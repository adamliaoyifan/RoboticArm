#pragma once
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace uld_occ_observer {
// Red-black node overhead for std::map nodes on LP64 (color+padding plus
// parent/left/right). A map node allocation is this plus the padded pair.
inline constexpr std::uint64_t kMapNodeOverhead = 8u + 3u * sizeof(void*);
// A make_shared control block is roughly two pointers folded into the same
// allocation as the object.
inline constexpr std::uint64_t kControlBlockBytes = 2u * sizeof(void*);
// Heap bytes actually allocated for a string: SSO strings (capacity within
// the local buffer) own nothing; a heap string owns capacity()+1 bytes
// including the terminator.
inline std::uint64_t heap_string_bytes(const std::string& text) {
  static const std::size_t kSsoCapacity = std::string{}.capacity();
  return text.capacity() > kSsoCapacity ? text.capacity() + 1 : 0;
}
// One map entry's allocated node: rb overhead plus the padded pair payload.
template <class Pair>
inline std::uint64_t map_node_bytes() {
  return kMapNodeOverhead + sizeof(Pair);
}

// Process-wide occupancy tracker. Every allocation site raises the class
// live level and every release lowers it, so peak(class) is the true
// max-over-time of simultaneously live bytes — not cumulative traffic. The
// same-instant composite peak over the read-view/cache scopes is maintained
// on every change (never a sum of per-class peaks measured at different
// times). Cumulative flow is kept separately and is never used by a gate.
class AllocationTotals {
 public:
  static AllocationTotals& instance() {
    static AllocationTotals totals;
    return totals;
  }
  void add(const char* name, std::uint64_t bytes) { change(name, bytes); }
  void subtract(const char* name, std::uint64_t bytes) { change(name, -static_cast<std::int64_t>(bytes)); }
  // Containers that report their whole current level at a mutation.
  void set(const char* name, std::uint64_t level) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::int64_t delta = static_cast<std::int64_t>(level)
        - static_cast<std::int64_t>(live_[name]);
    apply(name, delta);
    flow_[name] += delta > 0 ? static_cast<std::uint64_t>(delta) : 0;
  }
  std::uint64_t live(const char* name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return live_.count(name) ? live_.at(name) : 0;
  }
  std::uint64_t peak(const char* name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return peak_.count(name) ? peak_.at(name) : 0;
  }
  std::uint64_t flow(const char* name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return flow_.count(name) ? flow_.at(name) : 0;
  }
  // Same-instant composite peaks over the declared scopes.
  std::uint64_t read_view_peak() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return read_view_peak_;
  }
  std::uint64_t cache_peak() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cache_peak_;
  }
  std::uint64_t combined_peak() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return combined_peak_;
  }
  void reset_for_test() {
    std::lock_guard<std::mutex> lock(mutex_);
    live_.clear(); peak_.clear(); flow_.clear();
    read_view_live_ = 0; cache_live_ = 0;
    read_view_peak_ = 0; cache_peak_ = 0; combined_peak_ = 0;
  }
 private:
  static bool in_read_view(const std::string& name) {
    return name == "block_allocations" || name == "block_directory_nodes"
        || name == "snapshot_objects" || name == "handle_directory_nodes"
        || name == "entry_queue_retained" || name == "entry_queue_payload"
        || name == "receipt_ledger" || name == "completion_board"
        || name == "inflight_responses" || name == "reader_tracking";
  }
  static bool in_cache(const std::string& name) { return name == "surface_caches"; }
  void change(const char* name, std::int64_t delta) {
    if (delta == 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    apply(name, delta);
    if (delta > 0) flow_[name] += static_cast<std::uint64_t>(delta);
  }
  void apply(const std::string& name, std::int64_t delta) {
    std::int64_t next = static_cast<std::int64_t>(live_[name]) + delta;
    if (next < 0) next = 0;  // a release we did not observe an alloc for
    live_[name] = static_cast<std::uint64_t>(next);
    if (live_[name] > peak_[name]) peak_[name] = live_[name];
    if (in_read_view(name)) {
      read_view_live_ = read_view_live_ + delta > 0 ? read_view_live_ + delta : 0;
      if (read_view_live_ > read_view_peak_) read_view_peak_ = read_view_live_;
    }
    if (in_cache(name)) {
      cache_live_ = cache_live_ + delta > 0 ? cache_live_ + delta : 0;
      if (cache_live_ > cache_peak_) cache_peak_ = cache_live_;
    }
    const std::uint64_t combined = read_view_live_ + cache_live_;
    if (combined > combined_peak_) combined_peak_ = combined;
  }
  mutable std::mutex mutex_;
  std::map<std::string, std::uint64_t> live_, peak_, flow_;
  std::uint64_t read_view_live_{}, cache_live_{};
  std::uint64_t read_view_peak_{}, cache_peak_{}, combined_peak_{};
};

// Counts are ownership sums. Two scopes are declared:
//  - READ_VIEW/CACHE SCOPE (the O6 512 MiB gate): block allocations (shared by
//    the store and old views, counted once per unique block), block-directory
//    nodes (store registry + per-view registries + the handle directory),
//    surface caches, inflight responses, entry queue (nodes + payload), receipt
//    ledger, completion records. Every class in this scope is attributed with
//    a per-container node size derived from the actual container type; the
//    basis is published in node_bytes_basis(). certification
//    READ_VIEW_CACHE_ATTRIBUTED and uncovered_allocations=false state exactly
//    that scope and nothing broader.
//  - OUT OF SCOPE: middleware transport buffers and the tf buffer are not
//    observable from the node; the fingerprint table is store-side and not
//    counted. They stay listed in uncovered_classes and no gate may treat
//    them as attributed.
// A partially attributed scope keeps certification NOT_EVALUATED; a populated
// counter is never by itself a resource-gate pass.
//
// Peak semantics: the *_peak fields and class_peaks are occupancy peaks from
// the allocation/release tracker — max over time of simultaneously live
// bytes, decremented at every release, so transients between two usage()
// samples are captured. read_view_cache_bytes_peak is the same-instant sum
// over both scopes. class_flows keeps cumulative allocation traffic for
// audit only; no gate consumes it.
struct ResourceUsage {
  std::uint64_t shared_block_bytes{};
  std::uint64_t shared_block_bytes_peak{};
  // Old-view exclusive blocks: blocks kept alive only by view registries.
  // Already inside shared_block_bytes (the unique-ownership sum); reported
  // for observability and never added to it again.
  std::uint64_t old_view_block_bytes{};
  std::uint64_t view_directory_entries{};
  std::uint64_t view_directory_entries_peak{};
  // Lifecycle high-water of open handles specifically (mutation-path tracked),
  // inside the observed view_directory_entries_peak above.
  std::uint64_t handle_directory_entries_peak{};
  std::uint64_t view_directory_bytes{};
  std::uint64_t view_directory_bytes_peak{};
  std::uint64_t surface_cache_bytes{};
  std::uint64_t surface_cache_bytes_peak{};
  std::uint64_t inflight_response_bytes{};
  std::uint64_t inflight_response_bytes_peak{};
  std::uint64_t entry_queue_messages{};
  std::uint64_t entry_queue_messages_peak{};
  std::uint64_t entry_queue_payload_bytes{};
  std::uint64_t entry_queue_payload_bytes_peak{};
  std::uint64_t receipt_ledger_records{};
  std::uint64_t receipt_ledger_records_peak{};
  std::uint64_t receipt_ledger_bytes{};
  std::uint64_t receipt_ledger_bytes_peak{};
  std::uint64_t completion_records{};
  std::uint64_t completion_records_peak{};
  // Store-owned estimate from tree diagnostics. Reported separately; it never
  // enters the read-view/cache sum.
  std::uint64_t store_tree_bytes{};
  // read_view_bytes excludes the surface caches (the cache scope below) and
  // never re-adds old-view blocks. read_view + cache is the 512 MiB quantity
  // with each byte counted exactly once.
  std::uint64_t read_view_bytes{};
  std::uint64_t read_view_bytes_peak{};
  std::uint64_t cache_bytes{};
  std::uint64_t cache_bytes_peak{};
  // True occupancy peaks: max over time of simultaneously live bytes, from
  // the allocation/release tracker. read_view_cache_bytes_peak is the
  // same-instant sum over both scopes (never per-class peaks added across
  // different times). Cumulative flow lives in class_flows and is never used
  // by a gate.
  std::uint64_t read_view_cache_bytes_peak{};
  std::map<std::string, std::uint64_t> class_peaks{};
  std::map<std::string, std::uint64_t> class_flows{};
  // Per-class ownership sums for this scope; keys match node_bytes_basis().
  std::map<std::string, std::uint64_t> class_bytes{};
  // "class" -> "container type and node-size derivation" for every class in
  // class_bytes. A reader can recompute each class sum from the counts.
  std::map<std::string, std::string> class_basis{};
  bool uncovered_allocations{true};
  std::string certification{"NOT_EVALUATED"};
  std::vector<std::string> uncovered_classes{
      "middleware_transport_buffers",
      "tf_buffer",
      "fingerprint_table"};
};

struct ResourceAccounts {
  std::uint64_t entry_messages{};
  std::uint64_t entry_message_peak{};
  std::uint64_t entry_payload_bytes{};
  std::uint64_t entry_payload_bytes_peak{};
  // Retained deque capacity from the owning container's chunk model.
  std::uint64_t entry_queue_allocated_bytes{};
  std::uint64_t ledger_records{};
  std::uint64_t ledger_record_peak{};
  std::uint64_t ledger_bytes{};
  std::uint64_t ledger_bytes_peak{};
  std::uint64_t completion_records{};
  std::uint64_t completion_records_peak{};
  // Node-inclusive byte accounts straight from the owning containers; the
  // repository does not re-estimate these.
  std::uint64_t completion_bytes{};
  std::uint64_t completion_bytes_peak{};
};
}  // namespace uld_occ_observer
