#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/observation_ingress.hpp"
#include "uld_occ_observer/snapshot_wait_board.hpp"
#include <algorithm>
#include <set>

namespace uld_occ_observer {
InflightResponse::InflightResponse(SnapshotRepository* repository, std::uint64_t bytes)
    : repository_(bytes == 0 ? nullptr : repository), bytes_(bytes) {
  if (repository_ && bytes_) repository_->adjust_inflight(static_cast<std::int64_t>(bytes_));
}
InflightResponse::InflightResponse(InflightResponse&& other) noexcept
    : repository_(other.repository_), bytes_(other.bytes_) {
  other.repository_ = nullptr;
  other.bytes_ = 0;
}
InflightResponse& InflightResponse::operator=(InflightResponse&& other) noexcept {
  if (this == &other) return *this;
  release();
  repository_ = other.repository_;
  bytes_ = other.bytes_;
  other.repository_ = nullptr;
  other.bytes_ = 0;
  return *this;
}
InflightResponse::~InflightResponse() { release(); }
void InflightResponse::release() noexcept {
  if (repository_ && bytes_) repository_->adjust_inflight(-static_cast<std::int64_t>(bytes_));
  repository_ = nullptr;
  bytes_ = 0;
}
InflightResponse SnapshotRepository::begin_response(std::uint64_t bytes) {
  return InflightResponse(this, bytes);
}
void SnapshotRepository::adjust_inflight(std::int64_t delta) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (delta >= 0) inflight_bytes_ += static_cast<std::uint64_t>(delta);
  else {
    const auto decrease = static_cast<std::uint64_t>(-delta);
    inflight_bytes_ = decrease >= inflight_bytes_ ? 0 : inflight_bytes_ - decrease;
  }
  inflight_peak_ = std::max(inflight_peak_, inflight_bytes_);
  // Occupancy accounting: route the delta so the composite peak sees
  // same-instant inflight bytes.
  if (delta > 0)
    AllocationTotals::instance().add("inflight_responses",
                                     static_cast<std::uint64_t>(delta));
  else
    AllocationTotals::instance().subtract(
        "inflight_responses", static_cast<std::uint64_t>(-delta));
}
void SnapshotRepository::expire_unlocked(Clock::time_point now, std::uint64_t epoch) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    const bool will_erase = it->second.expires <= now
        || it->second.snapshot->identity().epoch != epoch;
    if (will_erase) {
      AllocationTotals::instance().subtract("handle_directory_nodes", handle_node_bytes());
      it = entries_.erase(it);
    } else ++it;
  }
}
void SnapshotRepository::track_unlocked(const std::shared_ptr<const Snapshot>& snapshot) {
  if (!snapshot) return;
  readers_.erase(std::remove_if(readers_.begin(), readers_.end(),
      [](const std::weak_ptr<const Snapshot>& reader) { return reader.expired(); }), readers_.end());
  readers_.push_back(snapshot);
  // Retained weak-slot capacity of the reader vector.
  AllocationTotals::instance().set(
      "reader_tracking",
      static_cast<std::uint64_t>(readers_.capacity()) * 2 * sizeof(void*));
}
SnapshotLease SnapshotRepository::acquire(std::int64_t now_ns, std::int64_t max_age_ns,
    bool allow_stale, std::uint64_t expected_epoch,
    std::optional<std::uint64_t> expected_revision) {
  // Copy the latest snapshot before taking the repository mutex so this method
  // never waits for the store lock while holding the handle table.
  const auto latest = store_.snapshot();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  if (latest->identity().epoch != epoch) throw Error("EPOCH_MISMATCH");
  const auto now = Clock::now();
  expire_unlocked(now, epoch);
  if (expected_epoch && expected_epoch != epoch) throw Error("EPOCH_MISMATCH");
  auto selected = latest;
  if (expected_revision && *expected_revision != latest->identity().map_revision) {
    selected.reset();
    for (const auto& entry : entries_) {
      if (entry.second.snapshot->identity().map_revision == *expected_revision) {
        selected = entry.second.snapshot; break;
      }
    }
    if (!selected) throw Error("REVISION_UNAVAILABLE");
  }
  const auto& identity = selected->identity();
  if (now_ns < identity.last_integrated_stamp_ns || max_age_ns < 0) throw Error("INVALID_QUERY_TIME");
  const auto age = now_ns - identity.last_integrated_stamp_ns;
  if (age > max_age_ns && !allow_stale) throw Error("STALE");
  std::set<std::uint64_t> revisions;
  revisions.insert(identity.map_revision);
  for (const auto& entry : entries_) revisions.insert(entry.second.snapshot->identity().map_revision);
  if (entries_.size() >= 8 || revisions.size() > 2) throw Error("SNAPSHOT_PRESSURE");
  const auto handle = std::to_string(identity.epoch) + ":" + std::to_string(++next_handle_);
  entries_.emplace(handle, Entry{selected, now + ttl_});
  // Allocation-lifecycle accounting: a handle node is allocated here (the
  // key is SSO-sized for epoch:counter strings).
  AllocationTotals::instance().add("handle_directory_nodes", handle_node_bytes());
  note_handle_directory_unlocked(+1);
  track_unlocked(selected);
  return {handle, identity, age, age > max_age_ns};
}
std::optional<SnapshotLease> SnapshotRepository::try_acquire(
    std::int64_t now_ns, std::int64_t max_age_ns, bool allow_stale,
    std::uint64_t expected_epoch, std::optional<std::uint64_t> expected_revision) {
  const auto latest = store_.try_snapshot();
  if (!latest) return std::nullopt;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  if (latest->identity().epoch != epoch) throw Error("EPOCH_MISMATCH");
  const auto now = Clock::now();
  expire_unlocked(now, epoch);
  if (expected_epoch && expected_epoch != epoch) throw Error("EPOCH_MISMATCH");
  auto selected = latest;
  if (expected_revision && *expected_revision != latest->identity().map_revision) {
    selected.reset();
    for (const auto& entry : entries_) {
      if (entry.second.snapshot->identity().map_revision == *expected_revision) {
        selected = entry.second.snapshot; break;
      }
    }
    if (!selected) throw Error("REVISION_UNAVAILABLE");
  }
  const auto& identity = selected->identity();
  if (now_ns < identity.last_integrated_stamp_ns || max_age_ns < 0) throw Error("INVALID_QUERY_TIME");
  const auto age = now_ns - identity.last_integrated_stamp_ns;
  if (age > max_age_ns && !allow_stale) throw Error("STALE");
  std::set<std::uint64_t> revisions;
  revisions.insert(identity.map_revision);
  for (const auto& entry : entries_) revisions.insert(entry.second.snapshot->identity().map_revision);
  if (entries_.size() >= 8 || revisions.size() > 2) throw Error("SNAPSHOT_PRESSURE");
  const auto handle = std::to_string(identity.epoch) + ":" + std::to_string(++next_handle_);
  entries_.emplace(handle, Entry{selected, now + ttl_});
  AllocationTotals::instance().add("handle_directory_nodes", handle_node_bytes());
  note_handle_directory_unlocked(+1);
  track_unlocked(selected);
  return SnapshotLease{handle, identity, age, age > max_age_ns};
}
std::shared_ptr<const Snapshot> SnapshotRepository::get(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto epoch = store_.published_epoch();
  expire_unlocked(Clock::now(), epoch);
  const auto entry = entries_.find(handle);
  if (entry == entries_.end()) throw Error("INVALID_OR_EXPIRED_HANDLE");
  track_unlocked(entry->second.snapshot);
  return entry->second.snapshot;
}
void SnapshotRepository::release(const std::string& handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (entries_.erase(handle) != 0)
    AllocationTotals::instance().subtract("handle_directory_nodes", handle_node_bytes());
}
SnapshotRepository::~SnapshotRepository() {
  std::lock_guard<std::mutex> lock(mutex_);
  AllocationTotals::instance().subtract(
      "handle_directory_nodes",
      static_cast<std::uint64_t>(entries_.size()) * handle_node_bytes());
}
void SnapshotRepository::note_handle_directory_unlocked(std::int64_t delta) {
  // A high-water mark: only opens raise it, so erase passes nothing. The live
  // count stays entries_.size().
  if (delta > 0)
    handle_directory_peak_ = std::max<std::uint64_t>(handle_directory_peak_, entries_.size());
}
void SnapshotRepository::set_ttl_for_test(std::chrono::milliseconds ttl) { ttl_ = ttl; }
std::uint64_t SnapshotRepository::handle_node_bytes() {
  constexpr std::uint64_t kMapNodeOverhead = 8u + 3u * sizeof(void*);
  return kMapNodeOverhead + sizeof(std::map<std::string, Entry>::value_type);
}
std::uint64_t SnapshotRepository::block_node_bytes() {
  constexpr std::uint64_t kMapNodeOverhead = 8u + 3u * sizeof(void*);
  return kMapNodeOverhead + sizeof(BlockRegistry::value_type);
}
void SnapshotRepository::set_resource_scope_complete_for_test(bool complete) {
  std::lock_guard<std::mutex> lock(mutex_);
  resource_scope_complete_ = complete;
}
void SnapshotRepository::account(const ResourceAccounts& accounts) {
  std::lock_guard<std::mutex> lock(mutex_);
  accounts_ = accounts;
}
ResourceUsage SnapshotRepository::usage() {
  std::vector<std::shared_ptr<const CellBlock>> store_blocks;
  store_.collect_blocks(store_blocks);
  const auto store_directory_count = store_.directory_entries();
  const auto retained = store_.retained_snapshot();
  std::vector<std::shared_ptr<const Snapshot>> views;
  std::uint64_t inflight = 0;
  std::uint64_t inflight_peak = 0;
  std::uint64_t handle_entries = 0;
  std::uint64_t handle_peak = 0;
  ResourceAccounts accounts;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire_unlocked(Clock::now(), store_.published_epoch());
    readers_.erase(std::remove_if(readers_.begin(), readers_.end(),
        [](const std::weak_ptr<const Snapshot>& reader) { return reader.expired(); }), readers_.end());
    for (const auto& entry : entries_) views.push_back(entry.second.snapshot);
    for (const auto& reader : readers_)
      if (auto snapshot = reader.lock()) views.push_back(std::move(snapshot));
    if (retained) views.push_back(retained);
    inflight = inflight_bytes_;
    inflight_peak = inflight_peak_;
    handle_entries = entries_.size();
    handle_peak = handle_directory_peak_;
    accounts = accounts_;
  }
  // Node sizes derive from the actual container types, not from a shared
  // constant: a std::map node is the red-black overhead (color+padding plus
  // parent/left/right = 32 B on LP64) plus the padded pair payload. Capacity
  // equals size for a map node, so entries*node is the allocated capacity.
  const auto block_node = block_node_bytes();
  const auto handle_node = handle_node_bytes();
  std::set<const CellBlock*> current;
  for (const auto& block : store_blocks) if (block) current.insert(block.get());
  std::set<const CellBlock*> unique = current;
  std::set<const Snapshot*> seen_snapshots;
  std::set<const SurfaceCache*> seen_caches;
  std::uint64_t view_directory_count = 0;
  std::uint64_t surface = 0;
  std::uint64_t old_bytes = 0;
  for (const auto& snapshot : views) {
    if (!snapshot || !seen_snapshots.insert(snapshot.get()).second) continue;
    view_directory_count += snapshot->directory_entries();
    if (const auto* cache = snapshot->surface_cache()) {
      // Cache object (with its shared control block) plus the column buffer
      // at allocated capacity.
      if (seen_caches.insert(cache).second)
        surface += sizeof(SurfaceCache) + kControlBlockBytes
            + snapshot->surface_cache_allocated_bytes();
    }
    std::vector<std::shared_ptr<const CellBlock>> owned;
    snapshot->collect_blocks(owned);
    for (const auto& block : owned) {
      if (!block || !unique.insert(block.get()).second) continue;
      if (!current.count(block.get()))
        old_bytes += sizeof(CellBlock) + kControlBlockBytes;
    }
  }
  // Externally held surface buffers: surface() hands the column vector to
  // callers, and a reader can outlive the cache and snapshot that built it.
  // The tracker counts those bytes (added at build, released by the
  // buffer's deleter at the last owner); the walk above only sees buffers
  // reachable through snapshots, so the difference is exactly the external
  // share — never dropped from the live sums because "the owner is not in
  // the repository directory".
  const auto tracked_cache_live = AllocationTotals::instance().live("surface_caches");
  const auto external_buffers = tracked_cache_live > surface
      ? tracked_cache_live - surface : 0;
  surface += external_buffers;
  // The reader-tracking vector's retained capacity (weak_ptr slots).
  std::uint64_t readers_bytes = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    readers_bytes = static_cast<std::uint64_t>(readers_.capacity()) * sizeof(void*) * 2;
  }
  // Unique-ownership block sum: every live block exactly once (object plus
  // shared control block), whether it is current, view-exclusive, or shared.
  // old_bytes is the view-exclusive share of the same set and never adds to
  // it again.
  const auto block_bytes = static_cast<std::uint64_t>(unique.size())
      * (sizeof(CellBlock) + kControlBlockBytes);
  const auto block_directory_bytes = (store_directory_count + view_directory_count) * block_node;
  const auto handle_bytes = handle_entries * handle_node;
  const auto snapshot_objects = static_cast<std::uint64_t>(seen_snapshots.size())
      * (sizeof(Snapshot) + kControlBlockBytes);
  const auto store_tree = store_.diagnostics().estimated_tree_bytes;
  // The queue class is the owning container's retained-capacity model (chunk
  // high-water, never freed on pop); payload bytes are live.
  const auto queue_bytes = accounts.entry_queue_allocated_bytes;
  // Completion bytes come from the owning board's own node-inclusive account;
  // the ledger likewise reports its accounted bytes. Neither is re-estimated.
  const auto completion_bytes = accounts.completion_bytes;
  // Scope sums. read_view excludes surface caches; cache is exactly the
  // caches. read_view + cache counts every in-scope byte once: blocks are not
  // re-added via old_bytes and caches are not in both sums.
  const auto read_view = block_bytes + block_directory_bytes + handle_bytes + snapshot_objects
      + readers_bytes + inflight + queue_bytes + accounts.entry_payload_bytes
      + accounts.ledger_bytes + completion_bytes;
  const auto directories = store_directory_count + view_directory_count + handle_entries;
  const auto directory_bytes = block_directory_bytes + handle_bytes;
  std::lock_guard<std::mutex> lock(mutex_);
  block_peak_ = std::max(block_peak_, block_bytes);
  directory_peak_ = std::max(directory_peak_, directories);
  surface_peak_ = std::max(surface_peak_, surface);
  directory_bytes_peak_ = std::max(directory_bytes_peak_, directory_bytes);
  read_view_peak_ = std::max(read_view_peak_, read_view);
  cache_peak_ = std::max(cache_peak_, surface);
  // Occupancy peaks come from the allocation/release tracker, not from these
  // observations (a transient between samples is still counted there).
  ResourceUsage usage;
  usage.shared_block_bytes = block_bytes;
  usage.shared_block_bytes_peak = block_peak_;
  usage.old_view_block_bytes = old_bytes;
  usage.view_directory_entries = directories;
  usage.view_directory_entries_peak = directory_peak_;
  usage.handle_directory_entries_peak = handle_peak;
  usage.view_directory_bytes = directory_bytes;
  usage.view_directory_bytes_peak = directory_bytes_peak_;
  usage.surface_cache_bytes = surface;
  usage.surface_cache_bytes_peak = surface_peak_;
  usage.inflight_response_bytes = inflight;
  usage.inflight_response_bytes_peak = std::max(inflight_peak, inflight_peak_);
  usage.entry_queue_messages = accounts.entry_messages;
  usage.entry_queue_messages_peak = accounts.entry_message_peak;
  usage.entry_queue_payload_bytes = accounts.entry_payload_bytes;
  usage.entry_queue_payload_bytes_peak = accounts.entry_payload_bytes_peak;
  usage.receipt_ledger_records = accounts.ledger_records;
  usage.receipt_ledger_records_peak = accounts.ledger_record_peak;
  usage.receipt_ledger_bytes = accounts.ledger_bytes;
  usage.receipt_ledger_bytes_peak = accounts.ledger_bytes_peak;
  usage.completion_records = accounts.completion_records;
  usage.completion_records_peak = accounts.completion_records_peak;
  usage.store_tree_bytes = store_tree;
  usage.read_view_bytes = read_view;
  usage.read_view_bytes_peak = read_view_peak_;
  usage.cache_bytes = surface;
  usage.cache_bytes_peak = cache_peak_;
  // Occupancy peaks and cumulative flow from the tracker; flow is audit
  // only and never consumed by a gate.
  auto& totals = AllocationTotals::instance();
  usage.read_view_cache_bytes_peak = totals.combined_peak();
  usage.read_view_bytes_peak = totals.read_view_peak();
  usage.cache_bytes_peak = totals.cache_peak();
  usage.class_peaks = {
      {"block_allocations", totals.peak("block_allocations")},
      {"block_directory_nodes", totals.peak("block_directory_nodes")},
      {"snapshot_objects", totals.peak("snapshot_objects")},
      {"handle_directory_nodes", totals.peak("handle_directory_nodes")},
      {"surface_caches", totals.peak("surface_caches")},
      {"inflight_responses", totals.peak("inflight_responses")},
      {"entry_queue_retained", totals.peak("entry_queue_retained")},
      {"entry_queue_payload", totals.peak("entry_queue_payload")},
      {"receipt_ledger", totals.peak("receipt_ledger")},
      {"completion_board", totals.peak("completion_board")},
  };
  usage.class_flows = {
      {"block_allocations", totals.flow("block_allocations")},
      {"block_directory_nodes", totals.flow("block_directory_nodes")},
      {"snapshot_objects", totals.flow("snapshot_objects")},
      {"handle_directory_nodes", totals.flow("handle_directory_nodes")},
      {"surface_caches", totals.flow("surface_caches")},
  };
  usage.class_bytes = {
      {"block_allocations", block_bytes},
      {"block_directory_nodes", block_directory_bytes},
      {"handle_directory_nodes", handle_bytes},
      {"snapshot_objects", snapshot_objects},
      {"reader_tracking", readers_bytes},
      {"surface_caches", surface},
      {"inflight_responses", inflight},
      {"entry_queue_retained", queue_bytes},
      {"entry_queue_payload", accounts.entry_payload_bytes},
      {"receipt_ledger", accounts.ledger_bytes},
      {"completion_board", completion_bytes},
  };
  usage.class_basis = {
      {"block_allocations",
       "unique live blocks x (sizeof(CellBlock) + control block)"},
      {"block_directory_nodes",
       "(store + view registry entries) x (32 + sizeof(BlockRegistry::value_type))"},
      {"handle_directory_nodes",
       "open handles x (32 + sizeof(pair<const string, Entry)); handle keys are "
       "SSO-sized (<16 bytes) so no key heap"},
      {"snapshot_objects", "unique live snapshots x (sizeof(Snapshot) + control block)"},
      {"reader_tracking", "reader vector capacity x 2 pointers (weak_ptr slots)"},
      {"surface_caches",
       "unique caches x (sizeof(SurfaceCache) + control block + column buffer capacity)"},
      {"inflight_responses", "live response payload bytes, lifecycle-tracked"},
      {"entry_queue_retained",
       "ingress chunk model: (high-water chunks + spare) x chunk bytes, never freed on pop"},
      {"entry_queue_payload", "queued point-cloud payload bytes, lifecycle-tracked"},
      {"receipt_ledger", "ledger accounted bytes (map node + pair + heap strings)"},
      {"completion_board",
       "board accounted bytes (map nodes + pairs + heap strings + order deque chunks)"},
  };
  if (resource_scope_complete_) {
    // Every class of the read-view/cache scope is attributed above. This
    // certifies that scope only; the out-of-scope classes stay listed and no
    // broader resource claim is made. Live sums are the ownership walk at
    // sample time; *_alloc_bytes are monotone allocation-lifecycle totals
    // raised at the allocation sites (create-and-release between samples
    // still raises them). The queue/ledger/board/inflight peaks are tracked
    // by the owning containers at mutation time.
    usage.uncovered_allocations = false;
    usage.certification = "READ_VIEW_CACHE_ATTRIBUTED";
  } else {
    usage.uncovered_allocations = true;
    usage.certification = "NOT_EVALUATED";
    usage.uncovered_classes = {
        "block_allocations",
        "block_directory_nodes",
        "handle_directory_nodes",
        "surface_caches",
        "inflight_responses",
        "entry_queue_nodes",
        "receipt_ledger",
        "completion_board",
        "middleware_transport_buffers",
        "tf_buffer",
        "fingerprint_table"};
  }
  return usage;
}
}  // namespace uld_occ_observer
