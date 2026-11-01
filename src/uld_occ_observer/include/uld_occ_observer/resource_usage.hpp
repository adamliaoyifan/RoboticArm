#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace uld_occ_observer {
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
// Peak semantics (split deliberately):
//  - lifecycle peaks (inflight, entry queue, ledger, completions, handle
//    directory) are updated on the allocation/free path itself, so they bound
//    transients that fall between two samples;
//  - observed peaks (blocks, block directories, surface caches, the read-view
//    sum) are high-water marks of every usage() observation. A store-block
//    allocation and its reclamation between two queries is not claimed to be
//    captured by them.
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
