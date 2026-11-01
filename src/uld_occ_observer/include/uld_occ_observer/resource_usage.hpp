#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace uld_occ_observer {
// Counts are ownership sums. Two scopes are declared:
//  - READ_VIEW/CACHE SCOPE (the O6 512 MiB gate): shared blocks, old-view
//    blocks, view-directory nodes, surface caches, inflight responses, entry
//    queue, receipt ledger, completion records. Every class in this scope is
//    attributed by construction; certification READ_VIEW_CACHE_ATTRIBUTED and
//    uncovered_allocations=false state exactly that and nothing broader.
//  - OUT OF SCOPE: middleware transport buffers and the tf buffer are not
//    observable from the node; the fingerprint table is store-side and not
//    counted. They stay listed in uncovered_classes and no gate may treat
//    them as attributed.
// A partially attributed scope keeps certification NOT_EVALUATED; a populated
// counter is never by itself a resource-gate pass.
struct ResourceUsage {
  std::uint64_t shared_block_bytes{};
  std::uint64_t shared_block_bytes_peak{};
  std::uint64_t old_view_block_bytes{};
  std::uint64_t view_directory_entries{};
  std::uint64_t view_directory_entries_peak{};
  // Allocated-node estimate for the handle directory: entries times the
  // per-node size of one map entry (key string + Entry + rb-tree node).
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
  std::uint64_t read_view_bytes{};
  std::uint64_t read_view_bytes_peak{};
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
};
}  // namespace uld_occ_observer
