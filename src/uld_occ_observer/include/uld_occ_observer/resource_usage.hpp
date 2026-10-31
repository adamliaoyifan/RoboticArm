#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace uld_occ_observer {
// Counts are ownership sums. certification stays NOT_EVALUATED while any class
// below is still outside the sum; a populated counter is not a resource-gate pass.
struct ResourceUsage {
  std::uint64_t shared_block_bytes{};
  std::uint64_t shared_block_bytes_peak{};
  std::uint64_t old_view_block_bytes{};
  std::uint64_t view_directory_entries{};
  std::uint64_t view_directory_entries_peak{};
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
  bool uncovered_allocations{true};
  std::string certification{"NOT_EVALUATED"};
  std::vector<std::string> uncovered_classes{
      "octree_internal_nodes",
      "std_map_directory_nodes",
      "ledger_map_nodes",
      "ingress_queue_nodes",
      "completion_map_nodes",
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
