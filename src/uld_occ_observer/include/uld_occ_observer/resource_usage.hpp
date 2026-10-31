#pragma once
#include <cstdint>

namespace uld_occ_observer {
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
  std::uint64_t receipt_ledger_records{};
  std::uint64_t receipt_ledger_records_peak{};
  // std::map nodes, OcTree internals, and middleware buffers are outside these sums.
  bool uncovered_allocations{true};
};
}  // namespace uld_occ_observer
