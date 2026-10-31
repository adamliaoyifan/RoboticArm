#include "uld_occ_observer/observation_terminal.hpp"

namespace uld_occ_observer {
namespace {
void fill_identity(UpdateReceipt& receipt, const std::string& sensor_id, const std::string& observation_id) {
  if (receipt.sensor_id.empty()) receipt.sensor_id = sensor_id;
  if (receipt.observation_id.empty()) receipt.observation_id = observation_id;
}
}

SealedTerminal seal_inflight(ObservationLedger& ledger, SnapshotWaitBoard& board,
                             std::uint64_t epoch, const std::string& sensor_id,
                             const std::string& observation_id, UpdateReceipt receipt,
                             std::chrono::steady_clock::time_point now, double queue_ms) {
  fill_identity(receipt, sensor_id, observation_id);
  SealedTerminal sealed;
  if (!ledger.complete(epoch, sensor_id, observation_id, receipt, now, queue_ms)) return sealed;
  const auto stored = ledger.query(epoch, sensor_id, observation_id);
  if (!stored) return sealed;
  sealed.stored = true;
  sealed.state = stored->state;
  sealed.receipt = stored->receipt;
  sealed.queue_ms = stored->queue_ms;
  SnapshotWaitBoard::Completion completion;
  completion.epoch = epoch;
  completion.sensor_id = sensor_id;
  completion.observation_id = observation_id;
  completion.input_stamp_ns = stored->receipt.input_stamp_ns;
  completion.observed_revision = stored->receipt.identity.observed_revision;
  if (stored->state == "COMPLETED") completion.status = "COMPLETED";
  else {
    completion.status = "REJECTED";
    completion.reason = stored->receipt.status;
  }
  board.note_completion(completion);
  return sealed;
}

void note_unstored_rejection(SnapshotWaitBoard& board, std::uint64_t epoch,
                             const std::string& sensor_id, const std::string& observation_id,
                             const std::string& reason, std::int64_t input_stamp_ns) {
  SnapshotWaitBoard::Completion completion;
  completion.epoch = epoch;
  completion.sensor_id = sensor_id;
  completion.observation_id = observation_id;
  completion.status = "REJECTED";
  completion.reason = reason;
  completion.input_stamp_ns = input_stamp_ns;
  board.note_completion(completion);
}
}  // namespace uld_occ_observer
