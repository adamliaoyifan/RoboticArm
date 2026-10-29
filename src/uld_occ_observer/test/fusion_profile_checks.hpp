#pragma once

namespace uld_occ_observer {
// Package checks for fusion breakdown timing, mode equivalence, and failure recovery.
class FusionProfileChecks {
 public:
  void check_modes_and_equivalence();
  void check_phase_lifecycle();
  void check_sequence_and_failure();
  void check_directory_readback();
};
}  // namespace uld_occ_observer
