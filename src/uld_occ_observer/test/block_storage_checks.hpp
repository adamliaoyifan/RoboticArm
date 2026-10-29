#pragma once

namespace uld_occ_observer {
// Addressing and one-frame block-directory checks. Cell representation checks
// stay beside these once the block storage changes.
class BlockStorageChecks {
 public:
  void check_addressing();
  void check_batch_lifecycle();
};
}  // namespace uld_occ_observer
