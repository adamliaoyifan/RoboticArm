#pragma once
#include "uld_occ_observer/preprocessing.hpp"
#include "uld_occ_observer/snapshot.hpp"
#include <octomap/OcTree.h>
#include <deque>
#include <mutex>

namespace uld_occ_observer {
struct UpdateReceipt {
  std::string status;
  std::string sensor_id, observation_id;
  std::int64_t input_stamp_ns{};
  SnapshotIdentity identity;
  std::size_t raw_points{}, accepted_points{}, free_keys{}, hit_keys{};
  std::size_t changed_keys{}, occupancy_changed_keys{}, rebuilt_blocks{};
  double preprocessing_ms{}, preparation_ms{}, fusion_ms{}, total_ms{};
};
struct Checkpoint {
  SnapshotIdentity identity;
  std::string tree_bytes;
  std::map<std::string, std::int64_t> sensor_stamps;
  std::map<std::pair<std::string, std::string>, std::string> fingerprints;
  std::deque<std::pair<std::string, std::string>> fingerprint_order;
};
struct Diagnostics {
  SnapshotIdentity identity;
  std::size_t tree_nodes{}, known_cells{}, blocks{}, estimated_tree_bytes{};
  std::uint64_t received{}, integrated{}, rejected{}, duplicates{}, full_serializations{};
};

// Single mutation authority. Every update is fully prepared before touching the tree.
// ROS scheduling/queues, file IO, handles and clock conversion live outside this core.
class ObservationStore {
 public:
  ObservationStore(GeometryDescriptor descriptor, double resolution, DepthZConfig depth,
                   std::string clock_domain, std::string config_digest);
  UpdateReceipt integrate(const OpticalObservation& observation);
  std::shared_ptr<const Snapshot> snapshot() const;
  Diagnostics diagnostics() const;
  std::uint64_t reset(std::uint64_t expected_epoch);
  std::shared_ptr<const UldDomain> domain() const { return domain_; }
  Checkpoint checkpoint() const;
  std::uint64_t restore(const Checkpoint& checkpoint, std::uint64_t expected_epoch);
 private:
  void configure_tree();
  SnapshotIdentity identity_unlocked() const;
  const std::shared_ptr<const UldDomain> domain_;
  const OpticalDepthPreprocessor preprocessor_;
  const std::string clock_domain_, config_digest_;
  mutable std::mutex mutex_;
  std::unique_ptr<octomap::OcTree> tree_;
  BlockRegistry blocks_;
  mutable std::shared_ptr<const Snapshot> cached_snapshot_;
  std::uint64_t epoch_{1}, map_revision_{}, observed_revision_{};
  std::int64_t last_integrated_stamp_ns_{};
  std::map<std::string, std::int64_t> sensor_stamps_;
  std::map<std::pair<std::string, std::string>, std::string> fingerprints_;
  std::deque<std::pair<std::string, std::string>> fingerprint_order_;
  std::size_t known_cells_{};
  std::uint64_t received_{}, integrated_{}, rejected_{}, duplicates_{};
  mutable std::uint64_t full_serializations_{};
};
std::string sha256(const std::string& bytes);
}  // namespace uld_occ_observer
