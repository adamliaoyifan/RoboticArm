#pragma once
#include "uld_occ_observer/occupancy_grid.hpp"
#include "uld_occ_observer/preprocessing.hpp"
#include "uld_occ_observer/snapshot.hpp"
#include "uld_occ_observer/profiling.hpp"
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

namespace uld_occ_observer {
struct UpdateReceipt {
  std::string status;
  std::string sensor_id, observation_id;
  std::int64_t input_stamp_ns{};
  SnapshotIdentity identity;
  std::size_t raw_points{}, accepted_points{}, free_keys{}, hit_keys{};
  std::size_t changed_keys{}, occupancy_changed_keys{}, rebuilt_blocks{};
  UpdateProfile profile;
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
                   std::string clock_domain, std::string config_digest,
                   bool discretize_endpoints = false);
  UpdateReceipt integrate(const OpticalObservation& observation,
                          ProfileMode profile_mode = ProfileMode::Off);
  std::shared_ptr<const Snapshot> snapshot() const;
  // Null when integrate holds the store mutex. Does not wait.
  std::shared_ptr<const Snapshot> try_snapshot() const;
  // The snapshot the store itself still owns. Does not build one and does not
  // require the caller to keep a handle. Empty before the first publish.
  std::shared_ptr<const Snapshot> retained_snapshot() const;
  Diagnostics diagnostics() const;
  std::uint64_t published_epoch() const;
  void collect_blocks(std::vector<std::shared_ptr<const CellBlock>>& out) const;
  std::uint64_t directory_entries() const;
  // Test probe. Invoked while integrate holds the store mutex. Production leaves it empty.
  void set_lock_probe_for_test(std::function<void()> probe);
  std::uint64_t reset(std::uint64_t expected_epoch);
  std::shared_ptr<const UldDomain> domain() const { return domain_; }
  Checkpoint checkpoint() const;
  std::uint64_t restore(const Checkpoint& checkpoint, std::uint64_t expected_epoch);
 private:
  SnapshotIdentity identity_unlocked() const;
  const std::shared_ptr<const UldDomain> domain_;
  const OpticalDepthPreprocessor preprocessor_;
  const std::string clock_domain_, config_digest_;
  const bool discretize_endpoints_;
  mutable std::mutex mutex_;
  OccupancyGrid grid_;
  BlockRegistry blocks_;
  mutable std::shared_ptr<const Snapshot> cached_snapshot_;
  std::uint64_t epoch_{1}, map_revision_{}, observed_revision_{};
  std::atomic<std::uint64_t> published_epoch_{1};
  std::function<void()> lock_probe_;
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
