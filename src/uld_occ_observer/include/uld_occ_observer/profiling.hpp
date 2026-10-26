#pragma once
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace uld_occ_observer {
enum class ProfileMode { Off, Stages, Detailed };
enum class ProfileStage : std::size_t {
  LockWait, Validation, Fingerprint, Dedup, DepthFilter, Transform,
  RayPreparation, Fusion, Finalization,
  RayGeneration, RayFilterDedup, TreeUpdate, BlockMaintenance, Count
};
inline constexpr std::size_t kExclusiveStages = 9;
inline constexpr std::size_t kProfileStages = static_cast<std::size_t>(ProfileStage::Count);
inline constexpr std::array<const char*, kProfileStages> kProfileStageNames{
  "lock_wait", "validation", "fingerprint", "dedup", "depth_filter", "transform",
  "ray_preparation", "fusion", "finalization", "ray_generation", "ray_filter_dedup",
  "tree_update", "block_maintenance"};
struct UpdateProfile {
  ProfileMode mode{ProfileMode::Off};
  std::array<double, kProfileStages> milliseconds{};
  std::array<bool, kProfileStages> visited{};
  double residual_ms{};
  std::uint64_t rays_attempted{}, ray_keys_generated{}, active_ray_key_visits{},
      inactive_ray_key_visits{}, rays_with_active_keys{}, rays_without_active_keys{},
      free_insert_duplicates{}, free_unique_before_hit_removal{}, copied_blocks{}, copied_cells{};
  bool enabled() const { return mode != ProfileMode::Off; }
  bool detailed() const { return mode == ProfileMode::Detailed; }
  void finish(double total_ms) {
    if (!enabled()) return;
    double accounted = 0;
    for (std::size_t i = 0; i < kExclusiveStages; ++i) accounted += milliseconds[i];
    residual_ms = total_ms-accounted;
  }
};
// Local scope accumulation only. Disabled scopes make no clock calls; clocks
// in the per-ray/per-key loops are enabled exclusively in Detailed (T3) mode.
class ProfileSpan {
 public:
  using Clock = std::chrono::steady_clock;
  ProfileSpan(UpdateProfile& profile, ProfileStage stage, bool nested = false)
      : profile_(profile), index_(static_cast<std::size_t>(stage)),
        enabled_(nested ? profile.detailed() : profile.enabled()) {
    if (enabled_) { profile_.visited[index_] = true; start_ = Clock::now(); }
  }
  ~ProfileSpan() {
    if (enabled_) profile_.milliseconds[index_] +=
      std::chrono::duration<double,std::milli>(Clock::now()-start_).count();
  }
  ProfileSpan(const ProfileSpan&) = delete;
  ProfileSpan& operator=(const ProfileSpan&) = delete;
 private:
  UpdateProfile& profile_;
  std::size_t index_;
  bool enabled_;
  Clock::time_point start_;
};
struct SurfaceProfile {
  bool cache_hit{};
  std::uint64_t scanned_cells{}, output_columns{};
  double lock_wait_ms{}, build_ms{}, total_ms{};
};
}  // namespace uld_occ_observer
