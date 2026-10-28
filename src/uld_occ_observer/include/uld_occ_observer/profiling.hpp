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
struct SurfaceProfile {
  bool cache_hit{};
  std::uint64_t scanned_cells{}, output_columns{};
  double lock_wait_ms{}, build_ms{}, total_ms{};
};
// Report snapshot. Callers record through OccupancyDiagnostic, not by writing these fields.
struct UpdateProfile {
  ProfileMode mode{ProfileMode::Off};
  std::array<double, kProfileStages> milliseconds{};
  std::array<bool, kProfileStages> visited{};
  double residual_ms{};
  std::uint64_t rays_attempted{}, ray_keys_generated{}, active_ray_key_visits{},
      inactive_ray_key_visits{}, rays_with_active_keys{}, rays_without_active_keys{},
      free_insert_duplicates{}, free_unique_before_hit_removal{}, copied_blocks{}, copied_cells{};
  double domain_check_ms{}, free_insert_ms{}, occupied_insert_ms{}, hit_erase_ms{};
  double release_free_ms{}, release_occupied_ms{};
  bool surface_recorded{};
  SurfaceProfile surface{};
  bool enabled() const { return mode != ProfileMode::Off; }
  bool detailed() const { return mode == ProfileMode::Detailed; }
  void finish(double total_ms) {
    if (!enabled()) return;
    double accounted = 0;
    for (std::size_t i = 0; i < kExclusiveStages; ++i) accounted += milliseconds[i];
    residual_ms = total_ms-accounted;
  }
};

// One diagnostic session. Off does not read the clock or increment counts.
// Split timers read the clock only in Detailed.
class OccupancyDiagnostic {
 public:
  using Clock = std::chrono::steady_clock;
  enum class Cost {
    DomainCheck, FreeInsert, OccupiedInsert, HitErase, ReleaseFree, ReleaseOccupied
  };

  explicit OccupancyDiagnostic(ProfileMode level) { report_.mode = level; }
  bool enabled() const { return report_.enabled(); }
  bool detailed() const { return report_.detailed(); }
  const UpdateProfile& report() const { return report_; }
  void finish(double total_ms) { report_.finish(total_ms); }

  class Span {
   public:
    Span(OccupancyDiagnostic& diagnostic, ProfileStage stage, bool nested)
        : diagnostic_(&diagnostic), index_(static_cast<std::size_t>(stage)),
          enabled_(nested ? diagnostic.detailed() : diagnostic.enabled()) {
      if (enabled_) {
        diagnostic_->report_.visited[index_] = true;
        start_ = Clock::now();
      }
    }
    Span(Span&& other) noexcept
        : diagnostic_(other.diagnostic_), index_(other.index_), enabled_(other.enabled_),
          start_(other.start_) {
      other.enabled_ = false;
    }
    Span(const Span&) = delete;
    Span& operator=(const Span&) = delete;
    ~Span() {
      if (!enabled_) return;
      diagnostic_->report_.milliseconds[index_] +=
          std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }
   private:
    OccupancyDiagnostic* diagnostic_;
    std::size_t index_;
    bool enabled_;
    Clock::time_point start_{};
  };

  class Timer {
   public:
    Timer(OccupancyDiagnostic& diagnostic, Cost cost)
        : diagnostic_(&diagnostic), cost_(cost), enabled_(diagnostic.detailed()) {
      if (enabled_) start_ = Clock::now();
    }
    Timer(Timer&& other) noexcept
        : diagnostic_(other.diagnostic_), cost_(other.cost_), enabled_(other.enabled_),
          start_(other.start_) {
      other.enabled_ = false;
    }
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
    ~Timer() {
      if (!enabled_) return;
      const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
      auto& report = diagnostic_->report_;
      switch (cost_) {
        case Cost::DomainCheck: report.domain_check_ms += elapsed; break;
        case Cost::FreeInsert: report.free_insert_ms += elapsed; break;
        case Cost::OccupiedInsert: report.occupied_insert_ms += elapsed; break;
        case Cost::HitErase: report.hit_erase_ms += elapsed; break;
        case Cost::ReleaseFree: report.release_free_ms += elapsed; break;
        case Cost::ReleaseOccupied: report.release_occupied_ms += elapsed; break;
      }
    }
   private:
    OccupancyDiagnostic* diagnostic_;
    Cost cost_;
    bool enabled_;
    Clock::time_point start_{};
  };

  Span span(ProfileStage stage) { return Span(*this, stage, false); }
  Span detail(ProfileStage stage) { return Span(*this, stage, true); }
  Timer domain_check() { return Timer(*this, Cost::DomainCheck); }
  Timer free_insert() { return Timer(*this, Cost::FreeInsert); }
  Timer occupied_insert() { return Timer(*this, Cost::OccupiedInsert); }
  Timer hit_erase() { return Timer(*this, Cost::HitErase); }
  Timer release_free() { return Timer(*this, Cost::ReleaseFree); }
  Timer release_occupied() { return Timer(*this, Cost::ReleaseOccupied); }

  void ray_attempted() { if (enabled()) ++report_.rays_attempted; }
  void keys_generated(std::uint64_t count) { if (enabled()) report_.ray_keys_generated += count; }
  void active_visit(bool inserted) {
    if (!enabled()) return;
    ++report_.active_ray_key_visits;
    if (!inserted) ++report_.free_insert_duplicates;
  }
  void inactive_visit() { if (enabled()) ++report_.inactive_ray_key_visits; }
  void classify_ray(bool had_active_keys) {
    if (!enabled()) return;
    if (had_active_keys) ++report_.rays_with_active_keys;
    else ++report_.rays_without_active_keys;
  }
  void free_keys_before_hit_removal(std::uint64_t count) {
    if (enabled()) report_.free_unique_before_hit_removal = count;
  }
  void block_copied(std::uint64_t cells) {
    if (!enabled()) return;
    ++report_.copied_blocks;
    report_.copied_cells += cells;
  }
  void record_surface(const SurfaceProfile& sample) {
    report_.surface = sample;
    report_.surface_recorded = true;
  }

 private:
  UpdateProfile report_;
};
}  // namespace uld_occ_observer
