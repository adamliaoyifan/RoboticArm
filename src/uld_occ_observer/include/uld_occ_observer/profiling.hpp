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
  // Block entries visited by this rebuild. A cache hit visits none.
  std::uint64_t scanned_cells{}, output_columns{};
  double lock_wait_ms{}, build_ms{}, total_ms{};
  // Domain geometry scan is separate from scanned_cells. Off leaves this false;
  // zero counts on an uncollected profile are not a measurement.
  bool domain_init_collected{false};
  bool domain_init_performed{false};
  std::uint64_t domain_init_scanned_cells{};
  double domain_init_ms{};
};
// Coarse fusion intervals. Count is not a recordable phase.
enum class FusionPhase { ApplyBatch, BlockBatch, Count };
// ApplyBatch includes probability reads/writes, tree maintenance, and mutation creation. BlockBatch is block
// lookup, copy, per-cell maintenance, and registry writeback. Neither interval
// is only tree time or only allocation. Uncollected durations stay at zero in
// memory and are serialized as null, not as a measured zero.
struct FusionBreakdown {
  bool collected = false;
  std::array<bool, 2> visited{};
  std::array<double, 2> milliseconds{};
  bool mutation_storage_collected = false;
  std::size_t mutation_size{}, mutation_capacity{}, mutation_element_bytes{};
};
// Nested, frame-level intervals; no per-key clock or diagnostic mutation.
enum class ApplyPhase : std::size_t { WriteLoop, InnerUpdate, Prune, Count };
struct ApplyBreakdown {
  bool collected = false;
  std::array<bool, 3> visited{};
  std::array<double, 3> milliseconds{};
  std::size_t keys_considered{}, unchanged_keys{}, written_keys{};
};
// Report snapshot. Callers record through OccupancyDiagnostic, not by writing these fields.
// Per-key domain, insert, erase, and release clocks are not provided.
struct UpdateProfile {
  ProfileMode mode{ProfileMode::Off};
  std::array<double, kProfileStages> milliseconds{};
  std::array<bool, kProfileStages> visited{};
  double residual_ms{};
  std::uint64_t rays_attempted{}, ray_keys_generated{}, active_ray_key_visits{},
      inactive_ray_key_visits{}, rays_with_active_keys{}, rays_without_active_keys{},
      free_insert_duplicates{}, free_unique_before_hit_removal{}, copied_blocks{}, copied_cells{};
  bool surface_recorded{};
  SurfaceProfile surface{};
  FusionBreakdown fusion_breakdown;
  ApplyBreakdown apply_breakdown;
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
// The nine top-level spans and the Detailed nested spans are the only clocks.
class OccupancyDiagnostic {
 public:
  using Clock = std::chrono::steady_clock;

  explicit OccupancyDiagnostic(ProfileMode level) {
    report_.mode = level;
    if (report_.enabled()) report_.fusion_breakdown.collected = true;
  }
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

  Span span(ProfileStage stage) { return Span(*this, stage, false); }
  Span detail(ProfileStage stage) { return Span(*this, stage, true); }

  // One frame uses two of these, around apply and around block writeback.
  // finish() records at most once. The destructor finishes if the caller did not.
  // A moved-from span does not record. Off does not read the clock or write counts.
  class FusionSpan {
   public:
    FusionSpan(OccupancyDiagnostic& diagnostic, FusionPhase phase)
        : diagnostic_(&diagnostic), index_(static_cast<std::size_t>(phase)),
          enabled_(diagnostic.enabled() && index_ < 2) {
      if (!enabled_) return;
      diagnostic_->report_.fusion_breakdown.visited[index_] = true;
      start_ = Clock::now();
    }
    FusionSpan(FusionSpan&& other) noexcept
        : diagnostic_(other.diagnostic_), index_(other.index_), enabled_(other.enabled_),
          start_(other.start_) {
      other.enabled_ = false;
      other.diagnostic_ = nullptr;
    }
    FusionSpan(const FusionSpan&) = delete;
    FusionSpan& operator=(const FusionSpan&) = delete;
    FusionSpan& operator=(FusionSpan&&) = delete;
    void finish() noexcept {
      if (!enabled_) return;
      enabled_ = false;
      diagnostic_->report_.fusion_breakdown.milliseconds[index_] +=
          std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }
    ~FusionSpan() { finish(); }

   private:
    OccupancyDiagnostic* diagnostic_;
    std::size_t index_;
    bool enabled_;
    Clock::time_point start_{};
  };

  FusionSpan fusion_span(FusionPhase phase) { return FusionSpan(*this, phase); }
  void record_mutation_storage(std::size_t size, std::size_t capacity,
                               std::size_t element_bytes) {
    if (!enabled()) return;
    report_.fusion_breakdown.mutation_storage_collected = true;
    report_.fusion_breakdown.mutation_size = size;
    report_.fusion_breakdown.mutation_capacity = capacity;
    report_.fusion_breakdown.mutation_element_bytes = element_bytes;
  }

  class ApplySpan {
   public:
    ApplySpan(OccupancyDiagnostic& diagnostic, ApplyPhase phase)
        : diagnostic_(&diagnostic), index_(static_cast<std::size_t>(phase)),
          enabled_(diagnostic.enabled() && index_ < 3) {
      if (!enabled_) return;
      diagnostic_->report_.apply_breakdown.collected = true;
      diagnostic_->report_.apply_breakdown.visited[index_] = true;
      start_ = Clock::now();
    }
    ApplySpan(const ApplySpan&) = delete;
    ApplySpan& operator=(const ApplySpan&) = delete;
    ApplySpan(ApplySpan&& other) noexcept
        : diagnostic_(other.diagnostic_), index_(other.index_), enabled_(other.enabled_),
          start_(other.start_) { other.enabled_ = false; }
    void finish() noexcept {
      if (!enabled_) return;
      enabled_ = false;
      diagnostic_->report_.apply_breakdown.milliseconds[index_] +=
          std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    }
    ~ApplySpan() { finish(); }
   private:
    OccupancyDiagnostic* diagnostic_;
    std::size_t index_;
    bool enabled_;
    Clock::time_point start_{};
  };
  ApplySpan apply_span(ApplyPhase phase) { return ApplySpan(*this, phase); }
  // Derived once from functional results, rather than incremented on every key.
  void record_apply_counts(std::size_t considered, std::size_t written) {
    if (!enabled()) return;
    auto& apply = report_.apply_breakdown;
    apply.collected = true;
    apply.keys_considered = considered;
    apply.written_keys = written;
    apply.unchanged_keys = considered - written;
  }

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
