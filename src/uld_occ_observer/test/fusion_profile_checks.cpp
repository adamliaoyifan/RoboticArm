#include "fusion_profile_checks.hpp"
#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/serialization.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace uld_occ_observer {
namespace {
void require(bool value, const std::string& message) {
  if (!value) throw Error(message);
}
OpticalObservation observation(std::string id, std::int64_t stamp, std::vector<Vec3> points) {
  OpticalObservation out;
  out.identity = {"sensor", std::move(id), "optical", "test_clock", "fixture_geometry", 1, stamp};
  out.uld_from_optical.stamp_ns = stamp;
  out.points_m = std::move(points);
  return out;
}
ObservationStore make_store(double resolution) {
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0.0, 1.2, {}};
  return ObservationStore(geometry, resolution, {}, "test_clock", "config");
}
const FusionBreakdown& breakdown(const UpdateReceipt& receipt) {
  return receipt.profile.fusion_breakdown;
}
double stage_ms(const UpdateProfile& profile, ProfileStage stage) {
  return profile.milliseconds[static_cast<std::size_t>(stage)];
}
void require_integrated_phases(const UpdateReceipt& receipt, const char* label) {
  const auto& timing = breakdown(receipt);
  require(receipt.status == "INTEGRATED", label);
  require(timing.collected, std::string(label) + " did not collect fusion breakdown");
  require(timing.visited[static_cast<std::size_t>(FusionPhase::ApplyBatch)] &&
              timing.visited[static_cast<std::size_t>(FusionPhase::BlockBatch)],
          std::string(label) + " missed a fusion phase");
  const auto apply = timing.milliseconds[static_cast<std::size_t>(FusionPhase::ApplyBatch)];
  const auto block = timing.milliseconds[static_cast<std::size_t>(FusionPhase::BlockBatch)];
  require(std::isfinite(apply) && std::isfinite(block) && apply >= 0 && block >= 0,
          std::string(label) + " phase duration");
  require(apply + block <= stage_ms(receipt.profile, ProfileStage::Fusion) + 0.1,
          std::string(label) + " phases exceed Fusion");
  double exclusive = 0;
  for (std::size_t i = 0; i < kExclusiveStages; ++i) exclusive += receipt.profile.milliseconds[i];
  require(exclusive <= receipt.total_ms + 0.1, std::string(label) + " exclusive spans exceed total");
  require(timing.mutation_storage_collected, std::string(label) + " missed mutation storage");
  require(timing.mutation_size == receipt.changed_keys, std::string(label) + " mutation size");
  require(timing.mutation_capacity >= timing.mutation_size, std::string(label) + " mutation capacity");
  require(timing.mutation_element_bytes == sizeof(OccupancyMutation),
          std::string(label) + " element size");
}
void require_off(const UpdateReceipt& receipt) {
  const auto& timing = breakdown(receipt);
  require(!receipt.profile.enabled(), "off mode enabled diagnostics");
  require(!timing.collected, "off collected fusion breakdown");
  require(!timing.visited[0] && !timing.visited[1], "off visited a fusion phase");
  require(!timing.mutation_storage_collected && timing.mutation_size == 0 &&
              timing.mutation_capacity == 0 && timing.mutation_element_bytes == 0 &&
              timing.milliseconds[0] == 0 && timing.milliseconds[1] == 0,
          "off wrote fusion breakdown counts");
  const YAML::Node node = profile_to_yaml(receipt.profile);
  const YAML::Node fusion = node["fusion_breakdown"];
  require(fusion["collected"].as<bool>() == false, "off yaml collected");
  require(fusion["apply_batch_ms"].IsDefined() && fusion["apply_batch_ms"].IsNull(),
          "off yaml presented apply time");
  require(fusion["block_batch_ms"].IsDefined() && fusion["block_batch_ms"].IsNull(),
          "off yaml presented block time");
  require(fusion["apply_batch_visited"].IsNull() && fusion["block_batch_visited"].IsNull(),
          "off yaml presented visited flags");
  require(fusion["mutation_storage"].as<std::string>() == "not_collected", "off yaml storage");
  require(fusion["mutation_size"].IsNull() && fusion["mutation_capacity"].IsNull() &&
              fusion["mutation_element_bytes"].IsNull(),
          "off yaml presented mutation capacity");
}
void require_same_identity(const SnapshotIdentity& left, const SnapshotIdentity& right,
                           const char* label) {
  require(left.epoch == right.epoch && left.map_revision == right.map_revision &&
              left.observed_revision == right.observed_revision &&
              left.last_integrated_stamp_ns == right.last_integrated_stamp_ns &&
              left.geometry_hash == right.geometry_hash &&
              left.config_digest == right.config_digest &&
              left.clock_domain == right.clock_domain,
          label);
}
void require_same_queries(ObservationStore& left, ObservationStore& right, const char* label) {
  require(left.checkpoint().tree_bytes == right.checkpoint().tree_bytes,
          std::string(label) + " tree bytes");
  const auto left_diag = left.diagnostics();
  const auto right_diag = right.diagnostics();
  require(left_diag.known_cells == right_diag.known_cells &&
              left_diag.tree_nodes == right_diag.tree_nodes &&
              left_diag.blocks == right_diag.blocks,
          std::string(label) + " counts");
  require_same_identity(left_diag.identity, right_diag.identity, label);
  if (!left_diag.identity.last_integrated_stamp_ns) return;
  const auto left_snapshot = left.snapshot();
  const auto right_snapshot = right.snapshot();
  const auto shape = left.domain()->shape();
  std::size_t cursor = 0;
  while (true) {
    const auto left_page = left_snapshot->read_cells({0, 0, 0}, shape, cursor, 4096);
    const auto right_page = right_snapshot->read_cells({0, 0, 0}, shape, cursor, 4096);
    require(left_page.cells.size() == right_page.cells.size() && left_page.done == right_page.done,
            std::string(label) + " roi page");
    for (std::size_t i = 0; i < left_page.cells.size(); ++i) {
      const auto& a = left_page.cells[i];
      const auto& b = right_page.cells[i];
      const bool same = a.index == b.index && a.active == b.active && a.known == b.known &&
                        a.value.occupied == b.value.occupied &&
                        (!a.known || a.value.log_odds == b.value.log_odds);
      if (!same) {
        throw Error(std::string(label) + " roi cell " + std::to_string(a.index.x) + "," +
                    std::to_string(a.index.y) + "," + std::to_string(a.index.z));
      }
    }
    if (left_page.done) break;
    cursor = left_page.next_cursor;
  }
  const auto left_surface = left_snapshot->surface();
  const auto right_surface = right_snapshot->surface();
  require(left_surface->size() == right_surface->size(), std::string(label) + " surface size");
  for (std::size_t i = 0; i < left_surface->size(); ++i) {
    const auto& a = (*left_surface)[i];
    const auto& b = (*right_surface)[i];
    const bool same = a.x == b.x && a.y == b.y && a.active_count == b.active_count &&
                      a.known_count == b.known_count && a.has_occupied == b.has_occupied &&
                      (!a.has_occupied || a.top_z == b.top_z);
    if (!same) {
      throw Error(std::string(label) + " surface column " + std::to_string(a.x) + "," +
                  std::to_string(a.y));
    }
  }
}
void check_resolution(double resolution) {
  ObservationStore off = make_store(resolution);
  ObservationStore stages = make_store(resolution);
  ObservationStore detailed = make_store(resolution);
  const std::vector<Vec3> points{{0.2, 0.2, 0.8}, {0.35, 0.15, 0.95}};
  std::vector<OpticalObservation> frames;
  frames.push_back(observation("a", 1000, points));
  frames.push_back(observation("b", 2000, points));
  auto exterior = observation("c", 3000, points);
  exterior.uld_from_optical.translation = {3, 0, 0};
  frames.push_back(exterior);
  frames.push_back(observation("d", 4000, {{0.1, 0.4, 0.7}}));
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto off_receipt = off.integrate(frames[i], ProfileMode::Off);
    const auto stages_receipt = stages.integrate(frames[i], ProfileMode::Stages);
    const auto detailed_receipt = detailed.integrate(frames[i], ProfileMode::Detailed);
    const auto label = "resolution " + std::to_string(resolution) + " frame " + std::to_string(i);
    require(off_receipt.status == stages_receipt.status &&
                stages_receipt.status == detailed_receipt.status,
            label + " status");
    require(off_receipt.free_keys == stages_receipt.free_keys &&
                stages_receipt.free_keys == detailed_receipt.free_keys &&
                off_receipt.hit_keys == detailed_receipt.hit_keys &&
                off_receipt.changed_keys == detailed_receipt.changed_keys &&
                off_receipt.occupancy_changed_keys == detailed_receipt.occupancy_changed_keys &&
                off_receipt.rebuilt_blocks == detailed_receipt.rebuilt_blocks,
            label + " receipt counts");
    require_same_identity(off_receipt.identity, stages_receipt.identity, label.c_str());
    require_same_identity(stages_receipt.identity, detailed_receipt.identity, label.c_str());
    require_off(off_receipt);
    if (stages_receipt.status == "INTEGRATED") {
      require_integrated_phases(stages_receipt, "stages");
      require_integrated_phases(detailed_receipt, "detailed");
      const YAML::Node node = profile_to_yaml(stages_receipt.profile);
      const YAML::Node fusion = node["fusion_breakdown"];
      require(fusion["collected"].as<bool>(), "stages yaml collected");
      require(fusion["apply_batch_visited"].as<bool>() && fusion["block_batch_visited"].as<bool>(),
              "stages yaml visited");
      require(fusion["apply_batch_ms"].as<double>() >= 0 && fusion["block_batch_ms"].as<double>() >= 0,
              "stages yaml duration");
      require(fusion["mutation_storage"].as<std::string>() == "collected", "stages yaml storage");
      require(fusion["mutation_size"].as<std::size_t>() == stages_receipt.changed_keys,
              "stages yaml mutation size");
    }
    require_same_queries(off, stages, label.c_str());
    require_same_queries(stages, detailed, label.c_str());
  }
}
}  // namespace

void FusionProfileChecks::check_phase_lifecycle() {
  OccupancyDiagnostic session(ProfileMode::Stages);
  double recorded = -1;
  {
    auto span = session.fusion_span(FusionPhase::ApplyBatch);
    auto moved = std::move(span);
    moved.finish();
    recorded = session.report().fusion_breakdown.milliseconds[0];
    moved.finish();
    require(session.report().fusion_breakdown.milliseconds[0] == recorded,
            "second finish recorded again");
  }
  require(session.report().fusion_breakdown.visited[0], "moved span did not record");
  require(session.report().fusion_breakdown.milliseconds[0] == recorded,
          "span destructor recorded a second time");
  OccupancyDiagnostic closed(ProfileMode::Stages);
  try {
    auto span = closed.fusion_span(FusionPhase::BlockBatch);
    throw std::runtime_error("leave scope");
  } catch (const std::runtime_error&) {}
  require(closed.report().fusion_breakdown.visited[1], "exception left the phase open");
  require(std::isfinite(closed.report().fusion_breakdown.milliseconds[1]) &&
              closed.report().fusion_breakdown.milliseconds[1] >= 0,
          "exception path duration");
  OccupancyDiagnostic off(ProfileMode::Off);
  {
    auto span = off.fusion_span(FusionPhase::ApplyBatch);
    span.finish();
  }
  off.record_mutation_storage(4, 8, sizeof(OccupancyMutation));
  require(!off.report().fusion_breakdown.collected, "off span collected");
  require(!off.report().fusion_breakdown.visited[0], "off span visited");
  require(!off.report().fusion_breakdown.mutation_storage_collected, "off storage call wrote");
  require(off.report().fusion_breakdown.milliseconds[0] == 0, "off span duration");
  OccupancyDiagnostic ignored(ProfileMode::Stages);
  {
    auto span = ignored.fusion_span(FusionPhase::Count);
    span.finish();
  }
  require(!ignored.report().fusion_breakdown.visited[0] &&
              !ignored.report().fusion_breakdown.visited[1],
          "Count wrote a phase slot");
}

void FusionProfileChecks::check_modes_and_equivalence() {
  check_resolution(0.01);
  check_resolution(0.05);
}

void FusionProfileChecks::check_sequence_and_failure() {
  const std::vector<Vec3> ray{{0.15, 0.15, 0.9}};
  for (double resolution : {0.01, 0.05}) {
    ObservationStore stages = make_store(resolution);
    ObservationStore off = make_store(resolution);
    int first_new = -1, first_old = -1, first_saturated = -1, last_changed = -1;
    std::size_t known_after_new = 0;
    for (int frame = 0; frame < 12; ++frame) {
      const auto sample = observation("ray" + std::to_string(frame), 1000 + frame, ray);
      const auto before = stages.diagnostics().known_cells;
      const auto receipt = stages.integrate(sample, ProfileMode::Stages);
      const auto shadow = off.integrate(sample, ProfileMode::Off);
      require(receipt.status == "INTEGRATED" && shadow.status == "INTEGRATED", "saturation status");
      require_integrated_phases(receipt, "saturation");
      require(receipt.changed_keys == shadow.changed_keys, "saturation changed_keys");
      if (frame == 0) {
        require(receipt.changed_keys > 0 && stages.diagnostics().known_cells > before, "first frame created no cells");
        first_new = frame;
        known_after_new = stages.diagnostics().known_cells;
      } else if (receipt.changed_keys > 0 && stages.diagnostics().known_cells == known_after_new &&
                 first_old < 0) {
        first_old = frame;
      }
      last_changed = static_cast<int>(receipt.changed_keys);
      if (receipt.changed_keys == 0 && first_saturated < 0) first_saturated = frame;
      if (receipt.changed_keys == 0) {
        require(breakdown(receipt).mutation_storage_collected && breakdown(receipt).mutation_size == 0,
                "saturated frame lost the collected-zero marker");
        const YAML::Node profile = profile_to_yaml(receipt.profile);
        const YAML::Node fusion = profile["fusion_breakdown"];
        require(fusion["mutation_storage"].as<std::string>() == "collected" &&
                    fusion["mutation_size"].as<std::size_t>() == 0,
                "saturated yaml dropped collected storage");
      }
    }
    require(first_new == 0 && first_old > 0 && first_saturated > first_old && first_saturated < 12 &&
                last_changed == 0,
            "saturation sequence did not record new, old, and unchanged frames");
    require_same_queries(stages, off, "saturated pair");
    const auto reversed = stages.integrate(observation("reverse", 2000, {{0.4, 0.3, 1.05}}),
                                           ProfileMode::Stages);
    const auto reversed_off = off.integrate(observation("reverse", 2000, {{0.4, 0.3, 1.05}}),
                                            ProfileMode::Off);
    require(reversed.status == "INTEGRATED" && reversed.changed_keys > 0 &&
                reversed.changed_keys == reversed_off.changed_keys,
            "reverse observation did not change");
    require_integrated_phases(reversed, "reverse");
    require_same_queries(stages, off, "reversed pair");
    std::cout << "fusion_profile saturation resolution=" << resolution
              << " new=" << first_new << " old=" << first_old
              << " saturated=" << first_saturated << " reversed=12\n";

    ObservationStore failed = make_store(resolution);
    ObservationStore reference = make_store(resolution);
    const auto prefix = observation("prefix", 1000, ray);
    require(failed.integrate(prefix, ProfileMode::Stages).status == "INTEGRATED", "failure prefix");
    require(reference.integrate(prefix, ProfileMode::Off).status == "INTEGRATED", "reference prefix");
    const auto before_tree = failed.checkpoint().tree_bytes;
    const auto before_diag = failed.diagnostics();
    const auto bad = failed.integrate(observation("oor", 2000, {{1.0e6, 0.2, 0.8}}), ProfileMode::Stages);
    require(bad.status == "ENDPOINT_OUT_OF_KEY_RANGE", "out-of-key frame status");
    require(breakdown(bad).collected, "failed stages session dropped collection mode");
    require(!breakdown(bad).visited[0] && !breakdown(bad).visited[1], "failed collect entered fusion");
    require(!breakdown(bad).mutation_storage_collected, "failed collect recorded mutations");
    require(failed.checkpoint().tree_bytes == before_tree, "failed collect changed the tree");
    const auto after_diag = failed.diagnostics();
    require(after_diag.known_cells == before_diag.known_cells &&
                after_diag.tree_nodes == before_diag.tree_nodes &&
                after_diag.identity.map_revision == before_diag.identity.map_revision &&
                after_diag.identity.observed_revision == before_diag.identity.observed_revision &&
                after_diag.identity.last_integrated_stamp_ns == before_diag.identity.last_integrated_stamp_ns,
            "failed collect changed identity");
    const auto recovery = observation("recover", 3000, {{0.25, 0.2, 0.85}});
    require(failed.integrate(recovery, ProfileMode::Stages).status == "INTEGRATED", "recovery");
    require(reference.integrate(recovery, ProfileMode::Off).status == "INTEGRATED", "reference recovery");
    require_same_queries(failed, reference, "recovery");
  }
}

void FusionProfileChecks::check_directory_readback() {
  auto store = make_store(0.05);
  const auto receipt = store.integrate(
      observation("readback", 1000, {{0.0, 0.0, 0.8}, {0.2, -0.1, 0.9}}), ProfileMode::Stages);
  require(receipt.status == "INTEGRATED" && receipt.rebuilt_blocks > 0 && receipt.changed_keys > 0,
          "directory readback frame");
  require_integrated_phases(receipt, "directory readback");
  const auto snap = store.snapshot();
  const auto shape = store.domain()->shape();
  std::size_t known = 0;
  std::size_t cursor = 0;
  const auto total = static_cast<std::size_t>(shape.x) * shape.y * shape.z;
  while (cursor < total) {
    const auto page = snap->read_cells({0, 0, 0}, shape, cursor, 4096);
    for (const auto& cell : page.cells) {
      if (!cell.known) {
        require(cell.value.log_odds == 0.f && !cell.value.occupied, "unknown cell kept a value");
        continue;
      }
      require(cell.active, "known cell outside the domain");
      ++known;
    }
    require(page.next_cursor > cursor, "roi page did not advance");
    cursor = page.next_cursor;
    if (page.done) break;
  }
  require(cursor == total && known == store.diagnostics().known_cells && known > 0,
          "directory readback lost or invented a known cell");
  const CellIndex probe{shape.x / 2, shape.y / 2, shape.z / 2};
  const auto held = snap->cell(probe);
  require(store.integrate(observation("later", 2000, {{0.3, 0.3, 1.1}})).status == "INTEGRATED",
          "later directory frame");
  const auto again = snap->cell(probe);
  require(again.known == held.known && again.value.log_odds == held.value.log_odds &&
              again.value.occupied == held.value.occupied &&
              snap->known_entry_count() == known,
          "old snapshot changed after a later block commit");
}
}  // namespace uld_occ_observer
