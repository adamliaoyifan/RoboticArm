#include "fusion_profile_checks.hpp"
#include "query_content_digest.hpp"
#include "uld_occ_observer/geometry.hpp"
#include "uld_occ_observer/occupancy_grid.hpp"
#include "uld_occ_observer/serialization.hpp"
#include <octomap/OcTree.h>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace uld_occ_observer {
namespace {
void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void configure_reference(octomap::OcTree& tree) {
  tree.setProbHit(.7); tree.setProbMiss(.4); tree.setOccupancyThres(.5);
  tree.setClampingThresMin(.1192); tree.setClampingThresMax(.971);
}
std::string bytes(const octomap::OcTree& tree) {
  std::ostringstream stream(std::ios::binary);
  check(tree.write(stream), "reference serialization failed");
  return stream.str();
}
std::vector<OccupancyMutation> original_apply(octomap::OcTree& tree, const OccupancyScan& scan) {
  std::vector<OccupancyMutation> changes;
  auto update = [&](const octomap::OcTreeKey& key, bool hit) {
    const auto* before = tree.search(key);
    const bool existed = before != nullptr;
    const float old_value = existed ? before->getLogOdds() : 0.f;
    const bool old_occupied = existed && tree.isNodeOccupied(before);
    const auto* after = tree.updateNode(key, hit, false);
    const float new_value = after->getLogOdds();
    if (existed && old_value == new_value) return;
    changes.push_back({cell_index_from_key(key), hit, existed, old_occupied,
                       tree.isNodeOccupied(after), old_value, new_value});
  };
  for (const auto& key : scan.free_cells) update(key, false);
  for (const auto& key : scan.occupied_cells) update(key, true);
  return changes;
}
void same_mutations(const std::vector<OccupancyMutation>& a,
                    const std::vector<OccupancyMutation>& b) {
  check(a.size() == b.size(), "ApplyBatch mutation count mismatch");
  for (std::size_t i = 0; i < a.size(); ++i) {
    check(a[i].index == b[i].index && a[i].hit == b[i].hit &&
          a[i].existed == b[i].existed && a[i].old_occupied == b[i].old_occupied &&
          a[i].occupied == b[i].occupied && a[i].old_log_odds == b[i].old_log_odds &&
          a[i].new_log_odds == b[i].new_log_odds, "ApplyBatch mutation content mismatch");
  }
}
}  // namespace

void FusionProfileChecks::check_apply_batch() {
  check_probability_digest();
  for (const auto resolution : {0.01, 0.02, 0.05}) {
    const GeometryDescriptor geometry{"apply", "digest", .2, .2, 0, .2, {}};
    const UldDomain domain(geometry, resolution);
    for (const auto mode : {ProfileMode::Off, ProfileMode::Stages, ProfileMode::Detailed}) {
      OccupancyGrid grid(resolution);
      octomap::OcTree reference(resolution);
      configure_reference(reference);
      // All eight keys share an immediate parent. Include a known zero leaf
      // elsewhere, which must never be confused with unknown.
      const auto zero_key = key_from_cell_index({2, 2, 2});
      reference.setNodeValue(zero_key, 0.f);
      grid = OccupancyGrid::fromCheckpointBytes(bytes(reference), resolution);
      bool saw_unchanged = false, saw_probability_only = false, saw_coarse_leaf = false;
      for (int frame = 0; frame < 36; ++frame) {
        OccupancyDiagnostic diagnostic(mode);
        OccupancyScan scan;
        // collect completes even an empty scan. Populate its public unique-key
        // lists to exercise the batch boundary independently from ray casting.
        grid.collect(scan, domain.center({0, 0, 0}), {}, domain, {}, diagnostic);
        for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z) {
          const auto key = key_from_cell_index({x, y, z});
          const bool hit = frame < 10 || frame >= 28;
          // First reverse update splits a previously pruned sibling group.
          if (frame == 10 && (x != 0 || y != 0 || z != 0)) continue;
          if (hit) scan.occupied_cells.insert(key);
          else scan.free_cells.push_back(key);
        }
        if (frame == 0) scan.free_cells.push_back(zero_key);
        if (frame == 35) { scan.free_cells.clear(); scan.occupied_cells.clear(); }
        const auto expected = original_apply(reference, scan);
        const auto actual = grid.apply(scan, diagnostic);
        same_mutations(actual, expected);
        saw_unchanged |= !scan.occupied_cells.empty() && actual.empty();
        for (const auto& mutation : actual)
          saw_probability_only |= mutation.existed && mutation.old_occupied == mutation.occupied;
        const auto candidate = grid.serialize();
        check(canonical_probability_digest(candidate) == canonical_probability_digest(bytes(reference)),
              "ApplyBatch full probability/unknown state mismatch");
        // Verify all finest cells, including unknown space and untouched siblings.
        auto restored = OccupancyGrid::fromCheckpointBytes(candidate, resolution);
        restored.validateLeaves(domain);
        std::size_t reference_known = 0, candidate_known = 0;
        const auto shape = domain.shape();
        for (int x = 0; x < shape.x; ++x) for (int y = 0; y < shape.y; ++y)
          for (int z = 0; z < shape.z; ++z)
            if (reference.search(key_from_cell_index({x, y, z}))) ++reference_known;
        restored.forEachLeaf([&](CellIndex index, float value) {
          const auto* node = reference.search(key_from_cell_index(index));
          check(node && node->getLogOdds() == value, "restored finest probability differs");
          ++candidate_known;
        });
        check(candidate_known == reference_known, "restored known count differs");
        const auto& apply = diagnostic.report().apply_breakdown;
        if (mode == ProfileMode::Off) {
          check(!apply.collected && apply.keys_considered == 0 && apply.written_keys == 0,
                "Off ApplyBatch collected diagnostics");
          const auto yaml = profile_to_yaml(diagnostic.report())["apply_breakdown"];
          check(yaml["write_loop_ms"].IsNull() && yaml["written_keys"].IsNull(),
                "Off ApplyBatch serialized a measurement");
        } else {
          check(apply.collected && apply.keys_considered == scan.free_cells.size() + scan.occupied_cells.size()
                && apply.written_keys == actual.size()
                && apply.unchanged_keys + apply.written_keys == apply.keys_considered,
                "ApplyBatch diagnostic accounting differs");
          for (double time : apply.milliseconds)
            check(std::isfinite(time) && time >= 0, "ApplyBatch invalid duration");
        }
        if (frame == 9) {
          saw_coarse_leaf = reference.search(key_from_cell_index({0, 0, 0})) ==
                            reference.search(key_from_cell_index({1, 1, 1}));
        }
        if (frame == 15) grid = std::move(restored); // restore then continue the same sequence
      }
      check(saw_unchanged && saw_probability_only && saw_coarse_leaf,
            "ApplyBatch fixture missed saturation/probability/pruning path");
    }
  }
  // Batch span ownership: finish and move must not double-record; Off is inert.
  OccupancyDiagnostic diagnostic(ProfileMode::Stages);
  { auto first = diagnostic.apply_span(ApplyPhase::WriteLoop); auto moved = std::move(first);
    moved.finish(); const auto elapsed = diagnostic.report().apply_breakdown.milliseconds[0];
    moved.finish(); check(elapsed == diagnostic.report().apply_breakdown.milliseconds[0],
                          "ApplySpan finish double records"); }
}
}  // namespace uld_occ_observer
