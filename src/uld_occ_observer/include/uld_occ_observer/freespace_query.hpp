#pragma once
#include "uld_occ_observer/snapshot.hpp"
#include <cstddef>
#include <vector>

namespace uld_occ_observer {

// Frozen placement policy echoed in every response (contract §2). These are
// the existing placement-side defaults; the query never reads ROS parameters.
struct PlacementPolicy {
  double support_tol_m{0.05};
  double min_support_ratio{0.6};
  double clearance_margin_m{0.03};      // top clearance to the ceiling
  double same_layer_clearance_m{0.05};  // lateral clearance to occupied cells
};

// One query request against an immutable snapshot. A request names exactly
// one box size; yaw_mode selects yaw 0, yaw 90, or both. cursor/limit page
// through the FEASIBLE candidate list in (yaw, cx, cy) ascending order; the
// enumeration itself is recomputed deterministically per call, so a page is
// always consistent with the snapshot identity it echoes.
struct FreespaceRequest {
  double box_length_m{}, box_width_m{}, box_height_m{};
  int yaw_mode{};  // 0 = yaw 0, 1 = yaw 90, 2 = both
  std::size_t cursor{}, limit{};
};

struct PlacementCandidate {
  int cell_x{}, cell_y{}, cell_z{};    // min-corner cell of the box volume
  double origin_x{}, origin_y{}, origin_z{};  // min-corner world corner (ULD frame)
  int yaw_deg{};
  double support_ratio{};
  // 1 = at least one observed occupied column supports the box,
  // 2 = support is entirely the floor prior (the floor exists; that is not
  //     observed evidence and downstream must keep the distinction).
  int support_kind{};
  int verify_begin[3]{}, verify_end[3]{};  // volume cells, end exclusive
  bool operator==(const PlacementCandidate& other) const {
    return cell_x == other.cell_x && cell_y == other.cell_y && cell_z == other.cell_z
        && origin_x == other.origin_x && origin_y == other.origin_y
        && origin_z == other.origin_z && yaw_deg == other.yaw_deg
        && support_ratio == other.support_ratio && support_kind == other.support_kind
        && verify_begin[0] == other.verify_begin[0] && verify_begin[1] == other.verify_begin[1]
        && verify_begin[2] == other.verify_begin[2] && verify_end[0] == other.verify_end[0]
        && verify_end[1] == other.verify_end[1] && verify_end[2] == other.verify_end[2];
  }
};

struct FreespacePage {
  std::vector<PlacementCandidate> candidates;
  std::size_t next_cursor{}, feasible_total{}, grid_total{};
  std::size_t rejected_out_of_domain{}, rejected_collision{}, rejected_clearance{},
      rejected_unknown{}, rejected_support{};
  bool done{};
  // Verification scope: the prefix-sum scratch covered exactly this region.
  int region_begin[3]{}, region_end[3]{};
};

// Full-volume known-free candidate query. Stateless and deterministic: the
// same snapshot + request always produce the same page. Semantics frozen in
// test/benchmark/freespace_contract.py -- see the module docstring there:
//   - unknown is never free; a candidate is returned only when EVERY cell of
//     its oriented volume is known free (ray-confirmed) and inside the hull;
//   - the contact surface and the internal volume are judged separately:
//     support = observed occupied columns within support_tol below the box
//     bottom, or the floor prior while the bottom is within support_tol of
//     the floor plane;
//   - the box bottom rests exactly on the highest observed support (the
//     existing placement peak semantics); positions floating within the
//     tolerance are not enumerated;
//   - rejection priority: out_of_domain, collision (occupied inside the
//     volume or within same_layer_clearance laterally with z overlap),
//     clearance (ceiling margin), unknown (volume has unobserved cells),
//     support (ratio below min_support_ratio);
//   - grid_total == feasible + the five rejection counters, per response.
FreespacePage query_freespace(const Snapshot& snapshot, const FreespaceRequest& request,
                              const PlacementPolicy& policy = PlacementPolicy{});
}  // namespace uld_occ_observer
