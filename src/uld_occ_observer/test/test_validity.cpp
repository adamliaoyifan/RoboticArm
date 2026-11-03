// Offline checks for the reset validity state machine (plan
// OCTOMAP_RESET_VALIDITY_NEXT_20261103 §2/§3): boot semantics, readiness
// predicate, idempotent duplicate reset, latched event order, and that no
// acquire admits a non-VALID epoch.
#include "uld_occ_observer/validity.hpp"

#include <cassert>
#include <cstdio>

using uld_occ_observer::ValidityBoard;
using uld_occ_observer::ValidityState;

int main() {
  ValidityBoard board;
  // Boot: BUILDING(1) — an empty store accepts observations, admits nothing.
  auto v = board.load();
  assert(v->state == ValidityState::kBuilding && v->epoch == 1 && v->generation == 1);
  assert(!board.admits(1));
  assert(board.events().size() == 1);

  // Readiness refuses: zero known cells (no accepted observation yet).
  v = board.try_publish_valid(1, 0, 1, 1, 1'000'000'000, 1'400'000'000,
                              100, 1'000'000'000, "g", "c", "ros", "observed");
  assert(v->state == ValidityState::kBuilding);
  // Refuses: stale publish (age beyond the 1 s bound).
  v = board.try_publish_valid(1, 10, 1, 1, 1'000'000'000, 3'000'000'000,
                              101, 1'000'000'000, "g", "c", "ros", "observed");
  assert(v->state == ValidityState::kBuilding);
  // Refuses: wrong epoch (a floor prior / old snapshot can never publish).
  v = board.try_publish_valid(2, 10, 1, 1, 1'000'000'000, 1'100'000'000,
                              102, 1'000'000'000, "g", "c", "ros", "observed");
  assert(v->state == ValidityState::kBuilding);
  // Accepts: accepted observation, fresh, matching epoch -> VALID(1).
  v = board.try_publish_valid(1, 10, 3, 5, 1'000'000'000, 1'100'000'000,
                              103, 1'000'000'000, "g", "c", "ros", "observed");
  assert(v->state == ValidityState::kValid && v->epoch == 1 && v->generation == 2);
  assert(v->map_revision == 3 && v->observed_revision == 5);
  assert(v->geometry_hash == "g" && v->config_digest == "c" && v->clock_domain == "ros");
  assert(board.admits(1));
  assert(!board.admits(2));

  // Reset: INVALID(2), duplicate reset_id is an idempotent replay.
  v = board.commit_reset(2, "scene_inconsistency", "r1");
  assert(v->state == ValidityState::kInvalid && v->epoch == 2 && v->generation == 3);
  assert(v->reason == "scene_inconsistency" && v->reset_id == "r1");
  assert(!board.admits(1) && !board.admits(2));
  const auto before = board.load();
  v = board.commit_reset(2, "scene_inconsistency", "r1");  // duplicate
  assert(v == before && v->generation == 3);
  assert(board.last_reset_id() == "r1" && board.last_reset_epoch() == 2);

  // BUILDING(2) then a distinct reset during building invalidates it again.
  v = board.begin_building(2);
  assert(v->state == ValidityState::kBuilding && v->generation == 4);
  v = board.commit_reset(3, "identity_change", "r2");
  assert(v->state == ValidityState::kInvalid && v->epoch == 3 && v->generation == 5);
  board.begin_building(3);
  v = board.try_publish_valid(3, 7, 1, 2, 2'000'000'000, 2'500'000'000,
                              104, 1'000'000'000, "g2", "c2", "ros", "observed");
  assert(v->state == ValidityState::kValid && v->epoch == 3 && v->generation == 7);
  assert(board.admits(3));

  // Latched events: strictly increasing generations, replayable in order.
  const auto events = board.events();
  assert(events.size() == 7);
  for (std::size_t i = 1; i < events.size(); ++i) {
    assert(events[i].generation == events[i - 1].generation + 1);
  }
  assert(events.front().state == ValidityState::kBuilding);  // boot
  assert(events.back().state == ValidityState::kValid);

  // Restore publishes VALID for the restored epoch only when it matches.
  board.commit_reset(4, "explicit", "r3");
  board.begin_building(4);
  const auto restored = board.publish_restored(4, 9, 9, 4'000'000'000, "g3", "c3", "ros");
  assert(restored->state == ValidityState::kValid && restored->epoch == 4);
  assert(restored->reason == "restored");
  const auto mismatch = board.publish_restored(5, 1, 1, 0, "g", "c", "ros");
  assert(mismatch == restored);  // unchanged: epoch must match the board

  std::puts("validity checks passed");
  return 0;
}
