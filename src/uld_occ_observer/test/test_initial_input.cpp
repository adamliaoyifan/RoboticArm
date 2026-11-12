#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/initial_empty.hpp"
#include "uld_occ_observer/validity.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/checkpoint_io.hpp"
#include <iostream>
#include <functional>
#include <cstdio>
#include <unistd.h>
using namespace uld_occ_observer;
int failures = 0;
void check(bool condition, const char* name) {
  if (!condition) { std::cerr << "FAIL " << name << '\n'; ++failures; }
}
void refuses(const std::function<void()>& f, const char* name) {
  try { f(); check(false, name); } catch (const Error&) {}
}
int main() {
  GeometryDescriptor geometry{"geometry", "artifact", 1., 1., 0., 1., {}};
  ObservationStore store(geometry, .1, {}, "ros", "configuration");
  refuses([&] { store.initialize_empty(1, "", "operator", 100); }, "blank declaration");
  auto id = store.initialize_empty(1, "session-empty", "operator", 100);
  check(id.epoch == 2 && id.map_revision == 1 && id.observed_revision == 0 &&
      id.last_integrated_stamp_ns == 0 && id.initialization_stamp_ns == 100, "honest declaration identity");
  const auto before = store.snapshot();
  check(before->cell({5,5,5}).known && before->cell({5,5,5}).value.declared_free &&
      !before->cell({5,5,5}).value.occupied && !before->cell({10,5,5}).active, "exact free active mask");
  check(store.initialize_empty(1, "session-empty", "operator", 200).epoch == 2,
      "replay retains epoch and declaration time");
  refuses([&] { store.initialize_empty(2, "other", "operator", 200); }, "second declaration refused");
  ValidityBoard board;
  board.commit_reset(id.epoch, "initial", "session-empty"); board.begin_building(id.epoch);
  check(board.publish_initialized(id)->state == ValidityState::kValid &&
      board.load()->reason == "declared_initial_empty", "separate declaration validity");
  SnapshotRepository repository(store);
  auto lease = repository.acquire(1000000000, 1, false, id.epoch);
  check(!lease.stale && lease.age_ns > 1, "initial condition is not fabricated fresh sensor data");
  OpticalObservation lidar;
  lidar.identity = {"livox", "scan1", "lidar", "ros", "geometry", 2, 1000000000};
  lidar.uld_from_optical.stamp_ns = lidar.identity.stamp_ns;
  lidar.uld_from_optical.translation = {0,0,.7};
  lidar.points_m = {{.25,0,-.25}};
  auto old = lidar; old.identity.stamp_ns = 50; old.uld_from_optical.stamp_ns = 50;
  check(store.integrate(old).status == "OBSERVATION_PREDATES_INITIALIZATION", "old acquisition cannot cross declaration");
  check(store.integrate(lidar).status == "NO_VALID_POINTS", "negative optical Z filtered");
  lidar.lidar_xyz = true;
  auto receipt = store.integrate(lidar);
  check(receipt.status == "INTEGRATED" && receipt.hit_keys == 1, "XYZ LiDAR negative Z admitted");
  auto cell = store.snapshot()->cell(*store.domain()->index_of({.25,0,.45}));
  check(cell.known && cell.value.occupied && !cell.value.declared_free, "first hit overrides initial free");
  check(before->cell(cell.index).value.declared_free, "old snapshot immutable");
  check(store.diagnostics().known_cells == before->known_entry_count(), "no double counted initialized cells");
  lidar.identity.sensor_id = "camera"; lidar.identity.observation_id = "depth1";
  lidar.identity.stamp_ns += 1; lidar.uld_from_optical.stamp_ns += 1;
  lidar.points_m = {{0,0,.6}}; lidar.uld_from_optical.translation = {0,0,0}; lidar.lidar_xyz = false;
  check(store.integrate(lidar).status == "INTEGRATED" &&
      store.snapshot()->identity().observed_revision == 2, "shared mixed modality writer");
  auto checkpoint = store.checkpoint();
  const auto path = std::string("/tmp/initial-input-checkpoint-") + std::to_string(getpid());
  write_checkpoint(checkpoint, path);
  auto loaded = read_checkpoint(path); std::remove(path.c_str());
  ObservationStore restored(geometry, .1, {}, "ros", "configuration");
  restored.restore(loaded, 1);
  check(restored.snapshot()->identity().initialization_id == "session-empty" &&
      restored.snapshot()->cell(cell.index).value.occupied &&
      restored.snapshot()->cell({1,1,1}).value.declared_free, "checkpoint retains prior and sensor provenance");
  store.reset(2);
  refuses([&] { store.snapshot(); }, "reset does not renew initial free");
  refuses([&] { repository.get(lease.handle); }, "reset invalidates lease");
  refuses([&] { store.initialize_empty(3, "session-empty", "operator", 200); }, "reset cannot reuse declaration");
  std::cout << "initial-input failures=" << failures << '\n';
  return failures ? 1 : 0;
}
