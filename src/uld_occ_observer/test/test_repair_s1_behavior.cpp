#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/observation_ingress.hpp"
#include "uld_occ_observer/observation_ledger.hpp"
#include "uld_occ_observer/observation_worker.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/snapshot_wait_board.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace uld_occ_observer;

namespace {
int failures = 0;
void expect(bool value, const char* message) {
  if (value) return;
  std::cerr << "FAIL " << message << '\n';
  ++failures;
}
ObserverConfiguration load() {
  const char* geometry = std::getenv("OCC_S1_GEOMETRY");
  const char* config = std::getenv("OCC_S1_CONFIG");
  if (!geometry) geometry = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_cuboid.yaml";
  if (!config) config = "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_cuboid_r0p05.yaml";
  return load_configuration(geometry, config);
}
OpticalObservation observation(const ObserverConfiguration& cfg, const std::string& id, std::int64_t stamp) {
  OpticalObservation item;
  item.identity.sensor_id = "cam0";
  item.identity.observation_id = id;
  item.identity.optical_frame = "d555_color_optical_frame";
  item.identity.clock_domain = cfg.clock_domain;
  item.identity.geometry_hash = cfg.geometry.geometry_hash;
  item.identity.epoch = 1;
  item.identity.stamp_ns = stamp;
  item.uld_from_optical.rotation = {0, 0, 1, -1, 0, 0, 0, -1, 0};
  item.uld_from_optical.translation = {-1.15, 0, 1.20};
  item.uld_from_optical.stamp_ns = stamp;
  item.points_m.push_back({0, 0, 1.0});
  return item;
}
void test_read_while_store_locked(const ObserverConfiguration& cfg) {
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
  const auto first = store.integrate(observation(cfg, "seed", 1000000000));
  expect(first.status == "INTEGRATED", "seed integrate");
  ObservationWorker health_reader(store);
  SnapshotRepository repository(store);
  const auto lease = repository.acquire(2000000000, 5000000000, true);
  std::atomic<bool> holding{false};
  std::atomic<bool> release{false};
  store.set_lock_probe_for_test([&] {
    holding.store(true);
    while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });
  std::thread writer([&] { store.integrate(observation(cfg, "blocked", 2000000000)); });
  while (!holding.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const auto started = std::chrono::steady_clock::now();
  const auto snapshot = repository.get(lease.handle);
  const auto columns = snapshot->surface();
  const auto page = snapshot->read_cells({0, 0, 0}, {2, 2, 2}, 0, 8);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  expect(columns && !columns->empty(), "surface during store lock");
  expect(page.cells.size() == 8, "roi during store lock");
  expect(elapsed < std::chrono::milliseconds(200), "read finished while store lock held");
  expect(holding.load(), "store lock still held after the read");
  const auto health = health_reader.diagnostics();
  expect(health.store.identity.epoch == store.published_epoch(), "health cache is readable while the store lock is held");
  release.store(true);
  writer.join();
  store.set_lock_probe_for_test({});
}
void test_reset_ttl_pressure_and_bytes(const ObserverConfiguration& cfg) {
  expect(SnapshotRepository::kDefaultTtl == std::chrono::seconds(5), "default ttl is 5s");
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
  expect(store.integrate(observation(cfg, "a", 1000000000)).status == "INTEGRATED", "first frame");
  SnapshotRepository repository(store);
  repository.set_ttl_for_test(std::chrono::milliseconds(30));
  const auto first = repository.acquire(2000000000, 5000000000, true);
  const auto kept = repository.get(first.handle);
  const auto before = repository.usage().shared_block_bytes;
  const auto second = repository.acquire(2000000000, 5000000000, true);
  const auto shared = repository.usage().shared_block_bytes;
  expect(shared == before, "second handle does not duplicate shared blocks");
  expect(store.integrate(observation(cfg, "b", 3000000000)).status == "INTEGRATED", "second frame");
  const auto latest = repository.acquire(4000000000, 5000000000, true);
  const auto with_old = repository.usage();
  repository.release(first.handle);
  const auto after_release = repository.usage();
  expect(after_release.shared_block_bytes <= with_old.shared_block_bytes, "release drops unshared blocks");
  expect(kept->surface() && !kept->surface()->empty(), "read started before reset still completes");
  const auto epoch = store.reset(store.published_epoch());
  expect(epoch == 2, "reset advances epoch");
  bool expired = false;
  try { repository.get(latest.handle); }
  catch (const Error& error) { expired = std::string(error.what()) == "INVALID_OR_EXPIRED_HANDLE"; }
  expect(expired, "old epoch handle cannot be read again");
  auto renewed = observation(cfg, "c", 5000000000);
  renewed.identity.epoch = epoch;
  const auto renewed_receipt = store.integrate(renewed);
  if (renewed_receipt.status != "INTEGRATED") std::cerr << "post-reset status " << renewed_receipt.status << "\n";
  expect(renewed_receipt.status == "INTEGRATED", "post-reset frame");
  SnapshotRepository fresh(store);
  fresh.set_ttl_for_test(std::chrono::milliseconds(20));
  const auto short_lease = fresh.acquire(6000000000, 5000000000, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  bool ttl = false;
  try { fresh.get(short_lease.handle); }
  catch (const Error& error) { ttl = std::string(error.what()) == "INVALID_OR_EXPIRED_HANDLE"; }
  expect(ttl, "ttl removes the handle");
  SnapshotRepository limited(store);
  int acquired = 0;
  bool pressure = false;
  for (int i = 0; i < 9; ++i) {
    try { limited.acquire(6000000000, 5000000000, true); ++acquired; }
    catch (const Error& error) { pressure = std::string(error.what()) == "SNAPSHOT_PRESSURE"; }
  }
  expect(acquired == 8 && pressure, "ninth handle is pressure");
  const auto baseline = limited.usage();
  expect(baseline.uncovered_allocations, "map and octree bytes stay uncovered");
  expect(baseline.shared_block_bytes > 0, "shared blocks are counted once");
  (void)second;
}
void test_ledger_ingress_and_wait() {
  ObservationLedger ledger;
  const auto now = std::chrono::steady_clock::now();
  const auto admitted = ledger.admit(1, "cam0", "frame-1", now);
  expect(admitted.accepted && admitted.observation_id == "frame-1", "ledger admits the caller id");
  expect(!ledger.admit(1, "cam0", "frame-1", now).accepted, "duplicate id is not admitted again");
  UpdateReceipt receipt;
  receipt.status = "INTEGRATED";
  receipt.observation_id = "frame-1";
  receipt.identity.observed_revision = 5;
  receipt.identity.map_revision = 3;
  ledger.complete(1, "cam0", "frame-1", receipt, now);
  const auto found = ledger.query(1, "cam0", "frame-1");
  expect(found && found->state == "COMPLETED" && found->receipt.identity.observed_revision == 5,
         "terminal is stored before a caller would publish");
  expect(!ledger.query(1, "cam0", "missing"), "unknown id is not seen");
  for (std::size_t i = 0; i < ObservationLedger::kCapacity - 1; ++i)
    expect(ledger.admit(1, "cam0", "flight-" + std::to_string(i), now).accepted, "fill in-flight ledger");
  expect(ledger.size() == ObservationLedger::kCapacity, "ledger is full");
  expect(ledger.in_flight() == ObservationLedger::kCapacity - 1, "the completed record stays out of the in-flight set");
  const auto refused = ledger.admit(1, "cam0", "one-more", now);
  expect(!refused.accepted && refused.status == "LEDGER_FULL" && refused.observation_id == "one-more",
         "full ledger rejects the new id and keeps in-flight records");
  expect(ledger.in_flight() == ObservationLedger::kCapacity - 1, "in-flight records were not evicted");
  ObservationLedger aged;
  expect(aged.admit(1, "cam0", "old", now).accepted, "aged admit");
  aged.complete(1, "cam0", "old", receipt, now);
  aged.expire(now + std::chrono::seconds(10));
  expect(aged.query(1, "cam0", "old")->state == "COMPLETED", "young terminal stays");
  aged.expire(now + ObservationLedger::kRetention + std::chrono::seconds(1));
  expect(aged.query(1, "cam0", "old")->state == "EXPIRED", "old terminal expires");

  ObservationIngress ingress;
  ObservationIngress::Item first{"cam0", "q-1", 1, {}};
  ObservationIngress::Item second{"cam0", "q-2", 1, {}};
  ObservationIngress::Item third{"cam0", "q-3", 1, {}};
  expect(ingress.try_admit(first).accepted, "first raw message");
  expect(ingress.try_admit(second).accepted, "second raw message");
  const auto full = ingress.try_admit(third);
  expect(!full.accepted && full.status == "QUEUE_FULL" && full.observation_id == "q-3",
         "third raw message is refused with its own id");
  expect(ingress.try_pop()->observation_id == "q-1", "adapter pops in admission order");
  expect(ingress.peak() == 2, "entry peak stays at the capacity that was reached");

  SnapshotWaitBoard board;
  SnapshotWaitBoard::Request request;
  request.expected_epoch = 1;
  request.sensor_id = "cam0";
  request.observation_id = "target";
  request.max_age_ms = 2000;
  request.timeout_ms = 2000;
  request.started = now;
  expect(!board.try_begin(0, request, 1), "first wait is accepted");
  const auto pressure = board.try_begin(SnapshotWaitBoard::kMaxWaiting, request, 1);
  expect(pressure && pressure->status == "WAIT_PRESSURE", "ninth wait is pressure");
  const auto mismatch = board.try_begin(0, request, 2);
  expect(mismatch && mismatch->status == "EPOCH_MISMATCH", "reset epoch rejects the wait");
  SnapshotIdentity stale_map;
  stale_map.epoch = 1;
  stale_map.map_revision = 9;
  stale_map.observed_revision = 4;
  stale_map.last_integrated_stamp_ns = 1000;
  expect(!board.poll(request, now, 1000, 1, &stale_map), "a newer map revision without the observed revision does not complete");
  board.note_completion({1, "cam0", "target", "COMPLETED", 5});
  expect(!board.poll(request, now, 1000, 1, &stale_map), "completion newer than the snapshot keeps waiting");
  SnapshotIdentity ready = stale_map;
  ready.observed_revision = 5;
  ready.map_revision = 3;
  const auto ok = board.poll(request, now, 1000, 1, &ready);
  expect(ok && ok->status == "OK" && ok->observed_revision == 5 && ok->map_revision == 3,
         "wait completes from observed revision");
  SnapshotWaitBoard::Request rejected = request;
  rejected.observation_id = "bad";
  board.note_completion({1, "cam0", "bad", "REJECTED", 0});
  const auto rejection = board.poll(rejected, now, 1000, 1, &ready);
  expect(rejection && rejection->status == "REJECTED", "target rejection is returned");
  const auto timed = board.poll(request, now + std::chrono::milliseconds(2000), 1000, 1, nullptr);
  expect(timed && timed->status == "TIMEOUT", "wait times out without a new frame");
}
}  // namespace

int main() {
  try {
    const auto cfg = load();
    test_read_while_store_locked(cfg);
    test_reset_ttl_pressure_and_bytes(cfg);
    test_ledger_ingress_and_wait();
  } catch (const std::exception& error) {
    std::cerr << "FAIL exception " << error.what() << '\n';
    return 1;
  }
  if (failures) {
    std::cerr << failures << " failures\n";
    return 1;
  }
  std::cout << "repair s1 behavior passed\n";
  return 0;
}
