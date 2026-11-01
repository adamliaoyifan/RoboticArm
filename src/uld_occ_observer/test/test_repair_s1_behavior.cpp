#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/observation_ingress.hpp"
#include "uld_occ_observer/observation_ledger.hpp"
#include "uld_occ_observer/observation_worker.hpp"
#include "uld_occ_observer/observation_terminal.hpp"
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
  expect(kept->surface() && !kept->surface()->empty(), "surface is available before reset");
  const auto epoch = store.reset(store.published_epoch());
  const auto after_reset = kept->read_cells({0, 0, 0}, {2, 2, 2}, 0, 8);
  expect(after_reset.cells.size() == 8, "read started before reset still completes after reset");
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
  expect(!baseline.uncovered_allocations
             && baseline.certification == "READ_VIEW_CACHE_ATTRIBUTED",
         "the read-view/cache scope is fully attributed");
  expect(baseline.view_directory_bytes >= baseline.view_directory_entries * 48u,
         "directory nodes are counted as allocated capacity");
  expect(baseline.read_view_bytes >= baseline.shared_block_bytes + baseline.surface_cache_bytes,
         "the read-view sum covers blocks, directory, and cache");
  expect(baseline.store_tree_bytes > 0, "store tree bytes are reported separately");
  limited.set_resource_scope_complete_for_test(false);
  const auto regressed = limited.usage();
  expect(regressed.uncovered_allocations && regressed.certification == "NOT_EVALUATED"
             && !regressed.uncovered_classes.empty(),
         "a class dropped from the sums returns the report to NOT_EVALUATED");
  limited.set_resource_scope_complete_for_test(true);
  expect(baseline.shared_block_bytes > 0, "shared blocks are counted once");
  (void)second;
}
void test_ledger_ingress_and_wait() {
  ObservationLedger ledger;
  const auto now = std::chrono::steady_clock::now();
  const auto admitted = ledger.admit(1, "cam0", "frame-1", now, 123456789);
  expect(admitted.accepted && admitted.observation_id == "frame-1", "ledger admits the caller id");
  expect(!ledger.admit(1, "cam0", "frame-1", now).accepted, "duplicate id is not admitted again");
  UpdateReceipt receipt;
  receipt.status = "INTEGRATED";
  receipt.observation_id = "frame-1";
  receipt.identity.observed_revision = 5;
  receipt.identity.map_revision = 3;
  ledger.complete(1, "cam0", "frame-1", receipt, now);
  expect(ledger.query(1, "cam0", "frame-1")->receipt.callback_enter_ns == 123456789,
         "completion carries the callback entry stamped at admission");
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
  SnapshotWaitBoard::Request overdue = request;
  overdue.observation_id = "full";
  overdue.timeout_ms = 10;
  SnapshotWaitBoard::Completion queued;
  queued.epoch = 1;
  queued.sensor_id = "cam0";
  queued.observation_id = "full";
  queued.status = "REJECTED";
  queued.reason = "QUEUE_FULL";
  board.note_completion(queued);
  const auto late = board.poll(overdue, now + std::chrono::milliseconds(50), 1000, 1, nullptr);
  expect(late && late->status == "REJECTED" && late->reason == "QUEUE_FULL",
         "a recorded rejection beats a later timeout");
}
void test_resource_ownership(const ObserverConfiguration& cfg) {
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
  expect(store.integrate(observation(cfg, "seed", 1000000000)).status == "INTEGRATED", "ownership seed");
  SnapshotRepository repo(store);
  const auto first = repo.acquire(2000000000, 5000000000, true);
  auto held = repo.get(first.handle);
  auto cache = held->surface();
  const auto one = repo.usage();
  const auto directories = one.view_directory_entries;
  const auto second = repo.acquire(2000000000, 5000000000, true);
  const bool same = repo.get(second.handle).get() == held.get();
  const auto two = repo.usage();
  expect(same && one.surface_cache_bytes > 0 && two.surface_cache_bytes == one.surface_cache_bytes,
         "same cache must be counted once for two handles");
  expect(two.view_directory_entries == directories, "same snapshot directory is counted once");
  expect(two.shared_block_bytes == one.shared_block_bytes, "same blocks stay deduplicated");
  store.reset(1);
  repo.release(first.handle);
  repo.release(second.handle);
  const auto retained = repo.usage();
  std::vector<std::shared_ptr<const CellBlock>> blocks;
  held->collect_blocks(blocks);
  const auto readable = held->read_cells({0, 0, 0}, {2, 2, 2}, 0, 8);
  expect(!blocks.empty() && retained.shared_block_bytes >= blocks.size() * sizeof(CellBlock),
         "released handle with live reader must retain resource attribution");
  expect(retained.surface_cache_bytes >= held->surface_cache_allocated_bytes(),
         "live reader cache must remain attributed after reset and release");
  expect(readable.cells.size() == 8, "live reader still sees the pre-reset cells");
  expect(retained.certification == "READ_VIEW_CACHE_ATTRIBUTED" && !retained.uncovered_allocations,
         "the scoped attribution stays complete across reset and release");
  expect(!retained.uncovered_classes.empty(), "out-of-scope classes stay listed");
  for (const auto& name : retained.uncovered_classes)
    expect(name == "middleware_transport_buffers" || name == "tf_buffer"
               || name == "fingerprint_table",
           "only declared out-of-scope classes remain uncovered");
  held.reset();
  cache.reset();
  const auto released = repo.usage();
  expect(released.shared_block_bytes == 0 && released.surface_cache_bytes == 0,
         "last owner release returns to the empty post-reset baseline");

  ObservationStore versions(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
  expect(versions.integrate(observation(cfg, "v1", 1000000000)).status == "INTEGRATED", "version one");
  SnapshotRepository version_repo(versions);
  const auto old_lease = version_repo.acquire(2000000000, 5000000000, true);
  auto old_view = version_repo.get(old_lease.handle);
  old_view->surface();
  const auto old_usage = version_repo.usage();
  expect(versions.integrate(observation(cfg, "v2", 3000000000)).status == "INTEGRATED", "version two");
  const auto new_lease = version_repo.acquire(4000000000, 5000000000, true);
  auto new_view = version_repo.get(new_lease.handle);
  new_view->surface();
  const auto both = version_repo.usage();
  expect(old_view.get() != new_view.get(), "a changed map publishes a new snapshot");
  expect(both.surface_cache_bytes >= old_usage.surface_cache_bytes + new_view->surface_cache_allocated_bytes()
             || old_view->surface_cache() == new_view->surface_cache(),
         "two live versions count each distinct surface cache");
  expect(both.old_view_block_bytes > 0, "blocks retained only by the old version stay attributed");
  version_repo.release(old_lease.handle);
  version_repo.release(new_lease.handle);
  old_view.reset();
  const auto baseline = version_repo.usage();
  expect(baseline.surface_cache_bytes == new_view->surface_cache_allocated_bytes(),
         "dropping the old reader leaves the current snapshot cache as the baseline");
  expect(baseline.old_view_block_bytes == 0, "old-version blocks leave with their last reader");
  new_view.reset();

  SnapshotRepository flights(versions);
  auto first_response = flights.begin_response(1000);
  auto second_response = flights.begin_response(2500);
  expect(flights.usage().inflight_response_bytes == 3500, "concurrent responses add");
  first_response = InflightResponse();
  expect(flights.usage().inflight_response_bytes == 2500, "releasing one response keeps the other");
  const auto peak = flights.usage().inflight_response_bytes_peak;
  second_response = InflightResponse();
  const auto done = flights.usage();
  expect(done.inflight_response_bytes == 0 && done.inflight_response_bytes_peak == peak && peak >= 3500,
         "the last response release returns inflight bytes to zero and keeps the peak");
}
void test_terminal_seal_and_bounds() {
  const auto now = std::chrono::steady_clock::now();
  ObservationLedger ledger;
  SnapshotWaitBoard board;
  UpdateReceipt missing;
  missing.status = "INTEGRATED";
  expect(!ledger.complete(1, "cam0", "absent", missing, now), "complete does not insert an unknown id");
  expect(!ledger.query(1, "cam0", "absent"), "an unknown id stays unseen");
  expect(ledger.admit(1, "cam0", "once", now, 42).accepted, "seal admit");
  UpdateReceipt queued;
  queued.status = "QUEUE_FULL";
  queued.sensor_id = "cam0";
  queued.observation_id = "once";
  queued.input_stamp_ns = 42;
  const auto sealed = seal_inflight(ledger, board, 1, "cam0", "once", queued, now, 1.5);
  expect(sealed.stored && sealed.state == "REJECTED" && sealed.receipt.status == "QUEUE_FULL",
         "seal stores the rejection before a caller publishes");
  expect(sealed.receipt.callback_enter_ns == 42 && sealed.receipt.input_stamp_ns == 42,
         "the sealed rejection carries the entry stamp and keeps the cloud stamp separate");
  UpdateReceipt later;
  later.status = "INTEGRATED";
  later.identity.observed_revision = 9;
  expect(!seal_inflight(ledger, board, 1, "cam0", "once", later, now).stored,
         "a second seal does not overwrite the terminal");
  const auto stored = ledger.query(1, "cam0", "once");
  expect(stored && stored->state == "REJECTED" && stored->receipt.status == "QUEUE_FULL" &&
             stored->receipt.input_stamp_ns == 42 && stored->queue_ms == 1.5 &&
             stored->receipt.callback_enter_ns == 42,
         "query returns the sealed receipt with its entry stamp intact");
  SnapshotWaitBoard::Request wait;
  wait.expected_epoch = 1;
  wait.sensor_id = "cam0";
  wait.observation_id = "once";
  wait.max_age_ms = 2000;
  wait.timeout_ms = 2000;
  wait.started = now + std::chrono::seconds(5);
  const auto rejection = board.poll(wait, now + std::chrono::seconds(5), 0, 1, nullptr);
  expect(rejection && rejection->status == "REJECTED" && rejection->reason == "QUEUE_FULL",
         "the waiter reads the sealed rejection rather than timing out");
  note_unstored_rejection(board, 1, "cam0", "unsaved", "LEDGER_FULL", 7);
  expect(!ledger.query(1, "cam0", "unsaved"), "a full ledger does not gain a row for the rejected id");
  const auto unsaved = board.find(1, "cam0", "unsaved");
  expect(unsaved && unsaved->reason == "LEDGER_FULL", "the rejection is still visible to waiters");
  ledger.mark(1, "cam0", "once", "PROCESSING", true, now);
  expect(ledger.query(1, "cam0", "once")->state == "REJECTED", "a terminal record is not reopened");

  ObservationLedger aged;
  aged.set_retention(std::chrono::milliseconds(5));
  expect(aged.admit(1, "cam0", "old", now).accepted, "retention admit");
  UpdateReceipt integrated;
  integrated.status = "INTEGRATED";
  integrated.observation_id = "old";
  aged.complete(1, "cam0", "old", integrated, now);
  aged.expire(now);
  expect(aged.query(1, "cam0", "old")->state == "COMPLETED", "query before retention stays completed");
  aged.expire(now + std::chrono::milliseconds(6));
  expect(aged.query(1, "cam0", "old")->state == "EXPIRED", "query after retention is expired");

  SnapshotWaitBoard bounded;
  for (std::size_t i = 0; i < SnapshotWaitBoard::kMaxCompletions + 3; ++i) {
    SnapshotWaitBoard::Completion completion;
    completion.epoch = 1;
    completion.sensor_id = "cam0";
    completion.observation_id = "n" + std::to_string(i);
    completion.status = "REJECTED";
    completion.reason = "QUEUE_FULL";
    bounded.note_completion(completion);
  }
  expect(bounded.size() == SnapshotWaitBoard::kMaxCompletions, "completion history is capped");
  expect(!bounded.find(1, "cam0", "n0"), "the oldest completion is dropped");
  expect(bounded.find(1, "cam0", "n" + std::to_string(SnapshotWaitBoard::kMaxCompletions + 2)).has_value(),
         "the newest completion is kept");
  bounded.drop_other_epochs(2);
  expect(bounded.size() == 0, "reset drops completions from the previous epoch");
  expect(bounded.accounted_bytes_peak() > 0, "completion history bytes were accounted before the drop");
}
}  // namespace

int main() {
  try {
    const auto cfg = load();
    test_read_while_store_locked(cfg);
    test_reset_ttl_pressure_and_bytes(cfg);
    test_resource_ownership(cfg);
    test_ledger_ingress_and_wait();
    test_terminal_seal_and_bounds();
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
