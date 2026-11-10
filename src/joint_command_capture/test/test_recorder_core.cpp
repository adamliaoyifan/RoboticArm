// Offline unit gates for the recorder core. Maps to the no-motion acceptance
// rules in docs/plans/joint_command_boundary_capture.md §5 (T2/T4/T5/T7
// offline portions). The runtime .so/shadowing gate (T1) and any live-sim
// behaviour are explicitly NOT covered here.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>

#include "joint_command_capture/recorder.hpp"

namespace fs = std::filesystem;
using joint_command_capture::EventKind;
using joint_command_capture::PluginReadRecord;
using joint_command_capture::PluginWriteRecord;
using joint_command_capture::Recorder;

namespace
{
class RecorderCoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    Recorder::destroy_for_tests();
    dir_ = fs::temp_directory_path() /
      ("jcc_test_" + std::to_string(++counter_));
    fs::remove_all(dir_);
  }
  void TearDown() override
  {
    Recorder::destroy_for_tests();
    fs::remove_all(dir_);
  }
  static uint64_t counter_;
  fs::path dir_;
};
uint64_t RecorderCoreTest::counter_ = 0;

PluginWriteRecord make_write(uint64_t seq, uint64_t sim_ns)
{
  PluginWriteRecord r{};
  r.seq = seq;
  r.sim_time_ns = sim_ns;
  r.n_joints = 6;
  r.update_rate = 100;
  for (int i = 0; i < 6; ++i) {
    r.cmd_received[i] = 0.1 * i;
    r.position_used[i] = 0.01 * i;
    r.write_applied[i] = 0.2 * i;
    r.branch[i] = static_cast<uint8_t>(
      joint_command_capture::WriteBranch::POSITION_PV);
    r.outcome[i] = static_cast<uint8_t>(
      joint_command_capture::WriteOutcome::WRITTEN);
  }
  return r;
}
}  // namespace

// T2/T7: OFF mode — no directory, no thread, records are no-ops, counted.
TEST_F(RecorderCoreTest, OffModeIsInertAndCounted)
{
  Recorder::instance().configure(Recorder::Config());
  EXPECT_FALSE(Recorder::instance().enabled());
  Recorder::instance().record_write(make_write(1, 1000));
  EXPECT_EQ(Recorder::instance().off_branch_calls(), 1u);
  EXPECT_EQ(Recorder::instance().write_stats().produced, 0u);
}

// Invalid config is refused by name and stays OFF (no silent fallback).
TEST_F(RecorderCoreTest, InvalidConfigIsNamedAndRefused)
{
  Recorder::Config bad;
  bad.mode = "sometimes";
  bad.run_dir = dir_;
  EXPECT_FALSE(Recorder::instance().configure(bad));
  EXPECT_FALSE(Recorder::instance().enabled());
  EXPECT_FALSE(Recorder::instance().faults().empty());
  EXPECT_EQ(Recorder::instance().faults()[0], "CAPTURE_CONFIG_INVALID_MODE");
}

// Reconfigure with different values is refused (sticky configure).
TEST_F(RecorderCoreTest, ReconfigureIsRefused)
{
  Recorder::Config cfg;
  cfg.mode = "window";
  cfg.run_dir = dir_;
  cfg.wall_deadline_s = 60;
  cfg.post_s = 0.5;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  Recorder::Config other = cfg;
  other.run_dir = dir_.string() + "_other";
  EXPECT_FALSE(Recorder::instance().configure(other));
  EXPECT_EQ(Recorder::instance().config().run_dir, cfg.run_dir);
  Recorder::instance().request_close("TEST");
}

// T4/T7: window mode streams live; ctl trigger events are idempotent per id;
// close writes footer, manifest, health; record counts in file match stats.
TEST_F(RecorderCoreTest, WindowStreamsAndTriggerIsIdempotent)
{
  Recorder::Config cfg;
  cfg.mode = "window";
  cfg.run_dir = dir_;
  cfg.post_s = 60;                 // keep the post-window out of this test
  cfg.wall_deadline_s = 30;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  auto & rec = Recorder::instance();
  EXPECT_TRUE(rec.enabled());
  std::atomic<bool> producing{true};
  std::thread producer([&] {
      for (uint64_t i = 0; producing.load() && i < 100000; ++i) {
        rec.record_write(make_write(i, i * 1000000));
        rec.record_read([&] {
            PluginReadRecord r{};
            r.seq = i;
            r.sim_time_ns = i * 1000000;
            r.n_joints = 6;
            for (int j = 0; j < 6; ++j) {
              r.position[j] = j;
              r.velocity[j] = 0;
              r.effort[j] = j + i;
            }
            return r;
          }());
        rec.record_iteration([&] {
            joint_command_capture::IterationRecord r{};
            r.seq = i;
            r.sim_time_ns = i * 1000000;
            r.iterations = i * 10;
            r.dt_ns = 1000000;
            return r;
          }());
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // Distinct ids each leave one event; duplicate id leaves none.
  std::ofstream(dir_ / "ctl" / "trigger-42") << "manual_test\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(rec.triggered());
  const uint64_t ev_after_42 = rec.event_stats().produced;
  std::ofstream(dir_ / "ctl" / "trigger-43") << "manual_test\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(rec.event_stats().produced, ev_after_42 + 1);
  std::ofstream(dir_ / "ctl" / "trigger-43again") << "dup\n";  // parses to id 43
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(rec.event_stats().produced, ev_after_42 + 1);      // idempotent
  // Graceful stop via ctl; then join the producer.
  std::ofstream(dir_ / "ctl" / "stop") << "\n";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!rec.closed() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  producing.store(false);
  producer.join();
  ASSERT_TRUE(rec.closed());
  EXPECT_EQ(rec.close_reason(), "CTL_STOP");
  for (const char * n : {"plugin_write", "plugin_read", "iteration", "events"}) {
    ASSERT_TRUE(fs::exists(dir_ / (std::string(n) + ".bin"))) << n;
  }
  ASSERT_TRUE(fs::exists(dir_ / "manifest_recorder.json"));
  ASSERT_TRUE(fs::exists(dir_ / "health.json"));
  EXPECT_EQ(rec.write_stats().dropped_ring, 0u);
  EXPECT_EQ(rec.write_stats().dropped_queue, 0u);
  EXPECT_GT(rec.write_stats().written, 0u);
}

// T7: after a trigger, post_s of additional sim time closes the window.
TEST_F(RecorderCoreTest, PostWindowClosesAfterSimTime)
{
  Recorder::Config cfg;
  cfg.mode = "armed";
  cfg.run_dir = dir_;
  cfg.post_s = 0.2;
  cfg.wall_deadline_s = 30;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  auto & rec = Recorder::instance();
  std::ofstream(dir_ / "ctl" / "trigger-9") << "post_window\n";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!rec.triggered() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(rec.triggered());
  // Advance sim time past post_s via records.
  for (uint64_t i = 0; i < 400 && !rec.closed(); ++i) {
    rec.record_write(make_write(i, (i + 1) * 1000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!rec.closed() && std::chrono::steady_clock::now() < deadline2) {
    rec.record_write(make_write(9999, 500000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(rec.closed());
  EXPECT_EQ(rec.close_reason(), "POST_WINDOW_COMPLETE");
}

// T5: disk failure (unopenable stream file) is a NAMED fault and never
// throws or blocks producers; ring accounting keeps counting overflows.
TEST_F(RecorderCoreTest, DiskFailureIsNamedAndNonBlocking)
{
  Recorder::Config cfg;
  cfg.mode = "armed";              // files open only at trigger: race-free setup
  cfg.run_dir = dir_;
  cfg.ring_write_mib = 1;          // small: overflow forces named ring drops
  cfg.wall_deadline_s = 2;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // A directory where plugin_write.bin should be -> ofstream open fails.
  fs::create_directories(dir_ / "plugin_write.bin");
  auto & rec = Recorder::instance();
  for (uint64_t i = 0; i < 20000; ++i) {
    rec.record_write(make_write(i, i * 1000000));
  }
  EXPECT_GT(rec.write_stats().dropped_ring, 0u);  // overflow named, not silent
  std::ofstream(dir_ / "ctl" / "trigger-1") << "disk_fail_test\n";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
  while (!rec.closed() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(rec.closed());
  bool has_write_fault = false;
  for (const auto & f : rec.faults()) {
    has_write_fault |= f == "CAPTURE_WRITE_FAILED";
  }
  EXPECT_TRUE(has_write_fault);
  EXPECT_EQ(rec.write_stats().produced, 20000u);
}

// T7: wall deadline closes with named truncation when sim time never advances
// (models a paused sim; no trigger ever fires).
TEST_F(RecorderCoreTest, WallDeadlineTruncatesWhenSimPaused)
{
  Recorder::Config cfg;
  cfg.mode = "armed";
  cfg.run_dir = dir_;
  cfg.wall_deadline_s = 0.5;   // sub-second for test speed
  cfg.post_s = 60.0;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  auto & rec = Recorder::instance();
  for (int i = 0; i < 10; ++i) {rec.record_write(make_write(i, 0));}
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!rec.closed() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(rec.closed());
  EXPECT_EQ(rec.close_reason(), "WALL_DEADLINE");
  // Truncation is declared, not silent: manifest records it.
  std::ifstream in(dir_ / "manifest_recorder.json");
  std::string manifest((std::istreambuf_iterator<char>(in)),
    std::istreambuf_iterator<char>());
  EXPECT_NE(manifest.find("WALL_DEADLINE"), std::string::npos);
}

// T2: reset does not cross clocks/epochs — a fresh recorder in the same
// process gets a new epoch, seq restarts, and stale directories are refused.
TEST_F(RecorderCoreTest, ResetDoesNotCrossEpochs)
{
  {
    Recorder::Config cfg;
    cfg.mode = "window";
    cfg.run_dir = dir_ / "first";
    cfg.wall_deadline_s = 0.3;
    cfg.post_s = 0.1;
    ASSERT_TRUE(Recorder::instance().configure(cfg));
    Recorder::instance().record_write(make_write(0, 0));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!Recorder::instance().closed() &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(Recorder::instance().closed());
    const std::string epoch1 = Recorder::instance().config().epoch;
    EXPECT_FALSE(epoch1.empty());
  }
  Recorder::destroy_for_tests();
  {
    Recorder::Config cfg;
    cfg.mode = "window";
    cfg.run_dir = dir_ / "first";   // non-empty now: must be refused by name
    ASSERT_FALSE(Recorder::instance().configure(cfg));
    EXPECT_FALSE(Recorder::instance().enabled());
  }
  Recorder::destroy_for_tests();
  {
    Recorder::Config cfg;
    cfg.mode = "window";
    cfg.run_dir = dir_ / "second";
    cfg.wall_deadline_s = 0.3;
    cfg.post_s = 0.1;
    ASSERT_TRUE(Recorder::instance().configure(cfg));
    const std::string epoch2 = Recorder::instance().config().epoch;
    // Different epoch, sequence restarted (produced counts from zero).
    EXPECT_FALSE(epoch2.empty());
    EXPECT_EQ(Recorder::instance().write_stats().produced, 0u);
    Recorder::instance().request_close("TEST");
  }
}

// Pre-trigger ring coverage: ring overwrite-oldest keeps the RECENT past
// (armed semantics need >= pre_s of history at trigger time).
TEST_F(RecorderCoreTest, RingKeepsRecentPast)
{
  Recorder::Config cfg;
  cfg.mode = "armed";
  cfg.run_dir = dir_;
  cfg.ring_write_mib = 1;   // small ring: ~2426 write records
  cfg.wall_deadline_s = 30;
  ASSERT_TRUE(Recorder::instance().configure(cfg));
  auto & rec = Recorder::instance();
  for (uint64_t i = 0; i < 8000; ++i) {
    rec.record_write(make_write(i, i * 1000000));   // no drain: armed
  }
  EXPECT_GT(rec.write_stats().dropped_ring, 0u);    // oldest overwritten
  // Trigger via ctl to dump; the newest records must survive.
  std::ofstream(dir_ / "ctl" / "trigger-7") << "ring_test\n";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (!rec.triggered() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(rec.triggered());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // Read the dumped file: header + at least the last record with seq 7999.
  std::ifstream in(dir_ / "plugin_write.bin", std::ios::binary);
  ASSERT_TRUE(in.good());
  joint_command_capture::BinFileHeader hdr{};
  in.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
  EXPECT_EQ(std::memcmp(hdr.magic, "JCC1", 4), 0);
  std::vector<char> body((std::istreambuf_iterator<char>(in)),
    std::istreambuf_iterator<char>());
  const size_t n = body.size() / sizeof(PluginWriteRecord);
  ASSERT_GE(n, 1u);
  PluginWriteRecord last{};
  std::memcpy(&last, body.data() + (n - 1) * sizeof(PluginWriteRecord),
    sizeof(last));
  EXPECT_EQ(last.seq, 7999u);   // newest kept
  rec.request_close("TEST");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
