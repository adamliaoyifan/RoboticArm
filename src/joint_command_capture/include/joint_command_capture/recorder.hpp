// Recorder core for the joint command boundary capture. ROS- and Gazebo-free.
//
// Guarantees (docs/plans/joint_command_boundary_capture.md §2/§4):
//  - control/physics callbacks only memcpy fixed-length records into
//    preallocated rings: no allocation, no disk, no locks on hot paths;
//  - OFF mode: no thread, no file, no allocation; the remaining branch cost
//    of the instrumentation call itself is measurable (off_branch_calls);
//  - full ring / full staging queue / disk failure / truncation are NAMED
//    counters and events, never silent drops;
//  - control-directory polling runs ONLY on the flush thread;
//  - external trigger ids are idempotent (the manual service writes the same
//    control files with the same numbering);
//  - wall-clock deadline bounds a paused sim: closing is a named truncation.
#ifndef JOINT_COMMAND_CAPTURE__RECORDER_HPP_
#define JOINT_COMMAND_CAPTURE__RECORDER_HPP_

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "joint_command_capture/records.hpp"

namespace joint_command_capture
{

class Recorder
{
public:
  enum class Mode { OFF, ARMED, WINDOW };

  struct Config
  {
    std::string mode = "off";              // off | armed | window
    std::string run_dir;                   // fresh run directory
    double pre_s = 15.0;                   // armed ring must cover this
    double post_s = 60.0;                  // live capture after trigger
    double wall_deadline_s = 600.0;        // hard wall close
    size_t ring_write_mib = 48;
    size_t ring_read_mib = 12;
    size_t ring_iter_mib = 2;
    size_t ring_event_mib = 2;
    size_t queue_mib = 32;                 // staging queue budget
    size_t flush_mib = 256;                // max bytes per stream file
    std::string watermark;                 // version + git sha (build stamped)
    std::string epoch;                     // uuid hex; generated if empty

    static Config from_env();              // JOINT_CAPTURE_* variables
    bool valid(std::string & why) const;
  };

  struct StreamStats
  {
    uint64_t produced = 0, dropped_ring = 0, dropped_queue = 0, written = 0;
  };

  static Recorder & instance();

  // TEST-ONLY hook so the reset/epoch-isolation gate can run in-process.
  // Production code must never call this.
  static void destroy_for_tests();

  // First non-OFF configure wins; identical re-configure is idempotent,
  // anything else is refused (sticky). Configure after close is refused.
  bool configure(const Config & cfg);
  bool enabled() const { return active_.load(std::memory_order_relaxed); }
  Mode mode() const { return mode_; }
  bool triggered() const { return triggered_; }
  bool closed() const { return closed_.load(std::memory_order_relaxed); }
  const Config & config() const { return cfg_; }

  // Producer API (gz thread). Never throws, never blocks, never allocates
  // past the preallocated rings.
  void record_write(const PluginWriteRecord & rec);
  void record_read(const PluginReadRecord & rec);
  void record_iteration(const IterationRecord & rec);
  // External id: idempotent across manual-service and ctl-file sources.
  void record_event(uint64_t external_id, EventKind kind, const char * text);
  void record_event_internal(EventKind kind, const char * text);

  // Graceful close request from any thread (join happens in destructor).
  void request_close(const char * reason);
  std::string close_reason() const;

  const StreamStats & write_stats() const { return w_stats_; }
  const StreamStats & read_stats() const { return r_stats_; }
  const StreamStats & iter_stats() const { return i_stats_; }
  const StreamStats & event_stats() const { return e_stats_; }
  uint64_t off_branch_calls() const
  {
    return off_calls_.load(std::memory_order_relaxed);
  }
  const std::vector<std::string> & faults() const { return faults_; }
  std::string health_json() const;

  ~Recorder();

private:
  Recorder() = default;
  Recorder(const Recorder &) = delete;
  Recorder & operator=(const Recorder &) = delete;

  template<typename T>
  struct Ring
  {
    explicit Ring(size_t capacity_records)
    : mask(capacity_records - 1), slots(capacity_records) {}
    size_t mask;
    std::vector<T> slots;                  // preallocated, fixed length
    std::atomic<uint64_t> head{0};         // producer position (monotonic)
    std::atomic<uint64_t> tail{0};         // oldest valid position
    uint64_t dropped = 0;                  // overwritten-oldest count
    void push(const T & rec) noexcept;     // overwrite-oldest SPSC
  };

  struct FileStream
  {
    std::string path;
    std::ofstream out;
    uint64_t bytes_written = 0;
    uint64_t records_written = 0;
    uint32_t record_kind = 0;
    bool open = false;
  };

  void flush_loop();
  bool ensure_run_dir();
  bool open_stream_files();
  void close_stream_files(const char * reason);
  void poll_control_dir();
  void handle_trigger(uint64_t external_id, const std::string & reason);
  void note_fault(const std::string & what);
  template<typename T>
  void drain_ring(
    Ring<T> & ring, uint64_t & consumer, StreamStats & stats, FileStream & file);
  bool check_deadlines();
  void write_health();
  void write_manifest(const char * reason);

  Config cfg_;
  std::atomic<bool> active_{false};
  std::atomic<bool> closed_{false};
  Mode mode_{Mode::OFF};
  bool triggered_ = false;                 // flush-thread only
  bool configured_ = false;
  bool files_open_ = false;                // flush-thread only
  std::atomic<uint64_t> off_calls_{0};
  std::atomic<uint64_t> last_sim_ns_{0};
  uint64_t internal_event_counter_ = 0;    // guarded by event_mutex_
  uint64_t trigger_sim_ns_ = 0;            // flush-thread only
  uint64_t trigger_wall_ns_ = 0;           // flush-thread only
  uint64_t wall_start_ns_ = 0;             // flush-thread only
  uint64_t last_health_ns_ = 0;            // flush-thread only

  std::unique_ptr<Ring<PluginWriteRecord>> w_ring_;
  std::unique_ptr<Ring<PluginReadRecord>> r_ring_;
  std::unique_ptr<Ring<IterationRecord>> i_ring_;
  std::unique_ptr<Ring<EventRecord>> e_ring_;
  StreamStats w_stats_, r_stats_, i_stats_, e_stats_;

  // Flush-thread-only drain positions.
  uint64_t w_cons_ = 0, r_cons_ = 0, i_cons_ = 0, e_cons_ = 0;
  std::vector<FileStream> files_;          // [0]w [1]r [2]i [3]e

  std::thread flush_thread_;
  mutable std::mutex event_mutex_;         // event ring + close_reason_
  std::string close_reason_;               // guarded by event_mutex_
  std::vector<uint64_t> seen_trigger_ids_; // guarded by event_mutex_
  std::vector<std::string> faults_;
};

}  // namespace joint_command_capture

#endif  // JOINT_COMMAND_CAPTURE__RECORDER_HPP_
