// Copyright 2026 luggage packing campaign. Fixed-size POD records shared by the
// instrumented gz_ros2_control plugin, the recorder core and the Python reader.
//
// Discipline (docs/plans/joint_command_boundary_capture.md §1/§4):
//  - records are fixed-length PODs so control callbacks only memcpy;
//  - NaN and "not available" are preserved verbatim, never coerced;
//  - every row carries source timestamps + sequence numbers so layers can be
//    joined by control-cycle/iteration index first, wall-time matching only
//    with an explicit sync-error column (computed offline by the reader).
#ifndef JOINT_COMMAND_CAPTURE__RECORDS_HPP_
#define JOINT_COMMAND_CAPTURE__RECORDS_HPP_

#include <cstddef>
#include <cstdint>

namespace joint_command_capture
{

constexpr std::size_t kMaxJoints = 16;
constexpr std::size_t kEventTextLen = 96;

// Write() branch taken per joint inside GazeboSimSystem::write().
// Values verified against gz_ros2_control 0.7.20 gz_system.cpp:809-880.
enum class WriteBranch : uint8_t
{
  NONE = 0,
  SKIP_NULL_ENTITY = 1,     // sim_joint == kNullEntity
  VELOCITY = 2,             // joint_control_method & VELOCITY
  POSITION_PV = 3,          // position cmd -> velocity cmd P conversion
  EFFORT = 4,               // joint_control_method & EFFORT
  FALLBACK_ZERO_VEL = 5,    // is_actuated but no control method -> zero velocity
  NO_METHOD = 6,            // not actuated, nothing written
};

// What actually happened to the ECM component for this joint this call.
enum class WriteOutcome : uint8_t
{
  NONE = 0,
  CREATED = 1,              // component created with the value
  WRITTEN = 2,              // existing component overwritten
  RETAINED_EMPTY = 3,       // component data empty -> previous command retained
  SKIPPED = 4,              // branch wrote nothing
};

// Per-joint read() flags.
enum ReadFlag : uint8_t
{
  READ_OK = 0,
  READ_SKIP_NULL = 1 << 0,
  READ_MISSING_COMPONENT = 1 << 1,
};

#pragma pack(push, 1)
// One GazeboSimSystem::write() call (~1 kHz: called from PreUpdate every
// physics step, re-applying the current command interface values).
struct PluginWriteRecord
{
  uint64_t seq;                          // per-process write-call counter
  uint64_t sim_time_ns;                  // write() time argument (ROS sim time)
  uint64_t wall_mono_ns;                 // steady_clock at record time
  uint32_t n_joints;
  int32_t update_rate;                   // controller_manager update rate (Hz)
  double cmd_received[kMaxJoints];       // command interface value (per method)
  double position_used[kMaxJoints];      // joint_position used by conversion
  double write_applied[kMaxJoints];      // value written to ECM (target_vel...)
  uint8_t branch[kMaxJoints];
  uint8_t outcome[kMaxJoints];
  uint64_t dropped_before;               // stream drop counter at record time
};

// One GazeboSimSystem::read() call (100 Hz, gated in PostUpdate).
struct PluginReadRecord
{
  uint64_t seq;
  uint64_t sim_time_ns;                  // read() time argument
  uint64_t wall_mono_ns;
  uint32_t n_joints;
  uint8_t flags[kMaxJoints];
  double position[kMaxJoints];
  double velocity[kMaxJoints];
  double effort[kMaxJoints];             // transmitted-wrench scalar (valid in sim)
  uint64_t dropped_before;
};

// One gated PostUpdate control cycle (100 Hz) from the world plugin.
struct IterationRecord
{
  uint64_t seq;
  uint64_t iterations;                   // gz UpdateInfo.iterations
  uint64_t sim_time_ns;                  // UpdateInfo.simTime
  uint64_t real_time_ns;                 // UpdateInfo.realTime
  uint64_t dt_ns;                        // UpdateInfo.dt
  uint8_t paused;
};

// Unified event stream: mode changes, triggers, faults, pause transitions.
struct EventRecord
{
  uint64_t event_id;                     // external id, or internal counter
  uint64_t wall_mono_ns;
  uint64_t sim_time_ns;                  // best known sim time (0 if unknown)
  uint8_t kind;                          // EventKind
  uint8_t reserved[7];
  char text[kEventTextLen];
};

enum class EventKind : uint8_t
{
  INTERNAL = 0,
  CONFIGURED = 1,
  TRIGGER = 2,
  CLOSED = 3,
  FAULT = 4,
  TRUNCATED = 5,
  PAUSE_CHANGE = 6,
};

constexpr uint64_t kInternalEventIdBit = 0x8000000000000000ULL;

// Binary file header written once per stream file. INSIDE the packed
// region: an aligned layout wrote 144/24-byte header/footer where every
// reader (and the Python schema) expects 140/20 — pinned by static_assert.
struct BinFileHeader
{
  char magic[4];                         // "JCC1"
  uint32_t header_size;
  uint32_t record_size;
  uint32_t record_kind;                  // 1=write 2=read 3=iter 4=event
  uint32_t reserved;
  uint64_t wall_mono_start_ns;
  uint64_t wall_system_start_ns;         // system_clock at configure (epoch link)
  char watermark[64];                    // version + git sha (NUL padded)
  char epoch[40];                        // uuid hex (NUL padded)
};
static_assert(sizeof(BinFileHeader) == 140, "reader schema expects 140");
static_assert(sizeof(PluginWriteRecord) == 456, "reader schema expects 456");
static_assert(sizeof(PluginReadRecord) == 436, "reader schema expects 436");
static_assert(sizeof(IterationRecord) == 41, "reader schema expects 41");
static_assert(sizeof(EventRecord) == 128, "reader schema expects 128");
#pragma pack(pop)

}  // namespace joint_command_capture

#endif  // JOINT_COMMAND_CAPTURE__RECORDS_HPP_
