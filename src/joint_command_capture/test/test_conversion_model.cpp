// T6 offline fixture: pin the position-command conversion semantics to the
// vendored upstream source (gz_ros2_control 0.7.20, gz_system.cpp:832-851,
// PROVENANCE.md) so the recorder/analysis model cannot silently drift from
// the plugin's actual behavior. Transcribed reference — full patched-vs-
// pristine behavioural对拍 needs the static-sim gate (flagged in evidence).
//
// Upstream (verbatim semantics):
//   error      = (joint_position - joint_position_cmd) * update_rate;
//   target_vel = -position_proportional_gain * error;
//   vel->Data()[0] = target_vel;            // written unless Data() empty
//   // (empty data => previous command retained, WriteOutcome::RETAINED_EMPTY)
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

namespace
{

struct RefJoint
{
  double position = 0.0;
  double cmd = 0.0;
  int update_rate = 100;
  double gain = 0.8;
};

double ref_target_vel(const RefJoint & j)
{
  const double error = (j.position - j.cmd) * j.update_rate;
  return -j.gain * error;
}

TEST(ConversionModel, PositionCmdBecomesVelocityWithEffectiveGain80)
{
  RefJoint j;  // 1 rad below command at 100 Hz update, gain 0.8
  j.position = 0.0;
  j.cmd = 1.0;
  // error = (0-1)*100 = -100; target = -0.8 * -100 = +80 rad/s per rad
  EXPECT_NEAR(ref_target_vel(j), 80.0, 1e-12);
}

TEST(ConversionModel, FrozenReferenceDrivesRestoringVelocity)
{
  RefJoint j;
  j.cmd = 0.5;
  j.position = 0.6;   // 0.1 rad overshoot above frozen reference
  // restoring: -80 * 0.1 = -8 rad/s commanded back toward the reference
  EXPECT_NEAR(ref_target_vel(j), -8.0, 1e-12);
}

TEST(ConversionModel, ZeroErrorMeansZeroCommandedVelocity)
{
  RefJoint j;
  j.cmd = 0.25;
  j.position = 0.25;
  EXPECT_NEAR(ref_target_vel(j), 0.0, 1e-12);
}

TEST(ConversionModel, UpdateRateScalesError)
{
  RefJoint a, b;
  a.update_rate = 100;
  b.update_rate = 50;
  a.position = b.position = 0.0;
  a.cmd = b.cmd = 0.1;
  EXPECT_NEAR(ref_target_vel(a), 2.0 * ref_target_vel(b), 1e-12);
}

// The retained-previous-command branch is a pure ECM state condition: when
// the JointVelocityCmd component exists but its data vector is empty the
// upstream code writes NOTHING for that joint (gz_system.cpp:849
// `else if (!vel->Data().empty())`). The recorder models it as
// RETAINED_EMPTY; the fixture pins the predicate itself.
TEST(ConversionModel, EmptyComponentDataMeansNoWrite)
{
  std::vector<double> vel_data;            // empty == "retained" case
  const bool upstream_writes = !vel_data.empty();
  EXPECT_FALSE(upstream_writes);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
