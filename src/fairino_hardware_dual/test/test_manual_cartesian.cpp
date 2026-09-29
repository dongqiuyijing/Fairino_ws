#include <gtest/gtest.h>

#include <string>

#include "fairino_hardware_dual/manual_cartesian.hpp"

using fairino_hardware_dual::BoundaryOutcome;
using fairino_hardware_dual::RuntimeMode;
using fairino_hardware_dual::ServoKind;
using fairino_hardware_dual::WriteCommandKind;
using fairino_hardware_dual::enter_manual_cartesian_outcome;
using fairino_hardware_dual::exit_manual_cartesian_outcome;
using fairino_hardware_dual::mode_after_gripper;
using fairino_hardware_dual::plan_manual_cartesian_write;
using fairino_hardware_dual::plan_servo_write;
using fairino_hardware_dual::runtime_mode_name;

TEST(ManualCartesian, AutoWriteSelectsServoJOnly)
{
  const auto plan = plan_servo_write(RuntimeMode::SERVO_ACTIVE);
  EXPECT_EQ(plan.kind, WriteCommandKind::SERVOJ);
  EXPECT_FALSE(plan.session_boundary);
}

TEST(ManualCartesian, ManualWriteSelectsServoCartOnly)
{
  const auto plan = plan_manual_cartesian_write(0.008, 10.0, 0.0, true, false);
  EXPECT_EQ(plan.kind, WriteCommandKind::SERVOCART);
  EXPECT_EQ(plan.cart.mode, 1);
  EXPECT_NEAR(plan.cart.x_mm, 0.08, 1e-12);
  EXPECT_NEAR(plan.cart.cmd_t_s, 0.008, 1e-12);
  EXPECT_DOUBLE_EQ(plan.cart.y_mm, 0.0);
  EXPECT_DOUBLE_EQ(plan.cart.z_mm, 0.0);
  EXPECT_DOUBLE_EQ(plan.cart.rx_deg, 0.0);
  EXPECT_FALSE(plan.session_boundary);
  EXPECT_NE(plan.kind, WriteCommandKind::SERVOJ);
}

TEST(ManualCartesian, StopIsZeroIncrementWithoutEndingSession)
{
  const auto plan = plan_manual_cartesian_write(0.008, 0.0, 10.0, true, false);
  EXPECT_EQ(plan.kind, WriteCommandKind::SERVOCART);
  EXPECT_DOUBLE_EQ(plan.cart.x_mm, 0.0);
  EXPECT_FALSE(plan.watchdog_zeroed);
  EXPECT_FALSE(plan.session_boundary);
}

TEST(ManualCartesian, WatchdogForcesZeroAndDoesNotEndSessionOrSelectServoJ)
{
  const auto plan = plan_manual_cartesian_write(0.008, 10.0, 151.0, true, false);
  EXPECT_EQ(plan.kind, WriteCommandKind::SERVOCART);
  EXPECT_DOUBLE_EQ(plan.cart.x_mm, 0.0);
  EXPECT_TRUE(plan.watchdog_zeroed);
  EXPECT_FALSE(plan.session_boundary);
  EXPECT_NE(plan.kind, WriteCommandKind::SERVOJ);
  EXPECT_STRNE(runtime_mode_name(RuntimeMode::MANUAL_CARTESIAN), "SERVO_ACTIVE");
}

TEST(ManualCartesian, LargeTimerPeriodCannotCreateALargeIncrement)
{
  const auto plan = plan_manual_cartesian_write(1.0, 10.0, 0.0, true, false);
  EXPECT_NEAR(plan.cart.cmd_t_s, 0.016, 1e-12);
  EXPECT_NEAR(plan.cart.x_mm, 0.16, 1e-12);
}

TEST(ManualCartesian, EnterBoundaryOrderIsEndSyncStartThenCartesian)
{
  const BoundaryOutcome failed_end = enter_manual_cartesian_outcome(14, false, 0);
  EXPECT_FALSE(failed_end.success);
  EXPECT_EQ(failed_end.mode, RuntimeMode::SERVO_ACTIVE);
  EXPECT_EQ(failed_end.resume_kind, ServoKind::SERVOJ);

  const BoundaryOutcome failed_sync = enter_manual_cartesian_outcome(0, false, 0);
  EXPECT_FALSE(failed_sync.success);
  EXPECT_EQ(failed_sync.mode, RuntimeMode::SERVO_RESTART);

  const BoundaryOutcome failed_start = enter_manual_cartesian_outcome(0, true, 14);
  EXPECT_FALSE(failed_start.success);
  EXPECT_EQ(failed_start.mode, RuntimeMode::SERVO_RESTART);
  EXPECT_NE(failed_start.mode, RuntimeMode::MANUAL_CARTESIAN);

  const BoundaryOutcome ok = enter_manual_cartesian_outcome(0, true, 0);
  EXPECT_TRUE(ok.success);
  EXPECT_EQ(ok.mode, RuntimeMode::MANUAL_CARTESIAN);
  EXPECT_EQ(ok.resume_kind, ServoKind::MANUAL_CARTESIAN);
}

TEST(ManualCartesian, ExitBoundaryOrderIsEndSyncStartThenServoJ)
{
  const BoundaryOutcome failed_end = exit_manual_cartesian_outcome(14, false, 0);
  EXPECT_FALSE(failed_end.success);
  EXPECT_EQ(failed_end.mode, RuntimeMode::MANUAL_CARTESIAN);

  const BoundaryOutcome failed_sync = exit_manual_cartesian_outcome(0, false, 0);
  EXPECT_FALSE(failed_sync.success);
  EXPECT_EQ(failed_sync.mode, RuntimeMode::SERVO_RESTART);

  const BoundaryOutcome ok = exit_manual_cartesian_outcome(0, true, 0);
  EXPECT_TRUE(ok.success);
  EXPECT_EQ(ok.mode, RuntimeMode::SERVO_ACTIVE);
  EXPECT_EQ(ok.resume_kind, ServoKind::SERVOJ);
}

TEST(ManualCartesian, GripperRestoresTheServoKindItInterrupted)
{
  EXPECT_EQ(mode_after_gripper(ServoKind::SERVOJ), RuntimeMode::SERVO_ACTIVE);
  EXPECT_EQ(
    mode_after_gripper(ServoKind::MANUAL_CARTESIAN),
    RuntimeMode::MANUAL_CARTESIAN);
}

TEST(ManualCartesian, ServoCartErrorInhibitsNonzeroMotion)
{
  const auto plan = plan_manual_cartesian_write(0.008, 10.0, 0.0, true, true);
  EXPECT_EQ(plan.kind, WriteCommandKind::SERVOCART);
  EXPECT_DOUBLE_EQ(plan.cart.x_mm, 0.0);
  EXPECT_FALSE(plan.session_boundary);
}

TEST(ManualCartesian, RuntimeModeNamesStayDefined)
{
  EXPECT_STREQ(runtime_mode_name(RuntimeMode::SERVO_ACTIVE), "SERVO_ACTIVE");
  EXPECT_STREQ(runtime_mode_name(RuntimeMode::MANUAL_CARTESIAN), "MANUAL_CARTESIAN");
  EXPECT_STREQ(runtime_mode_name(RuntimeMode::MANUAL_ENTERING), "MANUAL_ENTERING");
  EXPECT_STREQ(runtime_mode_name(RuntimeMode::MANUAL_EXITING), "MANUAL_EXITING");
  EXPECT_STRNE(runtime_mode_name(RuntimeMode::SERVO_RESTART), "UNKNOWN");
}
