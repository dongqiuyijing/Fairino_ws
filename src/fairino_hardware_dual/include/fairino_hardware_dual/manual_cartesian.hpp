#ifndef FAIRINO_HARDWARE_DUAL__MANUAL_CARTESIAN_HPP_
#define FAIRINO_HARDWARE_DUAL__MANUAL_CARTESIAN_HPP_

#include <cmath>
#include <string>

namespace fairino_hardware_dual
{

// MANUAL_ENTERING / MANUAL_EXITING exist so a type change never shares a
// write() cycle with the other servo command, and so exit always emits one
// zero ServoCart before ServoMoveEnd. They are not a second state machine.
enum class RuntimeMode {
  SERVO_ACTIVE,
  MANUAL_ENTERING,
  MANUAL_CARTESIAN,
  MANUAL_EXITING,
  GRIPPER_COMMAND,
  GRIPPER_WAIT,
  SERVO_RESTART
};

enum class ServoKind {
  SERVOJ,
  MANUAL_CARTESIAN
};

enum class WriteCommandKind {
  SERVOJ,
  SERVOCART,
  NONE
};

struct ServoCartCommand {
  double x_mm{0.0};
  double y_mm{0.0};
  double z_mm{0.0};
  double rx_deg{0.0};
  double ry_deg{0.0};
  double rz_deg{0.0};
  double cmd_t_s{0.008};
  int mode{1};
};

struct WritePlan {
  WriteCommandKind kind{WriteCommandKind::NONE};
  ServoCartCommand cart{};
  bool watchdog_zeroed{false};
  bool session_boundary{false};
};

struct BoundaryOutcome {
  RuntimeMode mode{RuntimeMode::SERVO_ACTIVE};
  ServoKind resume_kind{ServoKind::SERVOJ};
  bool success{false};
};

inline constexpr double kNominalCommandPeriodS = 0.008;
inline constexpr double kMinCommandPeriodS = 0.001;
inline constexpr double kMaxCommandPeriodS = 0.016;
inline constexpr double kCommandWatchdogMs = 150.0;
inline constexpr double kMaxManualLinearSpeedMmS = 50.0;

inline const char * runtime_mode_name(RuntimeMode mode)
{
  switch (mode) {
    case RuntimeMode::SERVO_ACTIVE:
      return "SERVO_ACTIVE";
    case RuntimeMode::MANUAL_ENTERING:
      return "MANUAL_ENTERING";
    case RuntimeMode::MANUAL_CARTESIAN:
      return "MANUAL_CARTESIAN";
    case RuntimeMode::MANUAL_EXITING:
      return "MANUAL_EXITING";
    case RuntimeMode::GRIPPER_COMMAND:
      return "GRIPPER_COMMAND";
    case RuntimeMode::GRIPPER_WAIT:
      return "GRIPPER_WAIT";
    case RuntimeMode::SERVO_RESTART:
      return "SERVO_RESTART";
  }
  return "UNKNOWN";
}

inline double clamp_command_period_s(double period_s)
{
  if (!std::isfinite(period_s) || period_s <= 0.0) {
    return kNominalCommandPeriodS;
  }
  if (period_s < kMinCommandPeriodS) {
    return kMinCommandPeriodS;
  }
  if (period_s > kMaxCommandPeriodS) {
    return kMaxCommandPeriodS;
  }
  return period_s;
}

inline double clamp_manual_linear_mm_s(double velocity_mm_s)
{
  if (!std::isfinite(velocity_mm_s)) {
    return 0.0;
  }
  if (velocity_mm_s > kMaxManualLinearSpeedMmS) {
    return kMaxManualLinearSpeedMmS;
  }
  if (velocity_mm_s < -kMaxManualLinearSpeedMmS) {
    return -kMaxManualLinearSpeedMmS;
  }
  return velocity_mm_s;
}

// Fresh nonzero velocity becomes a base-frame X increment. A stale, missing,
// stopped, or fault-latched command stays inside MANUAL_CARTESIAN and sends
// a zero ServoCart. It does not end the servo session and it does not select
// ServoJ.
inline WritePlan plan_manual_cartesian_write(
  double period_s,
  double vx_mm_s,
  double command_age_ms,
  bool command_stamped,
  bool output_inhibited)
{
  WritePlan plan;
  plan.kind = WriteCommandKind::SERVOCART;
  plan.session_boundary = false;
  const double dt = clamp_command_period_s(period_s);
  plan.cart.cmd_t_s = dt;
  plan.cart.mode = 2;
  const bool fresh = command_stamped && std::isfinite(command_age_ms) &&
    command_age_ms >= 0.0 && command_age_ms <= kCommandWatchdogMs;
  double vx = 0.0;
  if (fresh && !output_inhibited) {
    vx = clamp_manual_linear_mm_s(vx_mm_s);
  }
  plan.watchdog_zeroed = command_stamped && !fresh && !output_inhibited &&
    std::isfinite(vx_mm_s) && vx_mm_s != 0.0;
  plan.cart.x_mm = vx * dt;
  return plan;
}

inline WritePlan plan_servo_write(RuntimeMode mode)
{
  WritePlan plan;
  plan.session_boundary = false;
  if (mode == RuntimeMode::SERVO_ACTIVE) {
    plan.kind = WriteCommandKind::SERVOJ;
    return plan;
  }
  plan.kind = WriteCommandKind::NONE;
  return plan;
}

// ServoJ -> ServoCart. End failure leaves the existing ServoJ session in
// SERVO_ACTIVE. Sync or Start failure parks in SERVO_RESTART, which the
// existing restart path retries without sending either servo command.
inline BoundaryOutcome enter_manual_cartesian_outcome(
  int end_ret, bool sync_ok, int start_ret)
{
  BoundaryOutcome outcome;
  outcome.resume_kind = ServoKind::SERVOJ;
  if (end_ret != 0) {
    outcome.mode = RuntimeMode::SERVO_ACTIVE;
    outcome.success = false;
    return outcome;
  }
  if (!sync_ok || start_ret != 0) {
    outcome.mode = RuntimeMode::SERVO_RESTART;
    outcome.success = false;
    return outcome;
  }
  outcome.mode = RuntimeMode::MANUAL_CARTESIAN;
  outcome.resume_kind = ServoKind::MANUAL_CARTESIAN;
  outcome.success = true;
  return outcome;
}

// ServoCart -> ServoJ. End failure stays in MANUAL_CARTESIAN so the caller
// does not activate the trajectory controller against an open Cartesian session.
inline BoundaryOutcome exit_manual_cartesian_outcome(
  int end_ret, bool sync_ok, int start_ret)
{
  BoundaryOutcome outcome;
  if (end_ret != 0) {
    outcome.mode = RuntimeMode::MANUAL_CARTESIAN;
    outcome.resume_kind = ServoKind::MANUAL_CARTESIAN;
    outcome.success = false;
    return outcome;
  }
  if (!sync_ok || start_ret != 0) {
    outcome.mode = RuntimeMode::SERVO_RESTART;
    outcome.resume_kind = ServoKind::SERVOJ;
    outcome.success = false;
    return outcome;
  }
  outcome.mode = RuntimeMode::SERVO_ACTIVE;
  outcome.resume_kind = ServoKind::SERVOJ;
  outcome.success = true;
  return outcome;
}

inline RuntimeMode mode_after_gripper(ServoKind kind_before_gripper)
{
  if (kind_before_gripper == ServoKind::MANUAL_CARTESIAN) {
    return RuntimeMode::MANUAL_CARTESIAN;
  }
  return RuntimeMode::SERVO_ACTIVE;
}

inline std::string boundary_message(const BoundaryOutcome & outcome)
{
  std::string message = runtime_mode_name(outcome.mode);
  if (!outcome.success) {
    message += ": boundary failed";
  }
  return message;
}

}  // namespace fairino_hardware_dual

#endif
