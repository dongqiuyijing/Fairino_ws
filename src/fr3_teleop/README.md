# FR3 dual-arm GUI teleoperation

This independent ROS 2 package supplies a working desktop GUI, exclusive input
manager, watchdog, and mock backend. It does **not** send a robot motion in the
shipped configuration.

## Why the real backend is intentionally blocked

The live dual hardware plugin (`fairino_hardware_dual`) opens one `FRRobot` per
arm, calls `ServoMoveStart()` on activation, and calls `ServoJ()` from its
125 Hz `write()` loop.  The installed SDK does expose `StartJOG`, `StopJOG`,
and `ImmStopJOG`, but a second SDK client would compete with that active servo
session.  This package therefore refuses `teleop.backend: real`; it never
opens an SDK connection and never attempts a controller switch.

MoveIt Servo 2.5.9 is installed locally and supports
`trajectory_msgs/JointTrajectory` output. That is compatible with the existing
position-only `joint_trajectory_controller`: Servo emits joint positions and
the controller continues to feed the existing hardware plugin's ServoJ loop.
No velocity hardware interface is needed.

## Build and mock launch

```bash
cd ~/fairino_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_teleop
source install/setup.bash
ros2 launch fr3_teleop teleop.launch.py
```

The GUI title says `MOCK`. Click **Enable Manual Control**, hold a direction
button, then release it. The manager receives repeated motion intents at
20 Hz; release, changing arm/frame/speed, loss of window activation, closing
the GUI, or a 250 ms input timeout sends a stop to the backend. `STOP` is a
software normal-stop request, not a certified safety E-stop.

## Current ROS interface

`/fr3_teleop/command` and `/fr3_teleop/status` use JSON inside `std_msgs/String`
to avoid introducing a new interface package for the mock-only first revision.
All sources use the same `motion` command schema (`axis`, `sign`, `source`). A
future joystick adapter publishes that schema; it does not call a backend.

## Frames and feedback

The manager subscribes to `/joint_states` and accepts exactly six measured
joint positions per arm. It resolves the selected display frame to
`arm_*_gripper_tcp` through the existing TF tree, so TCP position and RPY are
displayed in `world`, selected `base`, or selected `tool` frame. The overlay
hardware plugin does not publish `RobotNonrtState`; fault status is therefore
explicitly `UNKNOWN`, rather than inferred from joint-state traffic.

## Servo staging, not real motion

`teleop_servo.launch.py` starts independent Arm A/B Servo nodes and sends their
output only to `/arm_[ab]_teleop_controller/joint_trajectory`. It starts no
controller and performs no controller switch. It is safe to inspect with an
existing bringup because the output has no active subscriber by default:

```bash
ros2 launch fr3_teleop teleop_servo.launch.py
```

`teleop_controllers.yaml` defines separate position JTCs that claim the same
joints as `arm_a_controller` and `arm_b_controller`. They must be loaded
inactive with `controller_manager/spawner -p`; a strict controller-manager
switch is required before Servo output could reach hardware. `ControllerLease`
implements and tests the required rule: reject a manual request while an auto
goal is active; only grant after strict auto-off/manual-on succeeds; restore
manual-off/auto-on on release.

The existing gripper bridge is implemented in `GripperBridgeClient`. It sends
only `fairino_msgs/srv/GripperBridge` requests to the configured existing
service. `allow_real_gripper_service` remains false by default.

## Required next implementation before real use

Complete the controller-manager service adapter and action-status monitor in
the manager, then perform a fully isolated mock Servo-output probe before any
real switch. A real deployment must prove strict switch behavior, halt latency,
fresh state, scene contents, and low-speed motion under supervised conditions.
