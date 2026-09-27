# FR3 dual-arm GUI teleoperation

This independent ROS 2 package supplies a working desktop GUI, exclusive input
manager, watchdog, and mock backend. It does **not** send a robot motion in the
shipped configuration.

## Real backend boundary

The live dual hardware plugin (`fairino_hardware_dual`) opens one `FRRobot` per
arm, calls `ServoMoveStart()` on activation, and calls `ServoJ()` from its
125 Hz `write()` loop.  The installed SDK does expose `StartJOG`, `StopJOG`,
and `ImmStopJOG`, but a second SDK client would compete with that active servo
session.  This package never opens an SDK connection: manual motion remains on
the existing ros2_control position-command path and is enabled only after the
manager has completed the existing strict controller-ownership and initial-hold
checks.

MoveIt Servo 2.5.9 is installed locally and supports
`std_msgs/Float64MultiArray` output. The real teleop path receives six safe
joint velocities from Servo, integrates them into bounded position commands at
125 Hz, and sends those commands through
`position_controllers/JointGroupPositionController`; the existing automatic controllers remain position-only
`joint_trajectory_controller` instances.

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

## Servo output staging

`teleop_servo.launch.py` starts independent Arm A/B Servo nodes. Their raw
`Float64MultiArray` joint velocities are published under
`/fr3_teleop/arm_[ab]/servo_raw_commands`; the manager is the sole publisher
to `/arm_[ab]_teleop_controller/commands`, where it emits bounded, integrated
joint positions at 125 Hz. It does not automatically switch controllers:

```bash
ros2 launch fr3_teleop teleop_servo.launch.py
```

`teleop_controllers.yaml` defines separate
`JointGroupPositionController` instances that claim the same joints as
`arm_a_controller` and `arm_b_controller`. They must be loaded
inactive with `controller_manager/spawner -p`; a strict controller-manager
switch is required before Servo output could reach hardware. `ControllerLease`
implements and tests the required rule: reject a manual request while an auto
goal is active; only grant after strict auto-off/manual-on succeeds; restore
manual-off/auto-on on release.

The existing gripper bridge is implemented in `GripperBridgeClient`. It sends
only `fairino_msgs/srv/GripperBridge` requests to the configured existing
service. `allow_real_gripper_service` remains false by default.

Real use still requires the explicit controller-manager manual request,
fresh-state/TF checks, initial hold, and a supervised low-speed cell test.
