"""Read-only FR3 SDK status check for Arm A.

Connects with Robot.RPC, reads software version, joint position, and TCP pose,
then closes the RPC connection. This program does not enable, reset, or move
the robot.
"""

import math
import sys

ARM_A_IP = "192.168.58.2"
SDK_ROOT = "/home/cyberbraindualarm/fairino-python-sdk/linux"
JOINT_COUNT = 6


def import_robot():
    """Import the already configured Fairino SDK without modifying it."""
    try:
        from fairino import Robot
    except ImportError:
        if SDK_ROOT not in sys.path:
            sys.path.insert(0, SDK_ROOT)
        from fairino import Robot
    return Robot


def sdk_code(result):
    """Return the SDK error code. Success is 0."""
    if isinstance(result, tuple):
        if not result:
            raise ValueError("SDK returned an empty tuple")
        return result[0]
    return result


def six_numbers(values):
    """Return six finite numbers, or None when the payload is not usable."""
    if not isinstance(values, (list, tuple)) or len(values) != JOINT_COUNT:
        return None
    numbers = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        numbers.append(number)
    return numbers


def read_version(robot):
    result = robot.GetSoftwareVersion()
    code = sdk_code(result)
    print(f"GetSoftwareVersion return: {result!r}")
    if code != 0 or not isinstance(result, tuple) or len(result) != 4:
        print(f"FAIL: GetSoftwareVersion error code {code}")
        return False
    robot_model, web_version, controller_version = result[1], result[2], result[3]
    if robot_model in (None, "") or controller_version in (None, ""):
        print("FAIL: software version fields are empty")
        return False
    print(f"robot_model: {robot_model}")
    print(f"web_version: {web_version}")
    print(f"controller_version: {controller_version}")
    return True


def read_joints(robot):
    # flag 0 is the SDK blocking read. Unit is degree, from the SDK docstring.
    result = robot.GetActualJointPosDegree(0)
    code = sdk_code(result)
    print(f"GetActualJointPosDegree return: {result!r}")
    if code != 0 or not isinstance(result, tuple) or len(result) != 2:
        print(f"FAIL: GetActualJointPosDegree error code {code}")
        return False
    joints = six_numbers(result[1])
    if joints is None:
        print("FAIL: joint position is not six finite numbers")
        return False
    print(f"actual_joint_pos_deg [j1, j2, j3, j4, j5, j6] unit=degree: {joints}")
    return True


def read_tcp(robot):
    # flag 0 is the SDK blocking read. Values are returned without conversion.
    # This SDK documents Cartesian pose as millimetres and degrees.
    result = robot.GetActualTCPPose(0)
    code = sdk_code(result)
    print(f"GetActualTCPPose return: {result!r}")
    if code != 0 or not isinstance(result, tuple) or len(result) != 2:
        print(f"FAIL: GetActualTCPPose error code {code}")
        return False
    pose = six_numbers(result[1])
    if pose is None:
        print("FAIL: TCP pose is not six finite numbers")
        return False
    print(
        "actual_tcp_pose [x, y, z, rx, ry, rz] "
        f"unit=mm,degree (SDK raw, no conversion): {pose}"
    )
    return True


def close_robot(robot):
    if robot is None:
        return
    try:
        robot.CloseRPC()
        print("CloseRPC finished")
    except Exception as exc:
        print(f"CloseRPC failed: {type(exc).__name__}: {exc}")


def main():
    print("=== joy_control_test sdk_read_test START ===")
    print("mode: read-only")
    print(f"arm: A {ARM_A_IP}")
    print("motion commands: none")

    robot = None
    code = 1
    try:
        Robot = import_robot()
        print(f"SDK module: {getattr(Robot, '__file__', Robot)}")
        print(f"connecting Robot.RPC({ARM_A_IP})")
        robot = Robot.RPC(ARM_A_IP)
        connected = bool(getattr(robot, "is_connect", False))
        print(f"RPC.is_connect: {connected}")
        if not connected:
            print("FAIL: SDK did not connect to Arm A")
        elif not read_version(robot):
            pass
        elif not read_joints(robot):
            pass
        elif not read_tcp(robot):
            pass
        else:
            code = 0
            print("PASS")
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}")
        code = 1
    finally:
        close_robot(robot)
        if code != 0:
            print("RESULT: FAIL")
        print("=== joy_control_test sdk_read_test END ===")
    sys.exit(code)


if __name__ == "__main__":
    sys.exit(main())
