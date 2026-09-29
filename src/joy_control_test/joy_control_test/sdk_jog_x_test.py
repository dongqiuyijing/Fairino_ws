"""One-shot Arm A base-frame X+ jog test.

Default mode connects and prints the plan. It does not call StartJOG.
Pass --execute to send one StartJOG and one StopJOG.
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

from joy_control_test.sdk_read_test import (
    ARM_A_IP,
    import_robot,
    sdk_code,
    six_numbers,
)

# StartJOG ref, from this SDK: 2 = base frame.
JOG_REF_BASE = 2
# StartJOG nb, from this SDK: 1 = X axis in a Cartesian frame.
JOG_NB_X = 1
# StartJOG dir, from this SDK: 1 = positive.
JOG_DIR_POSITIVE = 1
# StopJOG ref is not the StartJOG ref. Base-frame stop is 3.
STOP_REF_BASE = 3

# vel and acc are percentages in [0, 100], not mm/s.
VEL_PERCENT = 5.0
ACC_PERCENT = 20.0
# Cartesian max_dis is millimetres.
MAX_DIS_MM = 3.0
MAX_DURATION_S = 0.40
POLL_S = 0.05
SETTLE_TIMEOUT_S = 2.0
SETTLE_DELTA_MM = 0.20


def timestamp():
    return datetime.now().isoformat(timespec="milliseconds")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Arm A base X+ SDK jog test. Default is precheck only."
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Send one StartJOG. Without this flag the program only reads state.",
    )
    args, unknown = parser.parse_known_args()
    if unknown:
        print(f"ignored arguments: {unknown}")
    return args


def competing_controllers():
    """Return other FR3 control processes. Shell text that only mentions them is ignored."""
    needles = (
        "move_group",
        "ros2_control_node",
        "fr3_teleop",
        "dual_bringup",
        "sdk_read_test",
        "sdk_jog_x_test",
    )
    own_pid = os.getpid()
    found = []
    for pid_text in os.listdir("/proc"):
        if not pid_text.isdigit():
            continue
        pid = int(pid_text)
        if pid == own_pid:
            continue
        try:
            exe = os.path.basename(os.readlink(f"/proc/{pid}/exe"))
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\x00", b" ").decode(
                errors="replace"
            ).strip()
        except OSError:
            continue
        if exe in ("bash", "sh", "grep", "awk", "pgrep") or "cursorsandbox" in cmd:
            continue
        argv0 = os.path.basename(cmd.split(" ", 1)[0]) if cmd else ""
        matched = [name for name in needles if name in (exe, argv0)]
        if exe.startswith("python") and ("fairino" in cmd or "Robot.RPC" in cmd):
            matched.append("python-sdk-client")
        if matched:
            found.append((pid, exe, matched, cmd[:180]))
    return found


def existing_arm_sockets():
    try:
        import subprocess

        output = subprocess.check_output(["ss", "-ntp"], text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError) as exc:
        return [f"ss check failed: {exc}"]
    return [line.strip() for line in output.splitlines() if ARM_A_IP in line]


def read_six(robot, method_name):
    method = getattr(robot, method_name)
    result = method(0)
    code = sdk_code(result)
    if code != 0 or not isinstance(result, tuple) or len(result) != 2:
        raise RuntimeError(f"{method_name} failed: {result!r}")
    values = six_numbers(result[1])
    if values is None:
        raise RuntimeError(f"{method_name} payload is not six numbers: {result!r}")
    return values


def wait_for_state(robot, timeout=2.0):
    """Wait until CNDE has filled a live status field.

    frame_head stays 0. This SDK writes program_state and robot_state instead.
    A stopped robot reports 1 for both, so 0 means no packet yet.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        pkg = robot.robot_state_pkg
        if int(pkg.program_state) != 0 or int(pkg.robot_state) != 0:
            return True
        time.sleep(0.05)
    return False


def state_snapshot(robot):
    pkg = robot.robot_state_pkg
    return {
        "frame_head": int(pkg.frame_head),
        "robot_state": int(pkg.robot_state),
        "robot_mode": int(pkg.robot_mode),
        "main_code": int(pkg.main_code),
        "sub_code": int(pkg.sub_code),
        "rbt_enable_state": int(pkg.rbtEnableState),
        "motion_done": int(pkg.motion_done),
        "safety_code": robot.GetSafetyCode(),
    }


def print_plan(joints, tcp, state):
    print("PRECHECK PASS")
    print("")
    print("Planned motion:")
    print("Arm A")
    print("Base X+")
    print(f"velocity = {VEL_PERCENT} percent, SDK range [0, 100], default 20")
    print(f"acceleration = {ACC_PERCENT} percent, SDK range [0, 100], default 100")
    print(f"max_dis = {MAX_DIS_MM} mm")
    print(f"software maximum duration = {MAX_DURATION_S} s")
    print(f"normal stop = StopJOG({STOP_REF_BASE})")
    print("")
    print("READY FOR FIRST JOG TEST")
    print("")
    print("Arm: A")
    print(f"IP: {ARM_A_IP}")
    print(
        "SDK StartJOG exact call: "
        f"StartJOG({JOG_REF_BASE}, {JOG_NB_X}, {JOG_DIR_POSITIVE}, "
        f"{MAX_DIS_MM}, vel={VEL_PERCENT}, acc={ACC_PERCENT})"
    )
    print(f"SDK StopJOG exact call: StopJOG({STOP_REF_BASE})")
    print("Frame: BASE (StartJOG ref=2)")
    print("Axis: X (StartJOG nb=1)")
    print("Direction: positive (StartJOG dir=1)")
    print(f"Velocity: {VEL_PERCENT} percent")
    print(f"Acceleration: {ACC_PERCENT} percent")
    print(f"Maximum distance: {MAX_DIS_MM} mm")
    print(f"Software maximum duration: {MAX_DURATION_S} s")
    print(f"TCP before: {tcp}")
    print(f"joint before: {joints}")
    print(
        "robot state: "
        f"mode={state['robot_mode']} (1 manual, 0 auto), "
        f"motion_state={state['robot_state']} (1 stop, 2 run, 3 pause, 4 drag), "
        f"enable={state['rbt_enable_state']}, "
        f"main_code={state['main_code']}, sub_code={state['sub_code']}, "
        f"safety_code={state['safety_code']}, motion_done={state['motion_done']}"
    )


def xyz_delta(before, after):
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    dz = after[2] - before[2]
    return dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz)


def watch_motion(robot, tcp_before):
    """Wait until max_dis or the time limit. Does not call StartJOG again."""
    started = time.monotonic()
    last = tcp_before
    while time.monotonic() - started < MAX_DURATION_S:
        time.sleep(POLL_S)
        last = read_six(robot, "GetActualTCPPose")
        _dx, _dy, _dz, distance = xyz_delta(tcp_before, last)
        if distance >= MAX_DIS_MM:
            print(f"distance reached {distance} mm, leaving the wait")
            break
    print(f"jog wait elapsed_s: {time.monotonic() - started:.3f}")
    return last


def settled(robot):
    """Return the latest TCP and whether it stopped changing."""
    previous = read_six(robot, "GetActualTCPPose")
    deadline = time.monotonic() + SETTLE_TIMEOUT_S
    stable_count = 0
    while time.monotonic() < deadline:
        time.sleep(0.15)
        current = read_six(robot, "GetActualTCPPose")
        _dx, _dy, _dz, step = xyz_delta(previous, current)
        previous = current
        if step <= SETTLE_DELTA_MM:
            stable_count += 1
            if stable_count >= 2:
                return current, True
        else:
            stable_count = 0
    return previous, False


def stop_jog(robot):
    stop_ret = robot.StopJOG(STOP_REF_BASE)
    print(f"StopJOG({STOP_REF_BASE}) ret: {stop_ret}")
    imm_ret = None
    if stop_ret != 0:
        print("NORMAL JOG STOP FAILED")
        imm_ret = robot.ImmStopJOG()
        print(f"ImmStopJOG() ret: {imm_ret}")
    return stop_ret, imm_ret


def execute_once(robot, joints_before, tcp_before):
    print(f"timestamp: {timestamp()}")
    print(f"joint_before: {joints_before}")
    print(f"tcp_before: {tcp_before}")
    start_call = (
        f"StartJOG({JOG_REF_BASE}, {JOG_NB_X}, {JOG_DIR_POSITIVE}, "
        f"{MAX_DIS_MM}, vel={VEL_PERCENT}, acc={ACC_PERCENT})"
    )
    print(f"calling {start_call}")
    start_ret = robot.StartJOG(
        JOG_REF_BASE,
        JOG_NB_X,
        JOG_DIR_POSITIVE,
        MAX_DIS_MM,
        vel=VEL_PERCENT,
        acc=ACC_PERCENT,
    )
    print(f"StartJOG ret: {start_ret}")
    if start_ret != 0:
        print("StartJOG failed. No second motion attempt.")
        return start_ret

    watch_motion(robot, tcp_before)
    return start_ret


def print_motion_result(joints_before, tcp_before, joints_after, tcp_after, start_ret, stop_ret, stopped):
    dx, dy, dz, distance = xyz_delta(tcp_before, tcp_after)
    print(f"joint_after: {joints_after}")
    print(f"tcp_after: {tcp_after}")
    print("Requested direction: Base X+")
    print("Actual TCP displacement:")
    print(f"dx: {dx}")
    print(f"dy: {dy}")
    print(f"dz: {dz}")
    print(f"distance: {distance}")
    print(f"max_dis: {MAX_DIS_MM}")
    print(f"distance error vs max_dis: {distance - MAX_DIS_MM}")
    print(f"StartJOG ret: {start_ret}")
    print(f"StopJOG ret: {stop_ret}")
    print(f"robot physically stopped: {stopped}")
    if dx <= 0.0:
        print("DIRECTION CHECK: X did not increase. No further jog will be sent.")


def run(execute):
    print("=== FR3 SDK JOG TEST ===")
    print("")
    print("WARNING:")
    print("Arm A will move.")
    print("")
    print(f"Robot: {ARM_A_IP}")
    print("Frame: BASE")
    print("Axis: X+")
    print("Motion type: SDK JOG")
    print("MODE: EXECUTE ONCE" if execute else "MODE: PRECHECK ONLY")
    print("")

    others = competing_controllers()
    sockets = existing_arm_sockets()
    if others or sockets:
        print("BLOCKED: another FR3 control process is running")
        for item in others:
            print(f"process: {item}")
        for line in sockets:
            print(f"socket: {line}")
        return 2

    print("No other controller: PASS")
    robot = None
    joints = None
    tcp = None
    motion_attempted = False
    start_ret = None
    stop_ret = None
    try:
        Robot = import_robot()
        print(f"SDK module: {getattr(Robot, '__file__', Robot)}")
        robot = Robot.RPC(ARM_A_IP)
        if not bool(getattr(robot, "is_connect", False)):
            print("FAIL: SDK did not connect to Arm A")
            return 1
        print(f"RPC.is_connect: True")
        version = robot.GetSoftwareVersion()
        print(f"GetSoftwareVersion return: {version!r}")
        if sdk_code(version) != 0:
            print("FAIL: GetSoftwareVersion")
            return 1
        if not wait_for_state(robot):
            print("FAIL: CNDE state frame was not received")
            return 1

        joints = read_six(robot, "GetActualJointPosDegree")
        tcp = read_six(robot, "GetActualTCPPose")
        state = state_snapshot(robot)
        print(f"timestamp: {timestamp()}")
        print(f"joint_before: {joints}")
        print(f"tcp_before: {tcp}")
        print_plan(joints, tcp, state)
        if state["safety_code"] != 0 or state["main_code"] != 0:
            print("ROBOT FAULT OR SAFETY STOP IS PRESENT")
            print("No StartJOG will be sent.")
            return 2
        if not execute:
            print("")
            print("NO MOTION COMMAND SENT")
            return 0

        motion_attempted = True
        start_ret = execute_once(robot, joints, tcp)
        return 0 if start_ret == 0 else 1
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        return 1
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if robot is not None and motion_attempted:
            try:
                stop_ret, imm_ret = stop_jog(robot)
                tcp_after, stopped = settled(robot)
                if not stopped and imm_ret is None:
                    print("NORMAL JOG STOP FAILED")
                    print("TCP still changing after StopJOG")
                    imm_ret = robot.ImmStopJOG()
                    print(f"ImmStopJOG() ret: {imm_ret}")
                    tcp_after, stopped = settled(robot)
                joints_after = read_six(robot, "GetActualJointPosDegree")
                if joints is not None and tcp is not None and tcp_after is not None:
                    print_motion_result(
                        joints,
                        tcp,
                        joints_after,
                        tcp_after,
                        start_ret,
                        stop_ret,
                        stopped,
                    )
                print(f"robot state after: {state_snapshot(robot)}")
            except Exception as exc:
                print(f"stop or readback failed: {type(exc).__name__}: {exc}")
                try:
                    print("NORMAL JOG STOP FAILED")
                    print(f"ImmStopJOG() ret: {robot.ImmStopJOG()}")
                except Exception as stop_exc:
                    print(f"ImmStopJOG failed: {type(stop_exc).__name__}: {stop_exc}")
        if robot is not None:
            try:
                robot.CloseRPC()
                print("CloseRPC finished")
            except Exception as exc:
                print(f"CloseRPC failed: {type(exc).__name__}: {exc}")
        print("=== FR3 SDK JOG TEST END ===")


def main():
    args = parse_args()
    sys.exit(run(args.execute))


if __name__ == "__main__":
    main()
