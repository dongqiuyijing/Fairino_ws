from fairino import Robot
import time

ROBOT_IP = "192.168.58.2"
GRIPPER_ID = 1

# ===== 必须换成你自己实际示教得到的数据 =====
POINT_A = [
    -142.4733137376237, -143.0885481126237, -32.5634765625, -46.92597946318069, 2.514455058787129, -96.0568388381807
]

POINT_B = [
    -139.5313708616955, -131.977563234839, -124.3828560102104, -17.70604501856435, 91.7312712716584, -47.987
]

POINT_C = [
    -139.5313708616955, -131.977563234839, -124.3828560102104, -17.70604501856435, 91.7312712716584, 129.402
]

POINT_D = [
    -363.3529663085937, -295.1261291503906, -39.32107543945312, 175.4273529052734, 0.7961405515670776, 178.0016937255859
]

robot = Robot.RPC(ROBOT_IP)

print("已连接 FR3")

# 全局速度限制为 10%
ret = robot.SetSpeed(40)
print("SetSpeed ret =", ret)

ret = robot.ActGripper(GRIPPER_ID, 0)

print("夹爪复位 ret =", ret)

time.sleep(2)
ret = robot.ActGripper(GRIPPER_ID, 1)

print("夹爪激活 ret =", ret)

time.sleep(2)

input("""
请确认：
1. 机器人周围无人
2. A→B 整个区域无障碍物
3. 急停按钮在手边
4. 当前准备进行 10% 速度测试

确认后按 Enter...
""")
ret = robot.MoveGripper(
    GRIPPER_ID,    # 夹爪编号
    0,           # 位置
    20,            # 速度 20%
    20,            # 力矩 20%
    5000,          # 最长等待 5000 ms
    0,             # 阻塞
    0,             # 普通平行夹爪
    0,
    0,
    0
)
input("先移动到 A 点")

ret = robot.MoveJ(
    joint_pos=POINT_A,
    tool=0,
    user=0,
    vel=100.0,
    blendT=-1.0
)


print("Move A ret =", ret)

if ret != 0:
    print("移动到 A 失败，停止测试")
    robot.CloseRPC()
    exit()

ret = robot.MoveGripper(
    GRIPPER_ID,    # 夹爪编号
    80,           # 位置
    20,            # 速度 20%
    20,            # 力矩 20%
    5000,          # 最长等待 5000 ms
    0,             # 阻塞
    0,             # 普通平行夹爪
    0,
    0,
    0
)

input("A 点正常。确认可以执行 A → B 后按 Enter...")

print("A → B ")

ret = robot.MoveJ(
    joint_pos=POINT_B,
    tool=0,
    user=0,
    vel=100.0,
    blendT=-1.0
)


# input("B 点正常。确认可以执行 B → C 后按 Enter...")

print("B → C")

ret = robot.MoveJ(
    joint_pos=POINT_C,
    tool=0,
    user=0,
    vel=100.0,
    blendT=-1.0
)


print("C → A")

ret = robot.MoveJ(
    joint_pos=POINT_A,
    tool=0,
    user=0,
    vel=100.0,
    blendT=-1.0
)

if ret != 0:
    print("移动到 A 失败，停止测试")
    robot.CloseRPC()
    exit()

input("A 点正常。确认可以夹爪复位后按 Enter...")

ret = robot.ActGripper(GRIPPER_ID, 0)

robot.CloseRPC()

print("测试结束")