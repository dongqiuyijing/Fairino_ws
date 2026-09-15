from fairino import Robot

ROBOT_IP = "192.168.58.2"

# ===== 必须换成你自己实际示教得到的数据 =====
POINT_A = [
    -138.645720529084, -137.2681872679455, -46.7423663753094, -62.49306253867574, -7.337126779084158, 55.7313582920792
]

POINT_B = [
    -129.9401976330445, -194.7445225479579, -57.66647489944306, -22.61704246596534, -3.815845451732673, 50.01020072710396
]

POINT_C = [
    -129.9401976330445, -194.7445225479579, -57.66647489944306, -22.61704246596534, -3.815845451732673, -124.8392781172648
]

POINT_D = [
    -133.6426989866955, -188.5334400139232, -28.10759591584158, -138.6955397199876, 89.6638836246906, -129.231851407797
]

robot = Robot.RPC(ROBOT_IP)

print("已连接 FR3")

# 全局速度限制为 10%
ret = robot.SetSpeed(60)
print("SetSpeed ret =", ret)

input("""
请确认：
1. 机器人周围无人
2. A→B 整个区域无障碍物
3. 急停按钮在手边
4. 当前准备进行 10% 速度测试

确认后按 Enter...
""")

print("先移动到 A 点")

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

input("A 点正常。确认可以执行 A → B 后按 Enter...")

print("A → B")

ret = robot.MoveJ(
    joint_pos=POINT_B,
    tool=0,
    user=0,
    vel=100.0,
    blendT=-1.0
)

print("Move B ret =", ret)

robot.CloseRPC()

print("测试结束")