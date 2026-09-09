"""FR3 垂直夹取 small_part，并依次展示三个可见面的 Gazebo 任务。

这是“按预设关节角运动”的演示程序，不是 MoveIt 的碰撞规划程序。
它不会在发出轨迹前检查桌子、地面或其它模型是否挡在路径上；如果改动本文件中的
关节角，请先在 Gazebo 中慢速观察轨迹，避免让机械臂撞向桌面。
"""
import time

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -----------------------------------------------------------------------------
# 一、最常需要手动修改的参数
# -----------------------------------------------------------------------------

# 六个机械臂关节的固定顺序。下面每一个机械臂姿态数组都必须严格按这个顺序填写，
# 单位是弧度（rad），不是角度（°）。例如 90° = 1.570796 rad。
ARM = ["j1", "j2", "j3", "j4", "j5", "j6"]

# HKV 夹爪主关节 gripper_joint：0.0 张开，增大则闭合。
# 指间距约 0.0979 - joint；5 mm 工件对应约 0.093。完全闭合为 0.1。
# 阶段 2 已把 Gazebo gripper_controller 换成 GripperActionController，
# 动作是 /gripper_controller/gripper_cmd，不再是 FollowJointTrajectory。
# 本演示的夹爪路径尚未迁移；抓取任务请等阶段 3 改用 fr_control.gripper。
GRIPPER = ["gripper_joint"]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.093

# 以下是三个关键姿态，均由 IK 求得，且工具朝下。
# 改姿态时最安全的方法是：一次只改一个关节，幅度不超过 0.1 rad，然后重新测试。
#
# 真夹爪比旧伪夹爪更长（wrist3 到 TCP 约 0.211 m，原先约 0.175 m）。
# 下列关节角仍按旧 TCP 标定，首次运行请慢速确认 GRASP 是否够低/会不会碰桌。
# ABOVE：零件正上方的安全高度；当前 wrist3 高度约为 0.30 m。
#        如果桌子升高、工件变高，或担心碰桌，优先增大这里的安全高度。
ABOVE = [0.205443, -2.171320, -1.270636, -1.270429, 1.570793, -1.365350]
# GRASP：从 ABOVE 垂直向下的抓取高度；当前 wrist3 高度约为 0.22 m。
#        此值过低会撞桌/压入零件；过高则夹爪接触不到零件。
GRASP = [0.205442, -2.295137, -1.417373, -0.999876, 1.570793, -1.365351]
# LIFT：抓住零件后向上抬起的位置；当前 wrist3 高度约为 0.34 m。
LIFT = [0.205443, -2.137139, -1.156210, -1.419037, 1.570793, -1.365350]

# 每段运动的计划时间，单位秒。数值越大，速度越慢、越容易在 Gazebo 中稳定。
# 它不是安全碰撞距离；减小它只会让运动更快，不能让机械臂“自动避开桌子”。
T_OPEN = 2.0
T_ABOVE = 6.0
T_DESCEND = 4.0
T_CLOSE = 2.0
T_LIFT = 5.0
T_INSPECT = 3.0
T_RETURN = 2.0


class PickInspect(Node):
    def __init__(self):
        # ROS 节点名，会出现在 ros2 node list 和终端日志中。
        super().__init__("fr3_pick_inspect_demo")

        # 两个 ActionClient 分别连接机械臂和夹爪控制器。
        # 若改了 controllers.yaml 中的控制器名称，这两个 topic 也必须同步修改。
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/fairino3_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    "/gripper_controller/follow_joint_trajectory")
        # 这两个 topic 控制的是 Gazebo 的“固定关节抓取”插件，和真实接触力无关。
        # attach 会把 small_part 固定到 wrist3_link；detach 会恢复其自由物理状态。
        self.attach = self.create_publisher(Empty, "/fr3/grasp/attach", 10)
        self.detach = self.create_publisher(Empty, "/fr3/grasp/detach", 10)

    def move(self, client, names, positions, seconds):
        """向一个控制器发送单点关节轨迹，并同步等待该段动作完成。

        参数：
        - client：self.arm 或 self.gripper，决定使用哪个控制器；
        - names：对应的关节名列表；
        - positions：目标关节位置；长度必须与 names 完全相同；
        - seconds：从当前关节状态到目标位置所允许的运动时间（秒）。

        注意：本函数只执行给定目标，不做 IK、碰撞检测、桌面距离检测或路径绕障。
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        # ROS 的时间消息由整数秒和纳秒构成，以下两行允许 seconds 填 2.5 之类的小数。
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [point]
        # 对大幅度的 Gazebo 关节运动，明确给出比控制器默认值更宽的完成窗口。
        # 否则控制器会在 time_from_start 后仅等待配置中的 goal_time（之前是 2 s）
        # 就中止轨迹，即使机械臂已经非常接近目标。
        if client is self.arm:
            goal.goal_tolerance = [
                JointTolerance(name=name, position=0.08, velocity=0.10)
                for name in names
            ]
            goal.goal_time_tolerance = Duration(sec=10)
        # 发送动作目标；下面两次 spin_until_future_complete 会阻塞到“接受目标”和“动作结束”。
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("轨迹被控制器拒绝")
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"轨迹失败: {result.error_string}")

    def publish_grasp_command(self, publisher, count=3):
        """重复发布 attach/detach，确保 ROS-Gazebo bridge 已收到短消息。

        count 通常无需修改。detach 是幂等的，多次释放不会产生额外动作；attach 也只在
        夹爪到达 GRASP 且已经闭合之后调用，不能把它提前到接近动作之前。
        """
        for _ in range(count):
            publisher.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.15)

    def run(self):
        """按“释放→张开→上方→下探→闭合→附着→抬升→观察→放回”的顺序执行。"""
        if not self.arm.wait_for_server(10.0) or not self.gripper.wait_for_server(10.0):
            raise RuntimeError("机械臂或夹爪控制器未启动")
        # DetachableJoint 在 Ignition Gazebo 6 中初始为 attached。先解除，防止
        # 工件在夹爪尚未接触时就被 wrist3_link 带着走；也可清除上次异常退出的状态。
        self.get_logger().info("确认工件处于自由状态")
        self.publish_grasp_command(self.detach, count=5)
        self.get_logger().info("打开夹爪并垂直接近零件上方")
        # 1. 先把夹爪张开，再移动到工件正上方的安全姿态。
        self.move(self.gripper, GRIPPER, [GRIPPER_OPEN], T_OPEN)
        self.move(self.arm, ARM, ABOVE, T_ABOVE)
        self.get_logger().info("垂直下探到抓取高度")
        # 2. 下探。ABOVE 与 GRASP 应只产生沿工具轴的接近运动；改动后须目视确认。
        self.move(self.arm, ARM, GRASP, T_DESCEND)
        self.get_logger().info("闭合夹爪，随后建立 Gazebo 固定抓取约束")
        # 3. 闭合夹指；这是接触外观动作。随后 attach 才是让工件随末端运动的物理约束。
        self.move(self.gripper, GRIPPER, [GRIPPER_CLOSE], T_CLOSE)
        self.publish_grasp_command(self.attach)
        self.get_logger().info("抬升零件")
        # 4. 先抬升至安全高度，再旋转 j6 展示三个面，避免在桌面附近旋转。
        self.move(self.arm, ARM, LIFT, T_LIFT)
        for name, j6 in (("面 1", LIFT[5]), ("面 2", LIFT[5] + 1.570796),
                         ("面 3", LIFT[5] - 1.570796)):
            # copy() 很重要：它生成一个新姿态，避免直接修改全局 LIFT。
            # 如需改观察角度，修改 1.570796（90°）；不要超出 j6 的关节限位。
            target = LIFT.copy(); target[5] = j6
            self.get_logger().info(f"展示{name}")
            self.move(self.arm, ARM, target, T_INSPECT)
        self.get_logger().info("放回零件并释放")
        # 5. 先回到抓取高度，再 detach；绝不能在空中 detach，否则工件会掉落。
        self.move(self.arm, ARM, GRASP, T_RETURN)
        self.publish_grasp_command(self.detach)
        self.move(self.gripper, GRIPPER, [GRIPPER_OPEN], T_OPEN)
        self.move(self.arm, ARM, ABOVE, T_DESCEND)


def main(args=None):
    # ROS 2 的固定启动/清理模板。通常无需修改。
    rclpy.init(args=args)
    node = PickInspect()
    try:
        node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
