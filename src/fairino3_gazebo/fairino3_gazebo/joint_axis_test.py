"""逐轴验证 FR3 Gazebo 仿真关节是否能接收轨迹命令。

程序以仿真初始姿态为基准，依次只让 j1 至 j6 中的一轴偏转约 11.5 度，
随后回到初始姿态。其余五轴在每段轨迹中保持不变，因此可直观看出是哪一轴
没有响应。该程序不控制夹爪开合。
"""

import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

# 与 gz_ros2_control.xacro 中的 initial_value 一致，单位为弧度。
HOME_POSITIONS = [2.4468, -1.6214, 1.5465, -1.5877, -1.6368, 0.0]

# 每次测试的偏移约为 11.5°。幅度较小，可减少与桌面或自身发生碰撞的风险。
TEST_OFFSET_RAD = math.radians(11.5)


class JointAxisTester(Node):
    """通过 fairino3_controller 发送单关节轨迹。"""

    def __init__(self):
        super().__init__("fr3_joint_axis_test")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fairino3_controller/follow_joint_trajectory",
        )

    def move_to(self, positions, duration_sec=2.0):
        """发送完整六轴目标，并返回该轨迹是否成功执行。"""
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1_000_000_000
        )
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"发送目标位置（弧度）: {[round(value, 4) for value in positions]}"
        )
        goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("轨迹目标被控制器拒绝。")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error("未收到轨迹执行结果。")
            return False

        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "轨迹执行失败："
                f"error_code={result.result.error_code}, "
                f"error_string={result.result.error_string}"
            )
            return False

        self.get_logger().info("轨迹执行成功。")
        return True

    def run_test(self):
        """依次测试 j1 到 j6，并在每次测试后回到初始姿态。"""
        self.get_logger().info(
            "等待 /fairino3_controller/follow_joint_trajectory 动作服务..."
        )
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "控制器动作服务在 10 秒内未出现。请先启动 sim.launch.py，"
                "并确认 fairino3_controller 为 active。"
            )
            return False

        for index, joint_name in enumerate(JOINT_NAMES):
            target = HOME_POSITIONS.copy()
            target[index] += TEST_OFFSET_RAD

            self.get_logger().info(
                f"========== 测试 {joint_name}：仅该轴增加 11.5° =========="
            )
            if not self.move_to(target):
                return False

            self.get_logger().info(f"========== {joint_name} 回到初始姿态 ==========")
            if not self.move_to(HOME_POSITIONS):
                return False

        self.get_logger().info("六个关节均已完成逐轴测试。")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = JointAxisTester()

    try:
        node.run_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
