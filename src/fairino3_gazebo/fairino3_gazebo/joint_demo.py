import math

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


class FR3GazeboDemo(Node):

    def __init__(self):

        super().__init__(
            "fr3_gazebo_demo"
        )

        self.client = ActionClient(

            self,

            FollowJointTrajectory,

            "/fairino3_controller/"
            "follow_joint_trajectory",
        )


    def move_joints(
        self,
        positions,
        duration=3,
    ):

        self.get_logger().info(
            "Waiting for controller..."
        )

        self.client.wait_for_server()


        trajectory = JointTrajectory()

        trajectory.joint_names = [
            "j1",
            "j2",
            "j3",
            "j4",
            "j5",
            "j6",
        ]


        point = JointTrajectoryPoint()

        point.positions = positions

        point.time_from_start.sec = duration


        trajectory.points.append(point)


        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory


        self.get_logger().info(
            f"Sending target: {positions}"
        )


        goal_future = (
            self.client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            goal_future,
        )


        goal_handle = goal_future.result()


        if not goal_handle.accepted:

            self.get_logger().error(
                "Trajectory rejected."
            )

            return False


        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )


        self.get_logger().info(
            "Motion finished."
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = FR3GazeboDemo()


    # FAIRINO 官方 pos1
    pos1 = [

        2.4468,

        -1.6214,

        1.5465,

        -1.5877,

        -1.6368,

        0.0,
    ]


    node.get_logger().info(
        "Move to Face 1"
    )

    node.move_joints(
        pos1,
        duration=4,
    )


    # ------------------------------
    # Face 2
    # J6 +90°
    # ------------------------------

    face2 = pos1.copy()

    face2[5] = math.pi / 2


    node.get_logger().info(
        "Rotate +90 deg"
    )

    node.move_joints(
        face2,
        duration=3,
    )


    # ------------------------------
    # Face 3
    # 从 +90° 到 -90°
    # 即反方向 180°
    # ------------------------------

    face3 = pos1.copy()

    face3[5] = -math.pi / 2


    node.get_logger().info(
        "Rotate -180 deg"
    )

    node.move_joints(
        face3,
        duration=4,
    )


    node.get_logger().info(
        "Gazebo demo complete."
    )


    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()