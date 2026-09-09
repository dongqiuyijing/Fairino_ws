"""在 Gazebo 模型生成后解除 DetachableJoint 的默认初始附着状态。"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class GraspReset(Node):
    def __init__(self):
        super().__init__("fr3_initial_grasp_reset")
        self.detach = self.create_publisher(Empty, "/fr3/grasp/detach", 10)

    def release(self):
        # 发布数次是为了跨越 Gazebo transport bridge 的启动时序，且 detach 是幂等操作。
        for _ in range(6):
            self.detach.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)
        self.get_logger().info("已解除 small_part 的初始附着状态")


def main(args=None):
    rclpy.init(args=args)
    node = GraspReset()
    try:
        node.release()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
