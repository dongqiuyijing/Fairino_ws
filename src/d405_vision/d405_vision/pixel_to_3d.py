#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge


class PixelTo3DNode(Node):

    def __init__(self):
        super().__init__('pixel_to_3d')

        # ============================================================
        # Configuration
        # ============================================================

        # 你这台 D405 已实测：
        # 1 raw depth unit = 1 mm = 0.001 m
        self.depth_scale = 0.001

        # 点击点周围 ROI 半径
        # radius = 5 -> 11 x 11 ROI
        self.roi_radius = 5

        # 至少需要多少有效深度像素
        self.min_valid_pixels = 5

        # ============================================================
        # Runtime state
        # ============================================================

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.latest_rgb = None
        self.latest_depth = None
        self.latest_depth_header = None

        self.clicked_u = None
        self.clicked_v = None

        self.last_xyz = None
        self.last_depth_mm = None
        self.last_valid_ratio = None

        # ============================================================
        # Subscribers
        # ============================================================

        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.point_pub = self.create_publisher(
            PointStamped,
            '/vision/target_point',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/vision/target_marker',
            10
        )

        # ============================================================
        # OpenCV window
        # ============================================================

        self.window_name = 'D405 RGB - Click Target'

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL
        )

        cv2.setMouseCallback(
            self.window_name,
            self.mouse_callback
        )

        # GUI refresh timer
        self.create_timer(
            1.0 / 30.0,
            self.gui_timer_callback
        )

        self.get_logger().info(
            'D405 click-to-3D node started.'
        )

        self.get_logger().info(
            'Click any valid point in the RGB window.'
        )

        self.get_logger().info(
            f'ROI = {2*self.roi_radius+1} x '
            f'{2*self.roi_radius+1}'
        )

    # ================================================================
    # CameraInfo
    # ================================================================

    def camera_info_callback(self, msg):

        if self.fx is not None:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            '\nCamera intrinsics received:\n'
            f'  frame = {self.camera_frame}\n'
            f'  fx    = {self.fx:.6f}\n'
            f'  fy    = {self.fy:.6f}\n'
            f'  cx    = {self.cx:.6f}\n'
            f'  cy    = {self.cy:.6f}'
        )

    # ================================================================
    # RGB
    # ================================================================

    def rgb_callback(self, msg):

        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(
                f'RGB conversion failed: {exc}'
            )

    # ================================================================
    # Depth
    # ================================================================

    def depth_callback(self, msg):

        if msg.encoding != '16UC1':
            self.get_logger().warning(
                f'Unexpected depth encoding: {msg.encoding}'
            )
            return

        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

            self.latest_depth_header = msg.header

        except Exception as exc:
            self.get_logger().error(
                f'Depth conversion failed: {exc}'
            )

    # ================================================================
    # Mouse
    # ================================================================

    def mouse_callback(
        self,
        event,
        x,
        y,
        flags,
        param
    ):

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.clicked_u = int(x)
        self.clicked_v = int(y)

        self.get_logger().info(
            f'Clicked pixel: '
            f'u={self.clicked_u}, '
            f'v={self.clicked_v}'
        )

        self.compute_clicked_point()

    # ================================================================
    # Pixel + ROI -> XYZ
    # ================================================================

    def compute_clicked_point(self):

        if self.latest_depth is None:
            self.get_logger().warning(
                'No aligned depth image received yet.'
            )
            return

        if self.fx is None:
            self.get_logger().warning(
                'CameraInfo has not been received yet.'
            )
            return

        u = self.clicked_u
        v = self.clicked_v

        height, width = self.latest_depth.shape[:2]

        if not (0 <= u < width and 0 <= v < height):
            self.get_logger().warning(
                f'Clicked pixel ({u},{v}) is outside '
                f'image {width}x{height}'
            )
            return

        # ------------------------------------------------------------
        # 11 x 11 ROI
        # ------------------------------------------------------------

        r = self.roi_radius

        u_min = max(0, u - r)
        u_max = min(width, u + r + 1)

        v_min = max(0, v - r)
        v_max = min(height, v + r + 1)

        roi = self.latest_depth[
            v_min:v_max,
            u_min:u_max
        ]

        # RealSense depth == 0 means invalid
        valid_depths = roi[roi > 0]

        valid_count = int(valid_depths.size)
        total_count = int(roi.size)

        if valid_count < self.min_valid_pixels:

            self.last_xyz = None
            self.last_depth_mm = None
            self.last_valid_ratio = (
                valid_count / total_count
                if total_count > 0 else 0.0
            )

            self.get_logger().warning(
                f'Not enough valid depth around '
                f'({u},{v}): '
                f'{valid_count}/{total_count}'
            )

            return

        # ------------------------------------------------------------
        # Median depth
        # ------------------------------------------------------------

        depth_raw = float(
            np.median(valid_depths)
        )

        z = depth_raw * self.depth_scale

        valid_ratio = (
            valid_count / total_count
        )

        # ------------------------------------------------------------
        # Pinhole back-projection
        #
        # optical frame:
        # +X = image right
        # +Y = image down
        # +Z = camera forward
        # ------------------------------------------------------------

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        self.last_xyz = (x, y, z)
        self.last_depth_mm = z * 1000.0
        self.last_valid_ratio = valid_ratio

        # ------------------------------------------------------------
        # Publish PointStamped
        # ------------------------------------------------------------

        point = PointStamped()

        point.header = self.latest_depth_header

        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        self.point_pub.publish(point)

        # ------------------------------------------------------------
        # Publish RViz Marker
        # ------------------------------------------------------------

        marker = Marker()

        marker.header = self.latest_depth_header

        marker.ns = 'd405_clicked_target'
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.w = 1.0

        # 调试阶段故意用 3 cm，方便看清
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # lifetime = 0 -> 一直保留
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

        # ------------------------------------------------------------
        # Log
        # ------------------------------------------------------------

        self.get_logger().info(
            '\n'
            '========== CLICKED TARGET ==========\n'
            f'Pixel        : ({u}, {v})\n'
            f'ROI          : {roi.shape[1]} x '
            f'{roi.shape[0]}\n'
            f'Valid depth  : '
            f'{valid_count}/{total_count} '
            f'({valid_ratio*100:.1f}%)\n'
            f'Median raw   : {depth_raw:.1f}\n'
            f'Depth        : {z:.4f} m '
            f'({z*1000:.1f} mm)\n'
            '\n'
            f'X = {x:.4f} m '
            f'({x*1000:.1f} mm)\n'
            f'Y = {y:.4f} m '
            f'({y*1000:.1f} mm)\n'
            f'Z = {z:.4f} m '
            f'({z*1000:.1f} mm)\n'
            '\n'
            f'Frame = '
            f'{self.latest_depth_header.frame_id}\n'
            '===================================='
        )

    # ================================================================
    # GUI
    # ================================================================

    def gui_timer_callback(self):

        if self.latest_rgb is None:
            return

        display = self.latest_rgb.copy()

        # ------------------------------------------------------------
        # Draw optical center
        # ------------------------------------------------------------

        if self.cx is not None:

            cx = int(round(self.cx))
            cy = int(round(self.cy))

            cv2.drawMarker(
                display,
                (cx, cy),
                (255, 0, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=18,
                thickness=1
            )

        # ------------------------------------------------------------
        # Draw clicked point + ROI
        # ------------------------------------------------------------

        if (
            self.clicked_u is not None
            and self.clicked_v is not None
        ):

            u = self.clicked_u
            v = self.clicked_v
            r = self.roi_radius

            cv2.circle(
                display,
                (u, v),
                5,
                (0, 0, 255),
                -1
            )

            cv2.rectangle(
                display,
                (u-r, v-r),
                (u+r, v+r),
                (0, 255, 255),
                1
            )

            cv2.putText(
                display,
                f'({u},{v})',
                (u + 10, v - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
                cv2.LINE_AA
            )

        # ------------------------------------------------------------
        # Show XYZ
        # ------------------------------------------------------------

        if self.last_xyz is not None:

            x, y, z = self.last_xyz

            text1 = (
                f'XYZ [mm]: '
                f'{x*1000:.1f}, '
                f'{y*1000:.1f}, '
                f'{z*1000:.1f}'
            )

            text2 = (
                f'ROI valid: '
                f'{self.last_valid_ratio*100:.1f}%'
            )

            cv2.putText(
                display,
                text1,
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                display,
                text2,
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        else:

            cv2.putText(
                display,
                'Click a target',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

        cv2.imshow(
            self.window_name,
            display
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.get_logger().info(
                'q pressed, shutting down.'
            )
            rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = PixelTo3DNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        cv2.destroyAllWindows()

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
