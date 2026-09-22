#!/usr/bin/env python3

import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


WINDOW_NAME = 'D405 Grayscale Depth'
SLIDER_MIN_NAME = 'Min Depth (mm)'
SLIDER_MAX_NAME = 'Max Depth (mm)'
SLIDER_MAX_MM = 1000
DEFAULT_MIN_MM = 100
DEFAULT_MAX_MM = 250
# 当前这台 D405 实测：1 raw unit = 1 mm。不要改成 0.0001。
DEFAULT_DEPTH_SCALE = 0.001
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
STREAM_TIMEOUT_SEC = 1.0
GUI_RATE_HZ = 30.0
PLACEHOLDER_SIZE = (480, 848)


def _ignore_trackbar(_value):
    return


def resolve_depth_range(min_depth_mm, max_depth_mm):
    # min >= max 时交换；相等时把上限加 1 mm，避免除零。
    try:
        min_mm = float(min_depth_mm)
        max_mm = float(max_depth_mm)
    except (TypeError, ValueError):
        return float(DEFAULT_MIN_MM), float(DEFAULT_MAX_MM), True

    adjusted = False
    if not np.isfinite(min_mm) or not np.isfinite(max_mm):
        return float(DEFAULT_MIN_MM), float(DEFAULT_MAX_MM), True
    if min_mm > max_mm:
        min_mm, max_mm = max_mm, min_mm
        adjusted = True
    if min_mm == max_mm:
        max_mm = min_mm + 1.0
        adjusted = True
    return min_mm, max_mm, adjusted


def map_depth_to_grayscale(
    depth_raw,
    min_depth_mm,
    max_depth_mm,
    depth_scale=DEFAULT_DEPTH_SCALE,
):
    # 固定窗口映射，不按每帧最小/最大深度自动归一化。
    # 近处亮、远处暗。raw == 0 保持黑色，不能当成 0 mm 的近处物体。
    # 窗口外的有效深度饱和到 1 或 255，不能当成无效深度。
    # 不修改 depth_raw，也不做空洞填充。
    depth = np.asarray(depth_raw)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[:, :, 0]

    if depth.ndim != 2:
        return np.zeros((0, 0), dtype=np.uint8)

    gray = np.zeros(depth.shape, dtype=np.uint8)
    if depth.size == 0:
        return gray

    min_mm, max_mm, _adjusted = resolve_depth_range(
        min_depth_mm,
        max_depth_mm,
    )
    valid = depth != 0
    if not np.any(valid):
        return gray

    # 先把 scale 合成 mm/unit。当前设备 0.001 * 1000 = 1，
    # raw 值就是毫米；分开乘会把 175 变成 175.00000000000003。
    mm_per_unit = float(depth_scale) * 1000.0
    depth_mm = depth.astype(np.float64) * mm_per_unit
    mapped = 1.0 + 254.0 * (max_mm - depth_mm) / (max_mm - min_mm)
    mapped = np.clip(mapped, 1.0, 255.0).astype(np.uint8)
    gray[valid] = mapped[valid]
    return gray


def center_depth_mm(depth_raw, depth_scale=DEFAULT_DEPTH_SCALE):
    depth = np.asarray(depth_raw)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[:, :, 0]
    if depth.ndim != 2 or depth.size == 0:
        return None
    raw = depth[depth.shape[0] // 2, depth.shape[1] // 2]
    if raw == 0:
        return None
    return float(raw) * float(depth_scale) * 1000.0


def valid_depth_ratio(depth_raw):
    depth = np.asarray(depth_raw)
    if depth.size == 0:
        return 0.0
    return float(np.count_nonzero(depth)) / float(depth.size)


def format_center_depth(depth_mm):
    if depth_mm is None:
        return 'Center Depth: Invalid'
    if abs(depth_mm - round(depth_mm)) < 1e-3:
        return f'Center Depth: {int(round(depth_mm))} mm'
    return f'Center Depth: {depth_mm:.1f} mm'


def _max_numbered_png(directory):
    max_index = 0
    prefix = 'depth_gray_'
    for path in directory.glob('depth_gray_*.png'):
        suffix = path.stem[len(prefix):]
        if len(suffix) == 6 and suffix.isdigit():
            max_index = max(max_index, int(suffix))
    return max_index


def save_grayscale_png(directory, gray, metadata_text):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if gray is None:
        raise ValueError('gray image is None')

    index = _max_numbered_png(directory) + 1
    while index <= 999999:
        png_path = directory / f'depth_gray_{index:06d}.png'
        txt_path = directory / f'depth_gray_{index:06d}.txt'
        if txt_path.exists():
            index += 1
            continue
        try:
            descriptor = os.open(
                str(png_path),
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                0o644,
            )
        except FileExistsError:
            index += 1
            continue
        os.close(descriptor)
        try:
            if not cv2.imwrite(str(png_path), gray):
                raise RuntimeError('cv2.imwrite returned False')
            txt_path.write_text(metadata_text, encoding='utf-8')
        except Exception:
            png_path.unlink(missing_ok=True)
            txt_path.unlink(missing_ok=True)
            raise
        return png_path, txt_path
    raise RuntimeError('no free depth_gray_######.png name left')


def render_overlay(gray, lines):
    display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if not lines:
        return display

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    pad = 8
    gap = 4
    sizes = []
    for line in lines:
        (text_w, text_h), baseline = cv2.getTextSize(
            line,
            font,
            scale,
            thickness,
        )
        sizes.append((text_w, text_h, baseline))

    panel_w = max(item[0] for item in sizes) + pad * 2
    panel_h = pad
    for _text_w, text_h, baseline in sizes:
        panel_h += text_h + baseline + gap
    panel_h += pad - gap

    image_h, image_w = display.shape[:2]
    panel_w = max(1, min(panel_w, image_w - 2))
    panel_h = max(1, min(panel_h, image_h - 2))
    cv2.rectangle(
        display,
        (1, 1),
        (1 + panel_w, 1 + panel_h),
        (0, 0, 0),
        -1,
    )

    y = 1 + pad
    for line, (_text_w, text_h, baseline) in zip(lines, sizes):
        y += text_h
        cv2.putText(
            display,
            line,
            (1 + pad, y),
            font,
            scale,
            (240, 240, 240),
            thickness,
            cv2.LINE_AA,
        )
        y += baseline + gap
    return display


class DepthGrayscaleNode(Node):

    def __init__(self):
        super().__init__('depth_grayscale')

        default_save_dir = str(
            Path.home() / 'datasets' / 'd405_depth_debug'
        )
        self.declare_parameter('min_depth_mm', DEFAULT_MIN_MM)
        self.declare_parameter('max_depth_mm', DEFAULT_MAX_MM)
        self.declare_parameter('depth_scale', DEFAULT_DEPTH_SCALE)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('save_dir', default_save_dir)

        self.depth_scale = self._read_depth_scale()
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        if not self.save_dir:
            self.save_dir = default_save_dir
        self.slider_min = self._read_mm('min_depth_mm', DEFAULT_MIN_MM)
        self.slider_max = self._read_mm('max_depth_mm', DEFAULT_MAX_MM)

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_depth = None
        self._latest_arrival = None
        self._received_frame = False
        self._logged_rate = False
        self._shutting_down = False
        self._warn_times = {}
        self._frame_count = 0
        self._fps_window_start = None
        self._fps = 0.0
        self.last_gray = None

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 540)
        cv2.createTrackbar(
            SLIDER_MIN_NAME,
            WINDOW_NAME,
            self.slider_min,
            SLIDER_MAX_MM,
            _ignore_trackbar,
        )
        cv2.createTrackbar(
            SLIDER_MAX_NAME,
            WINDOW_NAME,
            self.slider_max,
            SLIDER_MAX_MM,
            _ignore_trackbar,
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / GUI_RATE_HZ, self._gui_timer)

        self.get_logger().info(
            'D405 grayscale depth viewer started.\n'
            f'  topic = {self.depth_topic}\n'
            f'  depth_scale = {self.depth_scale} m/unit\n'
            f'  default range = {self.slider_min}-{self.slider_max} mm\n'
            f'  save_dir = {self.save_dir}\n'
            '  near = bright, far = dark, invalid = black\n'
            '  q: quit, s: save PNG visualization'
        )

    def _read_depth_scale(self):
        value = self.get_parameter('depth_scale').value
        try:
            scale = float(value)
        except (TypeError, ValueError):
            scale = -1.0
        if not np.isfinite(scale) or scale <= 0.0:
            self.get_logger().warning(
                'Invalid depth_scale, using 0.001 m/unit.'
            )
            return DEFAULT_DEPTH_SCALE
        return scale

    def _read_mm(self, name, default):
        value = self.get_parameter(name).value
        try:
            number = int(round(float(value)))
        except (TypeError, ValueError):
            self.get_logger().warning(
                f'Invalid {name}, using {default} mm.'
            )
            return default
        if number < 0 or number > SLIDER_MAX_MM:
            clamped = min(max(number, 0), SLIDER_MAX_MM)
            self.get_logger().warning(
                f'{name}={number} mm clamped to {clamped} mm.'
            )
            return clamped
        return number

    def _warn_throttled(self, key, message, period=2.0):
        now = time.monotonic()
        last = self._warn_times.get(key, 0.0)
        if now - last >= period:
            self.get_logger().warning(message)
            self._warn_times[key] = now

    def _request_shutdown(self, reason):
        # 不要在定时器回调里调用 rclpy.shutdown()，否则会和 spin 互相等待。
        if self._shutting_down:
            return
        self._shutting_down = True
        self.get_logger().info(reason)

    def _depth_callback(self, msg):
        if self._shutting_down:
            return
        if msg.encoding != '16UC1':
            self._warn_throttled(
                'encoding',
                f'Expected depth encoding 16UC1, got {msg.encoding}.',
            )
            return
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough',
            )
        except Exception as exc:
            self._warn_throttled(
                'convert',
                f'Depth conversion failed: {exc}',
            )
            return

        image = np.squeeze(np.asarray(image))
        if image.ndim != 2 or image.size == 0:
            self._warn_throttled(
                'shape',
                f'Unexpected depth shape: {image.shape}',
            )
            return

        copied = np.array(image, copy=True)
        now = time.monotonic()
        with self._lock:
            self._latest_depth = copied
            self._latest_arrival = now
        self._update_fps(now)
        if not self._received_frame:
            self._received_frame = True
            self.get_logger().info(
                'Received 16UC1 depth '
                f'{copied.shape[1]}x{copied.shape[0]}.'
            )

    def _update_fps(self, now):
        if self._fps_window_start is None:
            self._fps_window_start = now
            self._frame_count = 0
        self._frame_count += 1
        elapsed = now - self._fps_window_start
        if elapsed >= 1.0:
            self._fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_window_start = now
            if not self._logged_rate:
                self._logged_rate = True
                self.get_logger().info(
                    f'Depth stream rate about {self._fps:.1f} Hz.'
                )

    def _current_depth(self):
        with self._lock:
            return self._latest_depth, self._latest_arrival

    def _read_sliders(self):
        min_pos = cv2.getTrackbarPos(SLIDER_MIN_NAME, WINDOW_NAME)
        max_pos = cv2.getTrackbarPos(SLIDER_MAX_NAME, WINDOW_NAME)
        if min_pos >= 0:
            self.slider_min = int(min_pos)
        if max_pos >= 0:
            self.slider_max = int(max_pos)

    def _compose_lines(self, depth, arrival):
        lines = [
            f'Min Depth: {self.slider_min} mm',
            f'Max Depth: {self.slider_max} mm',
        ]
        min_mm, max_mm, adjusted = resolve_depth_range(
            self.slider_min,
            self.slider_max,
        )
        if adjusted:
            lines.append(f'Using: {min_mm:.0f}-{max_mm:.0f} mm')

        status = None
        if depth is None:
            status = 'Waiting for depth...'
        else:
            if (
                arrival is not None
                and time.monotonic() - arrival > STREAM_TIMEOUT_SEC
            ):
                status = 'Depth stream lost'
            ratio = valid_depth_ratio(depth)
            center = center_depth_mm(depth, self.depth_scale)
            lines.append(f'Valid Depth: {ratio * 100.0:.1f}%')
            lines.append(format_center_depth(center))
            if self._fps > 0.0:
                lines.append(f'Rate: {self._fps:.1f} Hz')
        if status is not None:
            lines.append(status)
        lines.append('q: quit    s: save')
        return lines

    def _gui_timer(self):
        if self._shutting_down or not rclpy.ok():
            return

        self._read_sliders()
        depth, arrival = self._current_depth()
        if depth is None:
            gray = np.zeros(PLACEHOLDER_SIZE, dtype=np.uint8)
        else:
            gray = map_depth_to_grayscale(
                depth,
                self.slider_min,
                self.slider_max,
                self.depth_scale,
            )
            if gray.size == 0:
                gray = np.zeros(PLACEHOLDER_SIZE, dtype=np.uint8)
            else:
                self.last_gray = gray

        lines = self._compose_lines(depth, arrival)
        display = render_overlay(gray, lines)
        try:
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            self._warn_throttled('gui', f'OpenCV GUI error: {exc}')
            return

        if key in (ord('q'), ord('Q')):
            self._request_shutdown('q pressed, shutting down.')
        elif key in (ord('s'), ord('S')):
            self.save_current_view()

    def save_current_view(self):
        gray = self.last_gray
        if gray is None:
            self.get_logger().warning('No depth frame to save.')
            return None

        min_mm, max_mm, adjusted = resolve_depth_range(
            self.slider_min,
            self.slider_max,
        )
        depth, _arrival = self._current_depth()
        if depth is None:
            center_text = 'Center Depth: Invalid'
            ratio = 0.0
        else:
            center_text = format_center_depth(
                center_depth_mm(depth, self.depth_scale)
            )
            ratio = valid_depth_ratio(depth)

        metadata = '\n'.join([
            'visualization_only: true',
            'note: 8-bit grayscale visualization of D405 depth.',
            'note: this PNG does not store raw 16-bit depth.',
            'note: millimeters cannot be recovered from this PNG.',
            f'min_depth_mm: {self.slider_min}',
            f'max_depth_mm: {self.slider_max}',
            f'effective_min_depth_mm: {min_mm:.0f}',
            f'effective_max_depth_mm: {max_mm:.0f}',
            f'range_adjusted: {str(adjusted).lower()}',
            f'depth_scale_m_per_unit: {self.depth_scale}',
            f'center_depth: {center_text}',
            f'valid_ratio: {ratio:.6f}',
            f'width: {gray.shape[1]}',
            f'height: {gray.shape[0]}',
            f'saved_at: {time.strftime("%Y-%m-%d %H:%M:%S")}',
        ]) + '\n'
        try:
            png_path, txt_path = save_grayscale_png(
                self.save_dir,
                gray.copy(),
                metadata,
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to save grayscale: {exc}')
            return None
        self.get_logger().info(
            f'Saved visualization: {png_path}\n  metadata: {txt_path}'
        )
        return png_path


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthGrayscaleNode()
        while rclpy.ok() and not node._shutting_down:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
