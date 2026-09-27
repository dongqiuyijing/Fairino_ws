"""Thread-safe PyQt5 desktop GUI for the fr3_teleop manager.

The GUI only publishes intents. ROS callbacks run in the executor thread and
must never manipulate Qt widgets directly; parsed status reaches the Qt thread
through ``status_received`` instead.
"""

from __future__ import annotations

import json
import os
import sys
from threading import Lock, Thread, current_thread
from typing import Any

from PyQt5.QtCore import QSignalBlocker, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QComboBox, QDoubleSpinBox, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QRadioButton, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class TeleopWindow(QMainWindow):
    """Hold-to-move UI. All QWidget access is confined to the Qt thread."""

    status_decode_error = pyqtSignal(str)

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: Thread | None = None
        self._closing = False
        self._ros_shutdown_started = False
        self._node_destroyed = False
        self._manager_status: dict[str, Any] = {}
        self._latest_status: dict[str, Any] | None = None
        self._last_applied_status: dict[str, Any] | None = None
        self._status_apply_count = 0
        self._status_lock = Lock()
        self._manager_arm = "arm_a"
        self._manager_frame = "world"
        self._was_active = False
        self._debug_events = os.environ.get("FR3_TELEOP_GUI_DEBUG_EVENTS", "").lower() in {"1", "true", "yes"}
        self._diagnostic_counts = {key: 0 for key in ("selected_arm", "frame", "manual_ready", "owner", "active_input_source")}
        self.held: tuple[str, int] | None = None
        self._motion_buttons: dict[tuple[str, int], QPushButton] = {}
        self._arm_buttons: dict[str, QRadioButton] = {}
        self._gripper_buttons: list[QPushButton] = []

        self.publisher = node.create_publisher(String, "/fr3_teleop/command", 30)
        node.create_subscription(String, "/fr3_teleop/status", self._status, 20)

        # Finish all widgets and all signal connections before ROS is spun.
        self.setWindowTitle("FR3 Dual Arm Teleoperation")
        self._build()
        self.status_decode_error.connect(self._show_status_decode_error)
        self.repeat = QTimer(self)
        self.repeat.setInterval(50)
        self.repeat.timeout.connect(self._repeat_motion)
        self.status_refresh = QTimer(self)
        self.status_refresh.setInterval(100)  # 10 Hz is ample for an operator display.
        self.status_refresh.timeout.connect(self._apply_latest_status)
        self.status_refresh.start()
        self.status_diagnostics = QTimer(self)
        self.status_diagnostics.setInterval(10_000)
        self.status_diagnostics.timeout.connect(self._report_status_diagnostics)
        self.status_diagnostics.start()
        self._set_motion_controls_enabled(False)

    def start_ros(self, executor: SingleThreadedExecutor) -> None:
        """Start spinning only after construction and signal wiring are complete."""
        if self._closing or self._spin_thread is not None:
            return
        self._executor = executor

        def spin_ros() -> None:
            try:
                executor.spin()
            except ExternalShutdownException:
                pass
            except Exception as exc:
                if not self._closing:
                    self.status_decode_error.emit(f"ROS executor error: {exc}")

        self._spin_thread = Thread(target=spin_ros, name="fr3_teleop_ros_spin", daemon=False)
        self._spin_thread.start()

    def _build(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        selection = QHBoxLayout()
        self.arm_group = QButtonGroup(self)
        for arm, label in (("arm_a", "Arm A"), ("arm_b", "Arm B")):
            button = QRadioButton(label)
            button.setChecked(arm == "arm_a")
            # clicked is user intent; setChecked() during status sync never emits it.
            button.clicked.connect(lambda _checked, name=arm: self._select_arm(name))
            self.arm_group.addButton(button)
            self._arm_buttons[arm] = button
            selection.addWidget(button)
        self.enable = QPushButton("Enable Manual Control")
        self.enable.setCheckable(True)
        self.enable.clicked.connect(self._request_manual)
        selection.addWidget(self.enable)
        layout.addLayout(selection)

        controls = QHBoxLayout()
        frame_box = QGroupBox("Reference frame")
        frame_layout = QVBoxLayout(frame_box)
        self.frame = QComboBox()
        self.frame.addItems(["world", "base", "tool"])
        # activated is user intent; programmatic current-text changes do not emit it.
        self.frame.activated[str].connect(self._set_frame)
        frame_layout.addWidget(self.frame)
        frame_layout.addWidget(QLabel("world: shared workcell; base: selected arm; tool: TCP axes"))
        controls.addWidget(frame_box)
        speed_box = QGroupBox("Speed limits")
        speed_layout = QFormLayout(speed_box)
        self.linear = QDoubleSpinBox()
        self.linear.setRange(0.1, 20.0)
        self.linear.setValue(5.0)
        self.linear.setSuffix(" mm/s")
        self.angular = QDoubleSpinBox()
        self.angular.setRange(0.1, 10.0)
        self.angular.setValue(2.0)
        self.angular.setSuffix(" deg/s")
        self.linear.editingFinished.connect(self._set_speed)
        self.angular.editingFinished.connect(self._set_speed)
        speed_layout.addRow("Linear", self.linear)
        speed_layout.addRow("Angular", self.angular)
        controls.addWidget(speed_box)
        layout.addLayout(controls)
        layout.addWidget(self._motion_box(
            "Translation",
            [("X+", "x", 1), ("X-", "x", -1), ("Y+", "y", 1), ("Y-", "y", -1), ("Z+", "z", 1), ("Z-", "z", -1)],
        ))
        layout.addWidget(self._motion_box(
            "Rotation about selected frame axes",
            [("Rx+", "rx", 1), ("Rx-", "rx", -1), ("Ry+", "ry", 1), ("Ry-", "ry", -1), ("Rz+", "rz", 1), ("Rz-", "rz", -1)],
        ))
        grip = QHBoxLayout()
        for text, command in (("Open gripper", "open"), ("Close gripper", "close")):
            button = QPushButton(text)
            button.clicked.connect(lambda _, value=command: self._send({"type": "gripper", "command": value}))
            self._gripper_buttons.append(button)
            grip.addWidget(button)
        layout.addLayout(grip)
        self.status_label = QLabel("Manager: OFFLINE / UNKNOWN — waiting for /fr3_teleop/status…")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        stop = QPushButton("STOP — software normal stop, NOT a safety E-stop")
        stop.setStyleSheet("font-size: 18px; font-weight: bold; background: #b00020; color: white; padding: 10px")
        stop.clicked.connect(lambda: self.stop("operator-stop"))
        layout.addWidget(stop)
        self.setCentralWidget(root)

    def _motion_box(self, title: str, entries: list[tuple[str, str, int]]) -> QGroupBox:
        box = QGroupBox(title)
        grid = QGridLayout(box)
        for index, (label, axis, sign) in enumerate(entries):
            button = QPushButton(label)
            button.pressed.connect(lambda a=axis, s=sign: self._press(a, s))
            button.released.connect(lambda: self.stop("button-released"))
            self._motion_buttons[(axis, sign)] = button
            grid.addWidget(button, index // 2, index % 2)
        return box

    def _send(self, data: dict[str, Any]) -> None:
        if self._closing:
            return
        try:
            data.setdefault("source", "gui")
            message = String()
            message.data = json.dumps(data)
            self.publisher.publish(message)
        except Exception as exc:
            self.status_decode_error.emit(f"GUI command publish failed: {exc}")

    def _log_user_event(self, text: str) -> None:
        """Opt-in trace used to prove status synchronization cannot create commands."""
        if self._debug_events:
            self.node.get_logger().info(f"[GUI USER EVENT] {text}")

    def _select_arm(self, arm: str) -> None:
        # This is a request only. Manager status remains the source of truth.
        self._log_user_event(f"select_arm {arm}")
        self.stop("robot-switch-request")
        self._send({"type": "select_arm", "arm": arm})

    def _set_frame(self, frame: str) -> None:
        self._log_user_event(f"frame {frame}")
        self.stop("frame-switch-request")
        self._send({"type": "set_frame", "frame": frame})

    def _request_manual(self, requested: bool) -> None:
        # Do not let optimistic local UI state masquerade as ownership.
        self._log_user_event(f"manual_enable {requested}")
        self._send({"type": "enable", "value": requested})

    def _set_speed(self) -> None:
        self._log_user_event(f"speed linear_mm_s={self.linear.value()} angular_deg_s={self.angular.value()}")
        self.stop("speed-change")
        self._send({"type": "set_speed", "linear_mm_s": self.linear.value(), "angular_deg_s": self.angular.value()})

    def _press(self, axis: str, sign: int) -> None:
        button = self._motion_buttons[(axis, sign)]
        if not button.isEnabled() or not bool(self._manager_status.get("manual_ready", False)):
            return
        self.held = (axis, sign)
        self._repeat_motion()
        self.repeat.start()

    def _repeat_motion(self) -> None:
        if self.held is not None and not self._closing:
            axis, sign = self.held
            self._send({"type": "motion", "arm": self._manager_arm, "axis": axis, "sign": sign})

    def stop(self, reason: str) -> None:
        if hasattr(self, "repeat"):
            self.repeat.stop()
        self.held = None
        self._send({"type": "stop", "reason": reason})

    def _status(self, msg: String) -> None:
        """ROS callback: keep only the newest status; never touch Qt widgets."""
        if self._closing:
            return
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError("manager status is not a JSON object")
            with self._status_lock:
                self._latest_status = value
        except Exception as exc:
            try:
                self.node.get_logger().error(f"unable to decode manager status: {exc}")
            except Exception:
                pass
            self.status_decode_error.emit(f"Status decode error: {exc}")

    def _apply_latest_status(self) -> None:
        """Qt-thread timer slot: coalesce high-rate ROS status into a 10 Hz display."""
        with self._status_lock:
            value = dict(self._latest_status) if self._latest_status is not None else None
        if value is not None:
            self._apply_status(value)

    def _apply_status(self, value: dict[str, Any]) -> None:
        """Qt-thread-only, state-diffed UI synchronization without command publication."""
        if self._closing:
            return
        previous = self._last_applied_status or {}
        if value == previous:
            return
        for key in self._diagnostic_counts:
            if key in previous and previous.get(key) != value.get(key):
                self._diagnostic_counts[key] += 1
        control_changed = self._changed(
            previous, value, "selected_arm", "frame", "manual_ready", "owner", "active_input_source",
        )
        if self._debug_events and (not previous or control_changed):
            self.node.get_logger().info(
                "[GUI STATUS APPLY] "
                f"selected_arm={value.get('selected_arm')} frame={value.get('frame')} "
                f"manual_ready={value.get('manual_ready')}"
            )
        self._manager_status = value
        self._manager_arm = str(value.get("selected_arm", self._manager_arm))
        self._manager_frame = str(value.get("frame", self._manager_frame))
        if previous.get("selected_arm") != self._manager_arm:
            self._sync_arm_widgets()
        if previous.get("frame") != self._manager_frame:
            self._sync_frame_widget()
        if self._changed(previous, value, "linear_speed_mm_s", "angular_speed_deg_s"):
            self._sync_speed_widgets(value)
        if self._changed(previous, value, "manual_transition", "manual_ready"):
            self._sync_manual_widgets()
        if self._changed(previous, value, "manual_ready", "servo_nonzero_output_enabled", "backend", "first_real_test_mode", "selected_arm", "frame"):
            self._set_motion_controls_enabled(bool(value.get("manual_ready", False)))
        text = self._status_text(value)
        if self.status_label.text() != text:
            self.status_label.setText(text)
        self._last_applied_status = value
        self._status_apply_count += 1

    @staticmethod
    def _changed(previous: dict[str, Any], current: dict[str, Any], *keys: str) -> bool:
        return any(previous.get(key) != current.get(key) for key in keys)

    def _sync_arm_widgets(self) -> None:
        if self._manager_arm not in self._arm_buttons:
            return
        blockers = [QSignalBlocker(button) for button in self._arm_buttons.values()]
        self._arm_buttons[self._manager_arm].setChecked(True)
        del blockers

    def _sync_frame_widget(self) -> None:
        if self._manager_frame not in {self.frame.itemText(i) for i in range(self.frame.count())}:
            return
        blocker = QSignalBlocker(self.frame)
        self.frame.setCurrentText(self._manager_frame)
        del blocker

    def _sync_speed_widgets(self, value: dict[str, Any]) -> None:
        blocker_linear, blocker_angular = QSignalBlocker(self.linear), QSignalBlocker(self.angular)
        if "linear_speed_mm_s" in value:
            self.linear.setValue(float(value["linear_speed_mm_s"]))
        if "angular_speed_deg_s" in value:
            self.angular.setValue(float(value["angular_speed_deg_s"]))
        del blocker_linear, blocker_angular

    def _sync_manual_widgets(self) -> None:
        transition = str(self._manager_status.get("manual_transition", "UNKNOWN"))
        ready = bool(self._manager_status.get("manual_ready", False))
        blocker = QSignalBlocker(self.enable)
        self.enable.setChecked(ready)
        if ready:
            self.enable.setText("Release Manual Control")
        elif transition in {"REQUESTING", "HOLD_PENDING", "RELEASING"}:
            self.enable.setText(f"Manual {transition.lower().replace('_', ' ')}…")
        elif transition in {"DENIED", "FAULT"}:
            self.enable.setText(f"Manual {transition.lower()} — retry")
        else:
            self.enable.setText("Enable Manual Control")
        self.enable.setEnabled(transition not in {"REQUESTING", "HOLD_PENDING", "RELEASING"})
        del blocker

    def _set_motion_controls_enabled(self, manual_ready: bool) -> None:
        first_test = bool(self._manager_status.get("first_real_test_mode", False))
        only_allowed = self._manager_arm == "arm_a" and self._manager_frame == "base"
        backend = str(self._manager_status.get("backend", ""))
        output_allowed = backend == "mock" or bool(self._manager_status.get("servo_nonzero_output_enabled", False))
        for key, button in self._motion_buttons.items():
            allowed = manual_ready and output_allowed
            if first_test:
                allowed = allowed and only_allowed and key == ("x", 1)
            button.setEnabled(allowed)
        if first_test:
            self._arm_buttons["arm_b"].setEnabled(False)
            for index in range(self.frame.count()):
                self.frame.model().item(index).setEnabled(self.frame.itemText(index) == "base")
            self.linear.setMaximum(10.0)
            if self.linear.value() > 10.0:
                blocker = QSignalBlocker(self.linear)
                self.linear.setValue(10.0)
                del blocker
            self.angular.setEnabled(False)
            for button in self._gripper_buttons:
                button.setEnabled(False)
        else:
            self._arm_buttons["arm_b"].setEnabled(True)
            for index in range(self.frame.count()):
                self.frame.model().item(index).setEnabled(True)
            self.linear.setMaximum(20.0)
            self.angular.setEnabled(True)
            for button in self._gripper_buttons:
                button.setEnabled(manual_ready)

    def _report_status_diagnostics(self) -> None:
        if not self._debug_events or self._closing:
            return
        self.node.get_logger().info(
            "[GUI STATUS DIAGNOSTICS /10s] " + " ".join(
                f"{key}={count}" for key, count in self._diagnostic_counts.items()
            )
        )
        for key in self._diagnostic_counts:
            self._diagnostic_counts[key] = 0

    @staticmethod
    def _status_text(value: dict[str, Any]) -> str:
        joints = TeleopWindow._display_value(value.get("joint_positions_rad", "UNKNOWN"))
        tcp_pose = TeleopWindow._display_value(value.get("tcp_pose", "UNKNOWN"))
        return (
            f"Arm: {value.get('selected_arm', 'UNKNOWN')} | frame: {value.get('frame', 'UNKNOWN')} | "
            f"owner: {value.get('owner', 'UNKNOWN')} | input: {value.get('active_input_source', 'none')} | "
            f"motion: {value.get('motion', 'UNKNOWN')} | backend: {value.get('backend', 'UNKNOWN')} | "
            f"connection: {value.get('connection', 'UNKNOWN')}\n"
            f"manual: {value.get('manual_transition', 'UNKNOWN')} | ready: {value.get('manual_ready', False)} | "
            f"auto: {value.get('auto_state', 'UNKNOWN')} | nonzero output: {value.get('servo_nonzero_output_enabled', False)}\n"
            f"cmd: {value.get('command_linear_mm_s', 0)} mm/s, {value.get('command_angular_deg_s', 0)} deg/s | "
            f"joint feedback: {value.get('joint_feedback', 'UNKNOWN')} | TCP feedback: {value.get('tcp_feedback', 'UNKNOWN')} | "
            f"fault: {value.get('fault_state', value.get('fault', 'UNKNOWN'))}\n"
            f"joints (rad): {joints}\n"
            f"TCP ({value.get('tcp_pose_note', 'UNKNOWN')}): {tcp_pose}"
        )

    @staticmethod
    def _display_value(value: Any) -> Any:
        """Suppress feedback noise below operator-display precision."""
        if isinstance(value, float):
            return round(value, 4)
        if isinstance(value, dict):
            return {key: TeleopWindow._display_value(item) for key, item in value.items()}
        if isinstance(value, list):
            return [TeleopWindow._display_value(item) for item in value]
        return value

    def _show_status_decode_error(self, text: str) -> None:
        if not self._closing:
            self.status_label.setText(text)

    def changeEvent(self, event) -> None:
        if event.type() == event.ActivationChange:
            active = self.isActiveWindow()
            if self._was_active and not active and not self._closing:
                self.stop("window-deactivated")
            self._was_active = active
        super().changeEvent(event)

    def shutdown_ros(self) -> None:
        """Idempotently stop ROS before Qt destroys the window and its children."""
        if self._ros_shutdown_started:
            return
        self._ros_shutdown_started = True
        self.status_refresh.stop()
        self.status_diagnostics.stop()
        self.stop("gui-close")
        self._closing = True
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if self._spin_thread is not None and self._spin_thread is not current_thread():
            self._spin_thread.join(timeout=2.0)
        if not self._node_destroyed:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            self._node_destroyed = True
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def closeEvent(self, event) -> None:
        self.shutdown_ros()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    rclpy.init()
    node = Node("fr3_teleop_gui")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    window = TeleopWindow(node)
    window.show()
    # Construction, widgets, and Qt signal/slot wiring are complete at this point.
    window.start_ros(executor)
    # Test-only hook: exercise the same closeEvent path as a user closing the window.
    # It is inert unless explicitly set and never enables manual control or motion.
    auto_close_ms = int(os.environ.get("FR3_TELEOP_GUI_AUTOCLOSE_MS", "0"))
    if auto_close_ms > 0:
        QTimer.singleShot(auto_close_ms, window.close)
    result = app.exec_()
    window.shutdown_ros()  # closeEvent and this call share one guard.
    sys.exit(result)
