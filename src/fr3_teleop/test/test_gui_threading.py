"""Regression tests for GUI ROS/Qt thread separation (no ROS graph required)."""

import json
import os
import threading
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication
from std_msgs.msg import String

from fr3_teleop.gui import TeleopWindow


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def error(self, _message):
        pass


class _Node:
    def __init__(self):
        self.publisher = _Publisher()
        self.callback = None
        self.destroyed = False

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, _type, _topic, callback, _depth):
        self.callback = callback
        return object()

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        self.destroyed = True


def _status(**overrides):
    value = {
        "selected_arm": "arm_a",
        "frame": "base",
        "owner": "none",
        "active_input_source": "none",
        "motion": "stopped",
        "backend": "moveit_servo",
        "connection": "ONLINE",
        "manual_transition": "AUTO",
        "manual_ready": False,
        "auto_state": "AUTO_IDLE",
        "servo_nonzero_output_enabled": False,
        "joint_feedback": "ONLINE",
        "tcp_feedback": "ONLINE",
        "fault_state": "",
        "joint_positions_rad": {"arm_a_j1": 0.0},
        "tcp_pose_note": "test",
        "tcp_pose": {"x": 0.0},
        "first_real_test_mode": True,
    }
    value.update(overrides)
    message = String()
    message.data = json.dumps(value)
    return message


class TestGuiThreading(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def _drain_events(self):
        for _ in range(20):
            self.app.processEvents()

    @staticmethod
    def _commands(window):
        return [json.loads(message.data) for message in window.node.publisher.messages]

    def test_manager_absent_starts_offline_and_motion_is_disabled(self):
        window = TeleopWindow(_Node())
        self.assertIn("OFFLINE / UNKNOWN", window.status_label.text())
        self.assertFalse(any(button.isEnabled() for button in window._motion_buttons.values()))
        window.close()

    def test_status_from_background_thread_updates_only_after_qt_event_delivery(self):
        window = TeleopWindow(_Node())
        thread = threading.Thread(target=window._status, args=(_status(manual_ready=True, servo_nonzero_output_enabled=True),))
        thread.start()
        thread.join()
        self._drain_events()
        window._apply_latest_status()
        self.assertIn("Arm: arm_a | frame: base", window.status_label.text())
        self.assertTrue(window._motion_buttons[("x", 1)].isEnabled())
        self.assertFalse(window._motion_buttons[("x", -1)].isEnabled())
        self.assertFalse(window._arm_buttons["arm_b"].isEnabled())
        window.close()

    def test_rapid_preexisting_manager_status_and_repeated_close(self):
        window = TeleopWindow(_Node())

        def publish_burst():
            for index in range(100):
                window._status(_status(selected_arm="arm_a" if index % 2 else "arm_b", frame="base"))

        thread = threading.Thread(target=publish_burst)
        thread.start()
        thread.join()
        self._drain_events()
        window._apply_latest_status()
        self.assertIn("frame: base", window.status_label.text())
        window.close()
        window.shutdown_ros()  # Must be idempotent after closeEvent.
        self.assertTrue(window.node.destroyed)

    def test_repeated_100hz_status_is_coalesced_and_never_publishes_commands(self):
        window = TeleopWindow(_Node())

        def publish_burst():
            for _ in range(1000):
                window._status(_status())

        thread = threading.Thread(target=publish_burst)
        thread.start()
        thread.join()
        window._apply_latest_status()
        self.assertEqual(self._commands(window), [])
        self.assertIn("Arm: arm_a | frame: base", window.status_label.text())
        self.assertEqual(window._status_apply_count, 1)
        for _ in range(100):
            window._apply_latest_status()
        self.assertEqual(window._status_apply_count, 1)
        window.close()

    def test_manager_selection_and_frame_sync_do_not_echo_commands(self):
        window = TeleopWindow(_Node())
        window._status(_status(selected_arm="arm_b", frame="tool"))
        window._apply_latest_status()
        self.assertTrue(window._arm_buttons["arm_b"].isChecked())
        self.assertEqual(window.frame.currentText(), "tool")
        self.assertEqual(self._commands(window), [])
        window.close()

    def test_user_requests_are_one_shot_while_status_sync_is_silent(self):
        window = TeleopWindow(_Node())
        window._arm_buttons["arm_b"].click()
        window.frame.activated[str].emit("base")
        window.linear.setValue(2.0)
        window.linear.editingFinished.emit()
        commands = self._commands(window)
        self.assertEqual(sum(command["type"] == "select_arm" for command in commands), 1)
        self.assertEqual(sum(command["type"] == "set_frame" for command in commands), 1)
        self.assertEqual(sum(command["type"] == "set_speed" for command in commands), 1)
        window._status(_status(selected_arm="arm_b", frame="base"))
        window._apply_latest_status()
        self.assertEqual(len(self._commands(window)), len(commands))
        window.close()
