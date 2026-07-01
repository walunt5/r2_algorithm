from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional, Tuple

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from r2_nav_interfaces.action import NavigateToPose
from techx_r2_arm_interfaces.action import ExecuteAction
from techx_r2_chassis_interfaces.action import LiftControl

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .control_logic import (
    SpeedSettings,
    motion_from_key,
    percent_text,
    speed_scale_from_key,
    velocity_from_motion,
)


ARM_ACTIONS = {
    "gripper_arm_flat": ("夹爪机械臂平放", 1, 257, 8000),
    "gripper_arm_rotate_90": ("夹爪机械臂旋转 90°", 1, 258, 8000),
    "gripper_arm_tilt": ("夹爪机械臂斜放", 1, 259, 8000),
    "gripper_open": ("夹爪打开", 4, 1025, 3000),
    "gripper_close": ("夹爪闭合", 4, 1026, 3000),
}


class UiSignals(QObject):
    log = pyqtSignal(str)
    arm_feedback = pyqtSignal(str)
    arm_result = pyqtSignal(bool, str)
    lift_feedback = pyqtSignal(int, int, int, str)
    lift_result = pyqtSignal(bool, str)
    nav_feedback = pyqtSignal(str, float, float)
    nav_result = pyqtSignal(bool, str)
    server_status = pyqtSignal(bool, bool, bool)


class OperatorConsoleNode(Node):
    def __init__(self, signals: UiSignals):
        super().__init__("r2_operator_console")
        self.signals = signals

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("arm_action_name", "/r2_arm/execute_action")
        self.declare_parameter("lift_action_name", "/r2_chassis/lift_control")
        self.declare_parameter("nav_action_name", "/r2_navigate_to_pose")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("default_vx", 0.50)
        self.declare_parameter("default_vy", 0.50)
        self.declare_parameter("default_wz", 1.00)
        self.declare_parameter("max_vx", 2.50)
        self.declare_parameter("max_vy", 2.50)
        self.declare_parameter("max_wz", 1.20)
        self.declare_parameter("speed_step", 0.01)
        self.declare_parameter("arm_pose_timeout_ms", 8000)
        self.declare_parameter("gripper_timeout_ms", 3000)
        self.declare_parameter("lift_timeout_sec", 15.0)
        self.declare_parameter("nav_timeout_sec", 60.0)
        self.declare_parameter("server_check_period_ms", 1000)

        self.cmd_vel_topic = self.get_str("cmd_vel_topic")
        self.arm_action_name = self.get_str("arm_action_name")
        self.lift_action_name = self.get_str("lift_action_name")
        self.nav_action_name = self.get_str("nav_action_name")
        self.goals_file = self.resolve_goals_file(self.get_str("goals_file"))

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.arm_client = ActionClient(self, ExecuteAction, self.arm_action_name)
        self.lift_client = ActionClient(self, LiftControl, self.lift_action_name)
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.nav_goal_handle = None

    def get_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def get_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def resolve_goals_file(self, configured: str) -> str:
        configured = str(configured or "").strip()
        if configured:
            return os.path.abspath(os.path.expanduser(configured))
        try:
            share = get_package_share_directory("r2_nav_bringup")
            return os.path.join(share, "config", "r2_nav_goals.yaml")
        except PackageNotFoundError:
            return ""

    def publish_velocity(self, vx: float, vy: float, vz: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def publish_zero(self) -> None:
        self.publish_velocity(0.0, 0.0, 0.0, 0.0)

    def check_servers(self) -> Tuple[bool, bool, bool]:
        return (
            self.arm_client.server_is_ready(),
            self.lift_client.server_is_ready(),
            self.nav_client.server_is_ready(),
        )

    def send_arm_goal(self, label: str, target_id: int, action_id: int, timeout_ms: int) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=0.05):
            self.signals.arm_result.emit(False, f"机械臂 Action 不可用：{self.arm_action_name}")
            return False

        goal = ExecuteAction.Goal()
        goal.target_id = int(target_id)
        goal.action_id = int(action_id)
        goal.timeout_ms = int(timeout_ms)
        goal.param = 0
        goal.flags = 0

        self.signals.log.emit(
            f"发送机械臂动作：{label} target_id={target_id} action_id={action_id} timeout_ms={timeout_ms}"
        )

        send_future = self.arm_client.send_goal_async(
            goal,
            feedback_callback=self._on_arm_feedback,
        )
        send_future.add_done_callback(lambda future: self._on_arm_goal_response(future, label))
        return True

    def _on_arm_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.signals.arm_feedback.emit(f"state={fb.state} {fb.message}")

    def _on_arm_goal_response(self, future, label: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.signals.arm_result.emit(False, f"机械臂发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self.signals.arm_result.emit(False, f"机械臂目标被拒绝：{label}")
            return
        self.signals.arm_feedback.emit(f"机械臂目标已接受：{label}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_arm_result(f, label))

    def _on_arm_result(self, future, label: str) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            msg = (
                f"{label} 完成 success={result.success} "
                f"final_state={result.final_state} error_code={result.error_code} {result.message}"
            )
            self.signals.arm_result.emit(bool(result.success), msg)
        except Exception as exc:
            self.signals.arm_result.emit(False, f"机械臂结果异常：{exc}")

    def send_lift_goal(self, target_h_mm: int, mask: int, timeout_sec: float) -> bool:
        if not self.lift_client.wait_for_server(timeout_sec=0.05):
            self.signals.lift_result.emit(False, f"升降 Action 不可用：{self.lift_action_name}")
            return False

        goal = LiftControl.Goal()
        goal.target_h_mm = int(target_h_mm)
        goal.mask = int(mask)
        goal.timeout_sec = float(timeout_sec)

        self.signals.log.emit(
            f"发送升降：target_h_mm={target_h_mm} mask={mask} timeout_sec={timeout_sec:.1f}"
        )

        send_future = self.lift_client.send_goal_async(
            goal,
            feedback_callback=self._on_lift_feedback,
        )
        send_future.add_done_callback(self._on_lift_goal_response)
        return True

    def _on_lift_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.signals.lift_feedback.emit(
            int(fb.lift1_mm),
            int(fb.lift2_mm),
            int(fb.lift3_mm),
            f"state={fb.state} {fb.message}",
        )

    def _on_lift_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.signals.lift_result.emit(False, f"升降发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self.signals.lift_result.emit(False, "升降目标被拒绝")
            return
        self.signals.lift_feedback.emit(0, 0, 0, "升降目标已接受")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_lift_result)

    def _on_lift_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            msg = (
                f"升降完成 success={result.success} "
                f"final_state={result.final_state} error_code={result.error_code} {result.message}"
            )
            self.signals.lift_result.emit(bool(result.success), msg)
        except Exception as exc:
            self.signals.lift_result.emit(False, f"升降结果异常：{exc}")

    def send_nav_goal(self, goal_name: str, control_mode: str, timeout_sec: float) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.05):
            self.signals.nav_result.emit(False, f"导航 Action 不可用：{self.nav_action_name}")
            return False

        goal = NavigateToPose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id = "map"
        goal.goal_name = str(goal_name)
        goal.control_mode = str(control_mode)
        goal.timeout_sec = float(timeout_sec)

        self.signals.log.emit(
            f"发送导航：goal_name={goal_name} control_mode={control_mode} timeout_sec={timeout_sec:.1f}"
        )

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._on_nav_feedback,
        )
        send_future.add_done_callback(self._on_nav_goal_response)
        return True

    def _on_nav_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.signals.nav_feedback.emit(
            str(fb.state),
            float(fb.distance_to_goal),
            float(fb.yaw_error),
        )

    def _on_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.signals.nav_result.emit(False, f"导航发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self.signals.nav_result.emit(False, "导航目标被拒绝")
            return
        self.nav_goal_handle = goal_handle
        self.signals.nav_feedback.emit("目标已接受", 0.0, 0.0)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            self.nav_goal_handle = None
            self.signals.nav_result.emit(bool(result.success), result.message)
        except Exception as exc:
            self.nav_goal_handle = None
            self.signals.nav_result.emit(False, f"导航结果异常：{exc}")

    def cancel_nav_goal(self) -> bool:
        if self.nav_goal_handle is None:
            self.signals.log.emit("没有正在执行的导航目标")
            return False
        self.signals.log.emit("请求取消导航")
        future = self.nav_goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _: self.signals.log.emit("已发送导航取消请求"))
        return True


class HoldButton(QPushButton):
    def __init__(self, text: str, motion: Tuple[int, int, int, int], parent=None):
        super().__init__(text, parent)
        self.motion = motion
        self.setMinimumHeight(44)


class OperatorConsoleWindow(QWidget):
    def __init__(self, node: OperatorConsoleNode, signals: UiSignals):
        super().__init__()
        self.node = node
        self.signals = signals

        self.speeds = SpeedSettings(
            vx=node.get_float("default_vx"),
            vy=node.get_float("default_vy"),
            wz=node.get_float("default_wz"),
            linear_z=node.get_float("default_vx"),
            max_vx=node.get_float("max_vx"),
            max_vy=node.get_float("max_vy"),
            max_wz=node.get_float("max_wz"),
            step=node.get_float("speed_step"),
        )
        self.speeds.clamp()

        self.manual_active = False
        self.manual_motion = (0, 0, 0, 0)
        self.nav_active = False
        self.lift_active = False
        self.arm_busy = False

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_manual_velocity)
        period_ms = max(10, int(1000.0 / max(1.0, node.get_float("publish_rate_hz"))))
        self.publish_timer.start(period_ms)

        self.server_timer = QTimer(self)
        self.server_timer.timeout.connect(self.update_server_status)
        self.server_timer.start(max(200, node.get_int("server_check_period_ms")))

        self.setWindowTitle("R2 综合遥控与动作客户端")
        self.resize(1180, 760)
        self.setFocusPolicy(Qt.StrongFocus)

        self.build_ui()
        self.connect_signals()
        self.refresh_goals()
        self.update_speed_labels()
        self.update_interlocks()
        self.update_server_status()

    def build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.addLayout(self.build_status_bar())

        panels = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()

        left.addWidget(self.build_manual_group())
        left.addWidget(self.build_lift_group())
        right.addWidget(self.build_arm_group())
        right.addWidget(self.build_nav_group())

        panels.addLayout(left, 1)
        panels.addLayout(right, 1)
        root.addLayout(panels, 1)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(140)
        root.addWidget(self.log_box)

        self.setStyleSheet(
            """
            QWidget { font-size: 14px; }
            QGroupBox { font-weight: bold; margin-top: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { min-height: 34px; }
            QPushButton:disabled { color: #777; }
            QLabel#ServerOk { color: #137333; font-weight: bold; }
            QLabel#ServerBad { color: #b3261e; font-weight: bold; }
            """
        )

    def build_status_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        title = QLabel("R2 综合遥控与动作客户端")
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title, 1)

        self.arm_server_label = QLabel("机械臂: 未检查")
        self.lift_server_label = QLabel("升降: 未检查")
        self.nav_server_label = QLabel("导航: 未检查")
        layout.addWidget(self.arm_server_label)
        layout.addWidget(self.lift_server_label)
        layout.addWidget(self.nav_server_label)
        return layout

    def build_manual_group(self) -> QGroupBox:
        group = QGroupBox("手动全向移动 / teleop_twist_keyboard")
        layout = QVBoxLayout(group)

        self.manual_state_label = QLabel("状态：空闲")
        layout.addWidget(self.manual_state_label)

        speed_grid = QGridLayout()
        self.vx_spin = self.make_speed_spin(self.speeds.vx, self.speeds.max_vx)
        self.vy_spin = self.make_speed_spin(self.speeds.vy, self.speeds.max_vy)
        self.wz_spin = self.make_speed_spin(self.speeds.wz, self.speeds.max_wz)
        self.vx_label = QLabel()
        self.vy_label = QLabel()
        self.wz_label = QLabel()

        speed_grid.addWidget(QLabel("vx 前后 m/s"), 0, 0)
        speed_grid.addWidget(self.vx_spin, 0, 1)
        speed_grid.addWidget(self.vx_label, 0, 2)
        speed_grid.addWidget(QLabel("vy 横移 m/s"), 1, 0)
        speed_grid.addWidget(self.vy_spin, 1, 1)
        speed_grid.addWidget(self.vy_label, 1, 2)
        speed_grid.addWidget(QLabel("wz 旋转 rad/s"), 2, 0)
        speed_grid.addWidget(self.wz_spin, 2, 1)
        speed_grid.addWidget(self.wz_label, 2, 2)
        layout.addLayout(speed_grid)

        self.vx_spin.valueChanged.connect(lambda v: self.set_speed("vx", v))
        self.vy_spin.valueChanged.connect(lambda v: self.set_speed("vy", v))
        self.wz_spin.valueChanged.connect(lambda v: self.set_speed("wz", v))

        button_grid = QGridLayout()
        buttons = [
            ("↖ 前左", (1, 1, 0, 0), 0, 0),
            ("↑ 前", (1, 0, 0, 0), 0, 1),
            ("↗ 前右", (1, -1, 0, 0), 0, 2),
            ("← 左移", (0, 1, 0, 0), 1, 0),
            ("停止", (0, 0, 0, 0), 1, 1),
            ("右移 →", (0, -1, 0, 0), 1, 2),
            ("↙ 后左", (-1, 1, 0, 0), 2, 0),
            ("↓ 后", (-1, 0, 0, 0), 2, 1),
            ("↘ 后右", (-1, -1, 0, 0), 2, 2),
            ("⟲ 左旋", (0, 0, 0, 1), 3, 0),
            ("零速度", (0, 0, 0, 0), 3, 1),
            ("右旋 ⟳", (0, 0, 0, -1), 3, 2),
        ]
        self.manual_buttons = []
        for text, motion, row, col in buttons:
            btn = HoldButton(text, motion)
            btn.pressed.connect(lambda m=motion: self.start_manual(m))
            btn.released.connect(self.stop_manual)
            if motion == (0, 0, 0, 0):
                btn.clicked.connect(self.force_zero)
            self.manual_buttons.append(btn)
            button_grid.addWidget(btn, row, col)
        layout.addLayout(button_grid)

        help_text = QLabel(
            "键盘：u/i/o/j/k/l/m/,/.；Shift 大写为全向横移；"
            "q/z、w/x、e/c 调整速度百分比；t/b 发布 linear.z，当前底盘串口节点不使用该分量，也不映射为升降。"
        )
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        return group

    def build_arm_group(self) -> QGroupBox:
        group = QGroupBox("机械臂 / 夹爪动作")
        layout = QGridLayout(group)

        self.arm_buttons = []
        specs = [
            ("平放", "gripper_arm_flat", 0, 0),
            ("旋转 90°", "gripper_arm_rotate_90", 0, 1),
            ("斜放", "gripper_arm_tilt", 0, 2),
            ("夹爪打开", "gripper_open", 1, 0),
            ("夹爪闭合", "gripper_close", 1, 1),
        ]
        for text, key, row, col in specs:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, k=key: self.send_arm(k))
            self.arm_buttons.append(btn)
            layout.addWidget(btn, row, col)

        self.arm_status_label = QLabel("状态：空闲")
        layout.addWidget(self.arm_status_label, 2, 0, 1, 3)
        return group

    def build_lift_group(self) -> QGroupBox:
        group = QGroupBox("整体升降")
        layout = QGridLayout(group)

        self.lift_height_spin = QSpinBox()
        self.lift_height_spin.setRange(2, 66)
        self.lift_height_spin.setValue(30)
        self.lift_timeout_spin = QDoubleSpinBox()
        self.lift_timeout_spin.setRange(1.0, 60.0)
        self.lift_timeout_spin.setSingleStep(1.0)
        self.lift_timeout_spin.setValue(self.node.get_float("lift_timeout_sec"))
        self.lift_timeout_spin.setSuffix(" s")

        self.lift_start_btn = QPushButton("发送升降 mask=7")
        self.lift_start_btn.clicked.connect(self.send_lift)
        self.lift_status_label = QLabel("状态：空闲")
        self.lift_xyz_label = QLabel("lift1=- lift2=- lift3=-")

        layout.addWidget(QLabel("目标高度 mm"), 0, 0)
        layout.addWidget(self.lift_height_spin, 0, 1)
        layout.addWidget(QLabel("超时"), 1, 0)
        layout.addWidget(self.lift_timeout_spin, 1, 1)
        layout.addWidget(self.lift_start_btn, 2, 0, 1, 2)
        layout.addWidget(self.lift_xyz_label, 3, 0, 1, 2)
        layout.addWidget(self.lift_status_label, 4, 0, 1, 2)
        return group

    def build_nav_group(self) -> QGroupBox:
        group = QGroupBox("预设点导航")
        layout = QGridLayout(group)

        self.goals_file_label = QLineEdit(self.node.goals_file)
        self.goals_file_label.setReadOnly(True)
        self.goal_combo = QComboBox()
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItems(["x_then_y", "fixed_map"])
        self.nav_timeout_spin = QDoubleSpinBox()
        self.nav_timeout_spin.setRange(1.0, 300.0)
        self.nav_timeout_spin.setValue(self.node.get_float("nav_timeout_sec"))
        self.nav_timeout_spin.setSuffix(" s")
        self.refresh_goals_btn = QPushButton("刷新目标点")
        self.refresh_goals_btn.clicked.connect(self.refresh_goals)

        self.nav_start_btn = QPushButton("开始导航")
        self.nav_start_btn.clicked.connect(self.send_nav)
        self.nav_cancel_btn = QPushButton("取消导航")
        self.nav_cancel_btn.clicked.connect(self.cancel_nav)
        self.nav_status_label = QLabel("状态：空闲")
        self.nav_feedback_label = QLabel("distance=- yaw_error=-")

        layout.addWidget(QLabel("goals 文件"), 0, 0)
        layout.addWidget(self.goals_file_label, 0, 1, 1, 3)
        layout.addWidget(QLabel("目标点"), 1, 0)
        layout.addWidget(self.goal_combo, 1, 1)
        layout.addWidget(self.refresh_goals_btn, 1, 2)
        layout.addWidget(QLabel("模式"), 2, 0)
        layout.addWidget(self.control_mode_combo, 2, 1)
        layout.addWidget(QLabel("超时"), 2, 2)
        layout.addWidget(self.nav_timeout_spin, 2, 3)
        layout.addWidget(self.nav_start_btn, 3, 0, 1, 2)
        layout.addWidget(self.nav_cancel_btn, 3, 2, 1, 2)
        layout.addWidget(self.nav_feedback_label, 4, 0, 1, 4)
        layout.addWidget(self.nav_status_label, 5, 0, 1, 4)
        return group

    def make_speed_spin(self, value: float, maximum: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(0.0, maximum)
        spin.setSingleStep(self.speeds.step)
        spin.setValue(value)
        return spin

    def connect_signals(self) -> None:
        self.signals.log.connect(self.append_log)
        self.signals.arm_feedback.connect(self.on_arm_feedback)
        self.signals.arm_result.connect(self.on_arm_result)
        self.signals.lift_feedback.connect(self.on_lift_feedback)
        self.signals.lift_result.connect(self.on_lift_result)
        self.signals.nav_feedback.connect(self.on_nav_feedback)
        self.signals.nav_result.connect(self.on_nav_result)
        self.signals.server_status.connect(self.on_server_status)

    def set_speed(self, axis: str, value: float) -> None:
        if axis == "vx":
            self.speeds.vx = float(value)
        elif axis == "vy":
            self.speeds.vy = float(value)
        elif axis == "wz":
            self.speeds.wz = float(value)
        self.speeds.clamp()
        self.update_speed_labels()

    def update_speed_labels(self) -> None:
        self.vx_label.setText(percent_text(self.speeds.vx, self.speeds.max_vx))
        self.vy_label.setText(percent_text(self.speeds.vy, self.speeds.max_vy))
        self.wz_label.setText(percent_text(self.speeds.wz, self.speeds.max_wz))

    def sync_spin_values(self) -> None:
        for spin, value in [
            (self.vx_spin, self.speeds.vx),
            (self.vy_spin, self.speeds.vy),
            (self.wz_spin, self.speeds.wz),
        ]:
            old = spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(old)
        self.update_speed_labels()

    def start_manual(self, motion: Tuple[int, int, int, int]) -> None:
        if self.nav_active or self.lift_active:
            self.append_log("导航或升降进行中，禁止手动移动")
            self.node.publish_zero()
            return
        if motion == (0, 0, 0, 0):
            self.force_zero()
            return
        self.manual_active = True
        self.manual_motion = motion
        self.manual_state_label.setText(f"状态：手动移动 {motion}")
        self.publish_manual_velocity()
        self.update_interlocks()

    def stop_manual(self) -> None:
        if self.manual_active:
            self.manual_active = False
            self.manual_motion = (0, 0, 0, 0)
            self.node.publish_zero()
            self.manual_state_label.setText("状态：空闲，已发布零速度")
            self.update_interlocks()

    def force_zero(self) -> None:
        self.manual_active = False
        self.manual_motion = (0, 0, 0, 0)
        self.node.publish_zero()
        self.manual_state_label.setText("状态：已发布零速度")
        self.update_interlocks()

    def publish_manual_velocity(self) -> None:
        if not self.manual_active or self.nav_active or self.lift_active:
            return
        vx, vy, vz, wz = velocity_from_motion(self.manual_motion, self.speeds)
        self.node.publish_velocity(vx, vy, vz, wz)

    def keyPressEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        text = event.text()
        scale = speed_scale_from_key(text)
        if scale is not None:
            self.speeds = self.speeds.scaled(scale[0], scale[1])
            self.sync_spin_values()
            self.append_log(
                f"速度调整：vx={self.speeds.vx:.2f} vy={self.speeds.vy:.2f} wz={self.speeds.wz:.2f}"
            )
            return

        motion = motion_from_key(text)
        if motion is not None:
            if text == "k":
                self.force_zero()
            else:
                self.start_manual(motion)
            return

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat():
            return
        if motion_from_key(event.text()) is not None:
            self.stop_manual()
            return
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event) -> None:
        self.force_zero()
        super().focusOutEvent(event)

    def send_arm(self, action_key: str) -> None:
        if self.arm_busy:
            return
        label, target_id, action_id, default_timeout = ARM_ACTIONS[action_key]
        if action_key in ["gripper_open", "gripper_close"]:
            timeout_ms = self.node.get_int("gripper_timeout_ms")
        else:
            timeout_ms = self.node.get_int("arm_pose_timeout_ms")
        if timeout_ms <= 0:
            timeout_ms = default_timeout
        if self.node.send_arm_goal(label, target_id, action_id, timeout_ms):
            self.arm_busy = True
            self.arm_status_label.setText(f"状态：执行中 {label}")
            self.update_interlocks()

    def on_arm_feedback(self, text: str) -> None:
        self.arm_status_label.setText(f"状态：{text}")
        self.append_log(f"机械臂反馈：{text}")

    def on_arm_result(self, success: bool, text: str) -> None:
        self.arm_busy = False
        self.arm_status_label.setText(("成功：" if success else "失败：") + text)
        self.append_log(("机械臂成功：" if success else "机械臂失败：") + text)
        self.update_interlocks()

    def send_lift(self) -> None:
        if self.manual_active:
            self.force_zero()
        if self.nav_active:
            self.append_log("导航进行中，禁止启动升降")
            return
        if self.lift_active:
            return
        target = int(self.lift_height_spin.value())
        timeout = float(self.lift_timeout_spin.value())
        if self.node.send_lift_goal(target, 7, timeout):
            self.lift_active = True
            self.force_zero()
            self.lift_status_label.setText(f"状态：升降到 {target} mm")
            self.update_interlocks()

    def on_lift_feedback(self, lift1: int, lift2: int, lift3: int, text: str) -> None:
        self.lift_xyz_label.setText(f"lift1={lift1} lift2={lift2} lift3={lift3}")
        self.lift_status_label.setText(f"状态：{text}")
        self.append_log(f"升降反馈：{text}")

    def on_lift_result(self, success: bool, text: str) -> None:
        self.lift_active = False
        self.lift_status_label.setText(("成功：" if success else "失败：") + text)
        self.append_log(("升降成功：" if success else "升降失败：") + text)
        self.update_interlocks()

    def refresh_goals(self) -> None:
        self.goal_combo.clear()
        path = self.node.goals_file
        self.goals_file_label.setText(path)
        if not path:
            self.append_log("未找到 r2_nav_goals.yaml：r2_nav_bringup 可能未 build/source")
            return
        if not os.path.exists(path):
            self.append_log(f"goals 文件不存在：{path}")
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            goals = data.get("goals", {}) or {}
            names = sorted(str(name) for name in goals.keys())
            self.goal_combo.addItems(names)
            self.append_log(f"已加载 {len(names)} 个导航目标：{path}")
        except Exception as exc:
            self.append_log(f"读取 goals 失败：{exc}")

    def send_nav(self) -> None:
        if self.manual_active:
            self.force_zero()
        if self.lift_active:
            self.append_log("升降进行中，禁止启动导航")
            return
        if self.nav_active:
            return
        goal_name = self.goal_combo.currentText().strip()
        if not goal_name:
            self.append_log("请选择导航目标点")
            return
        control_mode = self.control_mode_combo.currentText()
        timeout = float(self.nav_timeout_spin.value())
        if self.node.send_nav_goal(goal_name, control_mode, timeout):
            self.nav_active = True
            self.force_zero()
            self.nav_status_label.setText(f"状态：导航中 {goal_name}")
            self.update_interlocks()

    def cancel_nav(self) -> None:
        if self.node.cancel_nav_goal():
            self.nav_status_label.setText("状态：正在取消导航")

    def on_nav_feedback(self, state: str, distance: float, yaw_error: float) -> None:
        self.nav_feedback_label.setText(
            f"state={state} distance={distance:.3f} m yaw_error={yaw_error:.3f} rad"
        )

    def on_nav_result(self, success: bool, text: str) -> None:
        self.nav_active = False
        self.node.publish_zero()
        self.nav_status_label.setText(("成功：" if success else "失败：") + text)
        self.append_log(("导航成功：" if success else "导航失败/取消：") + text)
        self.update_interlocks()

    def update_interlocks(self) -> None:
        manual_enabled = not self.nav_active and not self.lift_active
        lift_enabled = not self.manual_active and not self.nav_active and not self.lift_active
        nav_enabled = not self.manual_active and not self.lift_active and not self.nav_active

        for btn in self.manual_buttons:
            btn.setEnabled(manual_enabled)
        self.lift_start_btn.setEnabled(lift_enabled)
        self.nav_start_btn.setEnabled(nav_enabled)
        self.nav_cancel_btn.setEnabled(self.nav_active)

        for btn in self.arm_buttons:
            btn.setEnabled(not self.arm_busy)

    def update_server_status(self) -> None:
        arm_ok, lift_ok, nav_ok = self.node.check_servers()
        self.signals.server_status.emit(arm_ok, lift_ok, nav_ok)

    def on_server_status(self, arm_ok: bool, lift_ok: bool, nav_ok: bool) -> None:
        self.set_server_label(self.arm_server_label, "机械臂", arm_ok)
        self.set_server_label(self.lift_server_label, "升降", lift_ok)
        self.set_server_label(self.nav_server_label, "导航", nav_ok)

    def set_server_label(self, label: QLabel, name: str, ok: bool) -> None:
        label.setText(f"{name}: {'可用' if ok else '不可用'}")
        label.setObjectName("ServerOk" if ok else "ServerBad")
        label.style().unpolish(label)
        label.style().polish(label)

    def append_log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log_box.append(f"[{stamp}] {text}")

    def closeEvent(self, event) -> None:
        self.force_zero()
        if self.nav_active:
            self.node.cancel_nav_goal()
        event.accept()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    signals = UiSignals()
    node = OperatorConsoleNode(signals)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    window = OperatorConsoleWindow(node, signals)
    window.show()

    try:
        rc = app.exec_()
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
