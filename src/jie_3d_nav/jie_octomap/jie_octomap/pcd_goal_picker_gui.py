#!/usr/bin/env python3

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from vtk.util import numpy_support
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk


MAX_PREVIEW_POINTS = 300000
MAX_COLLISION_MARKER_POINTS = 50000


class PcdGoalPickerWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._workspace_root = self._find_workspace_root()
        self._cloud_points: np.ndarray | None = None
        self._preview_points: np.ndarray | None = None
        self._selected_xyz: tuple[float, float, float] | None = None
        self._pcd_actor: vtk.vtkActor | None = None
        self._selected_actor: vtk.vtkActor | None = None
        self._footprint_actor: vtk.vtkActor | None = None
        self._footprint_edge_actor: vtk.vtkActor | None = None
        self._collision_actor: vtk.vtkActor | None = None
        self._nearest_actor: vtk.vtkActor | None = None
        self._yaw_arrow_actor: vtk.vtkActor | None = None
        self._yaw_handle_actor: vtk.vtkActor | None = None
        self._yaw_dragging = False
        self._right_press_pos: tuple[int, int] | None = None
        self._camera_interactor_style = vtk.vtkInteractorStyleTrackballCamera()
        self._yaw_interactor_style = vtk.vtkInteractorStyleUser()

        self._renderer = vtk.vtkRenderer()
        self._init_ui()

    def _init_ui(self) -> None:
        self.setWindowTitle("PCD 离线导航点位选择工具")
        self.resize(1320, 820)

        root = QHBoxLayout()
        left_panel = QVBoxLayout()

        file_group = QGroupBox("PCD 地图")
        file_form = QFormLayout()
        pcd_row = QHBoxLayout()
        self.pcd_edit = QLineEdit()
        self.pcd_edit.setText(str(self._default_pcd_path()))
        choose_pcd_btn = QPushButton("选择")
        choose_pcd_btn.clicked.connect(self._choose_pcd)
        load_pcd_btn = QPushButton("读取")
        load_pcd_btn.clicked.connect(self._load_pcd)
        pcd_row.addWidget(self.pcd_edit, 1)
        pcd_row.addWidget(choose_pcd_btn)
        pcd_row.addWidget(load_pcd_btn)
        file_form.addRow("PCD 文件", pcd_row)

        self.preview_voxel_spin = QDoubleSpinBox()
        self.preview_voxel_spin.setDecimals(3)
        self.preview_voxel_spin.setRange(0.0, 1.0)
        self.preview_voxel_spin.setSingleStep(0.02)
        self.preview_voxel_spin.setValue(0.05)
        file_form.addRow("预览降采样(m)", self.preview_voxel_spin)
        file_group.setLayout(file_form)

        check_group = QGroupBox("机器人半径检查")
        check_form = QFormLayout()
        self.frame_edit = QLineEdit("map")
        self.frame_edit.textChanged.connect(self._refresh_result_text)
        check_form.addRow("坐标系", self.frame_edit)

        self.robot_radius_spin = QDoubleSpinBox()
        self.robot_radius_spin.setDecimals(3)
        self.robot_radius_spin.setRange(0.02, 2.0)
        self.robot_radius_spin.setSingleStep(0.01)
        self.robot_radius_spin.setValue(0.25)
        self.robot_radius_spin.valueChanged.connect(self._recheck_current_selection)
        check_form.addRow("机器人半径(m)", self.robot_radius_spin)

        self.safety_margin_spin = QDoubleSpinBox()
        self.safety_margin_spin.setDecimals(3)
        self.safety_margin_spin.setRange(0.0, 1.0)
        self.safety_margin_spin.setSingleStep(0.01)
        self.safety_margin_spin.setValue(0.020)
        self.safety_margin_spin.valueChanged.connect(self._recheck_current_selection)
        check_form.addRow("额外安全余量(m)", self.safety_margin_spin)

        self.min_height_spin = QDoubleSpinBox()
        self.min_height_spin.setDecimals(3)
        self.min_height_spin.setRange(-0.5, 3.0)
        self.min_height_spin.setSingleStep(0.02)
        self.min_height_spin.setValue(-0.10)
        self.min_height_spin.valueChanged.connect(self._recheck_current_selection)
        check_form.addRow("碰撞高度下限(m)", self.min_height_spin)

        self.max_height_spin = QDoubleSpinBox()
        self.max_height_spin.setDecimals(3)
        self.max_height_spin.setRange(-0.5, 5.0)
        self.max_height_spin.setSingleStep(0.05)
        self.max_height_spin.setValue(0.20)
        self.max_height_spin.valueChanged.connect(self._recheck_current_selection)
        check_form.addRow("碰撞高度上限(m)", self.max_height_spin)

        self.keyboard_step_spin = QDoubleSpinBox()
        self.keyboard_step_spin.setDecimals(3)
        self.keyboard_step_spin.setRange(0.001, 1.0)
        self.keyboard_step_spin.setSingleStep(0.01)
        self.keyboard_step_spin.setValue(0.05)
        check_form.addRow("键盘步长(m)", self.keyboard_step_spin)

        recheck_btn = QPushButton("重新检查当前点")
        recheck_btn.clicked.connect(self._recheck_current_selection)
        check_form.addRow("", recheck_btn)
        check_group.setLayout(check_form)

        result_group = QGroupBox("当前点位")
        result_layout = QVBoxLayout()
        self.result_label = QLabel("读取 PCD 后，在右侧点云上右键轻点。")
        self.result_label.setWordWrap(True)
        result_layout.addWidget(self.result_label)
        self.snippet_edit = QPlainTextEdit()
        self.snippet_edit.setReadOnly(True)
        self.snippet_edit.setMinimumHeight(128)
        result_layout.addWidget(self.snippet_edit)
        copy_row = QHBoxLayout()
        copy_coord_btn = QPushButton("复制坐标")
        copy_coord_btn.clicked.connect(self._copy_coordinates)
        copy_yaml_btn = QPushButton("复制 YAML")
        copy_yaml_btn.clicked.connect(self._copy_yaml)
        copy_row.addWidget(copy_coord_btn)
        copy_row.addWidget(copy_yaml_btn)
        result_layout.addLayout(copy_row)
        result_group.setLayout(result_layout)

        save_group = QGroupBox("保存到 r2_nav_goals.yaml")
        save_form = QFormLayout()
        yaml_row = QHBoxLayout()
        self.yaml_edit = QLineEdit(str(self._default_nav_goals_path()))
        choose_yaml_btn = QPushButton("选择")
        choose_yaml_btn.clicked.connect(self._choose_nav_yaml)
        reload_yaml_btn = QPushButton("加载")
        reload_yaml_btn.clicked.connect(self._load_goal_names)
        yaml_row.addWidget(self.yaml_edit, 1)
        yaml_row.addWidget(choose_yaml_btn)
        yaml_row.addWidget(reload_yaml_btn)
        save_form.addRow("YAML 文件", yaml_row)

        self.goal_combo = QComboBox()
        self.goal_combo.setEditable(True)
        self.goal_combo.currentTextChanged.connect(self._refresh_result_text)
        save_form.addRow("点位名", self.goal_combo)

        self.yaw_spin = QDoubleSpinBox()
        self.yaw_spin.setDecimals(6)
        self.yaw_spin.setRange(-math.pi, math.pi)
        self.yaw_spin.setSingleStep(0.05)
        self.yaw_spin.setValue(0.0)
        self.yaw_spin.valueChanged.connect(self._on_yaw_changed)
        save_form.addRow("yaw(rad)", self.yaw_spin)

        save_btn = QPushButton("保存当前点到 YAML")
        save_btn.clicked.connect(self._save_current_goal)
        save_form.addRow("", save_btn)
        save_group.setLayout(save_form)

        self.status_label = QLabel("等待读取 PCD。")
        self.status_label.setWordWrap(True)

        left_panel.addWidget(file_group)
        left_panel.addWidget(check_group)
        left_panel.addWidget(result_group, 1)
        left_panel.addWidget(save_group)
        left_panel.addWidget(self.status_label)
        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(430)

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.setFocusPolicy(Qt.StrongFocus)
        self.vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        self._setup_renderer()
        self._initialize_interactor()

        root.addWidget(left_widget)
        root.addWidget(self.vtk_widget, 1)
        self.setLayout(root)
        QApplication.instance().installEventFilter(self)
        self._load_goal_names(show_warning=False)

    def closeEvent(self, event) -> None:
        QApplication.instance().removeEventFilter(self)
        super().closeEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self._handle_keyboard_nudge(event):
            return True
        if self._event_targets_vtk_widget(obj) and self._handle_vtk_mouse_event(event):
            return True
        return super().eventFilter(obj, event)

    def _event_targets_vtk_widget(self, obj) -> bool:
        current = obj
        while current is not None:
            if current is self.vtk_widget:
                return True
            current = current.parent()
        return False

    def _setup_renderer(self) -> None:
        self._renderer.SetBackground(0.04, 0.07, 0.09)
        self._renderer.GradientBackgroundOn()
        self._renderer.SetBackground2(0.12, 0.16, 0.19)

        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5, 1.5, 1.5)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        self._renderer.AddActor(axes)
        self._renderer.AddActor(self._make_ground_grid(24.0, 1.0))

    def _initialize_interactor(self) -> None:
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(self._camera_interactor_style)
        interactor.AddObserver("LeftButtonPressEvent", self._on_left_button_press, 1.0)
        interactor.AddObserver("LeftButtonReleaseEvent", self._on_left_button_release, 1.0)
        interactor.AddObserver("MouseMoveEvent", self._on_mouse_move, 1.0)
        interactor.Initialize()

    def _find_workspace_root(self) -> Path:
        candidates: list[Path] = []
        env_root = os.environ.get("R2_ALGORITHM_ROOT")
        if env_root:
            candidates.append(Path(env_root).expanduser())

        cwd = Path.cwd().resolve()
        candidates.extend([cwd, *cwd.parents])
        candidates.append(Path.home() / "techx_R2_algorithm" / "r2_algorithm")
        candidates.append(Path("/home/xie/techx_R2_algorithm/r2_algorithm"))

        for candidate in candidates:
            marker = candidate / "src" / "r2_nav_bringup" / "config" / "r2_nav_goals.yaml"
            if marker.exists():
                return candidate
        return cwd

    def _default_pcd_path(self) -> Path:
        for rel_path in (
            "chassis_maps/red/Relocation Map.pcd",
            "chassis_maps/blue/Relocation Map.pcd",
            "chassis_maps/test/Relocation Map2.pcd",
        ):
            path = self._workspace_root / rel_path
            if path.exists():
                return path
        maps_dir = self._workspace_root / "chassis_maps"
        if maps_dir.exists():
            return maps_dir
        return Path.home()

    def _default_nav_goals_path(self) -> Path:
        path = self._workspace_root / "src" / "r2_nav_bringup" / "config" / "r2_nav_goals.yaml"
        if path.exists():
            return path
        return self._workspace_root / "r2_nav_goals.yaml"

    def _choose_pcd(self) -> None:
        current = Path(self.pcd_edit.text().strip()).expanduser()
        start_dir = current.parent if current.is_file() else current
        if not start_dir.exists():
            start_dir = self._workspace_root / "chassis_maps"
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "选择 PCD 文件",
            str(start_dir),
            "Point Cloud Files (*.pcd);;All Files (*)",
        )
        if selected:
            self.pcd_edit.setText(selected)
            self._load_pcd()

    def _choose_nav_yaml(self) -> None:
        current = Path(self.yaml_edit.text().strip()).expanduser()
        start_dir = current.parent if current.parent.exists() else self._workspace_root
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "选择 r2_nav_goals.yaml",
            str(start_dir),
            "YAML Files (*.yaml *.yml);;All Files (*)",
        )
        if selected:
            self.yaml_edit.setText(selected)
            self._load_goal_names()

    def _load_pcd(self) -> None:
        pcd_path = Path(self.pcd_edit.text().strip()).expanduser()
        if not pcd_path.exists() or not pcd_path.is_file():
            QMessageBox.warning(self, "PCD 点位选择", f"PCD 文件不存在：{pcd_path}")
            return

        self.status_label.setText(f"正在读取 PCD：{pcd_path}")
        QApplication.processEvents()

        try:
            cloud = o3d.io.read_point_cloud(str(pcd_path))
        except Exception as exc:
            QMessageBox.critical(self, "PCD 点位选择", f"读取 PCD 失败：{exc}")
            self.status_label.setText("读取 PCD 失败。")
            return

        if not cloud.has_points():
            QMessageBox.warning(self, "PCD 点位选择", "PCD 文件里没有点。")
            self.status_label.setText("PCD 文件为空。")
            return

        self._cloud_points = np.asarray(cloud.points, dtype=np.float32)
        preview_cloud = cloud
        voxel_size = float(self.preview_voxel_spin.value())
        if voxel_size > 0.0:
            preview_cloud = cloud.voxel_down_sample(voxel_size)

        preview_points = np.asarray(preview_cloud.points, dtype=np.float32)
        if preview_points.shape[0] > MAX_PREVIEW_POINTS:
            step = int(math.ceil(preview_points.shape[0] / MAX_PREVIEW_POINTS))
            preview_points = preview_points[::step]
        self._preview_points = np.ascontiguousarray(preview_points, dtype=np.float32)

        self._selected_xyz = None
        self._yaw_dragging = False
        self._right_press_pos = None
        self._clear_selection_actors()
        self._show_point_cloud()
        self.result_label.setText("已读取 PCD。右键轻点点云选择候选导航点。")
        self.snippet_edit.clear()
        self.vtk_widget.setFocus()
        self.status_label.setText(
            f"已读取 {self._cloud_points.shape[0]} 个原始点，显示 {self._preview_points.shape[0]} 个预览点。"
        )

    def _show_point_cloud(self) -> None:
        if self._pcd_actor is not None:
            self._renderer.RemoveActor(self._pcd_actor)
            self._pcd_actor = None

        if self._preview_points is None or self._preview_points.size == 0:
            self.vtk_widget.GetRenderWindow().Render()
            return

        self._pcd_actor = self._build_point_cloud_actor(self._preview_points)
        self._renderer.AddActor(self._pcd_actor)
        self._renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()

    def _on_left_button_press(self, obj, _event) -> None:
        if self._preview_points is None or self._preview_points.size == 0:
            self.status_label.setText("请先读取 PCD。")
            return

        click_x, click_y = obj.GetEventPosition()
        self._try_start_yaw_drag(obj, click_x, click_y)

    def _handle_vtk_mouse_event(self, event) -> bool:
        if event.type() not in (QEvent.MouseButtonPress, QEvent.MouseButtonRelease):
            return False
        if event.button() != Qt.RightButton:
            return False

        if event.type() == QEvent.MouseButtonPress:
            self._on_right_button_press(event)
            return True

        self._on_right_button_release(event)
        return True

    def _on_right_button_press(self, event) -> None:
        if self._preview_points is None or self._preview_points.size == 0:
            self._right_press_pos = None
            return
        click_x, click_y = self._qt_event_pos_in_vtk_widget(event)
        self._right_press_pos = (int(click_x), int(click_y))

    def _on_right_button_release(self, event) -> None:
        if self._preview_points is None or self._preview_points.size == 0:
            self._right_press_pos = None
            return
        if self._right_press_pos is None:
            return

        click_x, click_y = self._qt_event_pos_in_vtk_widget(event)
        press_x, press_y = self._right_press_pos
        self._right_press_pos = None
        move_sq = (int(click_x) - press_x) ** 2 + (int(click_y) - press_y) ** 2
        if move_sq > 36:
            return

        display_x, display_y = self._qt_to_vtk_display_pos(click_x, click_y)
        self._mark_point_from_display(display_x, display_y)

    def _qt_to_vtk_display_pos(self, x: int, y: int) -> tuple[int, int]:
        return int(x), max(0, int(self.vtk_widget.height()) - int(y) - 1)

    def _qt_event_pos_in_vtk_widget(self, event) -> tuple[int, int]:
        pos = self.vtk_widget.mapFromGlobal(event.globalPos())
        return int(pos.x()), int(pos.y())

    def _mark_point_from_display(self, click_x: int, click_y: int) -> None:
        picker = vtk.vtkPointPicker()
        picker.SetTolerance(0.015)
        if self._pcd_actor is not None:
            picker.PickFromListOn()
            picker.AddPickList(self._pcd_actor)

        if picker.Pick(click_x, click_y, 0, self._renderer) == 0:
            self.status_label.setText("没有选中点云点。可以放大后再点。")
            return

        point_id = picker.GetPointId()
        if point_id < 0 or point_id >= self._preview_points.shape[0]:
            picked = picker.GetPickPosition()
            candidate = np.asarray(picked, dtype=np.float32)
        else:
            candidate = self._preview_points[int(point_id)]

        selected = self._nearest_cloud_point(candidate)
        self._selected_xyz = (float(selected[0]), float(selected[1]), float(selected[2]))
        self._evaluate_and_show_selection()
        self.vtk_widget.setFocus()
        self.status_label.setText(
            "点位已选择。右键轻点重新标点，方向键/WASD 微调 XY，Q/E 微调 Z，拖动黄色端点调整 yaw。"
        )

    def _on_left_button_release(self, obj, _event) -> None:
        if not self._yaw_dragging:
            return
        self._yaw_dragging = False
        obj.SetInteractorStyle(self._camera_interactor_style)
        self.status_label.setText(f"yaw 已设置为 {float(self.yaw_spin.value()):.6f} rad。")

    def _on_mouse_move(self, obj, _event) -> None:
        if not self._yaw_dragging:
            return
        click_x, click_y = obj.GetEventPosition()
        self._update_yaw_from_display(click_x, click_y)

    def _try_start_yaw_drag(self, obj, click_x: int, click_y: int) -> bool:
        if self._selected_xyz is None:
            return False
        if self._yaw_arrow_actor is None and self._yaw_handle_actor is None:
            return False

        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        pickable_actors = [
            actor
            for actor in (self._yaw_handle_actor, self._yaw_arrow_actor)
            if actor is not None
        ]
        for actor in pickable_actors:
            picker.AddPickList(actor)
        if picker.Pick(click_x, click_y, 0, self._renderer) == 0:
            return False

        picked_prop = picker.GetViewProp()
        if picked_prop not in pickable_actors:
            return False

        self._yaw_dragging = True
        obj.SetInteractorStyle(self._yaw_interactor_style)
        self._update_yaw_from_display(click_x, click_y)
        return True

    def _update_yaw_from_display(self, click_x: int, click_y: int) -> None:
        if self._selected_xyz is None:
            return

        plane_point = self._display_to_selected_plane(click_x, click_y)
        if plane_point is None:
            return

        sx, sy, _sz = self._selected_xyz
        dx = float(plane_point[0] - sx)
        dy = float(plane_point[1] - sy)
        if math.hypot(dx, dy) < 0.03:
            return

        self._set_yaw(math.atan2(dy, dx))

    def _display_to_selected_plane(self, click_x: int, click_y: int) -> np.ndarray | None:
        if self._selected_xyz is None:
            return None

        z_plane = float(self._selected_xyz[2])
        self._renderer.SetDisplayPoint(float(click_x), float(click_y), 0.0)
        self._renderer.DisplayToWorld()
        near = np.asarray(self._renderer.GetWorldPoint(), dtype=np.float64)
        self._renderer.SetDisplayPoint(float(click_x), float(click_y), 1.0)
        self._renderer.DisplayToWorld()
        far = np.asarray(self._renderer.GetWorldPoint(), dtype=np.float64)

        if abs(float(near[3])) < 1e-9 or abs(float(far[3])) < 1e-9:
            return None
        near = near[:3] / near[3]
        far = far[:3] / far[3]
        dz = float(far[2] - near[2])
        if abs(dz) < 1e-9:
            return None
        t = (z_plane - float(near[2])) / dz
        return near + t * (far - near)

    def _set_yaw(self, yaw: float) -> None:
        normalized = math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))
        self.yaw_spin.blockSignals(True)
        self.yaw_spin.setValue(normalized)
        self.yaw_spin.blockSignals(False)
        self._update_yaw_actor()
        self._refresh_result_text()

    def _on_yaw_changed(self, _value: float) -> None:
        self._update_yaw_actor()
        self._refresh_result_text()

    def _handle_keyboard_nudge(self, event) -> bool:
        if self._selected_xyz is None or self._yaw_dragging:
            return False
        if self._focus_is_text_editor():
            return False

        key = event.key()
        dx = dy = dz = 0.0
        if key in (Qt.Key_Up, Qt.Key_W):
            dx = 1.0
        elif key in (Qt.Key_Down, Qt.Key_S):
            dx = -1.0
        elif key in (Qt.Key_Left, Qt.Key_A):
            dy = 1.0
        elif key in (Qt.Key_Right, Qt.Key_D):
            dy = -1.0
        elif key == Qt.Key_Q:
            dz = -1.0
        elif key == Qt.Key_E:
            dz = 1.0
        else:
            return False

        step = float(self.keyboard_step_spin.value())
        modifiers = event.modifiers()
        if modifiers & Qt.ShiftModifier:
            step *= 5.0
        if modifiers & Qt.AltModifier:
            step *= 0.2

        self._nudge_selected_point(dx * step, dy * step, dz * step)
        return True

    def _focus_is_text_editor(self) -> bool:
        focus_widget = QApplication.focusWidget()
        return isinstance(
            focus_widget,
            (QLineEdit, QPlainTextEdit, QAbstractSpinBox, QComboBox),
        )

    def _nudge_selected_point(self, dx: float, dy: float, dz: float) -> None:
        if self._selected_xyz is None:
            return

        x, y, z = self._selected_xyz
        self._selected_xyz = (x + dx, y + dy, z + dz)
        self._evaluate_and_show_selection()
        nx, ny, nz = self._selected_xyz
        self.vtk_widget.setFocus()
        self.status_label.setText(
            f"键盘已移动点位：x={nx:.6f}, y={ny:.6f}, z={nz:.6f}。"
        )

    def _nearest_cloud_point(self, candidate: np.ndarray) -> np.ndarray:
        if self._cloud_points is None or self._cloud_points.size == 0:
            return candidate
        diff = self._cloud_points - candidate.reshape(1, 3)
        distances = np.einsum("ij,ij->i", diff, diff)
        return self._cloud_points[int(np.argmin(distances))]

    def _recheck_current_selection(self, *_args) -> None:
        if self._selected_xyz is not None:
            self._evaluate_and_show_selection()

    def _evaluate_and_show_selection(self) -> None:
        if self._selected_xyz is None or self._cloud_points is None:
            return

        selected = np.asarray(self._selected_xyz, dtype=np.float32)
        result = self._check_collision(selected)
        self._update_selection_actors(selected, result)
        self._update_result_text(selected, result)

    def _check_collision(self, selected: np.ndarray) -> dict:
        assert self._cloud_points is not None

        robot_radius = float(self.robot_radius_spin.value())
        effective_radius = robot_radius + float(self.safety_margin_spin.value())
        min_height = float(self.min_height_spin.value())
        max_height = float(self.max_height_spin.value())
        if max_height < min_height:
            min_height, max_height = max_height, min_height

        points = self._cloud_points
        dx = points[:, 0] - selected[0]
        dy = points[:, 1] - selected[1]
        xy_distance_sq = dx * dx + dy * dy
        z_min = selected[2] + min_height
        z_max = selected[2] + max_height
        height_mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        collision_mask = height_mask & (xy_distance_sq <= effective_radius * effective_radius)
        collision_points = points[collision_mask]

        nearest_point = None
        nearest_distance = math.inf
        if np.any(height_mask):
            height_indices = np.where(height_mask)[0]
            nearest_index_in_height = int(np.argmin(xy_distance_sq[height_indices]))
            nearest_index = int(height_indices[nearest_index_in_height])
            nearest_point = points[nearest_index]
            nearest_distance = math.sqrt(float(xy_distance_sq[nearest_index]))

        physical_clearance = nearest_distance - robot_radius
        effective_clearance = nearest_distance - effective_radius
        return {
            "safe": collision_points.shape[0] == 0,
            "collision_points": collision_points,
            "nearest_point": nearest_point,
            "nearest_distance": nearest_distance,
            "physical_clearance": physical_clearance,
            "effective_clearance": effective_clearance,
            "effective_radius": effective_radius,
            "z_min": z_min,
            "z_max": z_max,
        }

    def _update_result_text(self, selected: np.ndarray, result: dict) -> None:
        x, y, z = (float(selected[0]), float(selected[1]), float(selected[2]))
        goal_name = self._goal_name()
        status = "安全" if result["safe"] else "不安全"
        nearest_distance = result["nearest_distance"]
        if math.isfinite(nearest_distance):
            nearest_text = f"{nearest_distance:.3f} m"
            clearance_text = f"{result['physical_clearance']:.3f} m"
            margin_text = f"{result['effective_clearance']:.3f} m"
        else:
            nearest_text = "无"
            clearance_text = "无穷"
            margin_text = "无穷"

        self.result_label.setText(
            f"{status} | frame={self.frame_edit.text().strip() or 'map'} | "
            f"x={x:.6f}, y={y:.6f}, z={z:.6f}\n"
            f"检查半径={result['effective_radius']:.3f} m，"
            f"高度=[{result['z_min']:.3f}, {result['z_max']:.3f}]，"
            f"最近障碍水平距离={nearest_text}，"
            f"实体净空={clearance_text}，含余量净空={margin_text}"
        )
        self.snippet_edit.setPlainText(self._make_yaml_snippet(goal_name, x, y, z))
        self.status_label.setText(
            "当前点安全，可以复制或保存。"
            if result["safe"]
            else f"当前点半径内有 {result['collision_points'].shape[0]} 个碰撞高度点，建议换点或减小半径。"
        )

    def _refresh_result_text(self, *_args) -> None:
        if self._selected_xyz is None:
            return
        selected = np.asarray(self._selected_xyz, dtype=np.float32)
        result = self._check_collision(selected)
        self._update_result_text(selected, result)

    def _goal_name(self) -> str:
        text = self.goal_combo.currentText().strip()
        return text or "SELECTED_GOAL"

    def _make_yaml_snippet(self, goal_name: str, x: float, y: float, z: float) -> str:
        frame_id = self.frame_edit.text().strip() or "map"
        yaw = float(self.yaw_spin.value())
        return (
            f"{goal_name}:\n"
            f"  frame_id: {frame_id}\n"
            f"  x: {x:.6f}\n"
            f"  y: {y:.6f}\n"
            f"  z: {z:.6f}\n"
            f"  yaw: {yaw:.6f}\n"
        )

    def _copy_coordinates(self) -> None:
        if self._selected_xyz is None:
            self.status_label.setText("还没有选择点位。")
            return
        x, y, z = self._selected_xyz
        text = f"{x:.6f}, {y:.6f}, {z:.6f}"
        QApplication.clipboard().setText(text)
        self.status_label.setText(f"已复制坐标：{text}")

    def _copy_yaml(self) -> None:
        text = self.snippet_edit.toPlainText().strip()
        if not text:
            self.status_label.setText("还没有可复制的 YAML。")
            return
        QApplication.clipboard().setText(text)
        self.status_label.setText("已复制 YAML 片段。")

    def _load_goal_names(self, show_warning: bool = True) -> None:
        yaml_path = Path(self.yaml_edit.text().strip()).expanduser()
        current_text = self.goal_combo.currentText().strip()
        self.goal_combo.blockSignals(True)
        self.goal_combo.clear()
        self.goal_combo.blockSignals(False)

        if not yaml_path.exists():
            if show_warning:
                QMessageBox.warning(self, "点位 YAML", f"YAML 文件不存在：{yaml_path}")
            return

        try:
            with yaml_path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            if show_warning:
                QMessageBox.critical(self, "点位 YAML", f"读取 YAML 失败：{exc}")
            return

        goals = data.get("goals", {}) if isinstance(data, dict) else {}
        if isinstance(goals, dict):
            self.goal_combo.addItems(list(goals.keys()))
        if current_text:
            self.goal_combo.setCurrentText(current_text)
        self._refresh_result_text()

    def _save_current_goal(self) -> None:
        if self._selected_xyz is None:
            QMessageBox.warning(self, "保存点位", "请先在 PCD 点云上选择一个点。")
            return

        goal_name = self._goal_name()
        if not goal_name:
            QMessageBox.warning(self, "保存点位", "点位名不能为空。")
            return

        selected = np.asarray(self._selected_xyz, dtype=np.float32)
        result = self._check_collision(selected)
        if not result["safe"]:
            reply = QMessageBox.question(
                self,
                "保存点位",
                f"当前点半径内有 {result['collision_points'].shape[0]} 个碰撞高度点，仍然保存吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        yaml_path = Path(self.yaml_edit.text().strip()).expanduser()
        if not yaml_path.parent.exists():
            QMessageBox.warning(self, "保存点位", f"目录不存在：{yaml_path.parent}")
            return

        data = {"goals": {}}
        if yaml_path.exists():
            try:
                with yaml_path.open("r", encoding="utf-8") as stream:
                    loaded = yaml.safe_load(stream) or {}
            except Exception as exc:
                QMessageBox.critical(self, "保存点位", f"读取 YAML 失败：{exc}")
                return
            if not isinstance(loaded, dict):
                QMessageBox.critical(self, "保存点位", "YAML 顶层不是字典，已取消保存。")
                return
            data = loaded

        goals = data.setdefault("goals", {})
        if not isinstance(goals, dict):
            QMessageBox.critical(self, "保存点位", "YAML 中的 goals 不是字典，已取消保存。")
            return

        x, y, z = self._selected_xyz
        goals[goal_name] = {
            "frame_id": self.frame_edit.text().strip() or "map",
            "x": round(float(x), 6),
            "y": round(float(y), 6),
            "z": round(float(z), 6),
            "yaw": round(float(self.yaw_spin.value()), 6),
        }

        try:
            with yaml_path.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        except Exception as exc:
            QMessageBox.critical(self, "保存点位", f"写入 YAML 失败：{exc}")
            return

        self.status_label.setText(f"已保存 {goal_name} 到 {yaml_path}")
        self._load_goal_names(show_warning=False)

    def _update_selection_actors(self, selected: np.ndarray, result: dict) -> None:
        self._clear_selection_actors()
        color = (0.10, 0.85, 0.25) if result["safe"] else (0.95, 0.10, 0.08)

        self._selected_actor = self._build_sphere_actor(
            selected,
            max(0.05, float(self.robot_radius_spin.value()) * 0.16),
            color,
            1.0,
        )
        self._renderer.AddActor(self._selected_actor)

        radius = float(result["effective_radius"])
        z = float(selected[2]) + 0.025
        self._footprint_actor, self._footprint_edge_actor = self._build_footprint_actors(
            (float(selected[0]), float(selected[1]), z),
            radius,
            color,
        )
        self._renderer.AddActor(self._footprint_actor)
        self._renderer.AddActor(self._footprint_edge_actor)

        collision_points = result["collision_points"]
        if collision_points.size > 0:
            if collision_points.shape[0] > MAX_COLLISION_MARKER_POINTS:
                step = int(math.ceil(collision_points.shape[0] / MAX_COLLISION_MARKER_POINTS))
                collision_points = collision_points[::step]
            self._collision_actor = self._build_plain_point_actor(
                collision_points,
                (1.0, 0.02, 0.02),
                6.0,
            )
            self._renderer.AddActor(self._collision_actor)

        nearest_point = result["nearest_point"]
        if nearest_point is not None:
            self._nearest_actor = self._build_sphere_actor(
                nearest_point,
                0.05,
                (0.72, 0.20, 1.0),
                1.0,
            )
            self._renderer.AddActor(self._nearest_actor)

        self._update_yaw_actor(render=False)
        self.vtk_widget.GetRenderWindow().Render()

    def _clear_selection_actors(self) -> None:
        for actor in (
            self._selected_actor,
            self._footprint_actor,
            self._footprint_edge_actor,
            self._collision_actor,
            self._nearest_actor,
            self._yaw_arrow_actor,
            self._yaw_handle_actor,
        ):
            if actor is not None:
                self._renderer.RemoveActor(actor)
        self._selected_actor = None
        self._footprint_actor = None
        self._footprint_edge_actor = None
        self._collision_actor = None
        self._nearest_actor = None
        self._yaw_arrow_actor = None
        self._yaw_handle_actor = None
        if hasattr(self, "vtk_widget"):
            self.vtk_widget.GetRenderWindow().Render()

    def _update_yaw_actor(self, render: bool = True) -> None:
        for actor in (self._yaw_arrow_actor, self._yaw_handle_actor):
            if actor is not None:
                self._renderer.RemoveActor(actor)
        self._yaw_arrow_actor = None
        self._yaw_handle_actor = None

        if self._selected_xyz is None:
            if render:
                self.vtk_widget.GetRenderWindow().Render()
            return

        center = np.asarray(self._selected_xyz, dtype=np.float32)
        yaw = float(self.yaw_spin.value())
        radius = float(self.robot_radius_spin.value()) + float(self.safety_margin_spin.value())
        length = max(0.55, radius * 1.7)
        z = float(center[2]) + 0.14
        start = np.array([center[0], center[1], z], dtype=np.float32)
        end = np.array(
            [
                center[0] + math.cos(yaw) * length,
                center[1] + math.sin(yaw) * length,
                z,
            ],
            dtype=np.float32,
        )

        self._yaw_arrow_actor = self._build_yaw_arrow_actor(start, end)
        self._yaw_handle_actor = self._build_sphere_actor(
            end,
            max(0.07, radius * 0.16),
            (1.0, 0.78, 0.05),
            1.0,
        )
        self._renderer.AddActor(self._yaw_arrow_actor)
        self._renderer.AddActor(self._yaw_handle_actor)
        if render:
            self.vtk_widget.GetRenderWindow().Render()

    def _build_point_cloud_actor(self, points: np.ndarray) -> vtk.vtkActor:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.SetVerts(self._make_vertex_cells(points.shape[0]))

        colors = self._height_colors(points[:, 2])
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("height_color")
        polydata.GetPointData().SetScalars(vtk_colors)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)
        mapper.SetScalarModeToUsePointData()
        mapper.ScalarVisibilityOn()

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetPointSize(2.0)
        actor.GetProperty().RenderPointsAsSpheresOn()
        return actor

    def _build_plain_point_actor(
        self, points: np.ndarray, color: tuple[float, float, float], point_size: float
    ) -> vtk.vtkActor:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)
        polydata.SetVerts(self._make_vertex_cells(points.shape[0]))

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetPointSize(point_size)
        actor.GetProperty().RenderPointsAsSpheresOn()
        return actor

    def _build_sphere_actor(
        self,
        center: np.ndarray,
        radius: float,
        color: tuple[float, float, float],
        opacity: float,
    ) -> vtk.vtkActor:
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(float(center[0]), float(center[1]), float(center[2]))
        sphere.SetRadius(float(radius))
        sphere.SetThetaResolution(24)
        sphere.SetPhiResolution(24)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(opacity)
        return actor

    def _build_footprint_actors(
        self,
        center: tuple[float, float, float],
        radius: float,
        color: tuple[float, float, float],
    ) -> tuple[vtk.vtkActor, vtk.vtkActor]:
        disk = vtk.vtkRegularPolygonSource()
        disk.SetNumberOfSides(96)
        disk.SetCenter(center)
        disk.SetRadius(float(radius))
        disk.SetNormal(0.0, 0.0, 1.0)
        disk.GeneratePolygonOn()

        disk_mapper = vtk.vtkPolyDataMapper()
        disk_mapper.SetInputConnection(disk.GetOutputPort())
        disk_actor = vtk.vtkActor()
        disk_actor.SetMapper(disk_mapper)
        disk_actor.GetProperty().SetColor(*color)
        disk_actor.GetProperty().SetOpacity(0.22)

        edge = vtk.vtkRegularPolygonSource()
        edge.SetNumberOfSides(96)
        edge.SetCenter(center)
        edge.SetRadius(float(radius))
        edge.SetNormal(0.0, 0.0, 1.0)
        edge.GeneratePolygonOff()
        edge.GeneratePolylineOn()

        edge_mapper = vtk.vtkPolyDataMapper()
        edge_mapper.SetInputConnection(edge.GetOutputPort())
        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_mapper)
        edge_actor.GetProperty().SetColor(*color)
        edge_actor.GetProperty().SetLineWidth(3.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return disk_actor, edge_actor

    def _build_yaw_arrow_actor(self, start: np.ndarray, end: np.ndarray) -> vtk.vtkActor:
        line = vtk.vtkLineSource()
        line.SetPoint1(float(start[0]), float(start[1]), float(start[2]))
        line.SetPoint2(float(end[0]), float(end[1]), float(end[2]))

        tube = vtk.vtkTubeFilter()
        tube.SetInputConnection(line.GetOutputPort())
        tube.SetRadius(0.035)
        tube.SetNumberOfSides(16)
        tube.CappingOn()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tube.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1.0, 0.78, 0.05)
        actor.GetProperty().SetOpacity(1.0)
        return actor

    def _make_vertex_cells(self, count: int) -> vtk.vtkCellArray:
        cells = np.empty(count * 2, dtype=np.int64)
        cells[0::2] = 1
        cells[1::2] = np.arange(count, dtype=np.int64)
        vtk_cells = vtk.vtkCellArray()
        vtk_cells.SetCells(count, numpy_support.numpy_to_vtkIdTypeArray(cells, deep=True))
        return vtk_cells

    def _height_colors(self, z_values: np.ndarray) -> np.ndarray:
        z_min = float(np.min(z_values))
        z_max = float(np.max(z_values))
        if z_max > z_min:
            t = ((z_values - z_min) / (z_max - z_min)).reshape(-1, 1)
        else:
            t = np.zeros((z_values.shape[0], 1), dtype=np.float32)

        low_color = np.array([153.0, 230.0, 255.0], dtype=np.float32)
        mid_color = np.array([0.0, 220.0, 80.0], dtype=np.float32)
        high_color = np.array([255.0, 45.0, 35.0], dtype=np.float32)
        colors = np.empty((z_values.shape[0], 3), dtype=np.float32)
        first_half = t[:, 0] <= 0.5
        t_low = np.clip(t * 2.0, 0.0, 1.0)
        t_high = np.clip((t - 0.5) * 2.0, 0.0, 1.0)
        colors[first_half] = (1.0 - t_low[first_half]) * low_color + t_low[first_half] * mid_color
        colors[~first_half] = (
            (1.0 - t_high[~first_half]) * mid_color + t_high[~first_half] * high_color
        )
        return colors.astype(np.uint8)

    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData()
        half = int(size / step)
        for i in range(-half, half + 1):
            line_x = vtk.vtkLineSource()
            line_x.SetPoint1(-size, i * step, 0.0)
            line_x.SetPoint2(size, i * step, 0.0)
            append.AddInputConnection(line_x.GetOutputPort())
            line_y = vtk.vtkLineSource()
            line_y.SetPoint1(i * step, -size, 0.0)
            line_y.SetPoint2(i * step, size, 0.0)
            append.AddInputConnection(line_y.GetOutputPort())

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(append.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.24, 0.30, 0.34)
        actor.GetProperty().SetLineWidth(1.0)
        actor.GetProperty().SetOpacity(0.65)
        return actor


def main() -> None:
    app = QApplication([])
    window = PcdGoalPickerWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
