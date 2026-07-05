import sys

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont, QPainter
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from mission_manager_sim import MissionManagerSim


SCREEN_W = 900
SCREEN_H = 500

# 二区方块采用固定布局，不再依赖红方/蓝方选择。
BLOCK_LAYOUT = [
    [12, 11, 10],
    [9, 8, 7],
    [6, 5, 4],
    [3, 2, 1],
]


class TouchButton(QPushButton):
    def __init__(
        self,
        text: str,
        kind: str = "primary",
        height: int = 52,
        fixed_height: bool = False,
    ):
        super().__init__(text)
        self.setObjectName(f"Btn_{kind}")

        if fixed_height:
            self.setFixedHeight(height)
        else:
            self.setMinimumHeight(height)

        self.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed,
        )


class SmallButton(QPushButton):
    def __init__(
        self,
        text: str,
        kind: str = "secondary",
    ):
        super().__init__(text)
        self.setObjectName(f"Btn_{kind}")
        self.setMinimumHeight(42)
        self.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Fixed,
        )


class BlockButton(QPushButton):
    def __init__(self, block_id: int):
        super().__init__("")

        self.block_id = block_id
        self.display_height = 0
        self.display_selected = False
        self.display_order = 0
        self.display_has_kfs = False
        self.display_edit_mode = "ROUTE"

        self.setCheckable(True)
        self.setMinimumHeight(58)
        self.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.setObjectName("Block_0")
        self.setProperty("editMode", "ROUTE")

    def set_block_visual(
        self,
        height_mm: int,
        selected: bool,
        order: int,
        has_kfs: bool,
        edit_mode: str,
    ):
        self.display_height = int(height_mm)
        self.display_selected = bool(selected)
        self.display_order = int(order)
        self.display_has_kfs = bool(has_kfs)
        self.display_edit_mode = edit_mode

        if height_mm <= 0:
            self.setObjectName("Block_0")
        elif height_mm == 200:
            self.setObjectName("Block_200")
        elif height_mm == 400:
            self.setObjectName("Block_400")
        elif height_mm == 600:
            self.setObjectName("Block_600")
        else:
            self.setObjectName("Block_400")

        self.setProperty("editMode", edit_mode)
        self.setChecked(selected)

        # objectName 或动态属性改变后，强制重新应用 QSS。
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

    def paintEvent(self, event):
        # 先由 QPushButton/QSS 绘制背景、边框和选中状态。
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)

        rect = self.rect()
        width = rect.width()
        height = rect.height()

        if self.display_selected:
            prefix = (
                f"{self.display_order}. "
                f"#{self.block_id}"
            )
        else:
            prefix = f"#{self.block_id}"

        height_text = (
            f"{self.display_height}mm"
        )

        if self.display_has_kfs:
            kfs_text = "★ 有KFS ★"
            kfs_color = QColor("#ffd21f")
        else:
            kfs_text = "无KFS"
            kfs_color = QColor("#ffffff")

        if self.display_edit_mode == "KFS":
            kfs_text = f"[{kfs_text}]"

        top_font = QFont(
            "Microsoft YaHei",
            11,
        )
        top_font.setBold(True)

        middle_font = QFont(
            "Microsoft YaHei",
            10,
        )

        kfs_font = QFont(
            "Microsoft YaHei",
            9,
        )
        kfs_font.setBold(True)

        normal_color = QColor("#ffffff")

        line_height = max(
            20,
            height // 4,
        )
        y1 = int(height * 0.18)
        y2 = int(height * 0.45)
        y3 = int(height * 0.72)

        painter.setPen(normal_color)
        painter.setFont(top_font)
        painter.drawText(
            0,
            y1,
            width,
            line_height,
            Qt.AlignCenter,
            prefix,
        )

        painter.setPen(normal_color)
        painter.setFont(middle_font)
        painter.drawText(
            0,
            y2,
            width,
            line_height,
            Qt.AlignCenter,
            height_text,
        )

        painter.setPen(kfs_color)
        painter.setFont(kfs_font)
        painter.drawText(
            0,
            y3,
            width,
            line_height,
            Qt.AlignCenter,
            kfs_text,
        )

        painter.end()


class InfoCard(QFrame):
    def __init__(
        self,
        title: str,
        body: str = "",
    ):
        super().__init__()

        self.setObjectName("InfoCard")

        self.title = QLabel(title)
        self.title.setObjectName("CardTitle")

        self.body = QLabel(body)
        self.body.setObjectName("CardBody")
        self.body.setWordWrap(True)
        self.body.setAlignment(
            Qt.AlignLeft |
            Qt.AlignVCenter
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(
            14,
            10,
            14,
            10,
        )
        layout.setSpacing(5)
        layout.addWidget(self.title)
        layout.addWidget(self.body)

    def set_body(self, text: str):
        self.body.setText(text)


class HomePage(QWidget):
    def __init__(self, ui):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            14,
            18,
            14,
        )
        root.setSpacing(10)

        title = QLabel("R2 任务控制台")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        body = QHBoxLayout()
        body.setSpacing(14)
        root.addLayout(body, stretch=1)

        # 左侧状态区
        left = QVBoxLayout()
        left.setSpacing(8)
        body.addLayout(left, stretch=5)

        self.status_card = InfoCard(
            "当前状态",
            "空闲：请先启动基础系统",
        )
        left.addWidget(self.status_card)

        self.system_card = InfoCard(
            "基础系统",
            "未启动",
        )
        left.addWidget(self.system_card)

        self.system_progress = QProgressBar()
        self.system_progress.setRange(
            0,
            100,
        )
        self.system_progress.setFormat(
            "系统准备进度 %p%"
        )
        left.addWidget(
            self.system_progress
        )

        left.addWidget(
            InfoCard(
                "操作流程",
                "1. 一区/二区：点击“启动一区/二区配置”。\n"
                "2. 三区：点击“启动三区高速配置”。\n"
                "3. 等待对应基础系统准备完成。\n"
                "4. 进入对应区域并点击“开始任务”。",
            ),
            stretch=1,
        )

        # 右侧按钮区
        right = QVBoxLayout()
        right.setSpacing(8)
        body.addLayout(right, stretch=4)

        self.start_system_btn = TouchButton(
            "启动一区/二区配置",
            "boot",
            height=44,
            fixed_height=True,
        )
        self.start_system_btn.clicked.connect(
            ui.start_system
        )
        right.addWidget(
            self.start_system_btn
        )

        self.start_zone3_system_btn = TouchButton(
            "启动三区高速配置",
            "boot",
            height=44,
            fixed_height=True,
        )
        self.start_zone3_system_btn.clicked.connect(
            ui.start_zone3_system
        )
        right.addWidget(
            self.start_zone3_system_btn
        )

        self.gym_btn = TouchButton(
            "进入一区任务",
            "primary",
            height=48,
        )
        self.gym_btn.clicked.connect(
            ui.goto_gym_prepare
        )
        right.addWidget(self.gym_btn)

        self.meilin_btn = TouchButton(
            "进入二区任务",
            "primary",
            height=48,
        )
        self.meilin_btn.clicked.connect(
            ui.goto_meilin_prepare
        )
        right.addWidget(self.meilin_btn)

        self.zone3_btn = TouchButton(
            "进入三区任务",
            "primary",
            height=48,
        )
        self.zone3_btn.clicked.connect(
            ui.goto_zone3_prepare
        )
        right.addWidget(self.zone3_btn)

        action_row = QHBoxLayout()
        action_row.setSpacing(8)

        stop_btn = SmallButton(
            "急停",
            "danger",
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )

        reset_btn = SmallButton(
            "复位",
            "warning",
        )
        reset_btn.clicked.connect(
            ui.reset_all
        )

        action_row.addWidget(stop_btn)
        action_row.addWidget(reset_btn)
        right.addLayout(action_row)
        right.addStretch(1)

        self.update_system(
            {
                "system_started": False,
                "system_ready": False,
                "system_starting": False,
                "system_progress": 0,
                "system_step": "未启动",
                "system_profile": None,
            }
        )

    def update_system(self, data):
        system_started = bool(
            data.get(
                "system_started",
                False,
            )
        )
        system_ready = bool(
            data.get(
                "system_ready",
                False,
            )
        )
        system_starting = bool(
            data.get(
                "system_starting",
                False,
            )
        )
        system_profile = data.get(
            "system_profile"
        )
        system_step = data.get(
            "system_step",
            "未启动",
        )
        progress = int(
            data.get(
                "system_progress",
                0,
            )
        )

        self.system_progress.setValue(
            progress
        )

        if system_starting:
            if system_profile == "zone3":
                profile_text = "三区高速配置"
            else:
                profile_text = "一区/二区普通配置"

            self.system_card.set_body(
                f"正在启动：{profile_text}\n"
                f"{system_step}"
            )

        elif system_ready:
            if system_profile == "zone3":
                self.system_card.set_body(
                    "READY：三区高速配置已启动\n"
                    "Odin 相对移动使用"
                    " relative_move_zone3.yaml"
                )
            elif system_profile == "normal":
                self.system_card.set_body(
                    "READY：一区/二区普通配置已启动\n"
                    "Odin 相对移动使用"
                    " relative_move.yaml"
                )
            else:
                self.system_card.set_body(
                    "READY：基础系统已启动"
                )

        elif system_started:
            self.system_card.set_body(
                "基础系统进程已启动，"
                "正在准备"
            )

        else:
            self.system_card.set_body(
                "未启动：请选择普通配置"
                "或三区高速配置"
            )

        can_start_system = (
            not system_started
            and not system_starting
            and not system_ready
        )

        self.start_system_btn.setEnabled(
            can_start_system
        )
        self.start_zone3_system_btn.setEnabled(
            can_start_system
        )

        normal_ready = (
            system_ready
            and system_profile == "normal"
        )
        zone3_ready = (
            system_ready
            and system_profile == "zone3"
        )

        self.gym_btn.setEnabled(
            normal_ready
        )
        self.meilin_btn.setEnabled(
            normal_ready
        )
        self.zone3_btn.setEnabled(
            zone3_ready
        )


class SimplePage(QWidget):
    def __init__(
        self,
        title_text,
        body_text,
        ui,
        start_func=None,
    ):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            16,
            18,
            16,
        )
        root.setSpacing(10)

        title = QLabel(title_text)
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)

        body.addWidget(
            InfoCard(
                "说明",
                body_text,
            ),
            stretch=5,
        )

        right = QVBoxLayout()
        body.addLayout(right, stretch=4)

        if start_func:
            start_btn = TouchButton(
                "开始任务",
                "start",
            )
            start_btn.clicked.connect(
                start_func
            )
            right.addWidget(start_btn)

        back_btn = TouchButton(
            "返回主页",
            "secondary",
        )
        back_btn.clicked.connect(
            ui.goto_home
        )
        right.addWidget(back_btn)

        stop_btn = TouchButton(
            "急停",
            "danger",
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )
        right.addWidget(stop_btn)

        right.addStretch(1)


class GymRunningPage(QWidget):
    def __init__(self, ui):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            16,
            18,
            16,
        )
        root.setSpacing(8)

        title = QLabel("一区运行中")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)

        left = QVBoxLayout()
        body.addLayout(left, stretch=4)

        self.step_card = InfoCard(
            "当前步骤",
            "-",
        )
        left.addWidget(self.step_card)

        self.count_card = InfoCard(
            "端头计数",
            "0 / 1",
        )
        left.addWidget(self.count_card)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat("%p%")
        left.addWidget(self.progress)

        stop_btn = TouchButton(
            "停止任务",
            "danger",
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )
        left.addWidget(stop_btn)
        left.addStretch(1)

        self.log_box = QTextEdit()
        self.log_box.setObjectName("LogBox")
        self.log_box.setReadOnly(True)
        body.addWidget(
            self.log_box,
            stretch=5,
        )


class Zone3RunningPage(QWidget):
    def __init__(self, ui):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            16,
            18,
            16,
        )
        root.setSpacing(8)

        title = QLabel("三区运行中")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)

        left = QVBoxLayout()
        body.addLayout(left, stretch=4)

        self.step_card = InfoCard(
            "当前步骤",
            "-",
        )
        left.addWidget(self.step_card)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat("%p%")
        left.addWidget(self.progress)

        stop_btn = TouchButton(
            "停止任务",
            "danger",
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )
        left.addWidget(stop_btn)
        left.addStretch(1)

        self.log_box = QTextEdit()
        self.log_box.setObjectName("LogBox")
        self.log_box.setReadOnly(True)
        body.addWidget(
            self.log_box,
            stretch=5,
        )


class MeilinPreparePage(QWidget):
    def __init__(self, ui):
        super().__init__()

        self.ui = ui
        self.block_buttons = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(
            12,
            10,
            12,
            10,
        )
        root.setSpacing(8)

        top = QHBoxLayout()
        root.addLayout(top)

        title = QLabel(
            "二区梅林路线 / KFS 选择"
        )
        title.setObjectName("PageHeader")
        title.setAlignment(
            Qt.AlignLeft |
            Qt.AlignVCenter
        )
        top.addWidget(title, stretch=1)

        body = QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        main = QVBoxLayout()
        body.addLayout(main, stretch=1)

        self.route_card = InfoCard(
            "当前路线",
            "未选择",
        )
        self.route_card.setMinimumHeight(66)
        main.addWidget(self.route_card)

        self.kfs_card = InfoCard(
            "KFS 状态",
            "未标记",
        )
        self.kfs_card.setMinimumHeight(62)
        main.addWidget(self.kfs_card)

        grid_frame = QFrame()
        grid_frame.setObjectName(
            "BlockGridFrame"
        )

        self.grid = QGridLayout(
            grid_frame
        )
        self.grid.setContentsMargins(
            6,
            6,
            6,
            6,
        )
        self.grid.setSpacing(6)

        for block_id in range(1, 13):
            button = BlockButton(block_id)
            button.clicked.connect(
                lambda checked, block=block_id:
                ui.toggle_block(block)
            )
            self.block_buttons[
                block_id
            ] = button

        main.addWidget(
            grid_frame,
            stretch=1,
        )

        side = QVBoxLayout()
        side.setSpacing(8)
        body.addLayout(side)

        self.mode_btn = TouchButton(
            "模式\n路线",
            "warning",
            height=44,
        )
        self.mode_btn.clicked.connect(
            ui.toggle_meilin_edit_mode
        )
        side.addWidget(self.mode_btn)

        save_btn = TouchButton(
            "保存\n配置",
            "boot",
            height=44,
        )
        save_btn.clicked.connect(
            ui.save_meilin_config
        )
        side.addWidget(save_btn)

        clear_btn = TouchButton(
            "CLEAR\n清",
            "secondary",
            height=44,
        )
        clear_btn.clicked.connect(
            ui.clear_block_sequence
        )
        side.addWidget(clear_btn)

        start_btn = TouchButton(
            "START\n开",
            "start",
            height=48,
        )
        start_btn.clicked.connect(
            ui.start_meilin
        )
        side.addWidget(start_btn)

        back_btn = TouchButton(
            "BACK\n返",
            "secondary",
            height=44,
        )
        back_btn.clicked.connect(
            ui.goto_home
        )
        side.addWidget(back_btn)

        stop_btn = TouchButton(
            "急停",
            "danger",
            height=44,
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )
        side.addWidget(stop_btn)

        side.addStretch(1)

        self.update_state(
            [],
            {},
            "ROUTE",
            {},
        )

    def update_state(
        self,
        sequence,
        block_has_kfs=None,
        edit_mode="ROUTE",
        block_heights=None,
    ):
        block_has_kfs = (
            block_has_kfs or {}
        )
        block_heights = (
            block_heights or {}
        )

        if sequence:
            route_blocks = [
                f"B{x}"
                for x in sequence
            ]

            route_text = " → ".join(
                route_blocks
            )

            if "EXIT_ZONE" not in route_blocks:
                route_text += " → EXIT_ZONE"

            self.route_card.set_body(
                route_text
            )
        else:
            self.route_card.set_body(
                "未选择"
            )

        kfs_blocks = [
            f"B{i}"
            for i in range(1, 13)
            if bool(
                block_has_kfs.get(
                    i,
                    False,
                )
            )
        ]

        self.kfs_card.set_body(
            "，".join(kfs_blocks)
            if kfs_blocks
            else "未标记"
        )

        if edit_mode == "KFS":
            self.mode_btn.setText(
                "模式\nKFS"
            )
        else:
            self.mode_btn.setText(
                "模式\n路线"
            )

        while self.grid.count():
            item = self.grid.takeAt(0)

            if item.widget():
                item.widget().setParent(None)

        for row_index, row in enumerate(
            BLOCK_LAYOUT
        ):
            for column_index, block_id in enumerate(
                row
            ):
                self.grid.addWidget(
                    self.block_buttons[
                        block_id
                    ],
                    row_index,
                    column_index,
                )

        for block_id, button in (
            self.block_buttons.items()
        ):
            height = int(
                block_heights.get(
                    block_id,
                    0,
                )
            )
            has_kfs = bool(
                block_has_kfs.get(
                    block_id,
                    False,
                )
            )
            selected = (
                block_id in sequence
            )
            order = (
                sequence.index(block_id) + 1
                if selected
                else 0
            )

            button.set_block_visual(
                height,
                selected,
                order,
                has_kfs,
                edit_mode,
            )


class MeilinRunningPage(QWidget):
    def __init__(self, ui):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            16,
            18,
            16,
        )
        root.setSpacing(8)

        title = QLabel("二区运行中")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)

        left = QVBoxLayout()
        body.addLayout(left, stretch=4)

        self.route_card = InfoCard(
            "路线",
            "未选择",
        )
        left.addWidget(self.route_card)

        self.kfs_card = InfoCard(
            "KFS",
            "未标记",
        )
        left.addWidget(self.kfs_card)

        self.step_card = InfoCard(
            "当前步骤",
            "-",
        )
        left.addWidget(self.step_card)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setFormat("%p%")
        left.addWidget(self.progress)

        stop_btn = TouchButton(
            "停止任务",
            "danger",
        )
        stop_btn.clicked.connect(
            ui.stop_task
        )
        left.addWidget(stop_btn)
        left.addStretch(1)

        self.log_box = QTextEdit()
        self.log_box.setObjectName("LogBox")
        self.log_box.setReadOnly(True)
        body.addWidget(
            self.log_box,
            stretch=5,
        )


class FinishPage(QWidget):
    def __init__(self, ui):
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(
            18,
            16,
            18,
            16,
        )
        root.setSpacing(10)

        title = QLabel("任务完成")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        root.addWidget(
            InfoCard(
                "状态",
                "任务完成。",
            ),
            stretch=1,
        )

        row = QHBoxLayout()

        home_btn = TouchButton(
            "返回主页",
            "secondary",
        )
        home_btn.clicked.connect(
            ui.goto_home
        )

        reset_btn = TouchButton(
            "复位",
            "warning",
        )
        reset_btn.clicked.connect(
            ui.reset_all
        )

        row.addWidget(home_btn)
        row.addWidget(reset_btn)
        root.addLayout(row)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "R2 900x500 横屏控制台"
        )
        self.setFixedSize(
            SCREEN_W,
            SCREEN_H,
        )

        self.manager = MissionManagerSim()

        self.manager.state_changed.connect(
            self.on_state_changed
        )
        self.manager.log_emitted.connect(
            self.on_log
        )
        self.manager.error_emitted.connect(
            self.show_error
        )

        self.stack = QStackedWidget()

        self.home_page = HomePage(self)

        self.gym_prepare_page = SimplePage(
            "一区任务",
            "确认机器人处于一区起点。\n"
            "点击“开始任务”后执行：\n"
            "zone1_competition_task.xml",
            self,
            self.start_gym,
        )

        self.gym_running_page = (
            GymRunningPage(self)
        )

        self.gym_done_page = SimplePage(
            "一区完成",
            "一区行为树已经执行完成。",
            self,
        )

        self.meilin_prepare_page = (
            MeilinPreparePage(self)
        )

        self.meilin_running_page = (
            MeilinRunningPage(self)
        )

        self.zone3_prepare_page = SimplePage(
            "三区任务",
            "当前基础系统应使用三区高速配置。\n"
            "确认机器人处于三区起点后，"
            "点击“开始任务”执行：\n"
            "zone3_competition_task.xml",
            self,
            self.start_zone3,
        )

        self.zone3_running_page = (
            Zone3RunningPage(self)
        )

        self.zone3_done_page = SimplePage(
            "三区完成",
            "三区行为树已经执行完成。",
            self,
        )

        self.finish_page = FinishPage(self)

        pages = [
            self.home_page,
            self.gym_prepare_page,
            self.gym_running_page,
            self.gym_done_page,
            self.meilin_prepare_page,
            self.meilin_running_page,
            self.zone3_prepare_page,
            self.zone3_running_page,
            self.zone3_done_page,
            self.finish_page,
        ]

        for page in pages:
            self.stack.addWidget(page)

        root = QVBoxLayout(self)
        root.setContentsMargins(
            0,
            0,
            0,
            0,
        )
        root.addWidget(self.stack)

        self.apply_style()

        self.manager.emit_state(
            "空闲：请先启动基础系统"
        )

    def apply_style(self):
        self.setStyleSheet(
            """
            QWidget {
                background-color: #000000;
                color: #f8fafc;
                font-family:
                    "Microsoft YaHei",
                    "SimHei",
                    "Arial";
                font-size: 11px;
            }

            #MainTitle {
                font-size: 20px;
                font-weight: 900;
                color: #ffffff;
            }

            #PageHeader {
                background-color: #c7e6ee;
                color: #00172b;
                font-size: 18px;
                font-weight: 900;
                padding-left: 16px;
                min-height: 44px;
            }

            #InfoCard {
                background-color: #171717;
                border: 1px solid #2f2f2f;
                border-radius: 16px;
            }

            #CardTitle {
                font-size: 11px;
                color: #ffffff;
                font-weight: 800;
            }

            #CardBody {
                font-size: 11px;
                color: #ffffff;
                line-height: 1.28;
                font-weight: 700;
            }

            QPushButton {
                background-color: #2563eb;
                color: white;
                border: 1px solid #3b82f6;
                border-radius: 16px;
                padding: 7px;
                font-size: 13px;
                font-weight: 900;
            }

            QPushButton:checked {
                border: 4px solid #facc15;
            }

            QPushButton:disabled {
                background-color: #374151;
                color: #9ca3af;
                border: 1px solid #4b5563;
            }

            #Btn_start {
                background-color: #16a34a;
                border: 2px solid #86efac;
                font-size: 12px;
            }

            #Btn_boot {
                background-color: #facc15;
                color: #111827;
                border: 2px solid #fde68a;
                font-size: 13px;
                padding: 4px;
            }

            #Btn_secondary {
                background-color: #e8f4fb;
                color: #00172b;
                border: 1px solid #9fb7c5;
                font-size: 12px;
            }

            #Btn_danger {
                background-color: #dc2626;
                border: 1px solid #ef4444;
            }

            #Btn_warning {
                background-color: #d97706;
                border: 1px solid #f59e0b;
            }

            #BlockGridFrame {
                background-color: #9fd5df;
                border: 3px solid #7cc4d1;
                border-radius: 8px;
            }

            #Block_0 {
                background-color: #374151;
                border: 2px solid #6b7280;
                border-radius: 12px;
                font-size: 12px;
            }

            #Block_200 {
                background-color: #24733a;
                border: 2px solid #44a35d;
                border-radius: 12px;
                font-size: 12px;
            }

            #Block_400 {
                background-color: #145c25;
                border: 2px solid #2f8b46;
                border-radius: 12px;
                font-size: 12px;
            }

            #Block_600 {
                background-color: #a8c24a;
                border: 2px solid #d7ef5a;
                border-radius: 12px;
                font-size: 12px;
            }

            #Block_0:checked,
            #Block_200:checked,
            #Block_400:checked,
            #Block_600:checked {
                border: 6px solid #facc15;
                background-color: #0f766e;
                color: #ffffff;
            }

            #LogBox {
                background-color: #050505;
                border: 1px solid #333333;
                border-radius: 14px;
                color: #e5e7eb;
                padding: 8px;
                font-family:
                    "Consolas",
                    "Microsoft YaHei";
                font-size: 9px;
            }

            QProgressBar {
                border: 1px solid #333333;
                border-radius: 10px;
                text-align: center;
                background-color: #171717;
                color: #ffffff;
                height: 16px;
                font-size: 10px;
            }

            QProgressBar::chunk {
                background-color: #22c55e;
                border-radius: 10px;
            }
            """
        )

    def goto_home(self):
        self.stack.setCurrentWidget(
            self.home_page
        )

    def start_system(self):
        self.manager.start_system(
            "normal"
        )

    def start_zone3_system(self):
        self.manager.start_system(
            "zone3"
        )

    def goto_gym_prepare(self):
        if not self.manager.system_ready:
            self.show_error(
                "请先启动一区/二区配置"
            )
            return

        if self.manager.system_profile != "normal":
            self.show_error(
                "当前不是一区/二区普通配置。\n"
                "请先复位，再启动一区/二区配置。"
            )
            return

        self.stack.setCurrentWidget(
            self.gym_prepare_page
        )

    def goto_meilin_prepare(self):
        if not self.manager.system_ready:
            self.show_error(
                "请先启动一区/二区配置"
            )
            return

        if self.manager.system_profile != "normal":
            self.show_error(
                "当前不是一区/二区普通配置。\n"
                "请先复位，再启动一区/二区配置。"
            )
            return

        self.meilin_prepare_page.update_state(
            self.manager.manual_block_sequence,
            self.manager.block_has_kfs,
            self.manager.edit_mode,
            self.manager.block_heights,
        )

        self.stack.setCurrentWidget(
            self.meilin_prepare_page
        )

    def goto_zone3_prepare(self):
        if not self.manager.system_ready:
            self.show_error(
                "请先启动三区高速配置"
            )
            return

        if self.manager.system_profile != "zone3":
            self.show_error(
                "当前不是三区高速配置。\n"
                "请先复位，再启动三区高速配置。"
            )
            return

        self.stack.setCurrentWidget(
            self.zone3_prepare_page
        )

    def start_gym(self):
        if not self.manager.system_ready:
            self.show_error(
                "一区/二区普通配置尚未准备完成，"
                "不能开始一区"
            )
            return

        if self.manager.system_profile != "normal":
            self.show_error(
                "一区只能使用普通速度配置。\n"
                "请先复位，再启动一区/二区配置。"
            )
            return

        self.gym_running_page.log_box.clear()
        self.manager.start_gym()

    def toggle_block(self, block_id):
        self.manager.toggle_block(block_id)

        self.meilin_prepare_page.update_state(
            self.manager.manual_block_sequence,
            self.manager.block_has_kfs,
            self.manager.edit_mode,
            self.manager.block_heights,
        )

    def toggle_meilin_edit_mode(self):
        self.manager.toggle_edit_mode()

        self.meilin_prepare_page.update_state(
            self.manager.manual_block_sequence,
            self.manager.block_has_kfs,
            self.manager.edit_mode,
            self.manager.block_heights,
        )

    def save_meilin_config(self):
        ok, message = (
            self.manager.save_meilin_config()
        )

        if ok:
            QMessageBox.information(
                self,
                "保存成功",
                message,
            )
        else:
            QMessageBox.warning(
                self,
                "保存失败",
                message,
            )

    def clear_block_sequence(self):
        self.manager.clear_block_sequence()

        self.meilin_prepare_page.update_state(
            self.manager.manual_block_sequence,
            self.manager.block_has_kfs,
            self.manager.edit_mode,
            self.manager.block_heights,
        )

    def start_meilin(self):
        if not self.manager.system_ready:
            self.show_error(
                "一区/二区普通配置尚未准备完成，"
                "不能开始二区"
            )
            return

        if self.manager.system_profile != "normal":
            self.show_error(
                "二区只能使用普通速度配置。\n"
                "请先复位，再启动一区/二区配置。"
            )
            return

        if not (
            self.manager.manual_block_sequence
        ):
            self.show_error(
                "请先选择至少一个梅林方块"
            )
            return

        self.meilin_running_page.log_box.clear()
        self.manager.start_meilin()

    def start_zone3(self):
        if not self.manager.system_ready:
            self.show_error(
                "三区高速配置尚未准备完成，"
                "不能开始三区"
            )
            return

        if self.manager.system_profile != "zone3":
            self.show_error(
                "三区必须使用高速配置。\n"
                "请先复位，再启动三区高速配置。"
            )
            return

        self.zone3_running_page.log_box.clear()
        self.manager.start_zone3()

    def stop_task(self):
        # 行为树已经退出时，只返回主页，避免重复执行耗时的急停流程。
        has_running_tree = (
            self.manager.tree_running
            or self.manager.gym_bt_process
            is not None
            or self.manager.meilin_bt_process
            is not None
            or self.manager.zone3_bt_process
            is not None
        )

        if not has_running_tree:
            self.stack.setCurrentWidget(
                self.home_page
            )
            return

        self.manager.stop()
        self.stack.setCurrentWidget(
            self.home_page
        )

    def reset_all(self):
        self.manager.reset()

        self.home_page.update_system(
            {
                "system_started": False,
                "system_ready": False,
                "system_starting": False,
                "system_progress": 0,
                "system_step": "未启动",
                "system_profile": None,
            }
        )

        self.meilin_prepare_page.update_state(
            [],
            {},
            "ROUTE",
            self.manager.block_heights,
        )

        self.gym_running_page.log_box.clear()
        self.meilin_running_page.log_box.clear()
        self.zone3_running_page.log_box.clear()

        self.stack.setCurrentWidget(
            self.home_page
        )

    def closeEvent(self, event):
        has_running_process = (
            self.manager.tree_running
            or self.manager.system_started
            or self.manager.system_ready
            or self.manager.system_starting
            or self.manager.system_process
            is not None
            or self.manager.gym_bt_process
            is not None
            or self.manager.meilin_bt_process
            is not None
            or self.manager.zone3_bt_process
            is not None
        )

        if has_running_process:
            reply = QMessageBox.question(
                self,
                "确认退出",
                "检测到系统或行为树进程"
                "可能仍在运行。\n\n"
                "直接关闭窗口会自动执行复位：\n"
                "1. 发布 /cmd_vel=0\n"
                "2. 停止当前行为树\n"
                "3. Ctrl+C 关闭总 launch\n\n"
                "确定要退出吗？",
                QMessageBox.Yes |
                QMessageBox.No,
                QMessageBox.No,
            )

            if reply != QMessageBox.Yes:
                event.ignore()
                return

            try:
                self.manager.reset()
            except Exception as error:
                QMessageBox.warning(
                    self,
                    "退出时复位失败",
                    "关闭窗口时执行复位失败：\n"
                    f"{error}\n\n"
                    "窗口会继续关闭，但需要"
                    "手动检查 ROS 2 进程。",
                )

        event.accept()

    def on_state_changed(self, data):
        state = data.get(
            "state",
            "IDLE",
        )
        step = data.get(
            "current_step",
            "-",
        )
        progress = int(
            data.get(
                "progress",
                0,
            )
        )

        self.home_page.status_card.set_body(
            data.get(
                "message",
                "空闲",
            )
        )
        self.home_page.update_system(data)

        self.gym_running_page.step_card.set_body(
            step
        )
        self.gym_running_page.count_card.set_body(
            f'{data.get("assembly_count", 0)}'
            " / "
            f'{data.get("target_assembly_count", 1)}'
        )
        self.gym_running_page.progress.setValue(
            progress
        )

        self.zone3_running_page.step_card.set_body(
            step
        )
        self.zone3_running_page.progress.setValue(
            progress
        )

        self.meilin_prepare_page.update_state(
            data.get(
                "manual_block_sequence",
                [],
            ),
            data.get(
                "block_has_kfs",
                {},
            ),
            data.get(
                "edit_mode",
                "ROUTE",
            ),
            data.get(
                "block_heights",
                {},
            ),
        )

        self.meilin_running_page.route_card.set_body(
            data.get(
                "manual_route_text",
                "未选择",
            )
        )
        self.meilin_running_page.kfs_card.set_body(
            data.get(
                "kfs_text",
                "未标记",
            )
        )
        self.meilin_running_page.step_card.set_body(
            step
        )
        self.meilin_running_page.progress.setValue(
            progress
        )

        if state == "GYM_DONE_WAIT_LIFT":
            self.stack.setCurrentWidget(
                self.gym_done_page
            )

        elif state == "RUNNING_GYM":
            self.stack.setCurrentWidget(
                self.gym_running_page
            )

        elif state == "RUNNING_MEILIN":
            self.stack.setCurrentWidget(
                self.meilin_running_page
            )

        elif state == "RUNNING_ZONE3":
            self.stack.setCurrentWidget(
                self.zone3_running_page
            )

        elif state == "ZONE3_DONE":
            self.stack.setCurrentWidget(
                self.zone3_done_page
            )

        elif state == "ZONE3_FAILED":
            self.stack.setCurrentWidget(
                self.zone3_prepare_page
            )

        elif state == "MATCH_DONE":
            self.stack.setCurrentWidget(
                self.finish_page
            )

        elif state in {
            "MEILIN_FAILED",
            "MEILIN_EXITED",
        }:
            self.meilin_prepare_page.update_state(
                data.get(
                    "manual_block_sequence",
                    [],
                ),
                data.get(
                    "block_has_kfs",
                    {},
                ),
                data.get(
                    "edit_mode",
                    "ROUTE",
                ),
                data.get(
                    "block_heights",
                    {},
                ),
            )
            self.stack.setCurrentWidget(
                self.meilin_prepare_page
            )

        elif state == "GYM_FAILED":
            self.stack.setCurrentWidget(
                self.gym_prepare_page
            )

        elif state in {
            "SYSTEM_EXITED",
            "STOPPED",
        }:
            self.stack.setCurrentWidget(
                self.home_page
            )

    def on_log(self, text):
        self.gym_running_page.log_box.append(
            text
        )
        self.meilin_running_page.log_box.append(
            text
        )
        self.zone3_running_page.log_box.append(
            text
        )

    def show_error(self, text):
        QMessageBox.warning(
            self,
            "提示",
            text,
        )


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
