import sys
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QProgressBar,
    QMessageBox, QStackedWidget, QSizePolicy
)
from mission_manager_sim import MissionManagerSim

SCREEN_W = 900
SCREEN_H = 600

RED_LAYOUT = [[12, 11, 10], [9, 8, 7], [6, 5, 4], [3, 2, 1]]
BLUE_LAYOUT = [[10, 11, 12], [7, 8, 9], [4, 5, 6], [1, 2, 3]]


class TouchButton(QPushButton):
    def __init__(self, text: str, kind: str = "primary", height: int = 68, fixed_height: bool = False):
        super().__init__(text)
        self.setObjectName(f"Btn_{kind}")
        if fixed_height:
            self.setFixedHeight(height)
        else:
            self.setMinimumHeight(height)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class SmallButton(QPushButton):
    def __init__(self, text: str, kind: str = "secondary"):
        super().__init__(text)
        self.setObjectName(f"Btn_{kind}")
        self.setMinimumHeight(52)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class TeamButton(QPushButton):
    def __init__(self, text: str, kind: str):
        super().__init__(text)
        self.setObjectName(f"Btn_{kind}")
        self.setCheckable(True)
        self.setMinimumHeight(62)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


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
        self.setMinimumHeight(82)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setObjectName("Block_0")
        self.setProperty("editMode", "ROUTE")

    def set_block_visual(self, height_mm: int, selected: bool, order: int, has_kfs: bool, edit_mode: str):
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

        # objectName / dynamic property 改变后强制刷新 QSS。
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

    def paintEvent(self, event):
        # 先让 QPushButton/QSS 画背景、边框和选中状态。
        super().paintEvent(event)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)

        rect = self.rect()
        w = rect.width()
        h = rect.height()

        prefix = f"{self.display_order}. #{self.block_id}" if self.display_selected else f"#{self.block_id}"
        height_text = f"{self.display_height}mm"

        if self.display_has_kfs:
            kfs_text = "★ 有KFS ★"
            kfs_color = QColor("#ffd21f")  # 金色：醒目但不加边框
        else:
            kfs_text = "无KFS"
            kfs_color = QColor("#ffffff")  # 无 KFS：白色

        if self.display_edit_mode == "KFS":
            kfs_text = f"[{kfs_text}]"

        top_font = QFont("Microsoft YaHei", 16)
        top_font.setBold(True)
        mid_font = QFont("Microsoft YaHei", 15)
        kfs_font = QFont("Microsoft YaHei", 13)  # KFS 字体再小一点，避免三行文字太挤
        kfs_font.setBold(True)

        # 选中时整体背景较深，文字保持白/金；未选中也保持高对比度。
        normal_color = QColor("#ffffff")

        line_h = max(20, h // 4)
        y1 = int(h * 0.18)
        y2 = int(h * 0.45)
        y3 = int(h * 0.72)

        painter.setPen(normal_color)
        painter.setFont(top_font)
        painter.drawText(0, y1, w, line_h, Qt.AlignCenter, prefix)

        painter.setPen(normal_color)
        painter.setFont(mid_font)
        painter.drawText(0, y2, w, line_h, Qt.AlignCenter, height_text)

        painter.setPen(kfs_color)
        painter.setFont(kfs_font)
        painter.drawText(0, y3, w, line_h, Qt.AlignCenter, kfs_text)

        painter.end()


class InfoCard(QFrame):
    def __init__(self, title: str, body: str = ""):
        super().__init__()
        self.setObjectName("InfoCard")
        self.title = QLabel(title)
        self.title.setObjectName("CardTitle")
        self.body = QLabel(body)
        self.body.setObjectName("CardBody")
        self.body.setWordWrap(True)
        self.body.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(5)
        layout.addWidget(self.title)
        layout.addWidget(self.body)

    def set_body(self, text: str):
        self.body.setText(text)


class HomePage(QWidget):
    def __init__(self, ui):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(8)
        title = QLabel("R2 任务控制台")
        title.setObjectName("MainTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)
        body = QHBoxLayout(); body.setSpacing(14); root.addLayout(body, stretch=1)
        left = QVBoxLayout(); left.setSpacing(8); body.addLayout(left, stretch=5)
        self.status_card = InfoCard("当前状态", "空闲：请选择红方/蓝方"); left.addWidget(self.status_card)
        self.team_card = InfoCard("当前队伍", "未选择"); left.addWidget(self.team_card)
        self.system_card = InfoCard("系统启动", "未启动：请选择队伍后点击“启动系统”"); left.addWidget(self.system_card)
        self.system_progress = QProgressBar(); self.system_progress.setRange(0, 100); self.system_progress.setFormat("系统准备进度 %p%"); left.addWidget(self.system_progress)
        left.addWidget(InfoCard("操作流程", "1. 选择红/蓝方。\n2. 启动系统。\n3. 进入一区或二区任务。"), stretch=1)
        right = QVBoxLayout(); right.setSpacing(8); body.addLayout(right, stretch=4)
        team_row = QHBoxLayout(); team_row.setSpacing(8)
        self.red_btn = TeamButton("RED\n红方", "red"); self.blue_btn = TeamButton("BLUE\n蓝方", "blue")
        self.red_btn.clicked.connect(lambda: ui.select_team("RED")); self.blue_btn.clicked.connect(lambda: ui.select_team("BLUE"))
        team_row.addWidget(self.red_btn); team_row.addWidget(self.blue_btn); right.addLayout(team_row)
        self.start_system_btn = TouchButton("启动系统 / 加载配置", "boot", height=50, fixed_height=True); self.start_system_btn.clicked.connect(ui.start_system); right.addWidget(self.start_system_btn)
        self.gym_btn = TouchButton("进入一区任务", "primary", height=60); self.gym_btn.clicked.connect(ui.goto_gym_prepare); right.addWidget(self.gym_btn)
        self.meilin_btn = TouchButton("进入二区任务", "primary", height=60); self.meilin_btn.clicked.connect(ui.goto_meilin_prepare); right.addWidget(self.meilin_btn)
        row = QHBoxLayout(); row.setSpacing(8)
        btn_stop = SmallButton("急停", "danger"); btn_stop.clicked.connect(ui.stop_task)
        btn_reset = SmallButton("复位", "warning"); btn_reset.clicked.connect(ui.reset_all)
        row.addWidget(btn_stop); row.addWidget(btn_reset); right.addLayout(row); right.addStretch(1)

    def update_team(self, team):
        self.team_card.set_body("红方：将加载 red 配置" if team == "RED" else ("蓝方：将加载 blue 配置" if team == "BLUE" else "未选择"))
        self.red_btn.setChecked(team == "RED"); self.blue_btn.setChecked(team == "BLUE")

    def update_system(self, data):
        team = data.get("current_team", "UNKNOWN")
        system_started = data.get("system_started", False); system_ready = data.get("system_ready", False); system_starting = data.get("system_starting", False)
        system_step = data.get("system_step", "未启动"); progress = int(data.get("system_progress", 0)); state = data.get("state", "IDLE")
        self.system_progress.setValue(progress)
        if team == "UNKNOWN": self.system_card.set_body("未启动：请先选择红方或蓝方")
        elif system_starting: self.system_card.set_body(f"启动中：{system_step}")
        elif system_ready: self.system_card.set_body("READY：服务端和红/蓝方配置已加载完成")
        elif system_started: self.system_card.set_body("已启动，但尚未 ready，请检查日志")
        else: self.system_card.set_body("未启动：请选择队伍后点击“启动系统”")
        team_locked = system_started or system_starting or state in ["RUNNING_GYM", "RUNNING_MEILIN"]
        self.red_btn.setEnabled(not team_locked); self.blue_btn.setEnabled(not team_locked)
        self.start_system_btn.setEnabled((team != "UNKNOWN") and (not system_started) and (not system_starting))
        self.gym_btn.setEnabled(system_ready); self.meilin_btn.setEnabled(system_ready)


class SimplePage(QWidget):
    def __init__(self, title_text, body_text, ui, start_func=None):
        super().__init__()
        root = QVBoxLayout(self); root.setContentsMargins(18, 16, 18, 16); root.setSpacing(10)
        title = QLabel(title_text); title.setObjectName("MainTitle"); title.setAlignment(Qt.AlignCenter); root.addWidget(title)
        body = QHBoxLayout(); root.addLayout(body, stretch=1)
        body.addWidget(InfoCard("说明", body_text), stretch=5)
        right = QVBoxLayout(); body.addLayout(right, stretch=4)
        if start_func:
            start_btn = TouchButton("开始", "start"); start_btn.clicked.connect(start_func); right.addWidget(start_btn)
        back_btn = TouchButton("返回主页", "secondary"); back_btn.clicked.connect(ui.goto_home); right.addWidget(back_btn)
        stop_btn = TouchButton("急停", "danger"); stop_btn.clicked.connect(ui.stop_task); right.addWidget(stop_btn); right.addStretch(1)


class GymRunningPage(QWidget):
    def __init__(self, ui):
        super().__init__()
        root = QVBoxLayout(self); root.setContentsMargins(18,16,18,16); root.setSpacing(8)
        title = QLabel("一区运行中"); title.setObjectName("MainTitle"); title.setAlignment(Qt.AlignCenter); root.addWidget(title)
        body = QHBoxLayout(); root.addLayout(body, stretch=1)
        left = QVBoxLayout(); body.addLayout(left, stretch=4)
        self.step_card = InfoCard("当前步骤", "-"); left.addWidget(self.step_card)
        self.count_card = InfoCard("端头计数", "0 / 1"); left.addWidget(self.count_card)
        self.progress = QProgressBar(); self.progress.setRange(0,100); self.progress.setFormat("%p%"); left.addWidget(self.progress)
        stop_btn = TouchButton("停止任务", "danger"); stop_btn.clicked.connect(ui.stop_task); left.addWidget(stop_btn)
        self.log_box = QTextEdit(); self.log_box.setObjectName("LogBox"); self.log_box.setReadOnly(True); body.addWidget(self.log_box, stretch=5)


class MeilinPreparePage(QWidget):
    def __init__(self, ui):
        super().__init__(); self.ui = ui; self.block_buttons = {}
        root = QVBoxLayout(self); root.setContentsMargins(12, 10, 12, 10); root.setSpacing(8)
        top = QHBoxLayout(); root.addLayout(top)
        title = QLabel("二区梅林路线 / KFS 选择"); title.setObjectName("PageHeader"); title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter); top.addWidget(title, stretch=1)
        self.team_label = QLabel("队伍：未选择"); self.team_label.setObjectName("TeamLabel"); self.team_label.setAlignment(Qt.AlignCenter); top.addWidget(self.team_label)
        body = QHBoxLayout(); body.setSpacing(10); root.addLayout(body, stretch=1)
        main = QVBoxLayout(); body.addLayout(main, stretch=1)
        self.route_card = InfoCard("当前路线", "未选择"); self.route_card.setMinimumHeight(66); main.addWidget(self.route_card)
        self.kfs_card = InfoCard("KFS 状态", "未标记"); self.kfs_card.setMinimumHeight(62); main.addWidget(self.kfs_card)
        grid_frame = QFrame(); grid_frame.setObjectName("BlockGridFrame"); self.grid = QGridLayout(grid_frame); self.grid.setContentsMargins(6,6,6,6); self.grid.setSpacing(6)
        for block_id in range(1,13):
            btn = BlockButton(block_id); btn.clicked.connect(lambda checked, b=block_id: ui.toggle_block(b)); self.block_buttons[block_id]=btn
        main.addWidget(grid_frame, stretch=1)
        side = QVBoxLayout(); side.setSpacing(8); body.addLayout(side)
        self.mode_btn = TouchButton("模式\n路线", "warning", height=55); self.mode_btn.clicked.connect(ui.toggle_meilin_edit_mode); side.addWidget(self.mode_btn)
        save_btn = TouchButton("保存\n配置", "boot", height=55); save_btn.clicked.connect(ui.save_meilin_config); side.addWidget(save_btn)
        clear_btn = TouchButton("CLEAR\n清", "secondary", height=55); clear_btn.clicked.connect(ui.clear_block_sequence); side.addWidget(clear_btn)
        start_btn = TouchButton("START\n开", "start", height=62); start_btn.clicked.connect(ui.start_meilin); side.addWidget(start_btn)
        back_btn = TouchButton("BACK\n返", "secondary", height=55); back_btn.clicked.connect(ui.goto_home); side.addWidget(back_btn)
        stop_btn = TouchButton("急停", "danger", height=55); stop_btn.clicked.connect(ui.stop_task); side.addWidget(stop_btn); side.addStretch(1)
        self.update_state("UNKNOWN", [], {}, "ROUTE", {})

    def update_state(self, team, sequence, block_has_kfs=None, edit_mode="ROUTE", block_heights=None):
        block_has_kfs = block_has_kfs or {}; block_heights = block_heights or {}
        self.team_label.setText("队伍：" + ("红方" if team == "RED" else ("蓝方" if team == "BLUE" else "未选择")))
        if sequence:
            route_blocks = [f"B{x}" for x in sequence]
            route_text = " → ".join(route_blocks) + (" → EXIT_ZONE" if "EXIT_ZONE" not in route_blocks else "")
            self.route_card.set_body(route_text)
        else: self.route_card.set_body("未选择")
        kfs_blocks=[f"B{i}" for i in range(1,13) if bool(block_has_kfs.get(i,False))]
        self.kfs_card.set_body("，".join(kfs_blocks) if kfs_blocks else "未标记")
        self.mode_btn.setText("模式\nKFS" if edit_mode == "KFS" else "模式\n路线")
        layout = RED_LAYOUT if team != "BLUE" else BLUE_LAYOUT
        while self.grid.count():
            item=self.grid.takeAt(0)
            if item.widget(): item.widget().setParent(None)
        for r,row in enumerate(layout):
            for c,block_id in enumerate(row): self.grid.addWidget(self.block_buttons[block_id], r, c)
        for block_id,btn in self.block_buttons.items():
            height=int(block_heights.get(block_id,0)); has_kfs=bool(block_has_kfs.get(block_id,False)); selected=block_id in sequence; order=sequence.index(block_id)+1 if selected else 0
            btn.set_block_visual(height, selected, order, has_kfs, edit_mode)


class MeilinRunningPage(QWidget):
    def __init__(self, ui):
        super().__init__()
        root=QVBoxLayout(self); root.setContentsMargins(18,16,18,16); root.setSpacing(8)
        title=QLabel("二区运行中"); title.setObjectName("MainTitle"); title.setAlignment(Qt.AlignCenter); root.addWidget(title)
        body=QHBoxLayout(); root.addLayout(body, stretch=1)
        left=QVBoxLayout(); body.addLayout(left, stretch=4)
        self.team_card=InfoCard("队伍","未选择"); left.addWidget(self.team_card)
        self.route_card=InfoCard("路线","未选择"); left.addWidget(self.route_card)
        self.kfs_card=InfoCard("KFS","未标记"); left.addWidget(self.kfs_card)
        self.step_card=InfoCard("当前步骤","-"); left.addWidget(self.step_card)
        self.odin_card=InfoCard("Odin","正常"); left.addWidget(self.odin_card)
        self.progress=QProgressBar(); self.progress.setRange(0,100); self.progress.setFormat("%p%"); left.addWidget(self.progress)
        stop_btn=TouchButton("停止任务","danger"); stop_btn.clicked.connect(ui.stop_task); left.addWidget(stop_btn)
        self.log_box=QTextEdit(); self.log_box.setObjectName("LogBox"); self.log_box.setReadOnly(True); body.addWidget(self.log_box, stretch=5)


class FinishPage(QWidget):
    def __init__(self, ui):
        super().__init__()
        root=QVBoxLayout(self); root.setContentsMargins(18,16,18,16); root.setSpacing(10)
        title=QLabel("任务完成"); title.setObjectName("MainTitle"); title.setAlignment(Qt.AlignCenter); root.addWidget(title)
        root.addWidget(InfoCard("状态","任务完成。"), stretch=1)
        row=QHBoxLayout(); home_btn=TouchButton("返回主页","secondary"); home_btn.clicked.connect(ui.goto_home); reset_btn=TouchButton("复位","warning"); reset_btn.clicked.connect(ui.reset_all)
        row.addWidget(home_btn); row.addWidget(reset_btn); root.addLayout(row)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__(); self.setWindowTitle("R2 900x600 横屏控制台"); self.setFixedSize(SCREEN_W, SCREEN_H)
        self.manager=MissionManagerSim(); self.manager.state_changed.connect(self.on_state_changed); self.manager.log_emitted.connect(self.on_log); self.manager.error_emitted.connect(self.show_error)
        self.stack=QStackedWidget(); self.home_page=HomePage(self); self.gym_prepare_page=SimplePage("一区任务","机器人在一区起点。\n点击开始后模拟一区任务。",self,self.start_gym); self.gym_running_page=GymRunningPage(self); self.gym_done_page=SimplePage("一区完成","请人工把机器人抬回重试区。",self,self.goto_meilin_prepare); self.meilin_prepare_page=MeilinPreparePage(self); self.meilin_running_page=MeilinRunningPage(self); self.finish_page=FinishPage(self)
        for page in [self.home_page,self.gym_prepare_page,self.gym_running_page,self.gym_done_page,self.meilin_prepare_page,self.meilin_running_page,self.finish_page]: self.stack.addWidget(page)
        root=QVBoxLayout(self); root.setContentsMargins(0,0,0,0); root.addWidget(self.stack); self.apply_style(); self.manager.emit_state("空闲：请选择红方/蓝方，然后启动系统")

    def apply_style(self):
        self.setStyleSheet('''
        QWidget { background-color: #000000; color: #f8fafc; font-family: "Microsoft YaHei", "SimHei", "Arial"; font-size: 15px; }
        #MainTitle { font-size: 30px; font-weight: 900; color: #ffffff; }
        #PageHeader { background-color: #c7e6ee; color: #00172b; font-size: 28px; font-weight: 900; padding-left: 16px; min-height: 44px; }
        #TeamLabel { background-color: #c7e6ee; color: #00172b; font-size: 20px; font-weight: 900; padding: 0px 12px; min-height: 44px; }
        #InfoCard { background-color: #171717; border: 1px solid #2f2f2f; border-radius: 16px; }
        #CardTitle { font-size: 16px; color: #ffffff; font-weight: 800; }
        #CardBody { font-size: 16px; color: #ffffff; line-height: 1.28; font-weight: 700; }
        QPushButton { background-color: #2563eb; color: white; border: 1px solid #3b82f6; border-radius: 16px; padding: 7px; font-size: 19px; font-weight: 900; }
        QPushButton:checked { border: 4px solid #facc15; } QPushButton:disabled { background-color: #374151; color: #9ca3af; border: 1px solid #4b5563; }
        #Btn_start { background-color: #16a34a; border: 2px solid #86efac; font-size: 24px; } #Btn_boot { background-color: #facc15; color: #111827; border: 2px solid #fde68a; font-size: 19px; padding: 4px; }
        #Btn_secondary { background-color: #e8f4fb; color: #00172b; border: 1px solid #9fb7c5; font-size: 18px; } #Btn_danger { background-color: #dc2626; border: 1px solid #ef4444; } #Btn_warning { background-color: #d97706; border: 1px solid #f59e0b; }
        #Btn_red { background-color: #f8fafc; color: #ef4444; border: 3px solid #ef4444; font-size: 22px; } #Btn_red:checked { background-color: #ef4444; color: #ffffff; border: 4px solid #fecaca; }
        #Btn_blue { background-color: #f8fafc; color: #3b82f6; border: 3px solid #3b82f6; font-size: 22px; } #Btn_blue:checked { background-color: #3b82f6; color: #ffffff; border: 4px solid #bfdbfe; }
        #BlockGridFrame { background-color: #9fd5df; border: 3px solid #7cc4d1; border-radius: 8px; }
        #Block_0 { background-color: #374151; border: 2px solid #6b7280; border-radius: 12px; font-size: 18px; }
        /* 200mm 和 400mm 颜色互换：200 用原 400 颜色，400 用原 200 颜色 */
        #Block_200 { background-color: #24733a; border: 2px solid #44a35d; border-radius: 12px; font-size: 18px; }
        #Block_400 { background-color: #145c25; border: 2px solid #2f8b46; border-radius: 12px; font-size: 18px; }
        #Block_600 { background-color: #a8c24a; border: 2px solid #d7ef5a; border-radius: 12px; font-size: 18px; }
        #Block_0:checked, #Block_200:checked, #Block_400:checked, #Block_600:checked {
            border: 6px solid #facc15;
            background-color: #0f766e;
            color: #ffffff;
        }

        #LogBox { background-color: #050505; border: 1px solid #333333; border-radius: 14px; color: #e5e7eb; padding: 8px; font-family: "Consolas", "Microsoft YaHei"; font-size: 12px; }
        QProgressBar { border: 1px solid #333333; border-radius: 10px; text-align: center; background-color: #171717; color: #ffffff; height: 24px; font-size: 14px; } QProgressBar::chunk { background-color: #22c55e; border-radius: 10px; }
        ''')

    def goto_home(self): self.stack.setCurrentWidget(self.home_page)
    def goto_gym_prepare(self):
        if self.manager.current_team == "UNKNOWN": self.show_error("请先在主页选择红方或蓝方"); return
        if not self.manager.system_ready: self.show_error("请先点击“启动系统”，等待系统准备完成"); return
        self.stack.setCurrentWidget(self.gym_prepare_page)
    def goto_meilin_prepare(self):
        if self.manager.current_team == "UNKNOWN": self.show_error("请先在主页选择红方或蓝方"); return
        if not self.manager.system_ready: self.show_error("请先点击“启动系统”，等待系统准备完成"); return
        self.meilin_prepare_page.update_state(self.manager.current_team,self.manager.manual_block_sequence,self.manager.block_has_kfs,self.manager.edit_mode,self.manager.block_heights); self.stack.setCurrentWidget(self.meilin_prepare_page)
    def select_team(self, team):
        self.manager.select_team(team); self.home_page.update_team(self.manager.current_team); self.meilin_prepare_page.update_state(self.manager.current_team,self.manager.manual_block_sequence,self.manager.block_has_kfs,self.manager.edit_mode,self.manager.block_heights)
    def start_system(self): self.manager.start_system()
    def start_gym(self):
        if not self.manager.system_ready: self.show_error("系统尚未准备完成，不能开始一区"); return
        self.stack.setCurrentWidget(self.gym_running_page); self.gym_running_page.log_box.clear(); self.manager.start_gym()
    def toggle_block(self, block_id):
        self.manager.toggle_block(block_id); self.meilin_prepare_page.update_state(self.manager.current_team,self.manager.manual_block_sequence,self.manager.block_has_kfs,self.manager.edit_mode,self.manager.block_heights)
    def toggle_meilin_edit_mode(self):
        self.manager.toggle_edit_mode(); self.meilin_prepare_page.update_state(self.manager.current_team,self.manager.manual_block_sequence,self.manager.block_has_kfs,self.manager.edit_mode,self.manager.block_heights)
    def save_meilin_config(self):
        ok,msg=self.manager.save_meilin_config(); QMessageBox.information(self,"保存成功",msg) if ok else QMessageBox.warning(self,"保存失败",msg)
    def clear_block_sequence(self):
        self.manager.clear_block_sequence(); self.meilin_prepare_page.update_state(self.manager.current_team,self.manager.manual_block_sequence,self.manager.block_has_kfs,self.manager.edit_mode,self.manager.block_heights)
    def start_meilin(self):
        if self.manager.current_team == "UNKNOWN": self.show_error("请先在主页选择红方或蓝方"); return
        if not self.manager.system_ready: self.show_error("系统尚未准备完成，不能开始二区"); return
        if not self.manager.manual_block_sequence: self.show_error("请先选择至少一个梅林方块"); return
        self.stack.setCurrentWidget(self.meilin_running_page); self.meilin_running_page.log_box.clear(); self.manager.start_meilin()
    def stop_task(self):
        # 如果行为树已经自己退出了，就不要再执行急停流程，避免 UI 卡顿。
        if (
            not self.manager.tree_running
            and self.manager.gym_bt_process is None
            and self.manager.meilin_bt_process is None
        ):
            self.stack.setCurrentWidget(self.home_page)
            return

        self.manager.stop()
        self.stack.setCurrentWidget(self.home_page)
    def reset_all(self):
        self.manager.reset(); self.home_page.update_team("UNKNOWN"); self.home_page.update_system({"current_team":"UNKNOWN","system_started":False,"system_ready":False,"system_starting":False,"system_progress":0}); self.meilin_prepare_page.update_state("UNKNOWN", [], {}, "ROUTE", self.manager.block_heights); self.gym_running_page.log_box.clear(); self.meilin_running_page.log_box.clear(); self.stack.setCurrentWidget(self.home_page)
    def closeEvent(self, event):
        """
        用户点击窗口右上角 X 时触发。
        目的：
        1. 如果系统/行为树正在运行，提示用户确认
        2. 确认关闭后，自动执行 reset()
        3. 避免 UI 关闭后 ROS2 launch / 行为树进程继续后台运行
        """
        has_running_process = (
            self.manager.tree_running
            or self.manager.system_started
            or self.manager.system_ready
            or self.manager.system_starting
            or self.manager.system_process is not None
            or self.manager.gym_bt_process is not None
            or self.manager.meilin_bt_process is not None
        )

        if has_running_process:
            reply = QMessageBox.question(
                self,
                "确认退出",
                "检测到系统或行为树进程可能仍在运行。\n\n"
                "直接关闭窗口会自动执行复位：\n"
                "1. 发布 /cmd_vel=0\n"
                "2. 停止当前行为树\n"
                "3. Ctrl+C 关闭总 launch\n\n"
                "确定要退出吗？",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )

            if reply != QMessageBox.Yes:
                event.ignore()
                return

            try:
                self.manager.reset()
            except Exception as e:
                QMessageBox.warning(
                    self,
                    "退出时复位失败",
                    f"关闭窗口时执行复位失败：\n{e}\n\n"
                    "窗口会继续关闭，但你需要手动检查 ROS2 进程。",
                )

        event.accept()    
    def on_state_changed(self,data):
        state=data.get("state","IDLE"); step=data.get("current_step","-"); progress=int(data.get("progress",0)); team=data.get("current_team","UNKNOWN"); route_text=data.get("manual_route_text","未选择"); kfs_text=data.get("kfs_text","未标记"); team_text="红方" if team=="RED" else ("蓝方" if team=="BLUE" else "未选择")
        self.home_page.status_card.set_body(data.get("message","空闲")); self.home_page.update_team(team); self.home_page.update_system(data)
        self.gym_running_page.step_card.set_body(step); self.gym_running_page.count_card.set_body(f'{data.get("assembly_count",0)} / {data.get("target_assembly_count",1)}'); self.gym_running_page.progress.setValue(progress)
        self.meilin_prepare_page.update_state(team,data.get("manual_block_sequence",[]),data.get("block_has_kfs",{}),data.get("edit_mode","ROUTE"),data.get("block_heights",{}))
        self.meilin_running_page.team_card.set_body(team_text); self.meilin_running_page.route_card.set_body(route_text); self.meilin_running_page.kfs_card.set_body(kfs_text); self.meilin_running_page.step_card.set_body(step); self.meilin_running_page.odin_card.set_body("正常" if data.get("odin_ok") else "未稳定"); self.meilin_running_page.progress.setValue(progress)
        if state == "GYM_DONE_WAIT_LIFT":
            self.stack.setCurrentWidget(self.gym_done_page)

        elif state == "RUNNING_GYM":
            self.stack.setCurrentWidget(self.gym_running_page)

        elif state == "RUNNING_MEILIN":
            self.stack.setCurrentWidget(self.meilin_running_page)

        elif state == "MATCH_DONE":
            self.stack.setCurrentWidget(self.finish_page)

        elif state == "MEILIN_FAILED":
            # 二区行为树失败后，自动回到二区准备界面，方便重新选路线/重新开始
            self.meilin_prepare_page.update_state(
                team,
                data.get("manual_block_sequence", []),
                data.get("block_has_kfs", {}),
                data.get("edit_mode", "ROUTE"),
                data.get("block_heights", {}),
            )
            self.stack.setCurrentWidget(self.meilin_prepare_page)

        elif state == "MEILIN_EXITED":
            self.meilin_prepare_page.update_state(
                team,
                data.get("manual_block_sequence", []),
                data.get("block_has_kfs", {}),
                data.get("edit_mode", "ROUTE"),
                data.get("block_heights", {}),
            )
            self.stack.setCurrentWidget(self.meilin_prepare_page)

        elif state == "GYM_FAILED":
            self.stack.setCurrentWidget(self.gym_prepare_page)

        elif state == "SYSTEM_EXITED":
            self.stack.setCurrentWidget(self.home_page)
    def on_log(self,text): self.gym_running_page.log_box.append(text); self.meilin_running_page.log_box.append(text)
    def show_error(self,text): QMessageBox.warning(self,"提示",text)


def main():
    app=QApplication(sys.argv); window=MainWindow(); window.show(); sys.exit(app.exec_())
if __name__ == "__main__": main()
