import os
from pathlib import Path
from datetime import datetime

import yaml
from PyQt5.QtCore import QObject, pyqtSignal, QTimer


class MissionManagerSim(QObject):
    state_changed = pyqtSignal(dict)
    log_emitted = pyqtSignal(str)
    error_emitted = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.state = "IDLE"
        self.current_task = "无"
        self.current_team = "UNKNOWN"

        self.manual_block_sequence = []
        self.block_has_kfs = {i: False for i in range(1, 13)}
        self.block_heights = {i: 0 for i in range(1, 13)}
        self.edit_mode = "ROUTE"

        self.tree_running = False
        self.system_started = False
        self.system_ready = False
        self.system_starting = False
        self.system_step = "未启动"
        self.system_progress = 0
        self.active_profile = None

        self.odin_ok = True
        self.lower_mcu_ok = True
        self.current_step = "-"
        self.assembly_count = 0
        self.target_assembly_count = 1
        self.progress = 0

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick_sequence)
        self._sequence = []
        self._seq_index = 0
        self._on_sequence_done = None

        self.load_meilin_map_cache()


    def get_source_meilin_map_path(self):
        return (
            self.get_workspace_root()
            / "src"
            / "r2_bt_executor"
            / "config"
            / "meilin_map.yaml"
        )


    def get_install_meilin_map_path(self):
        return (
            self.get_workspace_root()
            / "install"
            / "r2_bt_executor"
            / "share"
            / "r2_bt_executor"
            / "config"
            / "meilin_map.yaml"
        )


    def save_yaml_to_path(self, yaml_path, data):
        yaml_path = Path(yaml_path)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)

        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                data,
                f,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            )

    @property
    def manual_route_text(self):
        if not self.manual_block_sequence:
            return "未选择"
        route_blocks = [f"B{x}" for x in self.manual_block_sequence]
        if "EXIT_ZONE" not in route_blocks:
            route_blocks.append("EXIT_ZONE")
        return " → ".join(route_blocks)

    @property
    def kfs_text(self):
        selected = [f"B{i}" for i in range(1, 13) if self.block_has_kfs.get(i, False)]
        return "，".join(selected) if selected else "未标记"

    def get_workspace_root(self):
        """
        优先从 UI 文件所在位置反推工程根目录。
        你的 UI 路径是：
            /home/xie/techx_R2_algorithm/r2_algorithm/ui
        所以 mission_manager_sim.py 的上一级目录的上一级就是 r2_algorithm。
        """
        env_root = os.environ.get("R2_ALGORITHM_ROOT", "").strip()
        if env_root:
            p = Path(env_root).expanduser().resolve()
            if (p / "src" / "r2_bt_executor" / "config").exists():
                return p

        here = Path(__file__).resolve()
        for parent in [here.parent] + list(here.parents):
            if parent.name == "r2_algorithm" and (parent / "src").exists():
                return parent

        fallback = Path.home() / "techx_R2_algorithm" / "r2_algorithm"
        return fallback

    def get_default_meilin_map_path(self):
        root = self.get_workspace_root()
        source_yaml = root / "src" / "r2_bt_executor" / "config" / "meilin_map.yaml"
        return str(source_yaml)

    def load_meilin_map_cache(self):
        yaml_path = self.get_default_meilin_map_path()
        if not os.path.exists(yaml_path):
            # 找不到 YAML 时只给 UI 一个默认显示，不保存。
            self.block_heights = {
                1: 400, 2: 200, 3: 400,
                4: 200, 5: 400, 6: 600,
                7: 400, 8: 600, 9: 400,
                10: 200, 11: 400, 12: 200,
            }
            self.block_has_kfs = {i: False for i in range(1, 13)}
            return

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            blocks = data.get("blocks", {}) or {}
            for i in range(1, 13):
                info = blocks.get(f"B{i}", {}) or {}
                self.block_heights[i] = int(info.get("height", 0))
                self.block_has_kfs[i] = bool(info.get("has_kfs", False))

            route_blocks = (
                ((data.get("routes", {}) or {}).get("zone2_main", {}) or {})
                .get("blocks", []) or []
            )

            self.manual_block_sequence = []
            for name in route_blocks:
                if isinstance(name, str) and name.startswith("B"):
                    try:
                        idx = int(name[1:])
                    except ValueError:
                        continue
                    if 1 <= idx <= 12:
                        self.manual_block_sequence.append(idx)

        except Exception as e:
            self.log(f"读取 meilin_map.yaml 失败：{e}")

    def emit_state(self, msg=""):
        data = {
            "state": self.state,
            "current_task": self.current_task,
            "current_team": self.current_team,
            "manual_block_sequence": list(self.manual_block_sequence),
            "manual_route_text": self.manual_route_text,
            "block_has_kfs": dict(self.block_has_kfs),
            "block_heights": dict(self.block_heights),
            "edit_mode": self.edit_mode,
            "kfs_text": self.kfs_text,
            "tree_running": self.tree_running,
            "system_started": self.system_started,
            "system_ready": self.system_ready,
            "system_starting": self.system_starting,
            "system_step": self.system_step,
            "system_progress": self.system_progress,
            "active_profile": self.active_profile,
            "odin_ok": self.odin_ok,
            "lower_mcu_ok": self.lower_mcu_ok,
            "current_step": self.current_step,
            "assembly_count": self.assembly_count,
            "target_assembly_count": self.target_assembly_count,
            "progress": self.progress,
            "message": msg,
        }
        self.state_changed.emit(data)
        if msg:
            self.log(msg)

    def log(self, text):
        self.log_emitted.emit(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")

    def _start_sequence(self, steps, on_done):
        self._timer.stop()
        self._sequence = steps
        self._seq_index = 0
        self._on_sequence_done = on_done
        self.progress = 0
        self._timer.start(800)

    def _start_system_sequence(self, steps, on_done):
        self._timer.stop()
        self._sequence = steps
        self._seq_index = 0
        self._on_sequence_done = on_done
        self.system_progress = 0
        self.progress = 0
        self._timer.start(700)

    def _tick_sequence(self):
        if self._seq_index >= len(self._sequence):
            self._timer.stop()
            if self._on_sequence_done:
                self._on_sequence_done()
            return

        step = self._sequence[self._seq_index]
        self.current_step = step

        if self.state == "SYSTEM_STARTING":
            self.system_step = step
            self.system_progress = int((self._seq_index + 1) / len(self._sequence) * 100)
            self.progress = self.system_progress
        else:
            self.progress = int((self._seq_index + 1) / len(self._sequence) * 100)

        self.emit_state(step)
        self._seq_index += 1

    def start_system(self):
        if self.tree_running:
            self.error_emitted.emit("任务运行中不能启动系统")
            return
        if self.system_starting:
            self.error_emitted.emit("系统正在启动中")
            return
        if self.system_ready:
            self.error_emitted.emit("系统已经准备完成")
            return
        if self.current_team not in ["RED", "BLUE"]:
            self.error_emitted.emit("请先选择红方或蓝方")
            return

        team_name = "红方" if self.current_team == "RED" else "蓝方"
        profile_dir = "red" if self.current_team == "RED" else "blue"
        self.active_profile = profile_dir
        self.system_started = True
        self.system_ready = False
        self.system_starting = True
        self.system_step = f"准备启动 {team_name} Profile"
        self.state = "SYSTEM_STARTING"
        self.current_task = "r2_task_real_bringup.launch.py"
        self.current_step = self.system_step
        self.emit_state(f"开始启动系统：team:={profile_dir}")

        self._start_system_sequence(
            [
                f"读取 field_profiles.yaml，选择 {profile_dir} Profile",
                "读取 meilin_map.yaml",
                "启动底盘服务端",
                "启动机械臂服务端",
                "启动 Odin 与导航 Action",
                "等待所有 ROS2 Action ready",
                "系统准备完成",
            ],
            self._system_ready_done,
        )

    def _system_ready_done(self):
        self.system_starting = False
        self.system_ready = True
        self.system_started = True
        self.system_progress = 100
        self.current_task = "无"
        self.state = "SYSTEM_READY"
        self.system_step = "系统准备完成"
        self.current_step = "系统准备完成"
        self.load_meilin_map_cache()
        self.emit_state("系统准备完成：配置已加载")

    def select_team(self, team):
        if self.tree_running:
            self.error_emitted.emit("任务运行中不能切换红/蓝方")
            return
        if self.system_started or self.system_starting:
            self.error_emitted.emit("系统已启动后不能切换红/蓝方；如需切换，请先复位")
            return
        if team not in ["RED", "BLUE"]:
            self.error_emitted.emit("队伍只能是红方或蓝方")
            return

        if self.current_team == team:
            self.current_team = "UNKNOWN"
            self.state = "IDLE"
            self.current_step = "-"
            self.system_step = "未启动"
            self.system_progress = 0
            self.active_profile = None
            self.progress = 0
            self.emit_state("已取消队伍选择")
            return

        self.current_team = team
        self.state = "TEAM_SELECTED"
        self.current_step = f"已选择{'红方' if team == 'RED' else '蓝方'}"
        self.system_step = "等待启动系统"
        self.system_progress = 0
        self.progress = 0
        self.emit_state(f"已选择{'红方' if team == 'RED' else '蓝方'}，请点击启动系统")

    def toggle_edit_mode(self):
        if self.tree_running or self.system_starting:
            self.error_emitted.emit("任务运行中或系统启动中不能切换编辑模式")
            return
        self.edit_mode = "KFS" if self.edit_mode == "ROUTE" else "ROUTE"
        self.state = "MEILIN_EDITING"
        self.emit_state("已切换到 KFS 标记模式" if self.edit_mode == "KFS" else "已切换到路线选择模式")

    def toggle_block(self, block_id):
        if self.tree_running or self.system_starting:
            self.error_emitted.emit("任务运行中或系统启动中不能修改方块")
            return
        if block_id not in range(1, 13):
            self.error_emitted.emit("梅林方块编号必须是 1~12")
            return

        if self.edit_mode == "KFS":
            self.block_has_kfs[block_id] = not self.block_has_kfs[block_id]
            self.state = "MEILIN_EDITING"
            self.emit_state(f"B{block_id} 已设置为：{'有 KFS' if self.block_has_kfs[block_id] else '无 KFS'}")
            return

        # 路线模式：不再限制起点，也不限制相邻关系。
        if block_id in self.manual_block_sequence:
            self.manual_block_sequence.remove(block_id)
            self.emit_state(f"已移除 B{block_id}，当前路线：{self.manual_route_text}")
        else:
            self.manual_block_sequence.append(block_id)
            self.emit_state(f"已加入 B{block_id}，当前路线：{self.manual_route_text}")

        self.state = "MEILIN_EDITING"

    def clear_block_sequence(self):
        if self.tree_running:
            self.error_emitted.emit("任务运行中不能清空路线")
            return
        self.manual_block_sequence.clear()
        self.state = "MEILIN_EDITING"
        self.current_step = "已清空二区方块序列"
        self.progress = 0
        self.emit_state("已清空二区方块序列")

    def save_meilin_config(self):
        if self.tree_running or self.system_starting:
            return False, "任务运行中或系统启动中不能保存配置"

        source_yaml_path = self.get_source_meilin_map_path()
        install_yaml_path = self.get_install_meilin_map_path()

        if not source_yaml_path.exists():
            return False, f"找不到源码 meilin_map.yaml：\n{source_yaml_path}"

        try:
            with open(source_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            data.setdefault("routes", {})
            data["routes"].setdefault("zone2_main", {})

            # 如果 UI 当前路线为空：只保存 KFS，不清空原路线
            if self.manual_block_sequence:
                route_blocks = [f"B{x}" for x in self.manual_block_sequence]

                if "EXIT_ZONE" not in route_blocks:
                    route_blocks.append("EXIT_ZONE")

                data["routes"]["zone2_main"]["start_block"] = "ENTRY"
                data["routes"]["zone2_main"]["start_height"] = 0
                data["routes"]["zone2_main"]["blocks"] = route_blocks

            data.setdefault("blocks", {})

            for i in range(1, 13):
                block_name = f"B{i}"
                data["blocks"].setdefault(block_name, {})
                data["blocks"][block_name]["has_kfs"] = bool(
                    self.block_has_kfs.get(i, False)
                )

            if "ENTRY" in data["blocks"]:
                data["blocks"]["ENTRY"]["has_kfs"] = False

            if "EXIT_ZONE" in data["blocks"]:
                data["blocks"]["EXIT_ZONE"]["has_kfs"] = False

            # 1. 保存源码配置
            self.save_yaml_to_path(source_yaml_path, data)

            # 2. 同步保存到 install/share，行为树实际读取这里
            self.save_yaml_to_path(install_yaml_path, data)

            msg = (
                "已保存二区配置\n\n"
                f"源码配置：\n{source_yaml_path}\n\n"
                f"运行配置：\n{install_yaml_path}\n\n"
                f"当前路线：{self.manual_route_text}\n"
                f"KFS：{self.kfs_text}"
            )

            self.emit_state(
                f"已保存二区配置：路线 {self.manual_route_text}，KFS：{self.kfs_text}"
            )

            return True, msg

        except Exception as e:
            return False, str(e)

    def start_gym(self):
        if self.tree_running:
            self.error_emitted.emit("当前已有任务正在运行")
            return
        if not self.system_ready:
            self.error_emitted.emit("系统尚未准备完成，不能开始一区")
            return
        self.state = "RUNNING_GYM"
        self.current_task = "gym_task.xml"
        self.tree_running = True
        self.assembly_count = 0
        self.target_assembly_count = 1
        self.current_step = "初始化一区黑板"
        self.progress = 0
        self.emit_state("一区任务开始")
        self._start_sequence(
            ["初始化一区黑板", "发送 PICK_HEAD", "等待 PICK_HEAD_DONE", "等待 ASSEMBLY_DONE", "打开夹爪", "机构恢复", "计数 +1", "一区完成"],
            self._gym_done,
        )

    def _gym_done(self):
        self.tree_running = False
        self.current_task = "无"
        self.state = "GYM_DONE_WAIT_LIFT"
        self.assembly_count = 1
        self.current_step = "一区完成，请抬回重试区"
        self.progress = 100
        self.emit_state("一区完成，请抬回重试区")

    def start_meilin(self):
        if self.tree_running:
            self.error_emitted.emit("当前已有任务正在运行")
            return
        if self.current_team not in ["RED", "BLUE"]:
            self.error_emitted.emit("请先在主页选择红方或蓝方")
            return
        if not self.system_ready:
            self.error_emitted.emit("系统尚未准备完成，不能开始二区")
            return
        if not self.manual_block_sequence:
            self.error_emitted.emit("请先选择至少一个梅林方块")
            return

        self.state = "RUNNING_MEILIN"
        self.current_task = "meilin_task.xml"
        self.tree_running = True
        self.progress = 0
        self.current_step = "检查 Odin"
        self.emit_state(f"二区任务开始：路线 {self.manual_route_text}，KFS {self.kfs_text}")
        self._start_sequence(
            [
                "检查 Odin：OK",
                f"锁定路线：{self.manual_route_text}",
                f"锁定 KFS：{self.kfs_text}",
                "加载 meilin_zone2_task.xml",
                "判断下一个方块 KFS",
                "执行 KFS 预吸取或跳过",
                "下发爬台阶 / 平走指令",
                "二区完成",
            ],
            self._meilin_done,
        )

    def _meilin_done(self):
        self.tree_running = False
        self.current_task = "无"
        self.state = "MATCH_DONE"
        self.current_step = "任务完成"
        self.progress = 100
        self.emit_state("二区完成")

    def stop(self):
        self._timer.stop()
        self.tree_running = False
        self.system_starting = False
        self.current_task = "无"
        self.state = "STOPPED"
        self.current_step = "已停止"
        self.progress = 0
        self.emit_state("已停止")

    def reset(self):
        self._timer.stop()
        self.state = "IDLE"
        self.current_task = "无"
        self.current_team = "UNKNOWN"
        self.manual_block_sequence.clear()
        self.block_has_kfs = {i: False for i in range(1, 13)}
        self.edit_mode = "ROUTE"
        self.tree_running = False
        self.system_started = False
        self.system_ready = False
        self.system_starting = False
        self.system_step = "未启动"
        self.system_progress = 0
        self.active_profile = None
        self.odin_ok = True
        self.lower_mcu_ok = True
        self.current_step = "-"
        self.assembly_count = 0
        self.progress = 0
        self.load_meilin_map_cache()
        self.emit_state("系统已复位")
