import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path
from datetime import datetime

import yaml
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

ZONE1_COMPETITION_BT_XML = "zone1_competition_task.xml"
ZONE2_COMPETITION_BT_XML = "zone2_competition_task.xml"
ZONE3_COMPETITION_BT_XML = "zone3_competition_task.xml"


class MissionManagerSim(QObject):
    state_changed = pyqtSignal(dict)
    log_emitted = pyqtSignal(str)
    error_emitted = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.state = "IDLE"
        self.current_task = "无"
        self.current_team = "UNKNOWN"
        self.active_zone = None

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

        self.system_process = None
        self.gym_bt_process = None
        self.meilin_bt_process = None
        self.zone3_bt_process = None
        self.process_log_dir = self.get_workspace_root() / "log" / "ui_process"
        self.process_log_dir.mkdir(parents=True, exist_ok=True)
        self.process_log_files = {}
        self.process_log_read_pos = {}
        self.last_bt_node_name = ""

        self.odin_ok = True
        self.lower_mcu_ok = True

        #定位状态：用于判断 map-> chassis_base_link是否可用
        self.localization_ok = False
        self.localization_status = "未检查"
        self.localization_last_error = ""
        self.localization_check_deadline = 0.0

        self.current_step = "-"
        self.assembly_count = 0
        self.target_assembly_count = 1
        self.progress = 0

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick_sequence)

        # 监控由 UI 启动的 ros2 launch / 行为树进程。
        # 行为树自己结束或失败时，UI 能自动显示结果，不需要手动点“停止任务”。
        # 同时读取行为树日志，把“当前执行到哪个行为树积木块”显示到 UI。
        self._process_monitor_timer = QTimer()
        self._process_monitor_timer.timeout.connect(self._monitor_processes)
        self._process_monitor_timer.start(500)

        # 启动系统后，周期性检测定位 TF 是否可用
        self._localization_check_timer = QTimer()
        self._localization_check_timer.timeout.connect(self._poll_localization_ready)

        self._sequence = []
        self._seq_index = 0
        self._on_sequence_done = None

        self.load_meilin_map_cache()

    # -------------------------
    # 基础路径
    # -------------------------
    def get_workspace_root(self):
        env_root = os.environ.get("R2_ALGORITHM_ROOT", "").strip()
        if env_root:
            p = Path(env_root).expanduser().resolve()
            if (p / "src" / "r2_bt_executor" / "config").exists():
                return p

        here = Path(__file__).resolve()
        for parent in [here.parent] + list(here.parents):
            if parent.name == "r2_algorithm" and (parent / "src").exists():
                return parent

        return Path.home() / "techx_R2_algorithm" / "r2_algorithm"

    def get_source_meilin_map_path(self):
        return self.get_workspace_root() / "src" / "r2_bt_executor" / "config" / "meilin_map.yaml"

    def get_install_meilin_map_path(self):
        return self.get_workspace_root() / "install" / "r2_bt_executor" / "share" / "r2_bt_executor" / "config" / "meilin_map.yaml"

    def get_field_profiles_path(self):
        return self.get_workspace_root() / "ui" / "config" / "field_profiles.yaml"

    def get_source_nav_params_path(self):
        return self.get_workspace_root() / "src" / "r2_nav_bringup" / "config" / "r2_nav_params.yaml"

    def get_install_nav_params_path(self):
        return self.get_workspace_root() / "install" / "r2_nav_bringup" / "share" / "r2_nav_bringup" / "config" / "r2_nav_params.yaml"

    def get_source_nav_goals_path(self):
        return self.get_workspace_root() / "src" / "r2_nav_bringup" / "config" / "r2_nav_goals.yaml"

    def get_install_nav_goals_path(self):
        return self.get_workspace_root() / "install" / "r2_nav_bringup" / "share" / "r2_nav_bringup" / "config" / "r2_nav_goals.yaml"

    def resolve_config_path(self, path_value: str):
        """支持绝对路径，也支持相对 r2_algorithm 根目录的路径。"""
        p = Path(str(path_value).strip()).expanduser()
        if not p.is_absolute():
            p = self.get_workspace_root() / p
        return p.resolve()

    def get_default_meilin_map_path(self):
        return str(self.get_source_meilin_map_path())

    def get_bt_xml_source_path(self, xml_file_name: str):
        return self.get_workspace_root() / "src" / "r2_bt_executor" / "config" / xml_file_name

    def get_bt_xml_install_path(self, xml_file_name: str):
        return self.get_workspace_root() / "install" / "r2_bt_executor" / "share" / "r2_bt_executor" / "config" / xml_file_name

    def check_bt_xml_ready(self, xml_file_name: str):
        install_path = self.get_bt_xml_install_path(xml_file_name)
        source_path = self.get_bt_xml_source_path(xml_file_name)

        if install_path.exists():
            return True, str(install_path)

        if source_path.exists():
            return (
                False,
                "行为树 XML 只在源码目录存在，但运行时 install/share 目录还没有。\n"
                f"源码文件：{source_path}\n"
                f"运行文件：{install_path}\n\n"
                "请先执行：\n"
                "  colcon build --symlink-install --packages-select r2_bt_executor r2_bt_bringup\n"
                "  source install/setup.bash\n"
                "或者手动复制 XML 到 install/share。",
            )

        return (
            False,
            "找不到行为树 XML：\n"
            f"源码文件：{source_path}\n"
            f"运行文件：{install_path}",
        )

    def save_yaml_to_path(self, yaml_path, data):
        yaml_path = Path(yaml_path)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, default_flow_style=False)

    def load_team_nav_profile(self, zone: str):
        """
        启动系统前，根据 UI 当前选择的 RED / BLUE 和 zone1 / zone3：
        1. 读取 ui/config/field_profiles.yaml
        2. 修改 src 和 install 里的 r2_nav_params.yaml
        3. 把对应红/蓝方 r2_nav_goals.yaml 同步到 src 和 install
        """
        if self.current_team not in ["RED", "BLUE"]:
            return False, "请先选择红方或蓝方"
        if zone not in ["zone1", "zone3"]:
            return False, "系统区域只能是 zone1 或 zone3"

        profile_key = "red" if self.current_team == "RED" else "blue"
        field_profiles_path = self.get_field_profiles_path()

        if not field_profiles_path.exists():
            return False, f"找不到红蓝方配置文件：\n{field_profiles_path}"

        try:
            with open(field_profiles_path, "r", encoding="utf-8") as f:
                profiles = yaml.safe_load(f) or {}
        except Exception as e:
            return False, f"读取 field_profiles.yaml 失败：{e}"

        if profile_key not in profiles:
            return False, f"field_profiles.yaml 中没有 {profile_key} 配置"

        team_profile = profiles.get(profile_key) or {}
        profile = team_profile.get(zone) or {}

        if not profile:
            return False, f"field_profiles.yaml 中没有 {profile_key}.{zone} 配置"

        required_keys = [
            "relocalization_bin_file",
            "relocalization_pcd_file",
            "map_package_dir",
        ]

        for key in required_keys:
            if key not in profile or not str(profile.get(key, "")).strip():
                return False, f"{profile_key}.{zone} 配置缺少字段：{key}"

        nav_goals_value = str(team_profile.get("nav_goals_file", "")).strip()
        if not nav_goals_value:
            return False, f"{profile_key} 配置缺少字段：nav_goals_file"

        relocalization_bin_file = self.resolve_config_path(profile["relocalization_bin_file"])
        relocalization_pcd_file = self.resolve_config_path(profile["relocalization_pcd_file"])
        map_package_dir = self.resolve_config_path(profile["map_package_dir"])
        nav_goals_file = self.resolve_config_path(nav_goals_value)

        # 启动前检查文件/目录是否真的存在。这样可以避免 launch 启动一半才报错。
        if not relocalization_bin_file.is_file():
            return False, f"Odin 重定位 bin 地图不存在：\n{relocalization_bin_file}"

        if not relocalization_pcd_file.is_file():
            return False, f"PCD 可视化地图不存在：\n{relocalization_pcd_file}"

        if not map_package_dir.is_dir():
            return False, f"OctoMap 地图包目录不存在：\n{map_package_dir}"

        if not nav_goals_file.is_file():
            return False, f"红/蓝方导航目标点文件不存在：\n{nav_goals_file}"

        source_nav_params_path = self.get_source_nav_params_path()
        install_nav_params_path = self.get_install_nav_params_path()

        if not source_nav_params_path.exists():
            return False, f"找不到源码导航参数文件：\n{source_nav_params_path}"

        try:
            with open(source_nav_params_path, "r", encoding="utf-8") as f:
                nav_params = yaml.safe_load(f) or {}

            nav_params.setdefault("maps", {})
            nav_params["maps"]["relocalization_bin_file"] = str(relocalization_bin_file)
            nav_params["maps"]["relocalization_pcd_file"] = str(relocalization_pcd_file)
            nav_params["maps"]["map_package_dir"] = str(map_package_dir)

            # 保存到源码目录
            self.save_yaml_to_path(source_nav_params_path, nav_params)

            # 保存到 install/share 目录
            self.save_yaml_to_path(install_nav_params_path, nav_params)

            # 同步导航目标点文件
            source_nav_goals_path = self.get_source_nav_goals_path()
            install_nav_goals_path = self.get_install_nav_goals_path()

            source_nav_goals_path.parent.mkdir(parents=True, exist_ok=True)
            install_nav_goals_path.parent.mkdir(parents=True, exist_ok=True)

            shutil.copyfile(nav_goals_file, source_nav_goals_path)
            shutil.copyfile(nav_goals_file, install_nav_goals_path)

            msg = (
                f"已加载{profile_key}方 {zone} 导航配置\n"
                f"bin地图：{relocalization_bin_file}\n"
                f"pcd地图：{relocalization_pcd_file}\n"
                f"OctoMap目录：{map_package_dir}\n"
                f"导航目标点：{nav_goals_file}"
            )
            return True, msg

        except Exception as e:
            return False, f"加载红蓝方导航配置失败：{e}"

    # -------------------------
    # UI 显示文本
    # -------------------------
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

    def emit_state(self, msg=""):
        data = {
            "state": self.state,
            "current_task": self.current_task,
            "current_team": self.current_team,
            "active_zone": self.active_zone,
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
            "localization_ok": self.localization_ok,
            "localization_status": self.localization_status,
            "localization_last_error": self.localization_last_error,
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

    # -------------------------
    # ROS2 进程管理
    # -------------------------
    def make_ros_bash_cmd(self, ros_cmd: str) -> str:
        root = self.get_workspace_root()
        return (
            f"cd {root} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            f"{ros_cmd}"
        )
    
    def check_localization_tf_once(
        self,
        target_frame: str = "map",
        source_frame: str = "chassis_base_link",
        timeout_sec: float = 0.8,
        max_age_sec: float = 2.0,
        allow_static_tf: bool = True,
    ):
        """
        检查是否能查到 target_frame -> source_frame。

        对当前工程来说：
        - target_frame = map
        - source_frame = chassis_base_link

        能查到这个 TF，说明导航和 Odin PID 至少有可用位姿。
        """
        py_code = f"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

target_frame = {target_frame!r}
source_frame = {source_frame!r}
timeout_sec = {float(timeout_sec)!r}
max_age_sec = {float(max_age_sec)!r}
allow_static_tf = {bool(allow_static_tf)!r}

rclpy.init()
node = Node("ui_localization_tf_check")
buffer = Buffer()
listener = TransformListener(buffer, node, spin_thread=False)

deadline = time.time() + timeout_sec
last_error = ""

try:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = buffer.lookup_transform(target_frame, source_frame, Time())

            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now = node.get_clock().now().nanoseconds * 1e-9

            # 静态 TF 或某些驱动可能给 stamp=0。
            # 实机动态定位时，一般应该有正常时间戳。
            if stamp > 0.0:
                age = now - stamp
                if age > max_age_sec:
                    print(f"STALE TF age={{age:.3f}}s > {{max_age_sec:.3f}}s")
                    sys.exit(2)
            else:
                if not allow_static_tf:
                    print("TF exists, but stamp=0. It may be static/fake TF.")
                    sys.exit(3)

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z
            print(f"OK {{target_frame}} -> {{source_frame}} x={{x:.3f}} y={{y:.3f}} z={{z:.3f}}")
            sys.exit(0)

        except Exception as e:
            last_error = str(e)

    print(f"NO TF {{target_frame}} -> {{source_frame}}, last_error={{last_error}}")
    sys.exit(1)

finally:
    node.destroy_node()
    rclpy.shutdown()
"""

        ros_cmd = "python3 - <<'PY'\n" + py_code + "\nPY"

        try:
            result = subprocess.run(
                ["bash", "-lc", self.make_ros_bash_cmd(ros_cmd)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_sec + 2.0,
                check=False,
            )
            output = (result.stdout or "").strip()

            if result.returncode == 0:
                return True, output

            return False, output or f"TF 检查失败，returncode={result.returncode}"

        except subprocess.TimeoutExpired:
            return False, f"TF 检查超时：{target_frame} -> {source_frame}"
        except Exception as e:
            return False, f"TF 检查异常：{e}"

    def start_wait_localization(self, wait_timeout_sec: float = 20.0):
        """
        总系统启动后调用。
        在 wait_timeout_sec 时间内持续等待 map -> chassis_base_link。
        """
        self.localization_ok = False
        self.localization_status = "等待定位"
        self.localization_last_error = ""
        self.localization_check_deadline = time.time() + wait_timeout_sec

        self.system_ready = False
        self.system_started = True
        self.system_starting = False
        self.state = "SYSTEM_WAIT_LOCALIZATION"
        self.system_step = "系统已启动，正在等待定位 map -> chassis_base_link"
        self.current_step = self.system_step
        self.system_progress = 80
        self.progress = 80

        self.emit_state("系统已启动，正在等待定位成功后再允许开始任务")
        self._localization_check_timer.start(1000)

    def _poll_localization_ready(self):
        """
        QTimer 周期调用。
        定位成功后，把 system_ready 置为 True。
        """
        if not self.system_started or not self.is_process_running(self.system_process):
            self._localization_check_timer.stop()
            self.localization_ok = False
            self.localization_status = "系统未运行"
            self.localization_last_error = "总系统进程未运行"
            return

        ok, msg = self.check_localization_tf_once(
            target_frame="map",
            source_frame="chassis_base_link",
            timeout_sec=0.6,
            max_age_sec=2.0,
            allow_static_tf=True,
        )

        if ok:
            self._localization_check_timer.stop()

            self.localization_ok = True
            self.localization_status = "定位成功"
            self.localization_last_error = ""

            self.system_ready = True
            self.system_started = True
            self.system_starting = False
            self.state = "SYSTEM_READY"
            self.system_step = "系统已启动，定位成功"
            self.current_step = "系统已启动，定位成功，可以开始任务"
            self.system_progress = 100
            self.progress = 100

            self.load_meilin_map_cache()
            self.emit_state(f"定位成功：{msg}")
            return

        self.localization_ok = False
        self.localization_status = "等待定位"
        self.localization_last_error = msg

        if time.time() > self.localization_check_deadline:
            self._localization_check_timer.stop()

            self.localization_status = "定位失败"
            self.system_ready = False
            self.state = "SYSTEM_WAIT_LOCALIZATION"
            self.system_step = "系统已启动，但定位未成功"
            self.current_step = "系统已启动，但定位未成功，不能开始任务"
            self.system_progress = 80
            self.progress = 80

            self.emit_state("定位失败：未检测到 map -> chassis_base_link")
            self.error_emitted.emit(
                "系统已经启动，但当前还没有检测到定位成功。\n\n"
                "判断标准：TF 中能查到 map -> chassis_base_link。\n\n"
                f"最近一次错误：\n{msg}\n\n"
                "请检查 Odin 是否启动、地图是否加载、重定位是否成功。"
            )
            return

        self.current_step = "等待定位中：map -> chassis_base_link"
        self.system_step = self.current_step
        self.emit_state("等待定位中")    

    def is_process_running(self, proc) -> bool:
        return proc is not None and proc.poll() is None

    def start_ros_process(self, name: str, ros_cmd: str):
        log_file = self.process_log_dir / f"{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        bash_cmd = self.make_ros_bash_cmd(ros_cmd)
        f = open(log_file, "w", encoding="utf-8")
        proc = subprocess.Popen(
            ["bash", "-lc", bash_cmd],
            stdout=f,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        self.process_log_files[name] = log_file
        self.process_log_read_pos[name] = 0
        self.log(f"已启动进程 {name}，pid={proc.pid}")
        self.log(f"日志文件：{log_file}")
        return proc

    def read_process_log_tail(self, name: str, max_chars: int = 6000) -> str:
        log_file = self.process_log_files.get(name)
        if not log_file:
            return ""

        log_file = Path(log_file)
        if not log_file.exists():
            return ""

        try:
            text = log_file.read_text(encoding="utf-8", errors="ignore")
            return text[-max_chars:]
        except Exception as e:
            return f"读取日志失败：{e}"

    # -------------------------
    # 行为树日志 -> UI 当前步骤
    # -------------------------
    def bt_node_display_name(self, node_name: str) -> str:
        """把行为树节点名转换成 UI 上更容易看懂的中文说明。"""
        mapping = {
            "R2GetRouteFromYamlNode": "读取二区路线配置",
            "R2SetBlackboardStringNode": "设置当前方块",
            "R2SetBlackboardIntNode": "设置高度/路线索引",
            "R2CheckRouteFinishedNode": "检查路线是否完成",
            "R2GetNextManualBlockNode": "获取下一个目标方块",
            "R2GetTransitionInfoFromYamlNode": "读取方块间过渡信息",
            "R2CheckOdinLocalizationOkMockNode": "检查 Odin 定位状态",
            "R2NavigateToPoseActionNode": "导航到目标接近点",
            "R2OdinPosePidAlignActionNode": "执行 Odin PID 精对准",
            "R2GetBlockHeightFromYamlNode": "读取目标方块高度",
            "R2CheckBlockHasKfsFromYamlNode": "判断目标方块是否有 KFS",
            "R2BlackboardCheckBoolNode": "判断条件是否满足",
            "R2GetBlockKfsHeightFromYamlNode": "读取 KFS 高度",
            "R2CalculateHeightDeltaNode": "计算高度差",
            "R2BuildKfsPickActionIdFromYamlNode": "生成 KFS 吸取动作编号",
            "R2GetArmActionConfigFromYamlNode": "读取机械臂动作配置",
            "R2SetEndEffectorNode": "控制吸盘/气泵",
            "R2ExecuteArmActionNode": "执行机械臂动作",
            "R2BuildChassisCmdTypeFromYamlNode": "生成底盘爬台阶指令",
            "R2ChassisStepCommandNode": "执行底盘爬台阶动作",
            "R2IncrementIntNode": "更新路线索引",
            "R2BlackboardCheckStringNode": "检查是否到达终点",
            "R2ForceSuccess": "强制返回成功",
            "R2WaitForever": "等待",
        }
        return mapping.get(node_name, node_name)

    def extract_bt_node_name_from_log_line(self, line: str):
        """从一行行为树日志里提取 R2xxxNode / R2xxxActionNode 名字。"""
        match = re.search(r"\[(R2[A-Za-z0-9_]*(?:Node|ActionNode|MockNode|Success|Forever))\]", line)
        if match:
            return match.group(1)
        return None

    def handle_bt_log_line_for_ui(self, task_name: str, line: str):
        """根据行为树日志实时更新 UI 当前步骤。"""
        node_name = self.extract_bt_node_name_from_log_line(line)
        if not node_name:
            return

        # 避免同一个节点在日志里连续打印时重复刷新 UI。
        if node_name == self.last_bt_node_name:
            return

        self.last_bt_node_name = node_name
        display_name = self.bt_node_display_name(node_name)

        # 尝试把日志里的关键信息也带到 UI 上，方便你一眼看懂当前目标。
        detail = ""
        if "to_block=" in line:
            m = re.search(r"to_block=([A-Za-z0-9_]+)", line)
            if m:
                detail = f"\n目标方块：{m.group(1)}"
        elif "goal_name=" in line:
            m = re.search(r"goal_name=([A-Za-z0-9_]+)", line)
            if m:
                detail = f"\n导航目标：{m.group(1)}"
        elif "from_block=" in line and "to_block=" in line:
            m1 = re.search(r"from_block=([A-Za-z0-9_]+)", line)
            m2 = re.search(r"to_block=([A-Za-z0-9_]+)", line)
            if m1 and m2:
                detail = f"\n路径：{m1.group(1)} → {m2.group(1)}"

        if task_name == "meilin_bt":
            self.current_step = f"正在执行：{display_name}\n节点：{node_name}{detail}"
            self.emit_state(self.current_step)

        elif task_name == "gym_bt":
            self.current_step = f"一区正在执行：{display_name}\n节点：{node_name}{detail}"
            self.emit_state(self.current_step)

        elif task_name == "zone3_bt":
            self.current_step = f"三区正在执行：{display_name}\n节点：{node_name}{detail}"
            self.emit_state(self.current_step)

    def poll_process_log_new_lines(self, name: str, max_read_chars: int = 12000):
        """只读取某个进程日志中新增加的内容，用来实时更新 UI。"""
        log_file = self.process_log_files.get(name)
        if not log_file:
            return

        log_file = Path(log_file)
        if not log_file.exists():
            return

        try:
            last_pos = self.process_log_read_pos.get(name, 0)
            file_size = log_file.stat().st_size

            # 如果日志被清空或重建，重新从头读。
            if last_pos > file_size:
                last_pos = 0

            with open(log_file, "r", encoding="utf-8", errors="ignore") as f:
                f.seek(last_pos)
                new_text = f.read(max_read_chars)
                self.process_log_read_pos[name] = f.tell()

            if not new_text:
                return

            for line in new_text.splitlines():
                self.handle_bt_log_line_for_ui(name, line)

        except Exception as e:
            self.log(f"读取 {name} 实时日志失败：{e}")

    def bt_log_has_failure(self, text: str) -> bool:
        failure_keywords = [
            "Behavior tree finished with FAILURE",
            "Action did not succeed",
            "finished with FAILURE",
            "FAILED",
            "ERROR",
        ]
        return any(k in text for k in failure_keywords)

    def bt_log_has_success(self, text: str) -> bool:
        success_keywords = [
            "Behavior tree finished with SUCCESS",
            "finished with SUCCESS",
        ]
        return any(k in text for k in success_keywords)

    def _monitor_processes(self):
        # 行为树运行时，实时读取日志，把当前执行到的节点显示到 UI。
        if self.meilin_bt_process is not None and self.meilin_bt_process.poll() is None:
            self.poll_process_log_new_lines("meilin_bt")

        if self.gym_bt_process is not None and self.gym_bt_process.poll() is None:
            self.poll_process_log_new_lines("gym_bt")

        if self.zone3_bt_process is not None and self.zone3_bt_process.poll() is None:
            self.poll_process_log_new_lines("zone3_bt")

        self._monitor_meilin_bt_process()
        self._monitor_gym_bt_process()
        self._monitor_zone3_bt_process()
        self._monitor_system_process()

    def _monitor_meilin_bt_process(self):
        proc = self.meilin_bt_process
        if proc is None or proc.poll() is None:
            return

        # 进程已经退出时，再补读最后一次日志，避免漏掉最后一个节点。
        self.poll_process_log_new_lines("meilin_bt")

        returncode = proc.returncode
        tail = self.read_process_log_tail("meilin_bt")
        self.meilin_bt_process = None

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if returncode != 0 or self.bt_log_has_failure(tail):
            self.state = "MEILIN_FAILED"
            self.current_step = "二区行为树失败"
            if "Action did not succeed" in tail:
                self.odin_ok = False
            self.emit_state(
                f"二区行为树失败：returncode={returncode}。请查看 meilin_bt 日志。"
            )
            self.error_emitted.emit(
                "二区行为树已经失败并退出。\n\n"
                f"returncode={returncode}\n"
                "常见原因：导航失败、Odin/TF 不可用、目标点不可达。\n\n"
                "UI 已自动返回二区准备界面，当前任务已停止。"
            )
            return

        if self.bt_log_has_success(tail) or returncode == 0:
            self.state = "MATCH_DONE"
            self.current_step = "二区完成"
            self.emit_state("二区行为树成功结束")
            return

        self.state = "MEILIN_EXITED"
        self.current_step = "二区行为树已退出"
        self.emit_state(f"二区行为树已退出：returncode={returncode}")

    def _monitor_gym_bt_process(self):
        proc = self.gym_bt_process
        if proc is None or proc.poll() is None:
            return

        self.poll_process_log_new_lines("gym_bt")

        returncode = proc.returncode
        tail = self.read_process_log_tail("gym_bt")
        self.gym_bt_process = None

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if returncode != 0 or self.bt_log_has_failure(tail):
            self.state = "GYM_FAILED"
            self.current_step = "一区行为树失败"
            self.emit_state(f"一区行为树失败：returncode={returncode}")
            self.error_emitted.emit("一区行为树已经失败并退出。请查看 gym_bt 日志。")
            return

        self.state = "GYM_DONE_WAIT_LIFT"
        self.assembly_count = 1
        self.current_step = "一区完成，请抬回重试区"
        self.emit_state("一区行为树成功结束")

    def _monitor_zone3_bt_process(self):
        proc = self.zone3_bt_process
        if proc is None or proc.poll() is None:
            return

        self.poll_process_log_new_lines("zone3_bt")

        returncode = proc.returncode
        tail = self.read_process_log_tail("zone3_bt")
        self.zone3_bt_process = None

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if returncode != 0 or self.bt_log_has_failure(tail):
            self.state = "ZONE3_FAILED"
            self.current_step = "三区行为树失败"
            self.emit_state(f"三区行为树失败：returncode={returncode}")
            self.error_emitted.emit("三区行为树已经失败并退出。请查看 zone3_bt 日志。")
            return

        self.state = "ZONE3_DONE"
        self.current_step = "三区任务完成"
        self.emit_state("三区行为树成功结束")

    def _monitor_system_process(self):
        proc = self.system_process
        if proc is None or proc.poll() is None:
            return

        returncode = proc.returncode
        self.system_process = None

        if self.system_started or self.system_ready or self.system_starting:
            self.system_started = False
            self.system_ready = False
            self.system_starting = False
            self.system_progress = 0
            self.active_zone = None
            self.active_profile = None

            if self.state not in ["IDLE", "STOPPED"]:
                self.state = "SYSTEM_EXITED"
                self.current_step = "总系统已退出"
                self.system_step = "总系统已退出"
                self.emit_state(f"总 launch 已退出：returncode={returncode}")

    def stop_process(self, proc, name: str, sigint_timeout_sec: float = 5.0):
        if proc is None:
            return
        if proc.poll() is not None:
            self.log(f"{name} 已退出，returncode={proc.returncode}")
            return

        try:
            self.log(f"正在 Ctrl+C 停止 {name} ...")
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=sigint_timeout_sec)
                self.log(f"{name} 已正常退出，returncode={proc.returncode}")
                return
            except subprocess.TimeoutExpired:
                self.log(f"{name} Ctrl+C 后未退出，发送 SIGTERM ...")
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

            try:
                proc.wait(timeout=2.0)
                self.log(f"{name} 已通过 SIGTERM 退出，returncode={proc.returncode}")
                return
            except subprocess.TimeoutExpired:
                self.log(f"{name} SIGTERM 后仍未退出，发送 SIGKILL ...")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
                self.log(f"{name} 已强制退出，returncode={proc.returncode}")
        except Exception as e:
            self.log(f"停止 {name} 失败：{e}")

    def publish_zero_cmd_vel(self):
        ros_cmd = (
            "ros2 topic pub -r 20 --times 10 "
            "/cmd_vel geometry_msgs/msg/Twist "
            "'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
        )
        try:
            subprocess.run(["bash", "-lc", self.make_ros_bash_cmd(ros_cmd)], timeout=3.0, check=False)
            self.log("已连续发布 /cmd_vel=0")
        except Exception as e:
            self.log(f"发布 /cmd_vel=0 失败：{e}")

    def call_chassis_estop(self):
        ros_cmd = (
            "ros2 service call "
            "/r2_chassis/estop "
            "techx_r2_chassis_interfaces/srv/EStop "
            "'{trigger: true}'"
        )
        try:
            subprocess.run(["bash", "-lc", self.make_ros_bash_cmd(ros_cmd)], timeout=3.0, check=False)
            self.log("已调用 /r2_chassis/estop")
        except Exception as e:
            self.log(f"调用 /r2_chassis/estop 失败：{e}")

    def open_gripper_claw(self):
        ros_cmd = (
            "python3 ui/open_gripper_claw_until_feedback.py "
            "--target-id 4 "
            "--action-id 1025 "
            "--timeout-ms 3000 "
            "--param 0 "
            "--flags 0 "
            "--server-timeout 1.0 "
            "--goal-response-timeout 1.0 "
            "--feedback-timeout 1.0"
        )
        try:
            result = subprocess.run(
                ["bash", "-lc", self.make_ros_bash_cmd(ros_cmd)],
                timeout=4.0,
                check=False,
                capture_output=True,
                text=True,
            )
            output = "\n".join(
                part.strip()
                for part in (result.stdout, result.stderr)
                if part and part.strip()
            )
            if result.returncode == 0:
                if output:
                    self.log(f"夹爪打开首帧反馈：{output.splitlines()[-1]}")
                self.log("已下发夹爪打开命令：target_id=4 action_id=1025，不等待最终结果")
            else:
                self.log(
                    "夹爪打开命令未确认首帧反馈："
                    f"returncode={result.returncode}"
                    + (f"，{output.splitlines()[-1]}" if output else "")
                )
        except subprocess.TimeoutExpired:
            self.log("夹爪打开命令等待首帧反馈超时：请检查 /r2_arm/execute_action 是否可用或机械臂是否 busy")
        except Exception as e:
            self.log(f"夹爪打开命令失败：{e}")

    # -------------------------
    # 旧模拟进度保留，但真实流程不依赖它
    # -------------------------
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

    # -------------------------
    # meilin_map.yaml 缓存与保存
    # -------------------------
    def load_meilin_map_cache(self):
        yaml_path = self.get_default_meilin_map_path()
        if not os.path.exists(yaml_path):
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

            route_blocks = (((data.get("routes", {}) or {}).get("zone2_main", {}) or {}).get("blocks", []) or [])
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
                data["blocks"][block_name]["has_kfs"] = bool(self.block_has_kfs.get(i, False))

            if "ENTRY" in data["blocks"]:
                data["blocks"]["ENTRY"]["has_kfs"] = False
            if "EXIT_ZONE" in data["blocks"]:
                data["blocks"]["EXIT_ZONE"]["has_kfs"] = False

            self.save_yaml_to_path(source_yaml_path, data)
            self.save_yaml_to_path(install_yaml_path, data)

            msg = (
                "已保存二区配置\n\n"
                f"源码配置：\n{source_yaml_path}\n\n"
                f"运行配置：\n{install_yaml_path}\n\n"
                f"当前路线：{self.manual_route_text}\n"
                f"KFS：{self.kfs_text}"
            )
            self.emit_state(f"已保存二区配置：路线 {self.manual_route_text}，KFS：{self.kfs_text}")
            return True, msg
        except Exception as e:
            return False, str(e)

    # -------------------------
    # UI 按钮行为
    # -------------------------
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
            self.active_zone = None
            self.state = "IDLE"
            self.current_step = "-"
            self.system_step = "未启动"
            self.system_progress = 0
            self.active_profile = None
            self.progress = 0
            self.emit_state("已取消队伍选择")
            return

        self.current_team = team
        self.active_zone = None
        self.state = "TEAM_SELECTED"
        self.current_step = f"已选择{'红方' if team == 'RED' else '蓝方'}"
        self.system_step = "等待启动系统"
        self.system_progress = 0
        self.progress = 0
        self.emit_state(f"已选择{'红方' if team == 'RED' else '蓝方'}，请点击启动系统")

    def start_system(self, zone="zone1"):
        if self.tree_running:
            self.error_emitted.emit("任务运行中不能启动系统")
            return
        if self.system_starting:
            self.error_emitted.emit("系统正在启动中")
            return
        if self.system_ready or self.is_process_running(self.system_process):
            self.error_emitted.emit("系统已经启动或准备完成；如需重启，请先复位")
            return
        if self.current_team not in ["RED", "BLUE"]:
            self.error_emitted.emit("请先选择红方或蓝方")
            return
        if zone not in ["zone1", "zone3"]:
            self.error_emitted.emit("系统区域只能是 zone1 或 zone3")
            return

        team_name = "红方" if self.current_team == "RED" else "蓝方"
        profile_dir = "red" if self.current_team == "RED" else "blue"
        zone_name = "一区" if zone == "zone1" else "三区"

        ok, profile_msg = self.load_team_nav_profile(zone)
        if not ok:
            self.error_emitted.emit(profile_msg)
            self.emit_state("加载红蓝方导航配置失败")
            return
        self.log(profile_msg)

        self.active_zone = zone
        self.active_profile = f"{profile_dir}.{zone}"
        self.system_started = True
        self.system_ready = False
        self.system_starting = True
        self.system_step = f"正在启动 {team_name}{zone_name}总系统"
        self.state = "SYSTEM_STARTING"
        self.current_task = "r2_task_mock_bringup.launch.py"
        self.current_step = self.system_step
        self.system_progress = 10
        self.progress = 10
        self.emit_state(f"开始启动系统：队伍={profile_dir}，区域={zone}")

        try:
            self.system_process = self.start_ros_process(
                "system",
                "ros2 launch r2_bt_bringup r2_task_mock_bringup.launch.py",
            )

            self.current_task = "无"
            self.start_wait_localization(wait_timeout_sec=20.0)

        except Exception as e:
            self.system_process = None
            self.system_starting = False
            self.system_ready = False
            self.system_started = False
            self.system_progress = 0
            self.progress = 0
            self.current_task = "无"
            self.active_zone = None
            self.active_profile = None
            self.state = "IDLE"
            self.system_step = "启动失败"
            self.current_step = "启动失败"
            self.error_emitted.emit(f"启动系统失败：{e}")
            self.emit_state("启动系统失败")

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

    def start_gym(self):
        if self.tree_running:
            self.error_emitted.emit("当前已有任务正在运行")
            return
        if not self.system_ready:
            self.error_emitted.emit("系统尚未准备完成，不能开始一区")
            return
        if self.active_zone != "zone1":
            self.error_emitted.emit("当前不是一区系统，不能开始一区任务")
            return
        if not self.localization_ok:
            ok, msg = self.check_localization_tf_once(
                target_frame="map",
                source_frame="chassis_base_link",
                timeout_sec=1.0,
                max_age_sec=2.0,
                allow_static_tf=True,
            )
            if not ok:
                self.localization_ok = False
                self.localization_status = "定位失败"
                self.localization_last_error = msg
                self.emit_state("定位未成功，不能开始一区")
                self.error_emitted.emit(
                    "当前定位未成功，不能开始一区任务。\n\n"
                    "判断标准：TF 中能查到 map -> chassis_base_link。\n\n"
                    f"错误信息：\n{msg}"
                )
                return

            self.localization_ok = True
            self.localization_status = "定位成功"
            self.localization_last_error = ""

        xml_file_name = ZONE1_COMPETITION_BT_XML
        ok, info = self.check_bt_xml_ready(xml_file_name)
        if not ok:
            self.error_emitted.emit(info)
            return

        try:
            self.last_bt_node_name = ""
            self.gym_bt_process = self.start_ros_process(
                "gym_bt",
                f"ros2 launch r2_bt_bringup run_bt.launch.py xml_file_name:={xml_file_name}",
            )
            self.state = "RUNNING_GYM"
            self.current_task = xml_file_name
            self.tree_running = True
            self.assembly_count = 0
            self.target_assembly_count = 1
            self.current_step = "一区行为树已启动"
            self.progress = 0
            self.emit_state(f"一区行为树已启动：{xml_file_name}")
        except Exception as e:
            self.gym_bt_process = None
            self.tree_running = False
            self.current_task = "无"
            self.error_emitted.emit(f"启动一区行为树失败：{e}")

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
        if self.active_zone != "zone1":
            self.error_emitted.emit("当前不是一区系统，不能开始二区任务")
            return
        if not self.manual_block_sequence:
            self.error_emitted.emit("请先选择至少一个梅林方块")
            return

        xml_file_name = ZONE2_COMPETITION_BT_XML
        ok, info = self.check_bt_xml_ready(xml_file_name)
        if not ok:
            self.error_emitted.emit(info)
            return

        ok, msg = self.save_meilin_config()
        if not ok:
            self.error_emitted.emit(f"启动二区前保存配置失败：{msg}")
            return

        try:
            self.last_bt_node_name = ""
            self.meilin_bt_process = self.start_ros_process(
                "meilin_bt",
                f"ros2 launch r2_bt_bringup run_bt.launch.py xml_file_name:={xml_file_name}",
            )
            self.state = "RUNNING_MEILIN"
            self.current_task = xml_file_name
            self.tree_running = True
            self.progress = 0
            self.current_step = "二区行为树已启动"
            self.emit_state(f"二区行为树已启动：路线 {self.manual_route_text}，KFS {self.kfs_text}")
        except Exception as e:
            self.meilin_bt_process = None
            self.tree_running = False
            self.current_task = "无"
            self.error_emitted.emit(f"启动二区行为树失败：{e}")

    def start_zone3(self):
        if self.tree_running:
            self.error_emitted.emit("当前已有任务正在运行")
            return False
        if self.current_team not in ["RED", "BLUE"]:
            self.error_emitted.emit("请先在主页选择红方或蓝方")
            return False
        if not self.system_ready:
            self.error_emitted.emit("系统尚未准备完成，不能开始三区")
            return False
        if self.active_zone != "zone3":
            self.error_emitted.emit("当前不是三区系统，不能开始三区任务")
            return False

        xml_file_name = ZONE3_COMPETITION_BT_XML
        ok, info = self.check_bt_xml_ready(xml_file_name)
        if not ok:
            self.error_emitted.emit(info)
            return False

        try:
            self.last_bt_node_name = ""
            self.zone3_bt_process = self.start_ros_process(
                "zone3_bt",
                f"ros2 launch r2_bt_bringup run_bt.launch.py xml_file_name:={xml_file_name}",
            )
            self.state = "RUNNING_ZONE3"
            self.current_task = xml_file_name
            self.tree_running = True
            self.progress = 0
            self.current_step = "三区行为树已启动"
            self.emit_state(f"三区行为树已启动：{xml_file_name}")
            return True
        except Exception as e:
            self.zone3_bt_process = None
            self.tree_running = False
            self.current_task = "无"
            self.error_emitted.emit(f"启动三区行为树失败：{e}")
            return False

    def _meilin_done(self):
        self.tree_running = False
        self.current_task = "无"
        self.state = "MATCH_DONE"
        self.current_step = "任务完成"
        self.progress = 100
        self.emit_state("二区完成")

    def stop(self):
        self._timer.stop()
        self.log("执行急停：先发布 /cmd_vel=0")
        self.publish_zero_cmd_vel()
        self.log("执行急停：打开夹爪")
        self.open_gripper_claw()
        self.log("执行急停：调用 /r2_chassis/estop")
        self.call_chassis_estop()
        self.log("执行急停：停止当前行为树")
        self.stop_process(self.meilin_bt_process, "meilin_bt_process")
        self.stop_process(self.gym_bt_process, "gym_bt_process")
        self.stop_process(self.zone3_bt_process, "zone3_bt_process")
        self.meilin_bt_process = None
        self.gym_bt_process = None
        self.zone3_bt_process = None
        self.tree_running = False
        self.system_starting = False
        self.current_task = "无"
        self.state = "STOPPED"
        self.current_step = "已急停"
        self.progress = 0
        self.emit_state("已急停：底盘已停止，当前行为树已停止，总系统仍保持启动")

    def reset(self):
        self._timer.stop()
        self.log("执行复位：先发布 /cmd_vel=0")
        self.publish_zero_cmd_vel()
        self.log("执行复位：打开夹爪")
        self.open_gripper_claw()
        time.sleep(0.2)
        self.log("执行复位：停止行为树")
        self.stop_process(self.meilin_bt_process, "meilin_bt_process")
        self.stop_process(self.gym_bt_process, "gym_bt_process")
        self.stop_process(self.zone3_bt_process, "zone3_bt_process")
        self.meilin_bt_process = None
        self.gym_bt_process = None
        self.zone3_bt_process = None
        self.log("执行复位：Ctrl+C 总 launch")
        self.stop_process(self.system_process, "system_process")
        self.system_process = None

        self.state = "IDLE"
        self.current_task = "无"
        self.current_team = "UNKNOWN"
        self.active_zone = None
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
        self.last_bt_node_name = ""
        self.load_meilin_map_cache()
        self.emit_state("系统已复位：总 launch 已关闭，UI 已回到初始状态")
