import os
import re
import shlex
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import yaml
from PyQt5.QtCore import QObject, QTimer, pyqtSignal


ZONE1_COMPETITION_BT_XML = "zone1_competition_task.xml"
ZONE2_COMPETITION_BT_XML = "zone2_competition_task.xml"
ZONE3_COMPETITION_BT_XML = "zone3_competition_task.xml"

BASE_SYSTEM_LAUNCH_COMMAND = (
    "ros2 launch "
    "r2_bt_bringup "
    "r2_task_real_bringup.launch.py"
)

NORMAL_RELATIVE_MOVE_CONFIG = "relative_move.yaml"
ZONE3_RELATIVE_MOVE_CONFIG = "relative_move_zone3.yaml"

BT_LAUNCH_PACKAGE = "r2_bt_bringup"
BT_LAUNCH_FILE = "run_bt.launch.py"


class MissionManagerSim(QObject):
    state_changed = pyqtSignal(dict)
    log_emitted = pyqtSignal(str)
    error_emitted = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        # UI 状态
        self.state = "IDLE"
        self.current_task = "无"
        self.current_step = "-"
        self.progress = 0

        # 二区路线和 KFS 配置
        self.manual_block_sequence = []
        self.block_has_kfs = {
            i: False
            for i in range(1, 13)
        }
        self.block_heights = {
            i: 0
            for i in range(1, 13)
        }
        self.edit_mode = "ROUTE"

        # 基础系统状态
        self.system_started = False
        self.system_ready = False
        self.system_starting = False
        self.system_step = "未启动"
        self.system_progress = 0

        # normal：一区/二区普通速度
        # zone3：三区爬坡高速
        self.system_profile = None

        # 行为树状态
        self.tree_running = False
        self.assembly_count = 0
        self.target_assembly_count = 1
        self.last_bt_node_name = ""

        # 进程对象
        self.system_process = None
        self.gym_bt_process = None
        self.meilin_bt_process = None
        self.zone3_bt_process = None

        # 日志管理
        self.process_log_dir = (
            self.get_workspace_root()
            / "log"
            / "ui_process"
        )
        self.process_log_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.process_log_files = {}
        self.process_log_read_pos = {}
        self.process_log_handles = {}

        # 周期监控基础系统和三个行为树进程。
        self._process_monitor_timer = QTimer()
        self._process_monitor_timer.timeout.connect(
            self._monitor_processes
        )
        self._process_monitor_timer.start(500)

        self.load_meilin_map_cache()

    # =========================================================
    # 基础路径
    # =========================================================

    def get_workspace_root(self) -> Path:
        env_root = os.environ.get(
            "R2_ALGORITHM_ROOT",
            "",
        ).strip()

        if env_root:
            candidate = (
                Path(env_root)
                .expanduser()
                .resolve()
            )

            if (
                candidate
                / "src"
                / "r2_bt_executor"
                / "config"
            ).exists():
                return candidate

        current_file = Path(__file__).resolve()

        for parent in [
            current_file.parent,
            *current_file.parents,
        ]:
            if (
                parent.name == "r2_algorithm"
                and (parent / "src").exists()
            ):
                return parent

        return (
            Path.home()
            / "techx_R2_algorithm"
            / "r2_algorithm"
        )

    def get_source_meilin_map_path(self) -> Path:
        return (
            self.get_workspace_root()
            / "src"
            / "r2_bt_executor"
            / "config"
            / "meilin_map.yaml"
        )

    def get_install_meilin_map_path(self) -> Path:
        return (
            self.get_workspace_root()
            / "install"
            / "r2_bt_executor"
            / "share"
            / "r2_bt_executor"
            / "config"
            / "meilin_map.yaml"
        )

    def get_default_meilin_map_path(self) -> str:
        return str(
            self.get_source_meilin_map_path()
        )

    def get_relative_move_config_source_path(
        self,
        config_name: str,
    ) -> Path:
        return (
            self.get_workspace_root()
            / "src"
            / "r2_odin_relative_move"
            / "config"
            / config_name
        )

    def get_relative_move_config_install_path(
        self,
        config_name: str,
    ) -> Path:
        return (
            self.get_workspace_root()
            / "install"
            / "r2_odin_relative_move"
            / "share"
            / "r2_odin_relative_move"
            / "config"
            / config_name
        )

    def check_relative_move_config_ready(
        self,
        config_name: str,
    ):
        source_path = (
            self.get_relative_move_config_source_path(
                config_name
            )
        )
        install_path = (
            self.get_relative_move_config_install_path(
                config_name
            )
        )

        if install_path.is_file():
            return True, str(install_path)

        if source_path.is_file():
            return (
                False,
                "Odin 相对移动配置只在源码目录存在，"
                "但 install/share 中还没有。\n\n"
                f"源码文件：\n{source_path}\n\n"
                f"运行文件：\n{install_path}\n\n"
                "请执行：\n"
                f"  cd {self.get_workspace_root()}\n"
                "  colcon build --symlink-install "
                "--packages-select "
                "r2_odin_relative_move r2_bt_bringup\n"
                "  source install/setup.bash",
            )

        return (
            False,
            "找不到 Odin 相对移动配置：\n\n"
            f"源码文件：\n{source_path}\n\n"
            f"运行文件：\n{install_path}",
        )

    def get_bt_xml_source_path(
        self,
        xml_file_name: str,
    ) -> Path:
        return (
            self.get_workspace_root()
            / "src"
            / "r2_bt_executor"
            / "config"
            / xml_file_name
        )

    def get_bt_xml_install_path(
        self,
        xml_file_name: str,
    ) -> Path:
        return (
            self.get_workspace_root()
            / "install"
            / "r2_bt_executor"
            / "share"
            / "r2_bt_executor"
            / "config"
            / xml_file_name
        )

    def check_bt_xml_ready(
        self,
        xml_file_name: str,
    ):
        install_path = (
            self.get_bt_xml_install_path(
                xml_file_name
            )
        )
        source_path = (
            self.get_bt_xml_source_path(
                xml_file_name
            )
        )

        if install_path.exists():
            return True, str(install_path)

        if source_path.exists():
            return (
                False,
                "行为树 XML 只在源码目录存在，"
                "但 install/share 中还没有。\n\n"
                f"源码文件：\n{source_path}\n\n"
                f"运行文件：\n{install_path}\n\n"
                "请执行：\n"
                "  cd "
                f"{self.get_workspace_root()}\n"
                "  colcon build --symlink-install "
                "--packages-select "
                "r2_bt_executor r2_bt_bringup\n"
                "  source install/setup.bash",
            )

        return (
            False,
            "找不到行为树 XML：\n\n"
            f"源码文件：\n{source_path}\n\n"
            f"运行文件：\n{install_path}",
        )

    def save_yaml_to_path(
        self,
        yaml_path,
        data,
    ):
        yaml_path = Path(yaml_path)
        yaml_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        with open(
            yaml_path,
            "w",
            encoding="utf-8",
        ) as yaml_file:
            yaml.safe_dump(
                data,
                yaml_file,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            )

    # =========================================================
    # UI 显示数据
    # =========================================================

    @property
    def manual_route_text(self) -> str:
        if not self.manual_block_sequence:
            return "未选择"

        route_blocks = [
            f"B{block_id}"
            for block_id
            in self.manual_block_sequence
        ]

        route_blocks.append("EXIT_ZONE")

        return " → ".join(route_blocks)

    @property
    def kfs_text(self) -> str:
        selected = [
            f"B{block_id}"
            for block_id in range(1, 13)
            if self.block_has_kfs.get(
                block_id,
                False,
            )
        ]

        if selected:
            return "，".join(selected)

        return "未标记"

    def emit_state(self, message: str = ""):
        data = {
            "state": self.state,
            "current_task": self.current_task,
            "manual_block_sequence": list(
                self.manual_block_sequence
            ),
            "manual_route_text":
                self.manual_route_text,
            "block_has_kfs": dict(
                self.block_has_kfs
            ),
            "block_heights": dict(
                self.block_heights
            ),
            "edit_mode": self.edit_mode,
            "kfs_text": self.kfs_text,
            "tree_running":
                self.tree_running,
            "system_started":
                self.system_started,
            "system_ready":
                self.system_ready,
            "system_starting":
                self.system_starting,
            "system_step":
                self.system_step,
            "system_progress":
                self.system_progress,
            "system_profile":
                self.system_profile,
            "current_step":
                self.current_step,
            "assembly_count":
                self.assembly_count,
            "target_assembly_count":
                self.target_assembly_count,
            "progress": self.progress,
            "message": message,
        }

        self.state_changed.emit(data)

        if message:
            self.log(message)

    def log(self, text: str):
        timestamp = datetime.now().strftime(
            "%H:%M:%S"
        )
        self.log_emitted.emit(
            f"[{timestamp}] {text}"
        )

    # =========================================================
    # ROS 2 进程管理
    # =========================================================

    def make_ros_bash_cmd(
        self,
        ros_cmd: str,
    ) -> str:
        root = shlex.quote(
            str(self.get_workspace_root())
        )

        return (
            f"cd {root} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            f"{ros_cmd}"
        )

    def is_process_running(self, process) -> bool:
        return (
            process is not None
            and process.poll() is None
        )

    def start_ros_process(
        self,
        name: str,
        ros_cmd: str,
    ):
        if name in self.process_log_handles:
            self._close_process_log_handle(name)

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        log_file = (
            self.process_log_dir
            / f"{name}_{timestamp}.log"
        )

        bash_cmd = self.make_ros_bash_cmd(
            ros_cmd
        )

        log_handle = open(
            log_file,
            "w",
            encoding="utf-8",
        )

        try:
            process = subprocess.Popen(
                [
                    "bash",
                    "-lc",
                    bash_cmd,
                ],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
        except Exception:
            log_handle.close()
            raise

        self.process_log_files[
            name
        ] = log_file

        self.process_log_read_pos[
            name
        ] = 0

        self.process_log_handles[
            name
        ] = log_handle

        self.log(
            f"已启动进程 {name}，"
            f"pid={process.pid}"
        )
        self.log(
            f"日志文件：{log_file}"
        )

        return process

    def _close_process_log_handle(
        self,
        name: str,
    ):
        handle = self.process_log_handles.pop(
            name,
            None,
        )

        if handle is None:
            return

        try:
            handle.flush()
            handle.close()
        except Exception:
            pass

    def read_process_log_tail(
        self,
        name: str,
        max_chars: int = 6000,
    ) -> str:
        handle = self.process_log_handles.get(
            name
        )

        if handle is not None:
            try:
                handle.flush()
            except Exception:
                pass

        log_file = self.process_log_files.get(
            name
        )

        if not log_file:
            return ""

        log_file = Path(log_file)

        if not log_file.exists():
            return ""

        try:
            text = log_file.read_text(
                encoding="utf-8",
                errors="ignore",
            )
            return text[-max_chars:]
        except Exception as error:
            return (
                "读取日志失败："
                f"{error}"
            )

    def poll_process_log_new_lines(
        self,
        name: str,
        max_read_chars: int = 12000,
    ):
        handle = self.process_log_handles.get(
            name
        )

        if handle is not None:
            try:
                handle.flush()
            except Exception:
                pass

        log_file = self.process_log_files.get(
            name
        )

        if not log_file:
            return

        log_file = Path(log_file)

        if not log_file.exists():
            return

        try:
            last_position = (
                self.process_log_read_pos.get(
                    name,
                    0,
                )
            )
            file_size = log_file.stat().st_size

            if last_position > file_size:
                last_position = 0

            with open(
                log_file,
                "r",
                encoding="utf-8",
                errors="ignore",
            ) as log_reader:
                log_reader.seek(last_position)
                new_text = log_reader.read(
                    max_read_chars
                )
                self.process_log_read_pos[
                    name
                ] = log_reader.tell()

            if not new_text:
                return

            for line in new_text.splitlines():
                self.handle_bt_log_line_for_ui(
                    name,
                    line,
                )

        except Exception as error:
            self.log(
                f"读取 {name} 实时日志失败："
                f"{error}"
            )

    def stop_process(
        self,
        process,
        name: str,
        log_name: str = None,
        sigint_timeout_sec: float = 5.0,
    ):
        if process is None:
            if log_name:
                self._close_process_log_handle(
                    log_name
                )
            return

        if process.poll() is not None:
            self.log(
                f"{name} 已退出，"
                f"returncode={process.returncode}"
            )

            if log_name:
                self._close_process_log_handle(
                    log_name
                )
            return

        try:
            self.log(
                f"正在 Ctrl+C 停止 {name} ..."
            )

            os.killpg(
                os.getpgid(process.pid),
                signal.SIGINT,
            )

            try:
                process.wait(
                    timeout=sigint_timeout_sec
                )
                self.log(
                    f"{name} 已正常退出，"
                    f"returncode="
                    f"{process.returncode}"
                )

                if log_name:
                    self._close_process_log_handle(
                        log_name
                    )
                return

            except subprocess.TimeoutExpired:
                self.log(
                    f"{name} Ctrl+C 后未退出，"
                    "发送 SIGTERM ..."
                )

                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGTERM,
                )

            try:
                process.wait(timeout=2.0)
                self.log(
                    f"{name} 已通过 SIGTERM 退出，"
                    f"returncode="
                    f"{process.returncode}"
                )

                if log_name:
                    self._close_process_log_handle(
                        log_name
                    )
                return

            except subprocess.TimeoutExpired:
                self.log(
                    f"{name} SIGTERM 后仍未退出，"
                    "发送 SIGKILL ..."
                )

                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGKILL,
                )

                process.wait(timeout=2.0)

                self.log(
                    f"{name} 已强制退出，"
                    f"returncode="
                    f"{process.returncode}"
                )

        except ProcessLookupError:
            self.log(
                f"{name} 已经退出"
            )

        except Exception as error:
            self.log(
                f"停止 {name} 失败："
                f"{error}"
            )

        finally:
            if log_name:
                self._close_process_log_handle(
                    log_name
                )

    # =========================================================
    # 基础系统启动
    # =========================================================

    def start_system(self, profile="normal"):
        if profile not in {
            "normal",
            "zone3",
        }:
            self.error_emitted.emit(
                f"未知基础系统配置：{profile}"
            )
            return

        if self.tree_running:
            self.error_emitted.emit(
                "任务运行中不能启动基础系统"
            )
            return

        if self.system_starting:
            self.error_emitted.emit(
                "基础系统正在启动中"
            )
            return

        if (
            self.system_ready
            or self.is_process_running(
                self.system_process
            )
        ):
            self.error_emitted.emit(
                "基础系统已经启动；"
                "如需切换配置，请先复位"
            )
            return

        if profile == "zone3":
            config_name = (
                ZONE3_RELATIVE_MOVE_CONFIG
            )
            profile_text = (
                "三区爬坡高速配置"
            )
        else:
            config_name = (
                NORMAL_RELATIVE_MOVE_CONFIG
            )
            profile_text = (
                "一区/二区普通配置"
            )

        config_ready, config_info = (
            self.check_relative_move_config_ready(
                config_name
            )
        )

        if not config_ready:
            self.error_emitted.emit(
                config_info
            )
            return

        self.system_profile = profile
        self.system_started = False
        self.system_ready = False
        self.system_starting = True

        self.state = "SYSTEM_STARTING"
        self.current_task = (
            "r2_task_real_bringup.launch.py"
        )
        self.current_step = (
            f"正在启动{profile_text}"
        )
        self.system_step = (
            f"正在启动{profile_text}"
        )
        self.system_progress = 20
        self.progress = 20

        self.emit_state(
            f"开始启动{profile_text}："
            f"{config_name}"
        )

        try:
            launch_command = (
                f"{BASE_SYSTEM_LAUNCH_COMMAND} "
                "relative_move_config_name:="
                f"{shlex.quote(config_name)}"
            )

            self.system_process = (
                self.start_ros_process(
                    "system",
                    launch_command,
                )
            )

            QTimer.singleShot(
                3000,
                self._finish_system_start,
            )

        except Exception as error:
            self.system_process = None
            self.system_profile = None
            self.system_starting = False
            self.system_started = False
            self.system_ready = False
            self.system_progress = 0
            self.progress = 0
            self.current_task = "无"
            self.state = "IDLE"
            self.current_step = (
                "基础系统启动失败"
            )
            self.system_step = "启动失败"

            self.error_emitted.emit(
                "启动基础系统失败："
                f"{error}"
            )

            self.emit_state(
                "基础系统启动失败"
            )

    def _finish_system_start(self):
        # 用户可能在三秒内点击了复位。
        if not self.system_starting:
            return

        if not self.is_process_running(
            self.system_process
        ):
            tail = self.read_process_log_tail(
                "system",
                max_chars=3000,
            )

            self.system_process = None
            self._close_process_log_handle(
                "system"
            )

            self.system_profile = None
            self.system_starting = False
            self.system_started = False
            self.system_ready = False
            self.system_progress = 0
            self.progress = 0

            self.state = "SYSTEM_EXITED"
            self.current_task = "无"
            self.current_step = (
                "基础系统启动失败"
            )
            self.system_step = (
                "总 launch 已退出"
            )

            message = (
                "基础系统启动失败，"
                "总 launch 已经退出。"
            )

            if tail:
                message += (
                    "\n\n最近日志：\n"
                    f"{tail}"
                )

            self.error_emitted.emit(message)
            self.emit_state(
                "基础系统启动失败"
            )
            return

        self.system_starting = False
        self.system_started = True
        self.system_ready = True
        self.system_progress = 100
        self.progress = 100

        self.state = "SYSTEM_READY"
        self.current_task = "无"

        if self.system_profile == "zone3":
            profile_text = (
                "三区爬坡高速配置"
            )
        else:
            profile_text = (
                "一区/二区普通配置"
            )

        self.current_step = (
            f"{profile_text}已启动"
        )
        self.system_step = (
            f"{profile_text}准备完成"
        )

        self.load_meilin_map_cache()

        self.emit_state(
            f"{profile_text}已启动"
        )

    def bt_node_display_name(
        self,
        node_name: str,
    ) -> str:
        mapping = {
            "R2GetRouteFromYamlNode":
                "读取二区路线配置",
            "R2SetBlackboardStringNode":
                "设置当前方块",
            "R2SetBlackboardIntNode":
                "设置高度/路线索引",
            "R2CheckRouteFinishedNode":
                "检查路线是否完成",
            "R2GetNextManualBlockNode":
                "获取下一个目标方块",
            "R2GetTransitionInfoFromYamlNode":
                "读取方块间过渡信息",
            "R2GetBlockHeightFromYamlNode":
                "读取目标方块高度",
            "R2CheckBlockHasKfsFromYamlNode":
                "判断目标方块是否有 KFS",
            "R2BlackboardCheckBoolNode":
                "判断条件是否满足",
            "R2GetBlockKfsHeightFromYamlNode":
                "读取 KFS 高度",
            "R2CalculateHeightDeltaNode":
                "计算高度差",
            "R2BuildKfsPickActionIdFromYamlNode":
                "生成 KFS 吸取动作编号",
            "R2GetArmActionConfigFromYamlNode":
                "读取机械臂动作配置",
            "R2SetEndEffectorNode":
                "控制吸盘/气泵",
            "R2ExecuteArmActionNode":
                "执行机械臂动作",
            "R2BuildChassisCmdTypeFromYamlNode":
                "生成底盘动作编号",
            "R2ChassisStepCommandNode":
                "执行底盘动作",
            "R2IncrementIntNode":
                "更新路线索引",
            "R2BlackboardCheckStringNode":
                "检查是否到达终点",
            "R2OdinRelativeMoveActionNode":
                "执行 Odin 里程计相对移动",
            "R2OdinRelativeRotateActionNode":
                "执行 Odin 里程计相对旋转",
            "R2WeaponVisualServoActionNode":
                "执行武器视觉伺服",
            "R2VisionServoActionNode":
                "执行视觉伺服",
            "R2WaitForLightSignalActionNode":
                "等待灯光信号",
            "R2TimedCmdVelNode":
                "执行定时底盘运动",
            "R2LiftControlNode":
                "控制底盘升降",
            "R2OdinPosePidAlignActionNode":
                "执行 Odin PID 精对准",
            "R2NavigateToPoseActionNode":
                "导航到目标点",
            "R2ForceSuccess":
                "强制返回成功",
            "R2WaitForever":
                "等待",
        }

        return mapping.get(
            node_name,
            node_name,
        )

    def extract_bt_node_name_from_log_line(
        self,
        line: str,
    ):
        match = re.search(
            r"\["
            r"(R2[A-Za-z0-9_]*"
            r"(?:Node|ActionNode|MockNode|"
            r"Success|Forever))"
            r"\]",
            line,
        )

        if match:
            return match.group(1)

        return None

    def handle_bt_log_line_for_ui(
        self,
        task_name: str,
        line: str,
    ):
        node_name = (
            self.extract_bt_node_name_from_log_line(
                line
            )
        )

        if not node_name:
            return

        if node_name == self.last_bt_node_name:
            return

        self.last_bt_node_name = node_name
        display_name = self.bt_node_display_name(
            node_name
        )

        detail = ""

        if (
            "from_block=" in line
            and "to_block=" in line
        ):
            from_match = re.search(
                r"from_block=([A-Za-z0-9_]+)",
                line,
            )
            to_match = re.search(
                r"to_block=([A-Za-z0-9_]+)",
                line,
            )

            if from_match and to_match:
                detail = (
                    "\n路径："
                    f"{from_match.group(1)}"
                    " → "
                    f"{to_match.group(1)}"
                )

        elif "to_block=" in line:
            match = re.search(
                r"to_block=([A-Za-z0-9_]+)",
                line,
            )

            if match:
                detail = (
                    "\n目标方块："
                    f"{match.group(1)}"
                )

        elif "goal_name=" in line:
            match = re.search(
                r"goal_name=([A-Za-z0-9_]+)",
                line,
            )

            if match:
                detail = (
                    "\n目标："
                    f"{match.group(1)}"
                )

        if task_name == "gym_bt":
            prefix = "一区正在执行"
        elif task_name == "meilin_bt":
            prefix = "二区正在执行"
        elif task_name == "zone3_bt":
            prefix = "三区正在执行"
        else:
            prefix = "正在执行"

        self.current_step = (
            f"{prefix}：{display_name}\n"
            f"节点：{node_name}"
            f"{detail}"
        )

        self.emit_state(
            self.current_step
        )

    def bt_log_has_failure(
        self,
        text: str,
    ) -> bool:
        failure_keywords = [
            "Behavior tree finished with FAILURE",
            "finished with FAILURE",
            "Action did not succeed",
            "TreeNode threw exception",
            "XML_ERROR",
        ]

        return any(
            keyword in text
            for keyword in failure_keywords
        )

    def bt_log_has_success(
        self,
        text: str,
    ) -> bool:
        success_keywords = [
            "Behavior tree finished with SUCCESS",
            "finished with SUCCESS",
        ]

        return any(
            keyword in text
            for keyword in success_keywords
        )

    # =========================================================
    # 周期监控
    # =========================================================

    def _monitor_processes(self):
        if self.is_process_running(
            self.gym_bt_process
        ):
            self.poll_process_log_new_lines(
                "gym_bt"
            )

        if self.is_process_running(
            self.meilin_bt_process
        ):
            self.poll_process_log_new_lines(
                "meilin_bt"
            )

        if self.is_process_running(
            self.zone3_bt_process
        ):
            self.poll_process_log_new_lines(
                "zone3_bt"
            )

        self._monitor_gym_bt_process()
        self._monitor_meilin_bt_process()
        self._monitor_zone3_bt_process()
        self._monitor_system_process()

    def _monitor_gym_bt_process(self):
        process = self.gym_bt_process

        if (
            process is None
            or process.poll() is None
        ):
            return

        self.poll_process_log_new_lines(
            "gym_bt"
        )

        return_code = process.returncode
        tail = self.read_process_log_tail(
            "gym_bt"
        )

        self.gym_bt_process = None
        self._close_process_log_handle(
            "gym_bt"
        )

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if (
            return_code != 0
            or self.bt_log_has_failure(tail)
        ):
            self.state = "GYM_FAILED"
            self.current_step = (
                "一区行为树失败"
            )

            self.emit_state(
                "一区行为树失败："
                f"returncode={return_code}"
            )

            self.error_emitted.emit(
                "一区行为树已经失败并退出。\n\n"
                f"returncode={return_code}\n\n"
                "请查看 gym_bt 日志。"
            )
            return

        self.state = "GYM_DONE_WAIT_LIFT"
        self.assembly_count = 1
        self.current_step = "一区完成"

        self.emit_state(
            "一区行为树成功结束"
        )

    def _monitor_meilin_bt_process(self):
        process = self.meilin_bt_process

        if (
            process is None
            or process.poll() is None
        ):
            return

        self.poll_process_log_new_lines(
            "meilin_bt"
        )

        return_code = process.returncode
        tail = self.read_process_log_tail(
            "meilin_bt"
        )

        self.meilin_bt_process = None
        self._close_process_log_handle(
            "meilin_bt"
        )

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if (
            return_code != 0
            or self.bt_log_has_failure(tail)
        ):
            self.state = "MEILIN_FAILED"
            self.current_step = (
                "二区行为树失败"
            )

            self.emit_state(
                "二区行为树失败："
                f"returncode={return_code}"
            )

            self.error_emitted.emit(
                "二区行为树已经失败并退出。\n\n"
                f"returncode={return_code}\n\n"
                "请查看 meilin_bt 日志。"
            )
            return

        if (
            self.bt_log_has_success(tail)
            or return_code == 0
        ):
            self.state = "MATCH_DONE"
            self.current_step = "二区完成"

            self.emit_state(
                "二区行为树成功结束"
            )
            return

        self.state = "MEILIN_EXITED"
        self.current_step = (
            "二区行为树已退出"
        )

        self.emit_state(
            "二区行为树已退出："
            f"returncode={return_code}"
        )

    def _monitor_zone3_bt_process(self):
        process = self.zone3_bt_process

        if (
            process is None
            or process.poll() is None
        ):
            return

        self.poll_process_log_new_lines(
            "zone3_bt"
        )

        return_code = process.returncode
        tail = self.read_process_log_tail(
            "zone3_bt"
        )

        self.zone3_bt_process = None
        self._close_process_log_handle(
            "zone3_bt"
        )

        self.tree_running = False
        self.current_task = "无"
        self.progress = 100

        if (
            return_code != 0
            or self.bt_log_has_failure(tail)
        ):
            self.state = "ZONE3_FAILED"
            self.current_step = (
                "三区行为树失败"
            )

            self.emit_state(
                "三区行为树失败："
                f"returncode={return_code}"
            )

            self.error_emitted.emit(
                "三区行为树已经失败并退出。\n\n"
                f"returncode={return_code}\n\n"
                "请查看 zone3_bt 日志。"
            )
            return

        self.state = "ZONE3_DONE"
        self.current_step = "三区任务完成"

        self.emit_state(
            "三区行为树成功结束"
        )

    def _monitor_system_process(self):
        process = self.system_process

        if (
            process is None
            or process.poll() is None
        ):
            return

        return_code = process.returncode
        tail = self.read_process_log_tail(
            "system"
        )

        self.system_process = None
        self._close_process_log_handle(
            "system"
        )

        was_active = (
            self.system_started
            or self.system_ready
            or self.system_starting
        )

        self.system_started = False
        self.system_ready = False
        self.system_starting = False
        self.system_progress = 0
        self.system_profile = None
        self.system_step = "总系统已退出"

        if not was_active:
            return

        if self.state in {
            "IDLE",
            "STOPPED",
        }:
            return

        self.state = "SYSTEM_EXITED"
        self.current_task = "无"
        self.current_step = (
            "基础系统已退出"
        )
        self.progress = 0

        message = (
            "基础系统总 launch 已退出："
            f"returncode={return_code}"
        )

        self.emit_state(message)

        if return_code != 0:
            error_message = message

            if tail:
                error_message += (
                    "\n\n最近日志：\n"
                    f"{tail[-3000:]}"
                )

            self.error_emitted.emit(
                error_message
            )

    # =========================================================
    # 二区配置
    # =========================================================

    def load_meilin_map_cache(self):
        yaml_path = Path(
            self.get_default_meilin_map_path()
        )

        if not yaml_path.exists():
            self.block_heights = {
                1: 400,
                2: 200,
                3: 400,
                4: 200,
                5: 400,
                6: 600,
                7: 400,
                8: 600,
                9: 400,
                10: 200,
                11: 400,
                12: 200,
            }

            self.block_has_kfs = {
                i: False
                for i in range(1, 13)
            }
            return

        try:
            with open(
                yaml_path,
                "r",
                encoding="utf-8",
            ) as yaml_file:
                data = yaml.safe_load(
                    yaml_file
                ) or {}

            blocks = (
                data.get(
                    "blocks",
                    {},
                ) or {}
            )

            for block_id in range(1, 13):
                block_info = (
                    blocks.get(
                        f"B{block_id}",
                        {},
                    ) or {}
                )

                self.block_heights[
                    block_id
                ] = int(
                    block_info.get(
                        "height",
                        0,
                    )
                )

                self.block_has_kfs[
                    block_id
                ] = bool(
                    block_info.get(
                        "has_kfs",
                        False,
                    )
                )

            route_blocks = (
                (
                    (
                        data.get(
                            "routes",
                            {},
                        ) or {}
                    ).get(
                        "zone2_main",
                        {},
                    ) or {}
                ).get(
                    "blocks",
                    [],
                ) or []
            )

            self.manual_block_sequence = []

            for block_name in route_blocks:
                if not (
                    isinstance(
                        block_name,
                        str,
                    )
                    and block_name.startswith("B")
                ):
                    continue

                try:
                    block_id = int(
                        block_name[1:]
                    )
                except ValueError:
                    continue

                if 1 <= block_id <= 12:
                    self.manual_block_sequence.append(
                        block_id
                    )

        except Exception as error:
            self.log(
                "读取 meilin_map.yaml 失败："
                f"{error}"
            )

    def save_meilin_config(self):
        if (
            self.tree_running
            or self.system_starting
        ):
            return (
                False,
                "任务运行中或基础系统启动中，"
                "不能保存二区配置",
            )

        source_yaml_path = (
            self.get_source_meilin_map_path()
        )
        install_yaml_path = (
            self.get_install_meilin_map_path()
        )

        if not source_yaml_path.exists():
            return (
                False,
                "找不到源码 "
                "meilin_map.yaml：\n"
                f"{source_yaml_path}",
            )

        try:
            with open(
                source_yaml_path,
                "r",
                encoding="utf-8",
            ) as yaml_file:
                data = yaml.safe_load(
                    yaml_file
                ) or {}

            data.setdefault(
                "routes",
                {},
            )
            data["routes"].setdefault(
                "zone2_main",
                {},
            )

            if self.manual_block_sequence:
                route_blocks = [
                    f"B{block_id}"
                    for block_id
                    in self.manual_block_sequence
                ]

                route_blocks.append(
                    "EXIT_ZONE"
                )

                zone2_route = (
                    data["routes"][
                        "zone2_main"
                    ]
                )

                zone2_route[
                    "start_block"
                ] = "ENTRY"

                zone2_route[
                    "start_height"
                ] = 0

                zone2_route[
                    "blocks"
                ] = route_blocks

            data.setdefault(
                "blocks",
                {},
            )

            for block_id in range(1, 13):
                block_name = f"B{block_id}"

                data["blocks"].setdefault(
                    block_name,
                    {},
                )

                data["blocks"][
                    block_name
                ]["has_kfs"] = bool(
                    self.block_has_kfs.get(
                        block_id,
                        False,
                    )
                )

            for special_block in [
                "ENTRY",
                "EXIT_ZONE",
            ]:
                if special_block in data["blocks"]:
                    data["blocks"][
                        special_block
                    ]["has_kfs"] = False

            self.save_yaml_to_path(
                source_yaml_path,
                data,
            )
            self.save_yaml_to_path(
                install_yaml_path,
                data,
            )

            message = (
                "已保存二区配置\n\n"
                "源码配置：\n"
                f"{source_yaml_path}\n\n"
                "运行配置：\n"
                f"{install_yaml_path}\n\n"
                "当前路线："
                f"{self.manual_route_text}\n"
                "KFS："
                f"{self.kfs_text}"
            )

            self.emit_state(
                "已保存二区配置："
                f"路线 {self.manual_route_text}，"
                f"KFS {self.kfs_text}"
            )

            return True, message

        except Exception as error:
            return False, str(error)

    def toggle_edit_mode(self):
        if (
            self.tree_running
            or self.system_starting
        ):
            self.error_emitted.emit(
                "任务运行中或基础系统启动中，"
                "不能切换编辑模式"
            )
            return

        if self.edit_mode == "ROUTE":
            self.edit_mode = "KFS"
            message = (
                "已切换到 KFS 标记模式"
            )
        else:
            self.edit_mode = "ROUTE"
            message = (
                "已切换到路线选择模式"
            )

        self.state = "MEILIN_EDITING"
        self.emit_state(message)

    def toggle_block(self, block_id):
        if (
            self.tree_running
            or self.system_starting
        ):
            self.error_emitted.emit(
                "任务运行中或基础系统启动中，"
                "不能修改方块"
            )
            return

        if block_id not in range(1, 13):
            self.error_emitted.emit(
                "梅林方块编号必须是 1~12"
            )
            return

        self.state = "MEILIN_EDITING"

        if self.edit_mode == "KFS":
            new_value = not self.block_has_kfs.get(
                block_id,
                False,
            )

            self.block_has_kfs[
                block_id
            ] = new_value

            self.emit_state(
                f"B{block_id} 已设置为："
                f"{'有 KFS' if new_value else '无 KFS'}"
            )
            return

        if block_id in self.manual_block_sequence:
            self.manual_block_sequence.remove(
                block_id
            )

            self.emit_state(
                f"已移除 B{block_id}，"
                f"当前路线："
                f"{self.manual_route_text}"
            )
            return

        self.manual_block_sequence.append(
            block_id
        )

        self.emit_state(
            f"已加入 B{block_id}，"
            f"当前路线："
            f"{self.manual_route_text}"
        )

    def clear_block_sequence(self):
        if self.tree_running:
            self.error_emitted.emit(
                "任务运行中不能清空路线"
            )
            return

        self.manual_block_sequence.clear()
        self.state = "MEILIN_EDITING"
        self.current_step = (
            "已清空二区方块序列"
        )
        self.progress = 0

        self.emit_state(
            "已清空二区方块序列"
        )

    # =========================================================
    # 行为树启动
    # =========================================================

    def _make_bt_launch_command(
        self,
        xml_file_name: str,
    ) -> str:
        return (
            "ros2 launch "
            f"{BT_LAUNCH_PACKAGE} "
            f"{BT_LAUNCH_FILE} "
            "xml_file_name:="
            f"{shlex.quote(xml_file_name)}"
        )

    def _can_start_task(self) -> bool:
        if self.tree_running:
            self.error_emitted.emit(
                "当前已有任务正在运行"
            )
            return False

        if not self.system_ready:
            self.error_emitted.emit(
                "请先启动基础系统"
            )
            return False

        if not self.is_process_running(
            self.system_process
        ):
            self.system_started = False
            self.system_ready = False
            self.system_starting = False
            self.system_progress = 0
            self.system_profile = None
            self.system_step = (
                "基础系统未运行"
            )

            self.error_emitted.emit(
                "基础系统进程已经退出，"
                "请先重新启动基础系统"
            )
            self.emit_state(
                "基础系统未运行"
            )
            return False

        return True

    def start_gym(self):
        if not self._can_start_task():
            return

        if self.system_profile != "normal":
            self.error_emitted.emit(
                "一区只能使用一区/二区普通配置。"
                "请先复位，再启动普通配置。"
            )
            return

        xml_file_name = (
            ZONE1_COMPETITION_BT_XML
        )

        ready, info = self.check_bt_xml_ready(
            xml_file_name
        )

        if not ready:
            self.error_emitted.emit(info)
            return

        try:
            self.last_bt_node_name = ""

            self.gym_bt_process = (
                self.start_ros_process(
                    "gym_bt",
                    self._make_bt_launch_command(
                        xml_file_name
                    ),
                )
            )

            self.state = "RUNNING_GYM"
            self.current_task = xml_file_name
            self.tree_running = True
            self.assembly_count = 0
            self.target_assembly_count = 1
            self.current_step = (
                "一区行为树已启动"
            )
            self.progress = 0

            self.emit_state(
                "一区行为树已启动："
                f"{xml_file_name}"
            )

        except Exception as error:
            self.gym_bt_process = None
            self.tree_running = False
            self.current_task = "无"

            self.error_emitted.emit(
                "启动一区行为树失败："
                f"{error}"
            )

    def start_meilin(self):
        if not self._can_start_task():
            return

        if self.system_profile != "normal":
            self.error_emitted.emit(
                "二区只能使用一区/二区普通配置。"
                "请先复位，再启动普通配置。"
            )
            return

        if not self.manual_block_sequence:
            self.error_emitted.emit(
                "请先选择至少一个梅林方块"
            )
            return

        xml_file_name = (
            ZONE2_COMPETITION_BT_XML
        )

        ready, info = self.check_bt_xml_ready(
            xml_file_name
        )

        if not ready:
            self.error_emitted.emit(info)
            return

        saved, save_message = (
            self.save_meilin_config()
        )

        if not saved:
            self.error_emitted.emit(
                "启动二区前保存配置失败："
                f"{save_message}"
            )
            return

        try:
            self.last_bt_node_name = ""

            self.meilin_bt_process = (
                self.start_ros_process(
                    "meilin_bt",
                    self._make_bt_launch_command(
                        xml_file_name
                    ),
                )
            )

            self.state = "RUNNING_MEILIN"
            self.current_task = xml_file_name
            self.tree_running = True
            self.current_step = (
                "二区行为树已启动"
            )
            self.progress = 0

            self.emit_state(
                "二区行为树已启动："
                f"路线 {self.manual_route_text}，"
                f"KFS {self.kfs_text}"
            )

        except Exception as error:
            self.meilin_bt_process = None
            self.tree_running = False
            self.current_task = "无"

            self.error_emitted.emit(
                "启动二区行为树失败："
                f"{error}"
            )

    def start_zone3(self):
        if not self._can_start_task():
            return False

        if self.system_profile != "zone3":
            self.error_emitted.emit(
                "三区必须使用三区爬坡高速配置。"
                "请先复位，再启动三区配置。"
            )
            return False

        xml_file_name = (
            ZONE3_COMPETITION_BT_XML
        )

        ready, info = self.check_bt_xml_ready(
            xml_file_name
        )

        if not ready:
            self.error_emitted.emit(info)
            return False

        try:
            self.last_bt_node_name = ""

            self.zone3_bt_process = (
                self.start_ros_process(
                    "zone3_bt",
                    self._make_bt_launch_command(
                        xml_file_name
                    ),
                )
            )

            self.state = "RUNNING_ZONE3"
            self.current_task = xml_file_name
            self.tree_running = True
            self.current_step = (
                "三区行为树已启动"
            )
            self.progress = 0

            self.emit_state(
                "三区行为树已启动："
                f"{xml_file_name}"
            )

            return True

        except Exception as error:
            self.zone3_bt_process = None
            self.tree_running = False
            self.current_task = "无"

            self.error_emitted.emit(
                "启动三区行为树失败："
                f"{error}"
            )

            return False

    # =========================================================
    # 安全控制
    # =========================================================

    def publish_zero_cmd_vel(self):
        ros_cmd = (
            "ros2 topic pub "
            "-r 20 "
            "--times 10 "
            "/cmd_vel "
            "geometry_msgs/msg/Twist "
            "'{"
            "linear: {x: 0.0, y: 0.0, z: 0.0}, "
            "angular: {x: 0.0, y: 0.0, z: 0.0}"
            "}'"
        )

        try:
            subprocess.run(
                [
                    "bash",
                    "-lc",
                    self.make_ros_bash_cmd(
                        ros_cmd
                    ),
                ],
                timeout=3.0,
                check=False,
            )
            self.log(
                "已连续发布 /cmd_vel=0"
            )

        except Exception as error:
            self.log(
                "发布 /cmd_vel=0 失败："
                f"{error}"
            )

    def call_chassis_estop(self):
        ros_cmd = (
            "ros2 service call "
            "/r2_chassis/estop "
            "techx_r2_chassis_interfaces/srv/EStop "
            "'{trigger: true}'"
        )

        try:
            subprocess.run(
                [
                    "bash",
                    "-lc",
                    self.make_ros_bash_cmd(
                        ros_cmd
                    ),
                ],
                timeout=3.0,
                check=False,
            )
            self.log(
                "已调用 /r2_chassis/estop"
            )

        except Exception as error:
            self.log(
                "调用 /r2_chassis/estop 失败："
                f"{error}"
            )

    def open_gripper_claw(self):
        ros_cmd = (
            "python3 "
            "ui/open_gripper_claw_until_feedback.py "
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
                [
                    "bash",
                    "-lc",
                    self.make_ros_bash_cmd(
                        ros_cmd
                    ),
                ],
                timeout=4.0,
                check=False,
                capture_output=True,
                text=True,
            )

            output = "\n".join(
                part.strip()
                for part in (
                    result.stdout,
                    result.stderr,
                )
                if part and part.strip()
            )

            if result.returncode == 0:
                if output:
                    self.log(
                        "夹爪打开首帧反馈："
                        f"{output.splitlines()[-1]}"
                    )

                self.log(
                    "已下发夹爪打开命令："
                    "target_id=4 "
                    "action_id=1025，"
                    "不等待最终结果"
                )
                return

            self.log(
                "夹爪打开命令未确认首帧反馈："
                f"returncode={result.returncode}"
                + (
                    "，"
                    f"{output.splitlines()[-1]}"
                    if output
                    else ""
                )
            )

        except subprocess.TimeoutExpired:
            self.log(
                "夹爪打开命令等待首帧反馈超时："
                "请检查 /r2_arm/execute_action"
            )

        except Exception as error:
            self.log(
                "夹爪打开命令失败："
                f"{error}"
            )

    def stop(self):
        self.log(
            "执行急停：发布 /cmd_vel=0"
        )
        self.publish_zero_cmd_vel()

        self.log(
            "执行急停：停止当前行为树"
        )

        self.stop_process(
            self.gym_bt_process,
            "一区行为树",
            log_name="gym_bt",
        )
        self.stop_process(
            self.meilin_bt_process,
            "二区行为树",
            log_name="meilin_bt",
        )
        self.stop_process(
            self.zone3_bt_process,
            "三区行为树",
            log_name="zone3_bt",
        )

        self.gym_bt_process = None
        self.meilin_bt_process = None
        self.zone3_bt_process = None
        self.tree_running = False

        self.log(
            "执行急停：打开夹爪"
        )
        self.open_gripper_claw()

        self.current_task = "无"
        self.state = "STOPPED"
        self.current_step = "已急停"
        self.progress = 0
        self.last_bt_node_name = ""

        self.emit_state(
            "已急停：当前行为树已停止，"
            "基础系统仍保持启动"
        )

    def reset(self):
        # 防止延迟启动回调在复位后把系统重新标记为 READY。
        self.system_starting = False

        has_active_ros_process = any(
            [
                self.is_process_running(
                    self.system_process
                ),
                self.is_process_running(
                    self.gym_bt_process
                ),
                self.is_process_running(
                    self.meilin_bt_process
                ),
                self.is_process_running(
                    self.zone3_bt_process
                ),
            ]
        )

        if has_active_ros_process:
            self.log(
                "执行复位：发布 /cmd_vel=0"
            )
            self.publish_zero_cmd_vel()

        self.log(
            "执行复位：停止行为树"
        )

        self.stop_process(
            self.gym_bt_process,
            "一区行为树",
            log_name="gym_bt",
        )
        self.stop_process(
            self.meilin_bt_process,
            "二区行为树",
            log_name="meilin_bt",
        )
        self.stop_process(
            self.zone3_bt_process,
            "三区行为树",
            log_name="zone3_bt",
        )

        self.gym_bt_process = None
        self.meilin_bt_process = None
        self.zone3_bt_process = None
        self.tree_running = False

        if has_active_ros_process:
            self.log(
                "执行复位：打开夹爪"
            )
            self.open_gripper_claw()
            time.sleep(0.2)

        self.log(
            "执行复位：Ctrl+C 关闭基础总 launch"
        )

        self.stop_process(
            self.system_process,
            "基础系统",
            log_name="system",
        )
        self.system_process = None

        self.state = "IDLE"
        self.current_task = "无"
        self.current_step = "-"
        self.progress = 0

        self.assembly_count = 0
        self.target_assembly_count = 1
        self.last_bt_node_name = ""

        self.system_started = False
        self.system_ready = False
        self.system_starting = False
        self.system_step = "未启动"
        self.system_progress = 0
        self.system_profile = None

        self.edit_mode = "ROUTE"

        self.load_meilin_map_cache()

        self.emit_state(
            "系统已复位：基础总 launch 已关闭，"
            "UI 已回到初始状态"
        )
