import threading
import time

import rclpy
from rclpy.node import Node

import serial

from .protocol import (
    CMD_ACTION_FEEDBACK,
    CMD_EXECUTE_ACTION,
    CMD_IO_STATUS,
    CMD_SET_END_EFFECTOR,
    ERROR_MOTOR_ERROR,
    ERROR_OK,
    FRAME_TYPE_STM32_TO_HOST,
    IO_ARM_SUCTION_PUMP,
    IO_GRIPPER_CLAW,
    IO_REAR_STORAGE_PUMP,
    STATE_ACK,
    STATE_DONE,
    STATE_ERROR,
    STATE_RUNNING,
    TARGET_COMPOSITE_TASK,
    TARGET_GRIPPER_ARM,
    TARGET_KFS_SUCTION_ARM,
    action_id_to_name,
    bytes_to_hex,
    frame_to_bytes,
    io_id_to_name,
    pack_action_feedback_data,
    pack_frame,
    pack_io_status_data,
    parse_frames,
    target_id_to_name,
    unpack_execute_action_fields,
    unpack_set_end_effector_fields,
)


class MockArmStm32(Node):
    def __init__(self):
        super().__init__("mock_arm_stm32")

        self.declare_parameter("port", "/tmp/r2_arm_lower")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("done_delay_s", 1.0)
        self.declare_parameter("result_mode", "success")
        self.declare_parameter("debug_log_frames", True)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.done_delay_s = float(self.get_parameter("done_delay_s").value)
        self.result_mode = str(self.get_parameter("result_mode").value)
        self.debug_log_frames = bool(self.get_parameter("debug_log_frames").value)

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.02,
        )

        self.rx_buffer = bytearray()

        # 用于模拟 STM32 正在执行任务。
        # 如果同一个 seq 重发，不重新执行，只重发 ACK/IO_STATUS。
        # 如果不同 seq 进来，表示上位机发了新任务，mock 会回 MOTOR_ERROR。
        self.running_seq = None
        self.running_command = None
        self.lock = threading.Lock()

        self.read_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
        )
        self.read_thread.start()

        self.get_logger().info(
            f"mock_arm_stm32 started: port={self.port}, baudrate={self.baudrate}, "
            f"result_mode={self.result_mode}, debug_log_frames={self.debug_log_frames}"
        )

    def _read_loop(self):
        while rclpy.ok():
            try:
                data = self.serial.read(64)

                if not data:
                    continue

                if self.debug_log_frames:
                    self.get_logger().info(f"RX bytes: {bytes_to_hex(data)}")

                self.rx_buffer.extend(data)
                frames = parse_frames(self.rx_buffer)

                for frame in frames:
                    if self.debug_log_frames:
                        self.get_logger().info(
                            f"RX parsed frame: {bytes_to_hex(frame_to_bytes(frame))}"
                        )

                    self._handle_frame(frame)

            except Exception as exc:
                self.get_logger().error(f"mock serial read error: {exc}")
                time.sleep(0.1)

    def _handle_frame(self, frame):
        if frame.cmd_type == CMD_EXECUTE_ACTION:
            self._handle_execute_action(frame)
            return

        if frame.cmd_type == CMD_SET_END_EFFECTOR:
            self._handle_set_end_effector(frame)
            return

        self.get_logger().warn(
            f"ignore cmd_type=0x{frame.cmd_type:02X}, seq={frame.seq}"
        )

    def _handle_execute_action(self, frame):
        target_id, action_id, timeout_ms, param, flags = unpack_execute_action_fields(
            frame
        )

        self.get_logger().info(
            f"rx EXECUTE_ACTION seq={frame.seq}, "
            f"target_id=0x{target_id:02X}({target_id_to_name(target_id)}), "
            f"action_id=0x{action_id:04X}({action_id_to_name(action_id)}), "
            f"timeout_ms={timeout_ms}, param={param}, flags=0x{flags:02X}"
        )

        if self.result_mode == "no_ack":
            self.get_logger().warn("result_mode=no_ack, ignore EXECUTE_ACTION")
            return

        valid_target_ids = (
            TARGET_GRIPPER_ARM,
            TARGET_KFS_SUCTION_ARM,
            TARGET_COMPOSITE_TASK,
        )

        # 简化协议：
        # 所有显式失败统一使用 ERROR_MOTOR_ERROR = 0x0008。
        if target_id not in valid_target_ids:
            self.get_logger().warn("unsupported target_id, send ERROR + MOTOR_ERROR")

            self._send_action_feedback(
                seq=frame.seq,
                target_id=target_id,
                action_id=action_id,
                state=STATE_ERROR,
                error_code=ERROR_MOTOR_ERROR,
            )
            return

        command_key = ("execute_action", target_id, action_id)

        with self.lock:
            if self.running_seq is not None and self.running_seq != frame.seq:
                self.get_logger().warn(
                    f"busy: running_seq={self.running_seq}, new_seq={frame.seq}, "
                    "send ERROR + MOTOR_ERROR"
                )

                self._send_action_feedback(
                    seq=frame.seq,
                    target_id=target_id,
                    action_id=action_id,
                    state=STATE_ERROR,
                    error_code=ERROR_MOTOR_ERROR,
                )
                return

            if self.running_seq == frame.seq and self.running_command == command_key:
                self.get_logger().warn(
                    f"duplicate EXECUTE_ACTION seq={frame.seq}, resend ACK only"
                )

                self._send_action_feedback(
                    seq=frame.seq,
                    target_id=target_id,
                    action_id=action_id,
                    state=STATE_ACK,
                    error_code=ERROR_OK,
                )
                return

            self.running_seq = frame.seq
            self.running_command = command_key

        threading.Thread(
            target=self._simulate_execute_action_task,
            args=(frame.seq, target_id, action_id),
            daemon=True,
        ).start()

    def _handle_set_end_effector(self, frame):
        io_id, state, hold_ms = unpack_set_end_effector_fields(frame)

        self.get_logger().info(
            f"rx SET_END_EFFECTOR seq={frame.seq}, "
            f"io_id=0x{io_id:02X}({io_id_to_name(io_id)}), "
            f"state={state}, hold_ms={hold_ms}"
        )

        if self.result_mode == "no_ack":
            self.get_logger().warn("result_mode=no_ack, ignore SET_END_EFFECTOR")
            return

        valid_io_ids = (
            IO_GRIPPER_CLAW,
            IO_ARM_SUCTION_PUMP,
            IO_REAR_STORAGE_PUMP,
        )

        # 简化协议：
        # 所有 IO 显式失败也统一使用 ERROR_MOTOR_ERROR = 0x0008。
        if io_id not in valid_io_ids or state not in (0, 1):
            self.get_logger().warn("invalid io_id/state, send IO_STATUS error")

            self._send_io_status(
                seq=frame.seq,
                io_id=io_id,
                state=state,
                feedback_ok=False,
                error_code=ERROR_MOTOR_ERROR,
            )
            return

        command_key = ("set_end_effector", io_id, state)

        with self.lock:
            if self.running_seq is not None and self.running_seq != frame.seq:
                self.get_logger().warn(
                    f"busy: running_seq={self.running_seq}, new_seq={frame.seq}, "
                    "send IO_STATUS error"
                )

                self._send_io_status(
                    seq=frame.seq,
                    io_id=io_id,
                    state=state,
                    feedback_ok=False,
                    error_code=ERROR_MOTOR_ERROR,
                )
                return

            if self.running_seq == frame.seq and self.running_command == command_key:
                self.get_logger().warn(
                    f"duplicate SET_END_EFFECTOR seq={frame.seq}, resend IO_STATUS only"
                )

                self._send_io_status(
                    seq=frame.seq,
                    io_id=io_id,
                    state=state,
                    feedback_ok=True,
                    error_code=ERROR_OK,
                )
                return

            self.running_seq = frame.seq
            self.running_command = command_key

        threading.Thread(
            target=self._simulate_set_end_effector_task,
            args=(frame.seq, io_id, state),
            daemon=True,
        ).start()

    def _simulate_execute_action_task(
        self,
        seq: int,
        target_id: int,
        action_id: int,
    ):
        # 简化协议：
        # current_pose_id 不使用，固定为 0。
        self._send_action_feedback(
            seq=seq,
            target_id=target_id,
            action_id=action_id,
            state=STATE_ACK,
            error_code=ERROR_OK,
            current_pose_id=0,
        )

        time.sleep(0.2)

        self._send_action_feedback(
            seq=seq,
            target_id=target_id,
            action_id=action_id,
            state=STATE_RUNNING,
            error_code=ERROR_OK,
            current_pose_id=0,
        )

        time.sleep(self.done_delay_s)

        if self.result_mode == "success":
            self._send_action_feedback(
                seq=seq,
                target_id=target_id,
                action_id=action_id,
                state=STATE_DONE,
                error_code=ERROR_OK,
                current_pose_id=0,
            )

        elif self.result_mode == "error":
            self._send_action_feedback(
                seq=seq,
                target_id=target_id,
                action_id=action_id,
                state=STATE_ERROR,
                error_code=ERROR_MOTOR_ERROR,
                current_pose_id=0,
            )

        elif self.result_mode == "timeout":
            self.get_logger().warn(
                "result_mode=timeout, do not send DONE/ERROR"
            )
            return

        else:
            self.get_logger().warn(
                f"unknown result_mode={self.result_mode}, send ERROR + MOTOR_ERROR"
            )

            self._send_action_feedback(
                seq=seq,
                target_id=target_id,
                action_id=action_id,
                state=STATE_ERROR,
                error_code=ERROR_MOTOR_ERROR,
                current_pose_id=0,
            )

        with self.lock:
            if self.running_seq == seq:
                self.running_seq = None
                self.running_command = None

    def _simulate_set_end_effector_task(self, seq: int, io_id: int, state: int):
        time.sleep(0.05)

        if self.result_mode == "success":
            self._send_io_status(
                seq=seq,
                io_id=io_id,
                state=state,
                feedback_ok=True,
                error_code=ERROR_OK,
            )

        elif self.result_mode == "error":
            # 简化协议：
            # 没有真空检测，不再模拟 VACUUM_FAIL。
            # 所有显式失败统一用 MOTOR_ERROR = 0x0008。
            self._send_io_status(
                seq=seq,
                io_id=io_id,
                state=state,
                feedback_ok=False,
                error_code=ERROR_MOTOR_ERROR,
            )

        elif self.result_mode == "timeout":
            self.get_logger().warn(
                "result_mode=timeout, do not send IO_STATUS"
            )
            return

        else:
            self.get_logger().warn(
                f"unknown result_mode={self.result_mode}, send IO_STATUS ERROR + MOTOR_ERROR"
            )

            self._send_io_status(
                seq=seq,
                io_id=io_id,
                state=state,
                feedback_ok=False,
                error_code=ERROR_MOTOR_ERROR,
            )

        with self.lock:
            if self.running_seq == seq:
                self.running_seq = None
                self.running_command = None

    def _send_action_feedback(
        self,
        seq: int,
        target_id: int,
        action_id: int,
        state: int,
        error_code: int = ERROR_OK,
        current_pose_id: int = 0,
    ):
        # 简化协议：
        # current_pose_id 不使用，强制固定为 0。
        data = pack_action_feedback_data(
            target_id=target_id,
            action_id=action_id,
            state=state,
            error_code=error_code,
            current_pose_id=0,
        )

        frame = pack_frame(
            frame_type=FRAME_TYPE_STM32_TO_HOST,
            seq=seq,
            cmd_type=CMD_ACTION_FEEDBACK,
            data=data,
        )

        self.serial.write(frame)
        self.serial.flush()

        self.get_logger().info(
            f"tx ACTION_FEEDBACK seq={seq}, "
            f"target_id=0x{target_id:02X}({target_id_to_name(target_id)}), "
            f"action_id=0x{action_id:04X}({action_id_to_name(action_id)}), "
            f"state={state}, error_code=0x{error_code:04X}, current_pose_id=0"
        )

        if self.debug_log_frames:
            self.get_logger().info(f"TX raw frame: {bytes_to_hex(frame)}")

    def _send_io_status(
        self,
        seq: int,
        io_id: int,
        state: int,
        feedback_ok: bool,
        error_code: int = ERROR_OK,
    ):
        data = pack_io_status_data(
            io_id=io_id,
            state=state,
            feedback_ok=feedback_ok,
            error_code=error_code,
        )

        frame = pack_frame(
            frame_type=FRAME_TYPE_STM32_TO_HOST,
            seq=seq,
            cmd_type=CMD_IO_STATUS,
            data=data,
        )

        self.serial.write(frame)
        self.serial.flush()

        self.get_logger().info(
            f"tx IO_STATUS seq={seq}, "
            f"io_id=0x{io_id:02X}({io_id_to_name(io_id)}), "
            f"state={state}, feedback_ok={feedback_ok}, "
            f"error_code=0x{error_code:04X}"
        )

        if self.debug_log_frames:
            self.get_logger().info(f"TX raw frame: {bytes_to_hex(frame)}")


def main(args=None):
    rclpy.init(args=args)
    node = MockArmStm32()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()