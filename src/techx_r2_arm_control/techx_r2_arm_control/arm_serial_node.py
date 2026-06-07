import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import serial

from techx_r2_arm_interfaces.action import GetHead, SetEndEffector, ExecuteAction

from .protocol import (
    ACTION_TAKE_HEAD,
    CMD_ACTION_FEEDBACK,
    CMD_IO_STATUS,
    ERROR_ACTION_TIMEOUT,
    ERROR_BUSY,
    ERROR_INVALID_ID,
    ERROR_OK,
    ERROR_MOTOR_ERROR,
    FRAME_TYPE_STM32_TO_HOST,
    IO_ARM_SUCTION_PUMP,
    IO_GRIPPER_CLAW,
    IO_REAR_STORAGE_PUMP,
    STATE_ACK,
    STATE_DONE,
    STATE_ERROR,
    STATE_ESTOP,
    STATE_RUNNING,
    STATE_TIMEOUT,
    TARGET_COMPOSITE_TASK,
    TARGET_GRIPPER_ARM,
    TARGET_KFS_SUCTION_ARM,
    action_id_to_name,
    bytes_to_hex,
    error_to_name,
    frame_to_bytes,
    io_id_to_name,
    pack_execute_action_frame,
    pack_set_end_effector_frame,
    pack_take_head_frame,
    parse_frames,
    state_to_name,
    target_id_to_name,
    unpack_action_feedback,
    unpack_io_status,
)


class ArmSerialNode(Node):
    def __init__(self):
        super().__init__("arm_serial_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("ack_timeout_ms", 100)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("take_head_timeout_ms", 8000)
        self.declare_parameter("debug_log_frames", False)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.ack_timeout_s = float(self.get_parameter("ack_timeout_ms").value) / 1000.0
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.take_head_timeout_ms = int(self.get_parameter("take_head_timeout_ms").value)
        self.debug_log_frames = bool(self.get_parameter("debug_log_frames").value)

        self.seq = 0
        self.seq_lock = threading.Lock()

        self.task_lock = threading.Lock()
        self.pending_task = None

        self.rx_buffer = bytearray()

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.02,
            )
        except serial.SerialException as exc:
            self.get_logger().fatal(
                f"failed to open serial port {self.port}, baudrate={self.baudrate}: {exc}"
            )
            raise

        self.read_thread = threading.Thread(
            target=self._serial_read_loop,
            daemon=True,
        )
        self.read_thread.start()

        self.get_head_server = ActionServer(
            self,
            GetHead,
            "/r2_arm/get_head",
            self._execute_get_head,
        )

        self.set_end_effector_server = ActionServer(
            self,
            SetEndEffector,
            "/r2_arm/set_end_effector",
            self._execute_set_end_effector,
        )

        self.execute_action_server = ActionServer(
            self,
            ExecuteAction,
            "/r2_arm/execute_action",
            self._execute_execute_action,
        )

        self.get_logger().info(
            f"arm_serial_node started: port={self.port}, "
            f"baudrate={self.baudrate}, debug_log_frames={self.debug_log_frames}"
        )

    def destroy_node(self):
        try:
            if hasattr(self, "serial") and self.serial is not None and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass

        super().destroy_node()

    def _next_seq(self) -> int:
        with self.seq_lock:
            self.seq = (self.seq + 1) & 0xFF
            if self.seq == 0:
                self.seq = 1
            return self.seq

    def _execute_get_head(self, goal_handle):
        self.get_logger().info(
            f"GetHead goal received, timeout_ms={self.take_head_timeout_ms}"
        )

        with self.task_lock:
            if self.pending_task is not None:
                result = GetHead.Result()
                result.success = False
                result.final_state = STATE_ERROR
                result.error_code = ERROR_BUSY
                result.message = "arm serial node is busy"

                self.get_logger().warn(result.message)
                goal_handle.abort()
                return result

            seq = self._next_seq()

            pending_task = {
                "task_type": "take_head",
                "seq": seq,
                "target_id": TARGET_COMPOSITE_TASK,
                "action_id": ACTION_TAKE_HEAD,
                "ack_event": threading.Event(),
                "done_event": threading.Event(),
                "final_state": STATE_TIMEOUT,
                "error_code": ERROR_OK,
                "message": "",
                "goal_handle": goal_handle,
            }

            self.pending_task = pending_task

        frame = pack_take_head_frame(
            seq=seq,
            timeout_ms=self.take_head_timeout_ms,
        )

        result = GetHead.Result()

        try:
            got_ack = False

            for attempt in range(1, self.max_retries + 1):
                self.serial.write(frame)
                self.serial.flush()

                if self.debug_log_frames:
                    self.get_logger().info(
                        f"TX raw frame: {bytes_to_hex(frame)}"
                    )

                self.get_logger().info(
                    f"send TAKE_HEAD seq={seq}, attempt={attempt}"
                )

                if pending_task["ack_event"].wait(self.ack_timeout_s):
                    got_ack = True
                    break

                self.get_logger().warn(
                    f"ACK timeout: seq={seq}, attempt={attempt}"
                )

            if not got_ack:
                result.success = False
                result.final_state = STATE_TIMEOUT
                result.error_code = ERROR_ACTION_TIMEOUT
                result.message = "ACK timeout after retries"

                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            wait_done_timeout_s = max(self.take_head_timeout_ms / 1000.0, 1.0) + 1.0

            if not pending_task["done_event"].wait(wait_done_timeout_s):
                result.success = False
                result.final_state = STATE_TIMEOUT
                result.error_code = ERROR_ACTION_TIMEOUT
                result.message = "TAKE_HEAD task timeout waiting DONE"

                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            final_state = pending_task["final_state"]
            error_code = pending_task["error_code"]
            message = pending_task["message"]

            result.success = final_state == STATE_DONE
            result.final_state = final_state
            result.error_code = error_code
            result.message = message

            if result.success:
                self.get_logger().info("TAKE_HEAD success")
                goal_handle.succeed()
            else:
                self.get_logger().error(
                    f"TAKE_HEAD failed: "
                    f"final_state={state_to_name(final_state)}, "
                    f"error_code=0x{error_code:04X}({error_to_name(error_code)})"
                )
                goal_handle.abort()

            return result

        finally:
            with self.task_lock:
                if self.pending_task is pending_task:
                    self.pending_task = None

    def _execute_set_end_effector(self, goal_handle):
        goal = goal_handle.request

        self.get_logger().info(
            f"SetEndEffector goal received: "
            f"io_id=0x{goal.io_id:02X}({io_id_to_name(goal.io_id)}), "
            f"state={goal.state}, hold_ms={goal.hold_ms}"
        )

        valid_io_ids = (
            IO_GRIPPER_CLAW,
            IO_ARM_SUCTION_PUMP,
            IO_REAR_STORAGE_PUMP,
        )

        result = SetEndEffector.Result()

        if goal.io_id not in valid_io_ids or goal.state not in (0, 1):
            result.success = False
            result.final_state = STATE_ERROR
            result.error_code = ERROR_INVALID_ID
            result.feedback_ok = False
            result.message = (
                f"invalid io_id/state: io_id=0x{goal.io_id:02X}, state={goal.state}"
            )

            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        with self.task_lock:
            if self.pending_task is not None:
                result.success = False
                result.final_state = STATE_ERROR
                result.error_code = ERROR_BUSY
                result.feedback_ok = False
                result.message = "arm serial node is busy"

                self.get_logger().warn(result.message)
                goal_handle.abort()
                return result

            seq = self._next_seq()

            pending_task = {
                "task_type": "set_end_effector",
                "seq": seq,
                "io_id": int(goal.io_id),
                "state_cmd": int(goal.state),
                "ack_event": threading.Event(),
                "done_event": threading.Event(),
                "final_state": STATE_TIMEOUT,
                "error_code": ERROR_OK,
                "feedback_ok": False,
                "message": "",
                "goal_handle": goal_handle,
            }

            self.pending_task = pending_task

        frame = pack_set_end_effector_frame(
            seq=seq,
            io_id=int(goal.io_id),
            state=int(goal.state),
            hold_ms=int(goal.hold_ms),
        )

        try:
            got_response = False

            for attempt in range(1, self.max_retries + 1):
                self.serial.write(frame)
                self.serial.flush()

                if self.debug_log_frames:
                    self.get_logger().info(f"TX raw frame: {bytes_to_hex(frame)}")

                self.get_logger().info(
                    f"send SET_END_EFFECTOR seq={seq}, "
                    f"io_id=0x{goal.io_id:02X}({io_id_to_name(goal.io_id)}), "
                    f"state={goal.state}, attempt={attempt}"
                )

                if pending_task["ack_event"].wait(self.ack_timeout_s):
                    got_response = True
                    break

                self.get_logger().warn(
                    f"IO_STATUS timeout: seq={seq}, attempt={attempt}"
                )

            if not got_response:
                result.success = False
                result.final_state = STATE_TIMEOUT
                result.error_code = ERROR_ACTION_TIMEOUT
                result.feedback_ok = False
                result.message = "IO_STATUS timeout after retries"

                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            final_state = pending_task["final_state"]
            error_code = pending_task["error_code"]
            feedback_ok = pending_task["feedback_ok"]
            message = pending_task["message"]

            result.success = final_state == STATE_DONE
            result.final_state = final_state
            result.error_code = error_code
            result.feedback_ok = feedback_ok
            result.message = message

            if result.success:
                self.get_logger().info(
                    f"SET_END_EFFECTOR success: "
                    f"io_id=0x{goal.io_id:02X}({io_id_to_name(goal.io_id)}), "
                    f"state={goal.state}"
                )
                goal_handle.succeed()
            else:
                self.get_logger().error(
                    f"SET_END_EFFECTOR failed: "
                    f"final_state={state_to_name(final_state)}, "
                    f"error_code=0x{error_code:04X}({error_to_name(error_code)}), "
                    f"feedback_ok={feedback_ok}"
                )
                goal_handle.abort()

            return result

        finally:
            with self.task_lock:
                if self.pending_task is pending_task:
                    self.pending_task = None

    def _execute_execute_action(self, goal_handle):
        goal = goal_handle.request

        self.get_logger().info(
            f"ExecuteAction goal received: "
            f"target_id=0x{goal.target_id:02X}({target_id_to_name(goal.target_id)}), "
            f"action_id=0x{goal.action_id:04X}({action_id_to_name(goal.action_id)}), "
            f"timeout_ms={goal.timeout_ms}, param={goal.param}, flags=0x{goal.flags:02X}"
        )

        valid_target_ids = (
            TARGET_GRIPPER_ARM,
            TARGET_KFS_SUCTION_ARM,
            TARGET_COMPOSITE_TASK,
        )

        result = ExecuteAction.Result()

        if goal.target_id not in valid_target_ids:
            result.success = False
            result.final_state = STATE_ERROR
            result.error_code = ERROR_INVALID_ID
            result.message = f"invalid target_id=0x{goal.target_id:02X}"

            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        timeout_ms = int(goal.timeout_ms)
        if timeout_ms <= 0:
            timeout_ms = 3000

        with self.task_lock:
            if self.pending_task is not None:
                result.success = False
                result.final_state = STATE_ERROR
                result.error_code = ERROR_BUSY
                result.message = "arm serial node is busy"

                self.get_logger().warn(result.message)
                goal_handle.abort()
                return result

            seq = self._next_seq()

            pending_task = {
                "task_type": "execute_action",
                "seq": seq,
                "target_id": int(goal.target_id),
                "action_id": int(goal.action_id),
                "ack_event": threading.Event(),
                "done_event": threading.Event(),
                "final_state": STATE_TIMEOUT,
                "error_code": ERROR_OK,
                "message": "",
                "goal_handle": goal_handle,
            }

            self.pending_task = pending_task

        frame = pack_execute_action_frame(
            seq=seq,
            target_id=int(goal.target_id),
            action_id=int(goal.action_id),
            timeout_ms=timeout_ms,
            param=int(goal.param),
            flags=int(goal.flags),
        )

        try:
            got_ack = False

            for attempt in range(1, self.max_retries + 1):
                self.serial.write(frame)
                self.serial.flush()

                if self.debug_log_frames:
                    self.get_logger().info(f"TX raw frame: {bytes_to_hex(frame)}")

                self.get_logger().info(
                    f"send EXECUTE_ACTION seq={seq}, "
                    f"target_id=0x{goal.target_id:02X}({target_id_to_name(goal.target_id)}), "
                    f"action_id=0x{goal.action_id:04X}({action_id_to_name(goal.action_id)}), "
                    f"attempt={attempt}"
                )

                if pending_task["ack_event"].wait(self.ack_timeout_s):
                    got_ack = True
                    break

                self.get_logger().warn(
                    f"ACK timeout: seq={seq}, attempt={attempt}"
                )

            if not got_ack:
                result.success = False
                result.final_state = STATE_TIMEOUT
                result.error_code = ERROR_ACTION_TIMEOUT
                result.message = "ACK timeout after retries"

                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            wait_done_timeout_s = max(timeout_ms / 1000.0, 1.0) + 1.0

            if not pending_task["done_event"].wait(wait_done_timeout_s):
                result.success = False
                result.final_state = STATE_TIMEOUT
                result.error_code = ERROR_ACTION_TIMEOUT
                result.message = "EXECUTE_ACTION task timeout waiting DONE"

                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            final_state = pending_task["final_state"]
            error_code = pending_task["error_code"]
            message = pending_task["message"]

            result.success = final_state == STATE_DONE
            result.final_state = final_state
            result.error_code = error_code
            result.message = message

            if result.success:
                self.get_logger().info(
                    f"EXECUTE_ACTION success: "
                    f"target_id=0x{goal.target_id:02X}({target_id_to_name(goal.target_id)}), "
                    f"action_id=0x{goal.action_id:04X}({action_id_to_name(goal.action_id)})"
                )
                goal_handle.succeed()
            else:
                self.get_logger().error(
                    f"EXECUTE_ACTION failed: "
                    f"final_state={state_to_name(final_state)}, "
                    f"error_code=0x{error_code:04X}({error_to_name(error_code)})"
                )
                goal_handle.abort()

            return result

        finally:
            with self.task_lock:
                if self.pending_task is pending_task:
                    self.pending_task = None

    def _serial_read_loop(self):
        while rclpy.ok():
            try:
                data = self.serial.read(64)

                if not data:
                    continue

                if self.debug_log_frames:
                    self.get_logger().info(
                        f"RX bytes: {bytes_to_hex(data)}"
                    )

                self.rx_buffer.extend(data)
                frames = parse_frames(self.rx_buffer)

                for frame in frames:
                    if self.debug_log_frames:
                        self.get_logger().info(
                            f"RX parsed frame: {bytes_to_hex(frame_to_bytes(frame))}"
                        )
                    self._dispatch_frame(frame)

            except serial.SerialException as exc:
                self.get_logger().error(f"serial read error: {exc}")
                time.sleep(0.1)
            except Exception as exc:
                self.get_logger().error(f"unexpected read loop error: {exc}")
                time.sleep(0.1)

    def _dispatch_frame(self, frame):
        if frame.frame_type != FRAME_TYPE_STM32_TO_HOST:
            self.get_logger().warn(
                f"ignore frame with wrong frame_type=0x{frame.frame_type:02X}"
            )
            return

        if frame.cmd_type == CMD_ACTION_FEEDBACK:
            self._dispatch_action_feedback(frame)
            return

        if frame.cmd_type == CMD_IO_STATUS:
            self._dispatch_io_status(frame)
            return

        self.get_logger().warn(
            f"ignore unsupported cmd_type=0x{frame.cmd_type:02X}, seq={frame.seq}"
        )

    def _dispatch_action_feedback(self, frame):
        feedback = unpack_action_feedback(frame)

        self.get_logger().info(
            f"RX ACTION_FEEDBACK seq={feedback.seq}, "
            f"target_id=0x{feedback.target_id:02X}, "
            f"action_id=0x{feedback.action_id:04X}, "
            f"state={feedback.state}({state_to_name(feedback.state)}), "
            f"error_code=0x{feedback.error_code:04X}({error_to_name(feedback.error_code)}), "
            f"pose_id={feedback.current_pose_id}"
        )

        with self.task_lock:
            pending_task = self.pending_task

        if pending_task is None:
            self.get_logger().warn(
                f"received ACTION_FEEDBACK but no pending task, seq={feedback.seq}"
            )
            return

        if pending_task.get("task_type") not in ("take_head", "execute_action"):
            self.get_logger().warn(
                f"received ACTION_FEEDBACK but pending task is {pending_task.get('task_type')}"
            )
            return

        if feedback.seq != pending_task["seq"]:
            self.get_logger().warn(
                f"feedback seq mismatch: rx={feedback.seq}, pending={pending_task['seq']}"
            )
            return

        if feedback.target_id != pending_task["target_id"]:
            self.get_logger().warn(
                f"feedback target mismatch: rx=0x{feedback.target_id:02X}, "
                f"pending=0x{pending_task['target_id']:02X}"
            )
            return

        if feedback.action_id != pending_task["action_id"]:
            self.get_logger().warn(
                f"feedback action mismatch: rx=0x{feedback.action_id:04X}, "
                f"pending=0x{pending_task['action_id']:04X}"
            )
            return

        goal_handle = pending_task["goal_handle"]

        if pending_task.get("task_type") == "take_head":
            ros_feedback = GetHead.Feedback()
        else:
            ros_feedback = ExecuteAction.Feedback()

        ros_feedback.state = feedback.state

        if feedback.state == STATE_ACK:
            ros_feedback.message = "ACK"
            pending_task["ack_event"].set()
            goal_handle.publish_feedback(ros_feedback)
            return

        if feedback.state == STATE_RUNNING:
            ros_feedback.message = "RUNNING"

            # 有些 STM32 可能先回 RUNNING，没有单独回 ACK。
            # RUNNING 也表示命令已被接受。
            pending_task["ack_event"].set()

            goal_handle.publish_feedback(ros_feedback)
            return

        if feedback.state in (STATE_DONE, STATE_ERROR, STATE_TIMEOUT, STATE_ESTOP):
            pending_task["final_state"] = feedback.state
            pending_task["error_code"] = feedback.error_code

            if feedback.state == STATE_DONE:
                pending_task["message"] = "DONE"
            elif feedback.state == STATE_ERROR:
                pending_task["message"] = (
                    f"ERROR error_code=0x{feedback.error_code:04X}"
                    f"({error_to_name(feedback.error_code)})"
                )
            elif feedback.state == STATE_TIMEOUT:
                pending_task["message"] = (
                    f"STM32 TIMEOUT error_code=0x{feedback.error_code:04X}"
                    f"({error_to_name(feedback.error_code)})"
                )
            elif feedback.state == STATE_ESTOP:
                pending_task["message"] = (
                    f"ESTOP error_code=0x{feedback.error_code:04X}"
                    f"({error_to_name(feedback.error_code)})"
                )

            ros_feedback.message = pending_task["message"]
            goal_handle.publish_feedback(ros_feedback)

            pending_task["done_event"].set()
            return

        self.get_logger().warn(
            f"unknown ACTION_FEEDBACK state={feedback.state}, seq={feedback.seq}"
        )

    def _dispatch_io_status(self, frame):
        io_status = unpack_io_status(frame)

        self.get_logger().info(
            f"RX IO_STATUS seq={io_status.seq}, "
            f"io_id=0x{io_status.io_id:02X}({io_id_to_name(io_status.io_id)}), "
            f"state={io_status.state}, "
            f"feedback_ok={io_status.feedback_ok}, "
            f"error_code=0x{io_status.error_code:04X}({error_to_name(io_status.error_code)})"
        )

        with self.task_lock:
            pending_task = self.pending_task

        if pending_task is None:
            self.get_logger().warn(
                f"received IO_STATUS but no pending task, seq={io_status.seq}"
            )
            return

        if pending_task.get("task_type") != "set_end_effector":
            self.get_logger().warn(
                f"received IO_STATUS but pending task is {pending_task.get('task_type')}"
            )
            return

        if io_status.seq != pending_task["seq"]:
            self.get_logger().warn(
                f"IO_STATUS seq mismatch: rx={io_status.seq}, pending={pending_task['seq']}"
            )
            return

        if io_status.io_id != pending_task["io_id"]:
            self.get_logger().warn(
                f"IO_STATUS io_id mismatch: rx=0x{io_status.io_id:02X}, "
                f"pending=0x{pending_task['io_id']:02X}"
            )
            return

        goal_handle = pending_task["goal_handle"]

        ros_feedback = SetEndEffector.Feedback()
        ros_feedback.feedback_ok = bool(io_status.feedback_ok)

        final_error_code = io_status.error_code

        if io_status.error_code == ERROR_OK and not io_status.feedback_ok:
            final_error_code = ERROR_MOTOR_ERROR

        if io_status.feedback_ok and final_error_code == ERROR_OK:
            pending_task["final_state"] = STATE_DONE
            pending_task["error_code"] = ERROR_OK
            pending_task["feedback_ok"] = True
            pending_task["message"] = "DONE"

            ros_feedback.state = STATE_DONE
            ros_feedback.message = "DONE"
        else:
            pending_task["final_state"] = STATE_ERROR
            pending_task["error_code"] = final_error_code
            pending_task["feedback_ok"] = False
            pending_task["message"] = (
                f"ERROR error_code=0x{final_error_code:04X}"
                f"({error_to_name(final_error_code)})"
            )

            ros_feedback.state = STATE_ERROR
            ros_feedback.message = pending_task["message"]

        goal_handle.publish_feedback(ros_feedback)

        pending_task["ack_event"].set()
        pending_task["done_event"].set()


def main(args=None):
    rclpy.init(args=args)

    node = ArmSerialNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()