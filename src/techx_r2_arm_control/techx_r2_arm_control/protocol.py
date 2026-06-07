from dataclasses import dataclass
import struct


FRAME_LEN = 18

HEADER_0 = 0xAA
HEADER_1 = 0x55

FRAME_TYPE_HOST_TO_STM32 = 0x01
FRAME_TYPE_STM32_TO_HOST = 0x02

CMD_EXECUTE_ACTION = 0x10
CMD_ACTION_FEEDBACK = 0x11
CMD_SET_END_EFFECTOR = 0x20
CMD_IO_STATUS = 0x21
CMD_ESTOP = 0xE0

# 机械臂动作 target_id
TARGET_GRIPPER_ARM = 0x01
TARGET_KFS_SUCTION_ARM = 0x02

# 复合任务 target_id：
# 下位机内部自己完成多模块配合，例如取端头。
TARGET_COMPOSITE_TASK = 0x03

# 取端头复合任务：
# 下位机收到后，自己完成底盘定点、对准、机械臂夹取、检测、抬起等流程。
ACTION_TAKE_HEAD = 0x0301

# 夹爪机械臂动作
ACTION_GRIPPER_ARM_FLAT = 0x0101
ACTION_GRIPPER_ARM_LIFT_UP = 0x0102
ACTION_GRIPPER_ARM_TILT = 0x0103

# 中心吸盘机械臂动作
ACTION_SUCTION_ARM_HOME = 0x0201
ACTION_SUCTION_ARM_MEILIN_READY = 0x0202
ACTION_KFS_PICK_DELTA_0 = 0x0210
ACTION_KFS_PICK_DELTA_PLUS_200 = 0x0211
ACTION_KFS_PICK_DELTA_MINUS_200 = 0x0212
ACTION_KFS_PICK_DELTA_PLUS_400 = 0x0213
ACTION_KFS_PICK_DELTA_MINUS_400 = 0x0214
ACTION_KFS_LIFT_UP = 0x0220
ACTION_KFS_TRANSFER_TO_STORAGE = 0x0230
ACTION_SUCTION_ARM_CARRY_POSE = 0x0240

# SET_END_EFFECTOR 的 io_id
IO_GRIPPER_CLAW = 0x01
IO_ARM_SUCTION_PUMP = 0x02
IO_REAR_STORAGE_PUMP = 0x03

STATE_IDLE = 0
STATE_ACK = 1
STATE_RUNNING = 2
STATE_DONE = 3
STATE_ERROR = 4
STATE_TIMEOUT = 5
STATE_ESTOP = 6

ERROR_OK = 0x0000
ERROR_UNKNOWN_CMD = 0x0001
ERROR_BUSY = 0x0002
ERROR_INVALID_ID = 0x0003
ERROR_CRC_ERROR = 0x0004
ERROR_FRAME_FORMAT_ERROR = 0x0005
ERROR_ACTION_TIMEOUT = 0x0006
ERROR_LIMIT_TRIGGERED = 0x0007
ERROR_MOTOR_ERROR = 0x0008
ERROR_VACUUM_FAIL = 0x0009
ERROR_UNSYNC = 0x000A
ERROR_ESTOP_ACTIVE = 0x000B


@dataclass
class Frame:
    frame_type: int
    seq: int
    cmd_type: int
    data: bytes
    reserved: bytes
    crc: int


@dataclass
class ActionFeedback:
    seq: int
    target_id: int
    action_id: int
    state: int
    error_code: int
    current_pose_id: int


@dataclass
class IoStatus:
    seq: int
    io_id: int
    state: int
    feedback_ok: bool
    error_code: int


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


def pack_frame(frame_type: int, seq: int, cmd_type: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError("data length must be <= 8 bytes")

    data = data.ljust(8, b"\x00")
    reserved = b"\x00\x00\x00"

    raw = bytearray()
    raw.append(HEADER_0)
    raw.append(HEADER_1)
    raw.append(frame_type & 0xFF)
    raw.append(seq & 0xFF)
    raw.append(cmd_type & 0xFF)
    raw.extend(data)
    raw.extend(reserved)

    crc = crc16_modbus(bytes(raw))
    raw.extend(struct.pack("<H", crc))

    if len(raw) != FRAME_LEN:
        raise RuntimeError(f"frame length error: {len(raw)}")

    return bytes(raw)


def try_parse_one_frame(rx_buffer: bytearray):
    while len(rx_buffer) >= FRAME_LEN:
        if rx_buffer[0] != HEADER_0 or rx_buffer[1] != HEADER_1:
            del rx_buffer[0]
            continue

        raw = bytes(rx_buffer[:FRAME_LEN])

        recv_crc = struct.unpack_from("<H", raw, 16)[0]
        calc_crc = crc16_modbus(raw[:16])

        if recv_crc != calc_crc:
            del rx_buffer[0]
            continue

        frame = Frame(
            frame_type=raw[2],
            seq=raw[3],
            cmd_type=raw[4],
            data=raw[5:13],
            reserved=raw[13:16],
            crc=recv_crc,
        )

        del rx_buffer[:FRAME_LEN]
        return frame

    return None


def parse_frames(rx_buffer: bytearray) -> list[Frame]:
    frames = []

    while True:
        frame = try_parse_one_frame(rx_buffer)
        if frame is None:
            break
        frames.append(frame)

    return frames


def pack_execute_action_data(
    target_id: int,
    action_id: int,
    timeout_ms: int,
    param: int = 0,
    flags: int = 0,
) -> bytes:
    """
    EXECUTE_ACTION data:
    data0      target_id
    data1-2    action_id, little-endian
    data3-4    timeout_ms, little-endian
    data5-6    param, int16 little-endian
    data7      flags
    """
    return struct.pack(
        "<BHHhB",
        target_id & 0xFF,
        action_id & 0xFFFF,
        timeout_ms & 0xFFFF,
        int(param),
        flags & 0xFF,
    )


def pack_execute_action_frame(
    seq: int,
    target_id: int,
    action_id: int,
    timeout_ms: int,
    param: int = 0,
    flags: int = 0,
) -> bytes:
    data = pack_execute_action_data(
        target_id=target_id,
        action_id=action_id,
        timeout_ms=timeout_ms,
        param=param,
        flags=flags,
    )

    return pack_frame(
        frame_type=FRAME_TYPE_HOST_TO_STM32,
        seq=seq,
        cmd_type=CMD_EXECUTE_ACTION,
        data=data,
    )


def pack_take_head_frame(seq: int, timeout_ms: int = 8000) -> bytes:
    return pack_execute_action_frame(
        seq=seq,
        target_id=TARGET_COMPOSITE_TASK,
        action_id=ACTION_TAKE_HEAD,
        timeout_ms=timeout_ms,
        param=0,
        flags=0,
    )


def unpack_execute_action_fields(frame: Frame):
    """
    调试 / mock 用：解析 EXECUTE_ACTION data 区。
    """
    if frame.cmd_type != CMD_EXECUTE_ACTION:
        raise ValueError("frame is not EXECUTE_ACTION")

    target_id = frame.data[0]
    action_id = struct.unpack_from("<H", frame.data, 1)[0]
    timeout_ms = struct.unpack_from("<H", frame.data, 3)[0]
    param = struct.unpack_from("<h", frame.data, 5)[0]
    flags = frame.data[7]

    return target_id, action_id, timeout_ms, param, flags


def unpack_action_feedback(frame: Frame) -> ActionFeedback:
    """
    ACTION_FEEDBACK data:
    data0      target_id
    data1-2    action_id
    data3      state
    data4-5    error_code
    data6      current_pose_id
    data7      reserved
    """
    if frame.cmd_type != CMD_ACTION_FEEDBACK:
        raise ValueError("frame is not ACTION_FEEDBACK")

    target_id = frame.data[0]
    action_id = struct.unpack_from("<H", frame.data, 1)[0]
    state = frame.data[3]
    error_code = struct.unpack_from("<H", frame.data, 4)[0]
    current_pose_id = frame.data[6]

    return ActionFeedback(
        seq=frame.seq,
        target_id=target_id,
        action_id=action_id,
        state=state,
        error_code=error_code,
        current_pose_id=current_pose_id,
    )


def pack_action_feedback_data(
    target_id: int,
    action_id: int,
    state: int,
    error_code: int = ERROR_OK,
    current_pose_id: int = 0,
) -> bytes:
    """
    mock STM32 用：打包 ACTION_FEEDBACK data 区。
    上位机正式节点一般只需要 unpack_action_feedback。
    """
    return struct.pack(
        "<BHBHB",
        target_id & 0xFF,
        action_id & 0xFFFF,
        state & 0xFF,
        error_code & 0xFFFF,
        current_pose_id & 0xFF,
    ) + b"\x00"


def pack_set_end_effector_data(
    io_id: int,
    state: int,
    hold_ms: int = 0,
) -> bytes:
    """
    SET_END_EFFECTOR data:
    data0      io_id
    data1      state
    data2-3    hold_ms little-endian
    data4-7    reserved
    """
    return struct.pack(
        "<BBH",
        io_id & 0xFF,
        state & 0xFF,
        hold_ms & 0xFFFF,
    ) + b"\x00\x00\x00\x00"


def pack_set_end_effector_frame(
    seq: int,
    io_id: int,
    state: int,
    hold_ms: int = 0,
) -> bytes:
    data = pack_set_end_effector_data(
        io_id=io_id,
        state=state,
        hold_ms=hold_ms,
    )

    return pack_frame(
        frame_type=FRAME_TYPE_HOST_TO_STM32,
        seq=seq,
        cmd_type=CMD_SET_END_EFFECTOR,
        data=data,
    )


def unpack_set_end_effector_fields(frame: Frame):
    """
    调试 / mock 用：解析 SET_END_EFFECTOR data 区。
    """
    if frame.cmd_type != CMD_SET_END_EFFECTOR:
        raise ValueError("frame is not SET_END_EFFECTOR")

    io_id = frame.data[0]
    state = frame.data[1]
    hold_ms = struct.unpack_from("<H", frame.data, 2)[0]

    return io_id, state, hold_ms


def unpack_io_status(frame: Frame) -> IoStatus:
    """
    IO_STATUS data:
    data0      io_id
    data1      state
    data2      feedback_ok
    data3-4    error_code
    data5-7    reserved
    """
    if frame.cmd_type != CMD_IO_STATUS:
        raise ValueError("frame is not IO_STATUS")

    io_id = frame.data[0]
    state = frame.data[1]
    feedback_ok = frame.data[2] != 0
    error_code = struct.unpack_from("<H", frame.data, 3)[0]

    return IoStatus(
        seq=frame.seq,
        io_id=io_id,
        state=state,
        feedback_ok=feedback_ok,
        error_code=error_code,
    )


def pack_io_status_data(
    io_id: int,
    state: int,
    feedback_ok: bool,
    error_code: int = ERROR_OK,
) -> bytes:
    """
    mock STM32 用：打包 IO_STATUS data 区。
    """
    return struct.pack(
        "<BBBH",
        io_id & 0xFF,
        state & 0xFF,
        1 if feedback_ok else 0,
        error_code & 0xFFFF,
    ) + b"\x00\x00\x00"


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def frame_to_bytes(frame: Frame) -> bytes:
    raw = bytearray()
    raw.append(HEADER_0)
    raw.append(HEADER_1)
    raw.append(frame.frame_type & 0xFF)
    raw.append(frame.seq & 0xFF)
    raw.append(frame.cmd_type & 0xFF)
    raw.extend(frame.data.ljust(8, b"\x00")[:8])
    raw.extend(frame.reserved.ljust(3, b"\x00")[:3])
    raw.extend(struct.pack("<H", frame.crc & 0xFFFF))
    return bytes(raw)


def state_to_name(state: int) -> str:
    names = {
        STATE_IDLE: "IDLE",
        STATE_ACK: "ACK",
        STATE_RUNNING: "RUNNING",
        STATE_DONE: "DONE",
        STATE_ERROR: "ERROR",
        STATE_TIMEOUT: "TIMEOUT",
        STATE_ESTOP: "ESTOP",
    }
    return names.get(state, f"UNKNOWN_STATE_{state}")


def error_to_name(error_code: int) -> str:
    names = {
        ERROR_OK: "OK",
        ERROR_UNKNOWN_CMD: "UNKNOWN_CMD",
        ERROR_BUSY: "BUSY",
        ERROR_INVALID_ID: "INVALID_ID",
        ERROR_CRC_ERROR: "CRC_ERROR",
        ERROR_FRAME_FORMAT_ERROR: "FRAME_FORMAT_ERROR",
        ERROR_ACTION_TIMEOUT: "ACTION_TIMEOUT",
        ERROR_LIMIT_TRIGGERED: "LIMIT_TRIGGERED",
        ERROR_MOTOR_ERROR: "MOTOR_ERROR",
        ERROR_VACUUM_FAIL: "VACUUM_FAIL",
        ERROR_UNSYNC: "UNSYNC",
        ERROR_ESTOP_ACTIVE: "ESTOP_ACTIVE",
    }
    return names.get(error_code, f"UNKNOWN_ERROR_0x{error_code:04X}")


def target_id_to_name(target_id: int) -> str:
    names = {
        TARGET_GRIPPER_ARM: "GRIPPER_ARM",
        TARGET_KFS_SUCTION_ARM: "KFS_SUCTION_ARM",
        TARGET_COMPOSITE_TASK: "COMPOSITE_TASK",
    }
    return names.get(target_id, f"UNKNOWN_TARGET_0x{target_id:02X}")


def action_id_to_name(action_id: int) -> str:
    names = {
        ACTION_GRIPPER_ARM_FLAT: "GRIPPER_ARM_FLAT",
        ACTION_GRIPPER_ARM_LIFT_UP: "GRIPPER_ARM_LIFT_UP",
        ACTION_GRIPPER_ARM_TILT: "GRIPPER_ARM_TILT",
        ACTION_SUCTION_ARM_HOME: "SUCTION_ARM_HOME",
        ACTION_SUCTION_ARM_MEILIN_READY: "SUCTION_ARM_MEILIN_READY",
        ACTION_KFS_PICK_DELTA_0: "KFS_PICK_DELTA_0",
        ACTION_KFS_PICK_DELTA_PLUS_200: "KFS_PICK_DELTA_PLUS_200",
        ACTION_KFS_PICK_DELTA_MINUS_200: "KFS_PICK_DELTA_MINUS_200",
        ACTION_KFS_PICK_DELTA_PLUS_400: "KFS_PICK_DELTA_PLUS_400",
        ACTION_KFS_PICK_DELTA_MINUS_400: "KFS_PICK_DELTA_MINUS_400",
        ACTION_KFS_LIFT_UP: "KFS_LIFT_UP",
        ACTION_KFS_TRANSFER_TO_STORAGE: "KFS_TRANSFER_TO_STORAGE",
        ACTION_SUCTION_ARM_CARRY_POSE: "SUCTION_ARM_CARRY_POSE",
        ACTION_TAKE_HEAD: "TAKE_HEAD",
    }
    return names.get(action_id, f"UNKNOWN_ACTION_0x{action_id:04X}")


def io_id_to_name(io_id: int) -> str:
    names = {
        IO_GRIPPER_CLAW: "GRIPPER_CLAW",
        IO_ARM_SUCTION_PUMP: "ARM_SUCTION_PUMP",
        IO_REAR_STORAGE_PUMP: "REAR_STORAGE_PUMP",
    }
    return names.get(io_id, f"UNKNOWN_IO_0x{io_id:02X}")