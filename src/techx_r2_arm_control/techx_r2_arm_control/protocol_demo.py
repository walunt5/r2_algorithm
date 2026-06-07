from .protocol import (
    CMD_EXECUTE_ACTION,
    ACTION_TAKE_HEAD,
    TARGET_COMPOSITE_TASK,
    pack_take_head_frame,
)


def main():
    seq = 1
    timeout_ms = 8000

    frame = pack_take_head_frame(seq=seq, timeout_ms=timeout_ms)

    print("TAKE_HEAD frame:")
    print(" ".join(f"{b:02X}" for b in frame))
    print(f"length = {len(frame)}")
    print("")
    print("decoded meaning:")
    print(f"frame_type = 0x{frame[2]:02X}")
    print(f"seq        = 0x{frame[3]:02X}")
    print(f"cmd_type   = 0x{frame[4]:02X}  expected 0x{CMD_EXECUTE_ACTION:02X}")
    print(f"target_id  = 0x{frame[5]:02X}  expected 0x{TARGET_COMPOSITE_TASK:02X}")
    print(f"action_id  = 0x{frame[6] | (frame[7] << 8):04X} expected 0x{ACTION_TAKE_HEAD:04X}")
    print(f"timeout_ms = {frame[8] | (frame[9] << 8)}")
    print(f"param      = {frame[10] | (frame[11] << 8)}")
    print(f"flags      = 0x{frame[12]:02X}")