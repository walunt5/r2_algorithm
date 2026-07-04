#!/usr/bin/env python3
"""极简 GMK 武器头桥接 —— 收 Jetson 的 UDP，发布一个 ROS2 话题。

只有一个话题 /weapon_target (std_msgs/Float32MultiArray)：
    data = [valid, u, v, z_m, conf]
      valid : 1 有目标 / 0 无目标
      u, v  : 像素中心(左右对准用)
      z_m   : 深度(米，前后距离用)
      conf  : 置信度

标准 ament_python 包。两种跑法：
    # 1) colcon 编译后（推荐）
    colcon build --packages-select gmk_weapon_bridge && source install/setup.bash
    ros2 run gmk_weapon_bridge bridge_node
    # 2) 不编译直接跑（source ROS2 后，在仓库根目录）
    python3 -m gmk_weapon_bridge.bridge_node
可选参数(ROS2 参数)：
    ros2 run gmk_weapon_bridge bridge_node --ros-args -p udp_port:=12345 -p topic:=/weapon_target
"""
from __future__ import annotations

import select
import socket
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray
except ImportError:
    rclpy = None
    Node = object
    Float32MultiArray = None

from gmk_weapon_bridge.protocol import unpack_weapon_packet


class WeaponBridge(Node):
    def __init__(self) -> None:
        if rclpy is None or Float32MultiArray is None:
            raise RuntimeError("ROS2 Python packages are not available; source ROS setup.bash first")
        super().__init__("weapon_bridge")
        self.declare_parameter("udp_port", 12345)
        self.declare_parameter("udp_bind", "0.0.0.0")
        self.declare_parameter("topic", "/weapon_target")
        self.declare_parameter("publish_when_invalid", True)

        port = int(self.get_parameter("udp_port").value)
        bind = str(self.get_parameter("udp_bind").value)
        topic = str(self.get_parameter("topic").value)
        self.publish_when_invalid = bool(self.get_parameter("publish_when_invalid").value)

        # Vision control consumes the newest sample. A reliable queue with depth 5
        # can expose stale samples when a subscriber is slower than the camera.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Float32MultiArray, topic, qos)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind, port))
        self.sock.setblocking(False)
        self._stop_event = threading.Event()
        self._stats_lock = threading.Lock()
        self.last_seq = -1
        self.rx_frames = 0
        self.published_frames = 0
        self.superseded_frames = 0
        self.invalid_packets = 0
        self.last_receive_ns = 0
        self.receiver_thread = threading.Thread(
            target=self.receive_loop,
            name="weapon_udp_rx",
            daemon=True,
        )
        self.receiver_thread.start()
        self.stats_timer = self.create_timer(5.0, self.log_stats)
        self.get_logger().info(
            f"weapon_bridge: udp {bind}:{port} -> {topic}  "
            "qos=best_effort/depth1 data=[valid,u,v,z_m,conf]"
        )

    def receive_loop(self) -> None:
        """Receive immediately and discard stale queued frames after a stall."""
        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select([self.sock], [], [], 0.2)
            except (OSError, ValueError):
                if not self._stop_event.is_set():
                    self.get_logger().error("weapon_bridge: UDP select failed")
                break
            if not readable:
                continue

            # Bound each batch so continuous traffic can never starve publish.
            datagrams = []
            for _ in range(64):
                try:
                    data, _ = self.sock.recvfrom(64)
                except BlockingIOError:
                    break
                except OSError:
                    break
                datagrams.append(data)

            latest = None
            valid_packets = 0
            invalid_packets = 0
            for packet in datagrams:
                unpacked = unpack_weapon_packet(packet)
                if unpacked is None:
                    invalid_packets += 1
                    continue
                latest = unpacked
                valid_packets += 1

            now_ns = time.monotonic_ns()
            with self._stats_lock:
                self.rx_frames += valid_packets
                self.invalid_packets += invalid_packets
                self.superseded_frames += max(0, valid_packets - 1)
                if valid_packets:
                    self.last_receive_ns = now_ns

            if latest is not None:
                self.publish_packet(latest)

    def publish_packet(self, packet) -> None:
        seq, valid, u, v, z, conf = packet
        self.last_seq = seq
        if int(valid) == 0 and not self.publish_when_invalid:
            return
        msg = Float32MultiArray()
        msg.data = [float(valid), float(u), float(v), float(z), float(conf)]
        self.pub.publish(msg)
        with self._stats_lock:
            self.published_frames += 1

    def log_stats(self) -> None:
        with self._stats_lock:
            rx_frames = self.rx_frames
            published_frames = self.published_frames
            superseded_frames = self.superseded_frames
            invalid_packets = self.invalid_packets
            last_receive_ns = self.last_receive_ns
            self.rx_frames = 0
            self.published_frames = 0
            self.superseded_frames = 0
            self.invalid_packets = 0

        age_ms = -1.0
        if last_receive_ns:
            age_ms = (time.monotonic_ns() - last_receive_ns) / 1_000_000.0
        self.get_logger().info(
            f"weapon_bridge: rx={rx_frames / 5.0:.1f} pkt/s, "
            f"pub={published_frames / 5.0:.1f} msg/s, last_seq={self.last_seq}, "
            f"age={age_ms:.1f} ms, superseded={superseded_frames}, invalid={invalid_packets}"
        )

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.sock.close()
        except OSError:
            pass
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are not available; source ROS setup.bash first")
    rclpy.init()
    node = WeaponBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
