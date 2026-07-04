#!/usr/bin/env python3
"""Publish an ON/OFF ROS 2 signal from a bright orange light strip."""

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from light_signal_detector_core import (
    DetectionResult,
    DetectorConfig,
    SignalDebouncer,
    detect_orange_light,
)


class LightSignalDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("light_signal_detector")
        self._declare_parameters()
        self._read_detector_config().validate()

        self._bridge = CvBridge()
        self._debouncer = SignalDebouncer()
        self._last_image_time = self.get_clock().now()
        self._timed_out = False

        self._signal_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("signal_topic").value),
            10,
        )
        self._debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            2,
        )
        self._image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("color_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_callback)
        self.get_logger().info(
            "Light signal detector ready: orange HSV H=%d..%d S>=%d V>=%d, "
            "minimum area=%.1f px and %.2f%% of image"
            % (
                self.get_parameter("h_min").value,
                self.get_parameter("h_max").value,
                self.get_parameter("s_min").value,
                self.get_parameter("v_min").value,
                self.get_parameter("min_component_area").value,
                100.0 * self.get_parameter("min_component_area_ratio").value,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("signal_topic", "/light_signal/on")
        self.declare_parameter("debug_image_topic", "/light_signal/debug_image")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("h_min", 5)
        self.declare_parameter("h_max", 22)
        self.declare_parameter("s_min", 80)
        self.declare_parameter("v_min", 210)
        self.declare_parameter("white_s_max", 160)
        self.declare_parameter("white_v_min", 245)
        self.declare_parameter("white_adjacency_kernel_size", 9)
        self.declare_parameter("close_kernel_size", 7)
        self.declare_parameter("close_iterations", 2)
        self.declare_parameter("min_component_area", 600.0)
        self.declare_parameter("min_component_area_ratio", 0.01)
        self.declare_parameter("min_white_core_pixels", 50)
        self.declare_parameter("min_white_core_ratio", 0.05)

        self.declare_parameter("on_frames", 3)
        self.declare_parameter("off_frames", 5)
        self.declare_parameter("image_timeout_sec", 0.5)

    def _read_detector_config(self) -> DetectorConfig:
        return DetectorConfig(
            h_min=int(self.get_parameter("h_min").value),
            h_max=int(self.get_parameter("h_max").value),
            s_min=int(self.get_parameter("s_min").value),
            v_min=int(self.get_parameter("v_min").value),
            white_s_max=int(self.get_parameter("white_s_max").value),
            white_v_min=int(self.get_parameter("white_v_min").value),
            white_adjacency_kernel_size=int(
                self.get_parameter("white_adjacency_kernel_size").value
            ),
            close_kernel_size=int(self.get_parameter("close_kernel_size").value),
            close_iterations=int(self.get_parameter("close_iterations").value),
            min_component_area=float(self.get_parameter("min_component_area").value),
            min_component_area_ratio=float(
                self.get_parameter("min_component_area_ratio").value
            ),
            min_white_core_pixels=int(
                self.get_parameter("min_white_core_pixels").value
            ),
            min_white_core_ratio=float(
                self.get_parameter("min_white_core_ratio").value
            ),
        )

    def _image_callback(self, image_msg: Image) -> None:
        self._last_image_time = self.get_clock().now()
        self._timed_out = False
        try:
            image = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            result = detect_orange_light(image, self._read_detector_config())
        except (CvBridgeError, ValueError, cv2.error) as exc:
            self.get_logger().error(
                f"Light signal image processing failed: {exc}",
                throttle_duration_sec=2.0,
            )
            self._publish_frame_state(False)
            return

        state = self._publish_frame_state(result.detected)
        if bool(self.get_parameter("publish_debug_image").value):
            self._publish_debug_image(image_msg, image, result, state)

    def _publish_frame_state(self, detected: bool) -> bool:
        try:
            state = self._debouncer.update(
                detected,
                int(self.get_parameter("on_frames").value),
                int(self.get_parameter("off_frames").value),
            )
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            self._debouncer.force_off()
            state = False
        self._signal_pub.publish(Bool(data=state))
        return state

    def _watchdog_callback(self) -> None:
        timeout = float(self.get_parameter("image_timeout_sec").value)
        if timeout <= 0.0:
            return
        elapsed = (self.get_clock().now() - self._last_image_time).nanoseconds / 1.0e9
        if elapsed < timeout:
            return
        if not self._timed_out:
            self.get_logger().warn(
                f"No color image for {elapsed:.2f}s; forcing light signal OFF"
            )
            self._debouncer.force_off()
            self._timed_out = True
        # Keep publishing OFF so late subscribers do not inherit an unknown state.
        self._signal_pub.publish(Bool(data=False))

    def _publish_debug_image(
        self,
        source_msg: Image,
        image: np.ndarray,
        result: DetectionResult,
        state: bool,
    ) -> None:
        overlay = image.copy()
        overlay[result.orange_mask > 0] = (0, 165, 255)
        overlay[result.white_core_mask > 0] = (255, 255, 255)
        debug = cv2.addWeighted(image, 0.55, overlay, 0.45, 0.0)

        contours, _ = cv2.findContours(
            result.largest_component_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if contours:
            cv2.drawContours(debug, contours, -1, (0, 255, 0), 2)
            x, y, width, height = cv2.boundingRect(contours[0])
            cv2.rectangle(debug, (x, y), (x + width, y + height), (0, 255, 0), 2)

        status_color = (0, 255, 0) if state else (0, 0, 255)
        minimum_core_pixels = int(self.get_parameter("min_white_core_pixels").value)
        minimum_core_ratio = float(self.get_parameter("min_white_core_ratio").value)
        lines = (
            f"SIGNAL: {'ON' if state else 'OFF'} "
            f"frame={'detected' if result.detected else 'not detected'}",
            f"area={result.largest_area:.0f}/{result.required_component_area:.0f} "
            f"orange={result.orange_pixel_count} "
            f"core={result.largest_white_core_pixel_count}/{minimum_core_pixels} "
            f"core_ratio={result.largest_white_core_ratio:.1%}/{minimum_core_ratio:.1%}",
            "orange mask=orange, overexposed core=white, largest component=green",
        )
        for index, text in enumerate(lines):
            origin = (12, 30 + index * 26)
            cv2.putText(
                debug,
                text,
                origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                text,
                origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                status_color if index == 0 else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        debug_msg = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = source_msg.header
        self._debug_pub.publish(debug_msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node: Optional[LightSignalDetectorNode] = None
    try:
        node = LightSignalDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._debouncer.force_off()
            node._signal_pub.publish(Bool(data=False))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
