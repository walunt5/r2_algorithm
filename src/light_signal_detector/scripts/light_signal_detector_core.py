"""Core image processing and debounce logic for the light detector package."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    """Thresholds use OpenCV HSV ranges: H=[0, 179], S/V=[0, 255]."""

    h_min: int = 5
    h_max: int = 22
    s_min: int = 80
    v_min: int = 210
    white_s_max: int = 160
    white_v_min: int = 245
    white_adjacency_kernel_size: int = 9
    close_kernel_size: int = 7
    close_iterations: int = 2
    min_component_area: float = 600.0
    min_component_area_ratio: float = 0.01
    min_white_core_pixels: int = 50
    min_white_core_ratio: float = 0.05

    def validate(self) -> None:
        for name, value, maximum in (
            ("h_min", self.h_min, 179),
            ("h_max", self.h_max, 179),
            ("s_min", self.s_min, 255),
            ("v_min", self.v_min, 255),
            ("white_s_max", self.white_s_max, 255),
            ("white_v_min", self.white_v_min, 255),
        ):
            if not 0 <= value <= maximum:
                raise ValueError(f"{name} must be in [0, {maximum}], got {value}")
        if self.h_min > self.h_max:
            raise ValueError("h_min must not be greater than h_max for orange detection")
        for name, value in (
            ("white_adjacency_kernel_size", self.white_adjacency_kernel_size),
            ("close_kernel_size", self.close_kernel_size),
        ):
            if value <= 0 or value % 2 == 0:
                raise ValueError(f"{name} must be a positive odd integer, got {value}")
        if self.close_iterations < 0:
            raise ValueError("close_iterations must be non-negative")
        if self.min_component_area <= 0.0:
            raise ValueError("min_component_area must be positive")
        if not 0.0 <= self.min_component_area_ratio <= 1.0:
            raise ValueError("min_component_area_ratio must be in [0.0, 1.0]")
        if self.min_white_core_pixels < 0:
            raise ValueError("min_white_core_pixels must be non-negative")
        if not 0.0 <= self.min_white_core_ratio <= 1.0:
            raise ValueError("min_white_core_ratio must be in [0.0, 1.0]")


@dataclass(frozen=True)
class DetectionResult:
    detected: bool
    largest_area: float
    required_component_area: float
    orange_pixel_count: int
    white_core_pixel_count: int
    largest_white_core_pixel_count: int
    largest_white_core_ratio: float
    orange_mask: np.ndarray
    white_core_mask: np.ndarray
    combined_mask: np.ndarray
    largest_component_mask: np.ndarray


def detect_orange_light(image_bgr: np.ndarray, config: DetectorConfig) -> DetectionResult:
    """Detect a bright orange region and any adjacent overexposed white core."""
    config.validate()
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError(f"Expected a BGR image with shape HxWx3, got {image_bgr.shape}")
    if image_bgr.dtype != np.uint8:
        raise ValueError(f"Expected uint8 BGR image, got {image_bgr.dtype}")

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    orange_mask = cv2.inRange(
        hsv,
        (config.h_min, config.s_min, config.v_min),
        (config.h_max, 255, 255),
    )

    # A severely overexposed LED can lose saturation and therefore has no useful
    # hue. White pixels are accepted only when they touch a detected orange halo.
    white_candidate_mask = cv2.inRange(
        hsv,
        (0, 0, config.white_v_min),
        (179, config.white_s_max, 255),
    )
    adjacency_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (config.white_adjacency_kernel_size, config.white_adjacency_kernel_size),
    )
    orange_neighbourhood = cv2.dilate(orange_mask, adjacency_kernel)

    # Keep each complete bright component that touches the orange halo. The old
    # intersection-only approach retained just a thin boundary, making a large
    # overexposed centre appear to contain only a handful of core pixels.
    white_component_count, white_labels = cv2.connectedComponents(
        white_candidate_mask,
        connectivity=8,
    )
    white_core_mask = np.zeros_like(white_candidate_mask)
    if white_component_count > 1:
        touching_labels = np.unique(white_labels[orange_neighbourhood > 0])
        touching_labels = touching_labels[touching_labels != 0]
        if touching_labels.size:
            white_core_mask[np.isin(white_labels, touching_labels)] = 255

    combined_mask = cv2.bitwise_or(orange_mask, white_core_mask)
    if config.close_iterations:
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (config.close_kernel_size, config.close_kernel_size),
        )
        combined_mask = cv2.morphologyEx(
            combined_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=config.close_iterations,
        )

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        combined_mask,
        connectivity=8,
    )
    largest_area = 0.0
    largest_component_mask = np.zeros_like(combined_mask)
    if component_count > 1:
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_label = int(np.argmax(areas)) + 1
        largest_area = float(stats[largest_label, cv2.CC_STAT_AREA])
        largest_component_mask[labels == largest_label] = 255

    largest_white_core_mask = cv2.bitwise_and(
        white_core_mask,
        largest_component_mask,
    )
    largest_white_core_pixel_count = int(cv2.countNonZero(largest_white_core_mask))
    largest_white_core_ratio = (
        largest_white_core_pixel_count / largest_area if largest_area > 0.0 else 0.0
    )
    image_area = float(image_bgr.shape[0] * image_bgr.shape[1])
    required_component_area = max(
        config.min_component_area,
        image_area * config.min_component_area_ratio,
    )
    detected = (
        largest_area >= required_component_area
        and largest_white_core_pixel_count >= config.min_white_core_pixels
        and largest_white_core_ratio >= config.min_white_core_ratio
    )

    return DetectionResult(
        detected=detected,
        largest_area=largest_area,
        required_component_area=required_component_area,
        orange_pixel_count=int(cv2.countNonZero(orange_mask)),
        white_core_pixel_count=int(cv2.countNonZero(white_core_mask)),
        largest_white_core_pixel_count=largest_white_core_pixel_count,
        largest_white_core_ratio=largest_white_core_ratio,
        orange_mask=orange_mask,
        white_core_mask=white_core_mask,
        combined_mask=combined_mask,
        largest_component_mask=largest_component_mask,
    )


class SignalDebouncer:
    """Require consecutive frame decisions before changing the output state."""

    def __init__(self) -> None:
        self.state = False
        self.success_streak = 0
        self.failure_streak = 0

    def update(self, detected: bool, on_frames: int, off_frames: int) -> bool:
        if on_frames <= 0 or off_frames <= 0:
            raise ValueError("on_frames and off_frames must be positive")

        if detected:
            self.success_streak += 1
            self.failure_streak = 0
            if self.success_streak >= on_frames:
                self.state = True
        else:
            self.failure_streak += 1
            self.success_streak = 0
            if self.failure_streak >= off_frames:
                self.state = False
        return self.state

    def force_off(self) -> None:
        self.state = False
        self.success_streak = 0
        self.failure_streak = 0
