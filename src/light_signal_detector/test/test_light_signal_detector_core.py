"""Regression tests for bright-orange signal detection."""

from pathlib import Path
import sys

import cv2
import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from light_signal_detector_core import (  # noqa: E402
    DetectorConfig,
    SignalDebouncer,
    detect_orange_light,
)


def _blank_image() -> np.ndarray:
    return np.zeros((240, 320, 3), dtype=np.uint8)


def test_bright_orange_region_without_white_core_is_rejected() -> None:
    image = _blank_image()
    cv2.rectangle(image, (60, 80), (260, 105), (0, 140, 255), thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert not result.detected
    assert result.largest_area >= 600.0
    assert result.orange_pixel_count > 0
    assert result.largest_white_core_pixel_count == 0


def test_adjacent_overexposed_core_is_included() -> None:
    image = _blank_image()
    cv2.rectangle(image, (60, 80), (260, 115), (0, 140, 255), thickness=-1)
    cv2.rectangle(image, (70, 88), (250, 107), (255, 255, 255), thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert result.detected
    assert result.largest_white_core_pixel_count > 3000
    assert result.largest_white_core_ratio >= 0.05
    assert result.largest_area > result.orange_pixel_count


def test_warm_partly_saturated_overexposed_core_is_detected() -> None:
    image = _blank_image()
    cv2.rectangle(image, (60, 80), (260, 115), (0, 140, 255), thickness=-1)
    warm_core_hsv = np.uint8([[[25, 120, 255]]])
    warm_core_bgr = tuple(
        int(value)
        for value in cv2.cvtColor(warm_core_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    )
    cv2.rectangle(image, (70, 88), (250, 107), warm_core_bgr, thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert result.detected
    assert result.largest_white_core_pixel_count > 3000
    assert result.largest_white_core_ratio >= 0.05


def test_white_region_without_orange_halo_is_rejected() -> None:
    image = _blank_image()
    cv2.rectangle(image, (50, 60), (270, 150), (255, 255, 255), thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert not result.detected
    assert result.white_core_pixel_count == 0
    assert result.largest_area == 0.0


def test_dim_orange_region_is_rejected() -> None:
    image = _blank_image()
    hsv_color = np.uint8([[[15, 255, 120]]])
    bgr_color = tuple(int(value) for value in cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0])
    cv2.rectangle(image, (50, 60), (270, 150), bgr_color, thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert not result.detected


def test_bright_yellow_region_is_rejected() -> None:
    image = _blank_image()
    yellow_hsv = np.uint8([[[30, 255, 255]]])
    yellow_bgr = tuple(
        int(value) for value in cv2.cvtColor(yellow_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    )
    cv2.rectangle(image, (50, 60), (270, 150), yellow_bgr, thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert not result.detected
    assert result.orange_pixel_count == 0


def test_orange_hued_object_with_small_specular_highlight_is_rejected() -> None:
    image = _blank_image()
    orange_hsv = np.uint8([[[20, 255, 255]]])
    orange_bgr = tuple(
        int(value) for value in cv2.cvtColor(orange_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    )
    cv2.rectangle(image, (50, 60), (270, 150), orange_bgr, thickness=-1)
    cv2.rectangle(image, (155, 100), (164, 109), (255, 255, 255), thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert not result.detected
    assert result.largest_white_core_pixel_count > 0
    assert result.largest_white_core_ratio < 0.05


def test_small_light_like_region_is_rejected_by_image_area_ratio() -> None:
    image = _blank_image()
    cv2.rectangle(image, (100, 100), (130, 122), (0, 140, 255), thickness=-1)
    cv2.rectangle(image, (110, 107), (120, 114), (255, 255, 255), thickness=-1)

    result = detect_orange_light(image, DetectorConfig())

    assert result.required_component_area == 768.0
    assert result.largest_area >= 600.0
    assert result.largest_area < result.required_component_area
    assert result.largest_white_core_pixel_count >= 50
    assert result.largest_white_core_ratio >= 0.05
    assert not result.detected


def test_debouncer_requires_consecutive_frames() -> None:
    debouncer = SignalDebouncer()

    assert not debouncer.update(True, on_frames=3, off_frames=5)
    assert not debouncer.update(True, on_frames=3, off_frames=5)
    assert debouncer.update(True, on_frames=3, off_frames=5)
    for _ in range(4):
        assert debouncer.update(False, on_frames=3, off_frames=5)
    assert not debouncer.update(False, on_frames=3, off_frames=5)


def test_interrupted_on_sequence_does_not_trigger() -> None:
    debouncer = SignalDebouncer()

    for detected in (True, True, False, True, True):
        assert not debouncer.update(detected, on_frames=3, off_frames=5)
