import math

import pytest

from gmk_visual_servo.controller import (
    Phase,
    PolylineServoController,
    ResultCode,
    ServoConfig,
    TargetObservation,
)


def observation(now, *, u=320.0, depth=0.8, valid=True, confidence=0.9):
    return TargetObservation(valid, u, 240.0, depth, confidence, now)


def started_controller(config=None, *, timeout=10.0):
    controller = PolylineServoController(config or ServoConfig())
    controller.start(0.0, target_distance_m=0.5, timeout_sec=timeout)
    return controller


@pytest.mark.parametrize(
    ("u", "expected_sign"),
    [(400.0, -1.0), (240.0, 1.0)],
)
def test_y_alignment_direction_and_strict_axis(u, expected_sign):
    controller = started_controller()
    step = controller.step(0.0, observation(0.0, u=u))

    assert step.phase == Phase.ALIGNING_Y
    assert step.vx_mps == 0.0
    assert math.copysign(1.0, step.vy_mps) == expected_sign
    assert abs(step.vy_mps) <= 0.15


def test_y_must_settle_before_x_and_x_only_outputs_vx():
    controller = started_controller()

    first = controller.step(0.0, observation(0.0))
    not_settled = controller.step(0.29, observation(0.29))
    transitioned = controller.step(0.31, observation(0.31))
    x_command = controller.step(0.32, observation(0.32, depth=0.8))

    assert first.phase == Phase.ALIGNING_Y
    assert not_settled.phase == Phase.ALIGNING_Y
    assert transitioned.phase == Phase.ALIGNING_X
    assert transitioned.vx_mps == transitioned.vy_mps == 0.0
    assert x_command.phase == Phase.ALIGNING_X
    assert x_command.vx_mps > 0.0
    assert x_command.vy_mps == 0.0
    assert x_command.vx_mps <= 0.20


def test_x_reenters_y_after_sustained_pixel_drift():
    controller = started_controller()
    controller.step(0.0, observation(0.0))
    controller.step(0.31, observation(0.31))

    before_hold = controller.step(0.32, observation(0.32, u=337.0))
    after_hold = controller.step(0.43, observation(0.43, u=337.0))

    assert before_hold.phase == Phase.ALIGNING_X
    assert before_hold.vy_mps == 0.0
    assert after_hold.phase == Phase.ALIGNING_Y
    assert after_hold.vx_mps == after_hold.vy_mps == 0.0


def test_hysteresis_does_not_reenter_y_inside_outer_threshold():
    controller = started_controller()
    controller.step(0.0, observation(0.0))
    controller.step(0.31, observation(0.31))

    for now, u in [(0.32, 329.0), (0.45, 335.0), (0.60, 329.0)]:
        step = controller.step(now, observation(now, u=u, depth=0.8))
        assert step.phase == Phase.ALIGNING_X
        assert step.vy_mps == 0.0


def test_final_errors_must_settle_before_success():
    controller = started_controller()
    controller.step(0.0, observation(0.0))
    controller.step(0.31, observation(0.31))
    settling = controller.step(0.32, observation(0.32, depth=0.5))
    almost = controller.step(0.61, observation(0.61, depth=0.5))
    done = controller.step(0.63, observation(0.63, depth=0.5))

    assert settling.phase == Phase.SETTLING
    assert almost.terminal_code is None
    assert done.terminal_code == ResultCode.SUCCESS
    assert done.vx_mps == done.vy_mps == 0.0


def test_missing_input_stops_then_aborts_as_vision_timeout():
    controller = started_controller()

    waiting = controller.step(0.0, None)
    aborted = controller.step(0.31, None)

    assert waiting.phase == Phase.WAITING_TARGET
    assert waiting.vx_mps == waiting.vy_mps == 0.0
    assert aborted.terminal_code == ResultCode.VISION_TIMEOUT


def test_invalid_target_stops_then_aborts_as_target_lost():
    controller = started_controller()

    waiting = controller.step(0.0, observation(0.0, valid=False))
    aborted = controller.step(0.31, observation(0.31, valid=False))

    assert waiting.vx_mps == waiting.vy_mps == 0.0
    assert aborted.terminal_code == ResultCode.TARGET_LOST


def test_stale_input_stops_motion_immediately_then_aborts():
    controller = started_controller()
    sample = observation(0.0, u=400.0)

    moving = controller.step(0.0, sample)
    stopped = controller.step(0.11, sample)
    aborted = controller.step(0.42, sample)

    assert moving.vy_mps != 0.0
    assert stopped.phase == Phase.WAITING_TARGET
    assert stopped.vx_mps == stopped.vy_mps == 0.0
    assert aborted.terminal_code == ResultCode.VISION_TIMEOUT


def test_target_recovery_restarts_from_y_alignment():
    controller = started_controller()
    controller.step(0.0, observation(0.0, valid=False))

    recovered = controller.step(0.2, observation(0.2, u=400.0))

    assert recovered.phase == Phase.ALIGNING_Y
    assert recovered.vx_mps == 0.0
    assert recovered.vy_mps < 0.0


@pytest.mark.parametrize(
    "bad_observation",
    [
        observation(0.0, confidence=0.5),
        observation(0.0, depth=0.0),
        observation(0.0, depth=float("nan")),
        observation(0.0, u=float("inf")),
    ],
)
def test_unsafe_target_values_never_command_motion(bad_observation):
    controller = started_controller()
    step = controller.step(0.0, bad_observation)
    assert step.vx_mps == step.vy_mps == 0.0


def test_action_timeout_stops_motion():
    controller = started_controller(timeout=0.5)
    moving = controller.step(0.0, observation(0.0, u=400.0))
    timed_out = controller.step(0.5, observation(0.5, u=400.0))

    assert moving.vy_mps != 0.0
    assert timed_out.terminal_code == ResultCode.ACTION_TIMEOUT
    assert timed_out.vx_mps == timed_out.vy_mps == 0.0


def test_minimum_y_speed_preserves_direction():
    config = ServoConfig(vy_gain=0.0001, min_vy_mps=0.05, max_vy_mps=0.20)

    right = started_controller(config).step(0.0, observation(0.0, u=330.0))
    left = started_controller(config).step(0.0, observation(0.0, u=310.0))

    assert right.vy_mps == pytest.approx(-0.05)
    assert left.vy_mps == pytest.approx(0.05)
    assert right.vx_mps == left.vx_mps == 0.0


def test_minimum_x_speed_applies_only_outside_depth_tolerance():
    config = ServoConfig(vx_gain=0.01, min_vx_mps=0.05, max_vx_mps=0.20)
    controller = started_controller(config)
    controller.step(0.0, observation(0.0, depth=0.55))
    controller.step(0.31, observation(0.31, depth=0.55))

    moving = controller.step(0.32, observation(0.32, depth=0.55))
    settling = controller.step(0.33, observation(0.33, depth=0.5))

    assert moving.vx_mps == pytest.approx(0.05)
    assert moving.vy_mps == 0.0
    assert settling.vx_mps == settling.vy_mps == 0.0


def test_zero_minimum_speed_disables_floor():
    config = ServoConfig(vy_gain=0.0001, min_vy_mps=0.0)
    step = started_controller(config).step(0.0, observation(0.0, u=330.0))
    assert step.vy_mps == pytest.approx(-0.001)


def test_configuration_requires_hysteresis():
    with pytest.raises(ValueError, match="u_reentry_px"):
        PolylineServoController(ServoConfig(u_reentry_px=8.0))


@pytest.mark.parametrize(
    "config",
    [
        ServoConfig(min_vx_mps=0.21, max_vx_mps=0.20),
        ServoConfig(min_vy_mps=0.16, max_vy_mps=0.15),
        ServoConfig(min_vx_mps=-0.01),
    ],
)
def test_configuration_rejects_invalid_minimum_speed(config):
    with pytest.raises(ValueError, match="min_v"):
        PolylineServoController(config)
