"""Pure state machine for staged Y-then-X visual servo control."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional


class Phase(IntEnum):
    WAITING_TARGET = 0
    ALIGNING_Y = 1
    ALIGNING_X = 2
    SETTLING = 3


class ResultCode(IntEnum):
    SUCCESS = 0
    TARGET_LOST = 1
    VISION_TIMEOUT = 2
    ACTION_TIMEOUT = 3
    CANCELED = 4
    INTERNAL_ERROR = 5


@dataclass(frozen=True)
class ServoConfig:
    u_ref_px: float = 320.0
    u_tolerance_px: float = 8.0
    u_reentry_px: float = 16.0
    y_settle_time_sec: float = 0.3
    y_reentry_time_sec: float = 0.1
    depth_tolerance_m: float = 0.02
    final_settle_time_sec: float = 0.3
    vy_gain: float = 0.0015
    vx_gain: float = 0.8
    min_vy_mps: float = 0.05
    min_vx_mps: float = 0.05
    max_vy_mps: float = 0.15
    max_vx_mps: float = 0.20
    min_confidence: float = 0.6
    vision_timeout_sec: float = 0.1
    target_reacquire_sec: float = 0.3
    min_valid_depth_m: float = 0.05
    max_valid_depth_m: float = 10.0

    def validate(self) -> None:
        positive = {
            "u_tolerance_px": self.u_tolerance_px,
            "u_reentry_px": self.u_reentry_px,
            "y_settle_time_sec": self.y_settle_time_sec,
            "y_reentry_time_sec": self.y_reentry_time_sec,
            "depth_tolerance_m": self.depth_tolerance_m,
            "final_settle_time_sec": self.final_settle_time_sec,
            "vy_gain": self.vy_gain,
            "vx_gain": self.vx_gain,
            "max_vy_mps": self.max_vy_mps,
            "max_vx_mps": self.max_vx_mps,
            "vision_timeout_sec": self.vision_timeout_sec,
            "target_reacquire_sec": self.target_reacquire_sec,
            "min_valid_depth_m": self.min_valid_depth_m,
            "max_valid_depth_m": self.max_valid_depth_m,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and > 0")
        if self.u_reentry_px <= self.u_tolerance_px:
            raise ValueError("u_reentry_px must be greater than u_tolerance_px")
        minimum_speeds = {
            "min_vy_mps": self.min_vy_mps,
            "min_vx_mps": self.min_vx_mps,
        }
        for name, value in minimum_speeds.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
        if self.min_vy_mps > self.max_vy_mps:
            raise ValueError("min_vy_mps must not exceed max_vy_mps")
        if self.min_vx_mps > self.max_vx_mps:
            raise ValueError("min_vx_mps must not exceed max_vx_mps")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        if self.max_valid_depth_m <= self.min_valid_depth_m:
            raise ValueError("max_valid_depth_m must be greater than min_valid_depth_m")


@dataclass(frozen=True)
class TargetObservation:
    valid: bool
    u: float
    v: float
    depth_m: float
    confidence: float
    received_at: float


@dataclass(frozen=True)
class ControlStep:
    phase: Phase
    vx_mps: float
    vy_mps: float
    u: float
    v: float
    depth_m: float
    confidence: float
    error_u_px: float
    error_depth_m: float
    elapsed_sec: float
    terminal_code: Optional[ResultCode] = None
    message: str = ""


def _limit_speed(value: float, minimum: float, maximum: float) -> float:
    """Apply a signed minimum only to a non-zero motion command."""
    if value == 0.0:
        return 0.0
    magnitude = min(max(abs(value), minimum), maximum)
    return math.copysign(magnitude, value)


class PolylineServoController:
    """Strict polyline controller: lateral alignment before depth alignment."""

    def __init__(self, config: ServoConfig) -> None:
        config.validate()
        self.config = config
        self.phase = Phase.WAITING_TARGET
        self._active = False
        self._started_at = 0.0
        self._target_distance_m = 0.0
        self._timeout_sec = 0.0
        self._loss_since: Optional[float] = None
        self._loss_code = ResultCode.TARGET_LOST
        self._y_aligned_since: Optional[float] = None
        self._y_drift_since: Optional[float] = None
        self._final_settle_since: Optional[float] = None

    def start(self, now: float, target_distance_m: float, timeout_sec: float) -> None:
        if not math.isfinite(target_distance_m) or not (
            self.config.min_valid_depth_m <= target_distance_m <= self.config.max_valid_depth_m
        ):
            raise ValueError("target_distance_m is outside the configured valid depth range")
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be finite and > 0")
        self.phase = Phase.WAITING_TARGET
        self._active = True
        self._started_at = now
        self._target_distance_m = target_distance_m
        self._timeout_sec = timeout_sec
        self._loss_since = None
        self._loss_code = ResultCode.TARGET_LOST
        self._reset_alignment_timers()

    def step(self, now: float, observation: Optional[TargetObservation]) -> ControlStep:
        if not self._active:
            raise RuntimeError("controller must be started before step()")

        elapsed = max(0.0, now - self._started_at)
        if elapsed >= self._timeout_sec:
            return self._terminal(
                elapsed,
                observation,
                ResultCode.ACTION_TIMEOUT,
                "visual-servo action timed out",
            )

        invalid_code = self._invalid_observation_code(now, observation)
        if invalid_code is not None:
            if self._loss_since is None:
                self._loss_since = now
                self._loss_code = invalid_code
            self.phase = Phase.WAITING_TARGET
            self._reset_alignment_timers()
            if now - self._loss_since >= self.config.target_reacquire_sec:
                message = (
                    "vision input timed out"
                    if self._loss_code == ResultCode.VISION_TIMEOUT
                    else "target was lost"
                )
                return self._terminal(elapsed, observation, self._loss_code, message)
            return self._zero(elapsed, observation)

        self._loss_since = None
        assert observation is not None
        error_u = observation.u - self.config.u_ref_px
        error_depth = observation.depth_m - self._target_distance_m

        if self.phase == Phase.WAITING_TARGET:
            self.phase = Phase.ALIGNING_Y

        if self.phase == Phase.ALIGNING_Y:
            if abs(error_u) <= self.config.u_tolerance_px:
                if self._y_aligned_since is None:
                    self._y_aligned_since = now
                if now - self._y_aligned_since >= self.config.y_settle_time_sec:
                    self.phase = Phase.ALIGNING_X
                    self._y_drift_since = None
                    return self._zero(elapsed, observation, error_u, error_depth)
                return self._zero(elapsed, observation, error_u, error_depth)

            self._y_aligned_since = None
            vy = _limit_speed(
                -self.config.vy_gain * error_u,
                self.config.min_vy_mps,
                self.config.max_vy_mps,
            )
            return self._output(elapsed, observation, error_u, error_depth, 0.0, vy)

        if self.phase == Phase.ALIGNING_X:
            if abs(error_u) > self.config.u_reentry_px:
                if self._y_drift_since is None:
                    self._y_drift_since = now
                elif now - self._y_drift_since >= self.config.y_reentry_time_sec:
                    self.phase = Phase.ALIGNING_Y
                    self._y_aligned_since = None
                    self._y_drift_since = None
                    return self._zero(elapsed, observation, error_u, error_depth)
            else:
                self._y_drift_since = None

            if abs(error_depth) <= self.config.depth_tolerance_m:
                if abs(error_u) > self.config.u_tolerance_px:
                    self.phase = Phase.ALIGNING_Y
                    self._y_aligned_since = None
                    return self._zero(elapsed, observation, error_u, error_depth)
                self.phase = Phase.SETTLING
                self._final_settle_since = now
                return self._zero(elapsed, observation, error_u, error_depth)

            vx = _limit_speed(
                self.config.vx_gain * error_depth,
                self.config.min_vx_mps,
                self.config.max_vx_mps,
            )
            return self._output(elapsed, observation, error_u, error_depth, vx, 0.0)

        if self.phase == Phase.SETTLING:
            if abs(error_u) > self.config.u_tolerance_px:
                self.phase = Phase.ALIGNING_Y
                self._y_aligned_since = None
                self._final_settle_since = None
                return self._zero(elapsed, observation, error_u, error_depth)
            if abs(error_depth) > self.config.depth_tolerance_m:
                self.phase = Phase.ALIGNING_X
                self._final_settle_since = None
                return self._zero(elapsed, observation, error_u, error_depth)
            if self._final_settle_since is None:
                self._final_settle_since = now
            if now - self._final_settle_since >= self.config.final_settle_time_sec:
                return self._terminal(
                    elapsed,
                    observation,
                    ResultCode.SUCCESS,
                    "visual-servo target reached",
                    error_u,
                    error_depth,
                )
            return self._zero(elapsed, observation, error_u, error_depth)

        return self._terminal(
            elapsed,
            observation,
            ResultCode.INTERNAL_ERROR,
            "unknown visual-servo state",
            error_u,
            error_depth,
        )

    def _invalid_observation_code(
        self, now: float, observation: Optional[TargetObservation]
    ) -> Optional[ResultCode]:
        if observation is None or now - observation.received_at > self.config.vision_timeout_sec:
            return ResultCode.VISION_TIMEOUT
        values = (observation.u, observation.v, observation.depth_m, observation.confidence)
        if not observation.valid or not all(math.isfinite(value) for value in values):
            return ResultCode.TARGET_LOST
        if observation.confidence < self.config.min_confidence:
            return ResultCode.TARGET_LOST
        if not (
            self.config.min_valid_depth_m
            <= observation.depth_m
            <= self.config.max_valid_depth_m
        ):
            return ResultCode.TARGET_LOST
        return None

    def _reset_alignment_timers(self) -> None:
        self._y_aligned_since = None
        self._y_drift_since = None
        self._final_settle_since = None

    def _output(
        self,
        elapsed: float,
        observation: TargetObservation,
        error_u: float,
        error_depth: float,
        vx: float,
        vy: float,
    ) -> ControlStep:
        # This invariant is the defining property of the polyline strategy.
        if abs(vx) > 0.0 and abs(vy) > 0.0:
            raise RuntimeError("polyline controller cannot output vx and vy simultaneously")
        return ControlStep(
            phase=self.phase,
            vx_mps=vx,
            vy_mps=vy,
            u=observation.u,
            v=observation.v,
            depth_m=observation.depth_m,
            confidence=observation.confidence,
            error_u_px=error_u,
            error_depth_m=error_depth,
            elapsed_sec=elapsed,
        )

    def _zero(
        self,
        elapsed: float,
        observation: Optional[TargetObservation],
        error_u: Optional[float] = None,
        error_depth: Optional[float] = None,
    ) -> ControlStep:
        u = observation.u if observation is not None else 0.0
        v = observation.v if observation is not None else 0.0
        depth = observation.depth_m if observation is not None else 0.0
        confidence = observation.confidence if observation is not None else 0.0
        if error_u is None:
            error_u = u - self.config.u_ref_px if observation is not None else 0.0
        if error_depth is None:
            error_depth = depth - self._target_distance_m if observation is not None else 0.0
        return ControlStep(
            phase=self.phase,
            vx_mps=0.0,
            vy_mps=0.0,
            u=u,
            v=v,
            depth_m=depth,
            confidence=confidence,
            error_u_px=error_u,
            error_depth_m=error_depth,
            elapsed_sec=elapsed,
        )

    def _terminal(
        self,
        elapsed: float,
        observation: Optional[TargetObservation],
        code: ResultCode,
        message: str,
        error_u: Optional[float] = None,
        error_depth: Optional[float] = None,
    ) -> ControlStep:
        step = self._zero(elapsed, observation, error_u, error_depth)
        self._active = False
        return ControlStep(
            **{**step.__dict__, "terminal_code": code, "message": message}
        )
