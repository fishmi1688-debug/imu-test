#!/usr/bin/env python3

"""Realtime thigh IMU phase estimator used by the gait controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

EPSILON = 1e-9


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_phase(value: float) -> float:
    wrapped = value % 1.0
    return wrapped + 1.0 if wrapped < 0.0 else wrapped


def angular_delta_degrees(current: float, previous: float) -> float:
    """Shortest signed angular difference in degrees."""
    return ((float(current) - float(previous) + 180.0) % 360.0) - 180.0


def quaternion_to_euler_xyz_radians(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= EPSILON:
        return 0.0, 0.0, 0.0

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_to_euler_xyz_degrees(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    roll, pitch, yaw = quaternion_to_euler_xyz_radians(w, x, y, z)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quaternion_rotate_vector(
    w: float,
    x: float,
    y: float,
    z: float,
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a body-frame vector into the common reference frame."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= EPSILON:
        raise ValueError("quaternion norm is zero")

    w /= norm
    x /= norm
    y /= norm
    z /= norm
    vx, vy, vz = (float(component) for component in vector)

    # v' = v + 2*w*(q_vec x v) + 2*(q_vec x (q_vec x v)).
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _basis_axis(index: int) -> tuple[float, float, float]:
    return tuple(1.0 if component_index == int(index) else 0.0 for component_index in range(3))


def _dot3(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _sub3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale3(a: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _norm3(a: tuple[float, float, float]) -> float:
    return math.sqrt(_dot3(a, a))


def _normalize3(a: tuple[float, float, float]) -> Optional[tuple[float, float, float]]:
    norm = _norm3(a)
    if norm <= EPSILON:
        return None
    return _scale3(a, 1.0 / norm)


def _axis_index(axis: str | int, *, name: str) -> int:
    if isinstance(axis, int):
        value = int(axis)
        if 0 <= value <= 2:
            return value
        raise ValueError(f"{name} must be 0, 1, 2 or x, y, z")

    cleaned = str(axis or "").strip().lower()
    if cleaned.startswith(("-", "+")):
        cleaned = cleaned[1:]
    axis_map = {"x": 0, "0": 0, "y": 1, "1": 1, "z": 2, "2": 2}
    if cleaned not in axis_map:
        raise ValueError(f"{name} must be x, y, z, 0, 1, or 2")
    return axis_map[cleaned]


def _axis_vector(axis: str | int | tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    """Return a normalized xyz axis vector for a named axis or explicit vector."""
    if isinstance(axis, (tuple, list)):
        components = tuple(float(component) for component in axis)
        if len(components) != 3:
            raise ValueError("axis vector must contain three components")
    else:
        cleaned = str(axis or "").strip().lower()
        sign = -1.0 if cleaned.startswith("-") else 1.0
        index = _axis_index(cleaned, name="axis")
        components = tuple(sign if component_index == index else 0.0 for component_index in range(3))

    norm = math.sqrt(sum(component * component for component in components))
    if norm <= EPSILON:
        raise ValueError("axis vector norm must be positive")
    return tuple(component / norm for component in components)


def project_gyro_to_axis(
    gyro_xyz: tuple[float, float, float] | list[float],
    sagittal_axis: str | int | tuple[float, float, float] | list[float] = "y",
) -> float:
    """Project the three-axis gyro rate onto the thigh sagittal rotation axis."""
    gyro = tuple(float(component) for component in gyro_xyz)
    if len(gyro) != 3:
        raise ValueError("gyro_xyz must contain three components")
    axis = _axis_vector(sagittal_axis)
    return sum(rate * axis_component for rate, axis_component in zip(gyro, axis))


@dataclass
class ImuPhaseConfig:
    angle_sign: float = 1.0
    gyro_sign: float = 1.0
    gyro_unit: str = "deg"
    angle_source: str = "acc"
    gyro_source: str = "axis"
    acc_angle_numerator_axis: str | int = "z"
    acc_angle_denominator_axis: str | int = "x"
    acc_angle_numerator_sign: float = 1.0
    acc_angle_denominator_sign: float = 1.0
    gyro_axis: str | int = "y"
    gyro_sagittal_axis: tuple[float, float, float] | None = None
    quaternion_thigh_axis: str | int | tuple[float, float, float] = "x"
    quaternion_sagittal_forward_axis: str | int = "x"
    quaternion_sagittal_vertical_axis: str | int = "z"
    quaternion_gyro_sagittal_axis: str | int = "y"
    lowpass_cutoff_hz: float = 6.0
    initial_cycle_period_sec: float = 1.0
    min_cycle_period_sec: float = 0.70
    max_cycle_period_sec: float = 3.0
    gyro_zero_threshold_deg_s: float = 10.0
    min_swing_range_deg: float = 25.0
    complementary_acc_weight: float = 0.02
    phase_peak_ratio: float = 0.60
    phase_offset: float = 0.0
    startup_warmup_strides: int = 0
    clamp_phase: bool = True
    motion_timeout_sec: float = 0.45
    motion_window_sec: float = 0.5
    end_phase_stop_threshold: float = 0.98
    motion_swing_range_ratio: float = 0.25
    reject_abnormal_samples: bool = False
    max_abs_angle_deg: float = 170.0
    max_abs_gyro_deg_s: float = 800.0
    max_angle_jump_deg: float = 65.0
    max_gyro_jump_deg_s: float = 1000.0
    max_phase_rate_hz: float = 0.0
    startup_seed_enabled: bool = True
    startup_seed_min_window_sec: float = 0.10
    startup_seed_max_window_sec: float = 0.20
    startup_seed_gyro_threshold_deg_s: float = 15.0
    startup_seed_angle_threshold_deg: float = 3.0
    startup_seed_positive_gyro_phase_rad: float = 0.0
    startup_seed_negative_gyro_phase_rad: float = -math.pi
    startup_seed_ramp_sec: float = 0.30
    stop_detection_window_sec: float = 0.30
    stop_detection_hold_sec: float = 0.12
    stop_gyro_rms_threshold_deg_s: float = 8.0
    stop_angle_range_threshold_deg: float = 3.0
    stop_phase_rate_limit_hz: float = 4.0
    stop_phase_unstable_hold_sec: float = 0.25


@dataclass
class ImuPhaseOutput:
    angle_raw_deg: float
    angle_deg: float
    angular_velocity_raw_deg_s: float
    angular_velocity_deg_s: float
    phase_0_to_1: float
    phase_rad: float
    previous_cycle_period_sec: float
    previous_cycle_frequency_hz: float
    zero_event: bool
    zero_event_count: int
    motion_active: bool
    current_swing_range_deg: float
    time_since_last_zero_sec: Optional[float]
    time_since_last_motion_sec: Optional[float]
    recent_swing_range_deg: float = 0.0
    recent_swing_active: bool = False
    sample_rejected: bool = False
    reject_reason: str = ""
    phase_limited: bool = False
    angle_drift_removed_deg: float = 0.0
    temporary_phase_active: bool = False
    startup_ramp_scale: float = 1.0
    startup_seed_phase_rad: Optional[float] = None
    stopped_event: bool = False
    walking_mode: bool = True


class FirstOrderLowpass:
    def __init__(self, cutoff_hz: float):
        self.cutoff_hz = float(cutoff_hz)
        self.value: Optional[float] = None
        self.time_sec: Optional[float] = None

    def reset(self) -> None:
        self.value = None
        self.time_sec = None

    def update(self, value: float, time_sec: float) -> float:
        if self.value is None or self.time_sec is None or self.cutoff_hz <= 0.0:
            self.value = float(value)
            self.time_sec = float(time_sec)
            return float(value)

        dt = max(0.0, float(time_sec) - self.time_sec)
        if dt <= EPSILON:
            return self.value

        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (tau + dt)
        self.value = self.value + alpha * (float(value) - self.value)
        self.time_sec = float(time_sec)
        return self.value


class WrappedAngleLowpass:
    """First-order lowpass for wrapped angles using the shortest angular delta."""

    def __init__(self, cutoff_hz: float):
        self.cutoff_hz = float(cutoff_hz)
        self.value_rad: Optional[float] = None
        self.time_sec: Optional[float] = None

    def reset(self) -> None:
        self.value_rad = None
        self.time_sec = None

    def update(self, angle_deg: float, time_sec: float) -> float:
        current_rad = math.radians(float(angle_deg))
        if self.value_rad is None or self.time_sec is None or self.cutoff_hz <= 0.0:
            self.value_rad = math.atan2(math.sin(current_rad), math.cos(current_rad))
            self.time_sec = float(time_sec)
            return math.degrees(self.value_rad)

        dt = max(0.0, float(time_sec) - self.time_sec)
        if dt <= EPSILON:
            return math.degrees(self.value_rad)

        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (tau + dt)
        delta_rad = math.atan2(
            math.sin(current_rad - self.value_rad),
            math.cos(current_rad - self.value_rad),
        )
        self.value_rad = math.atan2(
            math.sin(self.value_rad + alpha * delta_rad),
            math.cos(self.value_rad + alpha * delta_rad),
        )
        self.time_sec = float(time_sec)
        return math.degrees(self.value_rad)


class ThighImuPhaseEstimator:
    """Estimate 0-1 gait phase from thigh sagittal angle and angular velocity.

    By default the event detector follows the earlier thigh IMU convention:
    angle = atan2(Acc_Z, Acc_X), angular velocity = Gyr_Y, phase zero is the
    positive-to-negative Gyr_Y zero crossing after sufficient swing amplitude.
    Quaternion Euler angle sources expose the original quaternion-to-Euler
    conversion for comparison. ``quat_sagittal`` rotates the configured thigh
    axis by the quaternion, removes heading by deriving the current forward
    direction from the rotated sagittal/lateral axis, then projects the thigh
    axis onto that dynamic forward/vertical plane. Angular velocity always
    comes from the IMU gyroscope axis; it is never computed by differencing the
    angle. The accelerometer is used as a low-weight gravity reference when a
    quaternion angle is available.
    During a cycle, phase advances using the previous observed cycle period.
    Axis fields in ImuPhaseConfig let mounted sensors remap this without
    changing the detector logic. Samples may be 6D [acc, gyro] or 10D
    [acc, gyro, quat_w, quat_x, quat_y, quat_z]; quaternion angle sources
    fall back to the accelerometer angle if quaternion values are absent.
    """

    def __init__(self, config: Optional[ImuPhaseConfig] = None):
        self.config = config or ImuPhaseConfig()
        self.acc_angle_numerator_index = _axis_index(
            self.config.acc_angle_numerator_axis,
            name="acc_angle_numerator_axis",
        )
        self.acc_angle_denominator_index = _axis_index(
            self.config.acc_angle_denominator_axis,
            name="acc_angle_denominator_axis",
        )
        self.acc_angle_numerator_sign = float(
            getattr(self.config, "acc_angle_numerator_sign", 1.0)
        )
        self.acc_angle_denominator_sign = float(
            getattr(self.config, "acc_angle_denominator_sign", 1.0)
        )
        self.gyro_index = 3 + _axis_index(self.config.gyro_axis, name="gyro_axis")
        self.gyro_sagittal_axis = _axis_vector(
            self.config.gyro_axis
            if self.config.gyro_sagittal_axis is None
            else self.config.gyro_sagittal_axis
        )
        self.quaternion_sagittal_forward_index = _axis_index(
            self.config.quaternion_sagittal_forward_axis,
            name="quaternion_sagittal_forward_axis",
        )
        self.quaternion_sagittal_vertical_index = _axis_index(
            self.config.quaternion_sagittal_vertical_axis,
            name="quaternion_sagittal_vertical_axis",
        )
        self.quaternion_gyro_sagittal_index = _axis_index(
            self.config.quaternion_gyro_sagittal_axis,
            name="quaternion_gyro_sagittal_axis",
        )
        self.quaternion_thigh_axis = _axis_vector(self.config.quaternion_thigh_axis)
        self.quaternion_sagittal_lateral_axis = _axis_vector(
            self.config.quaternion_gyro_sagittal_axis
        )
        self.angle_filter = WrappedAngleLowpass(self.config.lowpass_cutoff_hz)
        self.gyro_filter = FirstOrderLowpass(self.config.lowpass_cutoff_hz)
        self.reset()

    def reset(self) -> None:
        self.angle_filter.reset()
        self.gyro_filter.reset()
        self.current_cycle_start_time: Optional[float] = None
        self.last_zero_time: Optional[float] = None
        self.previous_cycle_period_sec = clamp(
            self.config.initial_cycle_period_sec,
            self.config.min_cycle_period_sec,
            self.config.max_cycle_period_sec,
        )
        self.zero_event_count = 0
        self.previous_sample: Optional[tuple[float, float, float]] = None
        self.candidate_peak_angle: Optional[float] = None
        self.cycle_min_angle: Optional[float] = None
        self.complementary_angle: Optional[float] = None
        self.complementary_time: Optional[float] = None
        self.max_positive_gyro_since_zero = 0.0
        self.last_motion_time: Optional[float] = None
        self.recent_angle_window: list[tuple[float, float]] = []
        self.last_raw_angle_deg: Optional[float] = None
        self.last_raw_gyro_deg_s: Optional[float] = None
        self.last_raw_time_sec: Optional[float] = None
        self.last_phase_rad: Optional[float] = None
        self.last_phase_time_sec: Optional[float] = None
        self.last_output: Optional[ImuPhaseOutput] = None

    def update_swing_threshold(self, threshold_deg: float) -> None:
        self.config.min_swing_range_deg = max(0.0, float(threshold_deg))

    def process_6d(
        self,
        sample_6d,
        sample_time: float,
        *,
        angle_source_override: Optional[str] = None,
    ) -> ImuPhaseOutput:
        """Process [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, ...]."""
        angle_raw, gyro_raw = self.extract_raw_signal(
            sample_6d,
            angle_source_override=angle_source_override,
        )
        angle_reference = angle_raw
        angle_source = str(
            angle_source_override
            if angle_source_override is not None
            else getattr(self.config, "angle_source", "acc")
        ).strip().lower()
        if angle_source.startswith("quat"):
            acc_angle = self._extract_accelerometer_angle(sample_6d)
            if math.isfinite(acc_angle) and math.isfinite(angle_raw):
                # Correct only the low-frequency reference. The reported raw
                # thigh angle remains the quaternion Euler angle.
                acc_weight = clamp(self.config.complementary_acc_weight, 0.0, 1.0)
                angle_reference = angle_raw + acc_weight * angular_delta_degrees(
                    acc_angle,
                    angle_raw,
                )
        return self.process_signal(
            angle_raw,
            gyro_raw,
            sample_time,
            reference_angle_deg=angle_reference,
            angle_source_override=angle_source,
        )

    def extract_raw_signal(
        self,
        sample_6d,
        *,
        angle_source_override: Optional[str] = None,
    ) -> tuple[float, float]:
        """Return configured raw sagittal angle and angular velocity in deg/deg-s."""
        values = [float(value) for value in sample_6d]
        if len(values) < 6:
            raise ValueError(f"IMU phase sample needs at least 6 values, got {len(values)}")
        acc_numerator = self.acc_angle_numerator_sign * values[self.acc_angle_numerator_index]
        acc_denominator = self.acc_angle_denominator_sign * values[self.acc_angle_denominator_index]
        angle_acc_deg = math.degrees(math.atan2(acc_numerator, acc_denominator))
        angle_source = str(
            angle_source_override
            if angle_source_override is not None
            else getattr(self.config, "angle_source", "acc")
        ).strip().lower()
        angle_selected_deg = angle_acc_deg
        if angle_source.startswith("quat") and len(values) >= 10:
            euler_x_deg, euler_y_deg, euler_z_deg = quaternion_to_euler_xyz_degrees(
                values[6],
                values[7],
                values[8],
                values[9],
            )
            if angle_source == "quat_sagittal":
                try:
                    angle_selected_deg = self._extract_heading_free_sagittal_angle(values)
                except Exception:
                    angle_selected_deg = euler_y_deg
            elif angle_source == "quat_x":
                angle_selected_deg = euler_x_deg
            elif angle_source == "quat_y":
                angle_selected_deg = euler_y_deg
            elif angle_source == "quat_z":
                angle_selected_deg = euler_z_deg
        angle_raw = self.config.angle_sign * angle_selected_deg
        gyro_raw = self.config.gyro_sign * project_gyro_to_axis(
            values[3:6],
            self.gyro_sagittal_axis,
        )
        if str(self.config.gyro_unit).strip().lower().startswith("rad"):
            gyro_raw = math.degrees(gyro_raw)
        return float(angle_raw), float(gyro_raw)

    def _extract_heading_free_sagittal_angle(self, values: list[float]) -> float:
        """Return thigh sagittal angle with the horizontal heading removed."""
        rotated_thigh_axis = quaternion_rotate_vector(
            values[6],
            values[7],
            values[8],
            values[9],
            self.quaternion_thigh_axis,
        )
        rotated_lateral_axis = quaternion_rotate_vector(
            values[6],
            values[7],
            values[8],
            values[9],
            self.quaternion_sagittal_lateral_axis,
        )
        vertical_axis = _basis_axis(self.quaternion_sagittal_vertical_index)
        fallback_forward_axis = _basis_axis(self.quaternion_sagittal_forward_index)

        lateral_vertical = _scale3(
            vertical_axis,
            _dot3(rotated_lateral_axis, vertical_axis),
        )
        lateral_horizontal = _sub3(rotated_lateral_axis, lateral_vertical)
        lateral_horizontal_unit = _normalize3(lateral_horizontal)
        if lateral_horizontal_unit is None:
            forward_axis = fallback_forward_axis
        else:
            # Body Y is mounted left on MI1. The left-axis crossed with the
            # vertical axis gives the user's instantaneous heading, so a
            # 180-degree turn no longer flips the sagittal angle sign.
            forward_axis = _normalize3(_cross3(lateral_horizontal_unit, vertical_axis))
            if forward_axis is None:
                forward_axis = fallback_forward_axis

        forward_component = _dot3(rotated_thigh_axis, forward_axis)
        vertical_component = _dot3(rotated_thigh_axis, vertical_axis)
        return math.degrees(math.atan2(forward_component, vertical_component))

    def _extract_accelerometer_angle(self, sample_6d) -> float:
        values = [float(value) for value in sample_6d]
        if len(values) < 6:
            return float("nan")
        numerator = self.acc_angle_numerator_sign * values[self.acc_angle_numerator_index]
        denominator = self.acc_angle_denominator_sign * values[self.acc_angle_denominator_index]
        if not math.isfinite(numerator) or not math.isfinite(denominator):
            return float("nan")
        return self.config.angle_sign * math.degrees(math.atan2(numerator, denominator))

    def process_signal(
        self,
        angle_raw_deg: float,
        gyro_raw_deg_s: float,
        sample_time: float,
        *,
        reference_angle_deg: Optional[float] = None,
        angle_source_override: Optional[str] = None,
    ) -> ImuPhaseOutput:
        """Process an already-computed sagittal angle signal."""
        angle_raw = float(angle_raw_deg)
        gyro_raw = float(gyro_raw_deg_s)
        sample_time = float(sample_time)
        reject_reason = self._abnormal_sample_reason(angle_raw, gyro_raw, sample_time)
        if reject_reason:
            return self._make_rejected_output(angle_raw, gyro_raw, reject_reason)
        if self.current_cycle_start_time is None:
            self.current_cycle_start_time = sample_time

        angle_reference = (
            angle_raw
            if reference_angle_deg is None or not math.isfinite(reference_angle_deg)
            else float(reference_angle_deg)
        )
        angle_acc = self.angle_filter.update(angle_reference, sample_time)
        gyro = self.gyro_filter.update(gyro_raw, sample_time)
        angle_source = str(
            angle_source_override
            if angle_source_override is not None
            else getattr(self.config, "angle_source", "acc")
        ).strip().lower()
        angle = (
            self._update_complementary_angle(angle_acc, gyro, sample_time)
            if angle_source.startswith("quat")
            else angle_acc
        )
        recent_swing_range = self._update_recent_swing_window(sample_time, angle)
        self._update_cycle_extrema(angle)
        zero_event = self._detect_gyro_zero_crossing(sample_time, angle, gyro)
        cycle_min = self.cycle_min_angle if self.cycle_min_angle is not None else angle
        cycle_max = self.candidate_peak_angle if self.candidate_peak_angle is not None else angle
        current_swing_range = max(0.0, cycle_max - cycle_min)

        motion_velocity_threshold = max(2.0, self.config.gyro_zero_threshold_deg_s * 0.35)
        motion_swing_range_threshold = max(
            2.0,
            self.config.min_swing_range_deg
            * clamp(self.config.motion_swing_range_ratio, 0.0, 1.0),
        )
        has_meaningful_swing = current_swing_range >= motion_swing_range_threshold
        recent_swing_active = recent_swing_range >= motion_swing_range_threshold
        has_velocity_motion = abs(gyro) >= motion_velocity_threshold and has_meaningful_swing
        if zero_event or recent_swing_active or has_velocity_motion:
            self.last_motion_time = float(sample_time)

        period = max(self.previous_cycle_period_sec, EPSILON)
        waiting_for_first_event = self.zero_event_count == 0 and not zero_event
        elapsed = (
            0.0
            if zero_event or waiting_for_first_event
            else max(0.0, sample_time - (self.current_cycle_start_time or sample_time))
        )
        phase_raw = elapsed / period + self.config.phase_offset
        phase = clamp(phase_raw, 0.0, 1.0) if self.config.clamp_phase else wrap_phase(phase_raw)
        phase_rad = phase * 2.0 * math.pi
        phase_limited = False
        phase_overrun_without_zero = bool(
            self.config.clamp_phase
            and not zero_event
            and phase_raw >= 1.0
        )
        time_since_last_zero = (
            None if self.last_zero_time is None else max(0.0, float(sample_time) - self.last_zero_time)
        )
        time_since_last_motion = (
            None
            if self.last_motion_time is None
            else max(0.0, float(sample_time) - self.last_motion_time)
        )
        phase_stalled_at_end = (
            self.config.clamp_phase
            and phase >= self.config.end_phase_stop_threshold
            and not zero_event
            and abs(gyro) < self.config.gyro_zero_threshold_deg_s
        )
        stale_cycle = (
            time_since_last_zero is None
            or time_since_last_zero > self.config.max_cycle_period_sec
        )
        low_motion_timeout = (
            time_since_last_motion is None
            or time_since_last_motion > self.config.motion_timeout_sec
        )
        motion_active = bool(
            self.zero_event_count > 0
            and not stale_cycle
            and not low_motion_timeout
            and not phase_stalled_at_end
            and not phase_overrun_without_zero
            and (recent_swing_active or zero_event)
        )

        phase_rad, phase_limited = self._limit_phase_rate(
            phase_rad,
            sample_time,
            zero_event=zero_event,
        )
        if phase_limited:
            phase = wrap_phase(phase_rad / (2.0 * math.pi))
            motion_active = False

        self.previous_sample = (sample_time, angle, gyro)
        self.last_raw_angle_deg = angle_raw
        self.last_raw_gyro_deg_s = gyro_raw
        self.last_raw_time_sec = sample_time
        self.last_phase_rad = phase_rad
        self.last_phase_time_sec = sample_time

        output = ImuPhaseOutput(
            angle_raw_deg=angle_raw,
            angle_deg=angle,
            angular_velocity_raw_deg_s=gyro_raw,
            angular_velocity_deg_s=gyro,
            phase_0_to_1=phase,
            phase_rad=phase_rad,
            previous_cycle_period_sec=period,
            previous_cycle_frequency_hz=1.0 / period,
            zero_event=zero_event,
            zero_event_count=self.zero_event_count,
            motion_active=motion_active,
            current_swing_range_deg=current_swing_range,
            time_since_last_zero_sec=time_since_last_zero,
            time_since_last_motion_sec=time_since_last_motion,
            recent_swing_range_deg=recent_swing_range,
            recent_swing_active=recent_swing_active,
            phase_limited=phase_limited,
        )
        self.last_output = output
        return output

    def _abnormal_sample_reason(self, angle_raw: float, gyro_raw: float, sample_time: float) -> str:
        if not bool(getattr(self.config, "reject_abnormal_samples", False)):
            return ""
        if not math.isfinite(angle_raw) or not math.isfinite(gyro_raw):
            return "non_finite"
        if abs(angle_raw) > float(getattr(self.config, "max_abs_angle_deg", 170.0)):
            return f"angle_abs>{float(getattr(self.config, 'max_abs_angle_deg', 170.0)):.1f}"
        if abs(gyro_raw) > float(getattr(self.config, "max_abs_gyro_deg_s", 800.0)):
            return f"gyro_abs>{float(getattr(self.config, 'max_abs_gyro_deg_s', 800.0)):.1f}"
        if self.last_raw_time_sec is None:
            return ""

        dt = float(sample_time) - float(self.last_raw_time_sec)
        if dt <= EPSILON or dt > 0.25:
            return ""
        if self.last_raw_angle_deg is not None:
            angle_jump = abs(angular_delta_degrees(angle_raw, self.last_raw_angle_deg))
            max_angle_jump = float(getattr(self.config, "max_angle_jump_deg", 65.0))
            if angle_jump > max_angle_jump:
                return f"angle_jump>{max_angle_jump:.1f}"
        if self.last_raw_gyro_deg_s is not None:
            gyro_jump = abs(float(gyro_raw) - float(self.last_raw_gyro_deg_s))
            max_gyro_jump = float(getattr(self.config, "max_gyro_jump_deg_s", 1000.0))
            if gyro_jump > max_gyro_jump:
                return f"gyro_jump>{max_gyro_jump:.1f}"
        return ""

    def _make_rejected_output(
        self,
        angle_raw: float,
        gyro_raw: float,
        reason: str,
    ) -> ImuPhaseOutput:
        previous = self.last_output
        if previous is not None:
            return ImuPhaseOutput(
                angle_raw_deg=float(angle_raw),
                angle_deg=float(previous.angle_deg),
                angular_velocity_raw_deg_s=float(gyro_raw),
                angular_velocity_deg_s=float(previous.angular_velocity_deg_s),
                phase_0_to_1=float(previous.phase_0_to_1),
                phase_rad=float(previous.phase_rad),
                previous_cycle_period_sec=float(previous.previous_cycle_period_sec),
                previous_cycle_frequency_hz=float(previous.previous_cycle_frequency_hz),
                zero_event=False,
                zero_event_count=int(previous.zero_event_count),
                motion_active=False,
                current_swing_range_deg=float(previous.current_swing_range_deg),
                time_since_last_zero_sec=previous.time_since_last_zero_sec,
                time_since_last_motion_sec=previous.time_since_last_motion_sec,
                recent_swing_range_deg=float(previous.recent_swing_range_deg),
                recent_swing_active=False,
                sample_rejected=True,
                reject_reason=str(reason),
            )

        period = max(self.previous_cycle_period_sec, EPSILON)
        return ImuPhaseOutput(
            angle_raw_deg=float(angle_raw),
            angle_deg=0.0,
            angular_velocity_raw_deg_s=float(gyro_raw),
            angular_velocity_deg_s=0.0,
            phase_0_to_1=0.0,
            phase_rad=0.0,
            previous_cycle_period_sec=period,
            previous_cycle_frequency_hz=1.0 / period,
            zero_event=False,
            zero_event_count=int(self.zero_event_count),
            motion_active=False,
            current_swing_range_deg=0.0,
            time_since_last_zero_sec=None,
            time_since_last_motion_sec=None,
            recent_swing_range_deg=0.0,
            recent_swing_active=False,
            sample_rejected=True,
            reject_reason=str(reason),
        )

    def _limit_phase_rate(
        self,
        phase_rad: float,
        sample_time: float,
        *,
        zero_event: bool,
    ) -> tuple[float, bool]:
        max_rate_hz = float(getattr(self.config, "max_phase_rate_hz", 3.0))
        if max_rate_hz <= 0.0 or self.last_phase_rad is None or self.last_phase_time_sec is None:
            return float(phase_rad), False
        dt = float(sample_time) - float(self.last_phase_time_sec)
        if dt <= EPSILON or dt > 0.25:
            return float(phase_rad), False
        if zero_event:
            return float(phase_rad % (2.0 * math.pi)), False

        max_delta = 2.0 * math.pi * max_rate_hz * dt
        forward_delta = (float(phase_rad) - float(self.last_phase_rad)) % (2.0 * math.pi)
        if forward_delta <= max_delta:
            return float(phase_rad % (2.0 * math.pi)), False
        limited = (float(self.last_phase_rad) + max_delta) % (2.0 * math.pi)
        return float(limited), True

    def _update_complementary_angle(
        self,
        acc_angle_deg: float,
        gyro_deg_s: float,
        time_sec: float,
    ) -> float:
        if self.complementary_angle is None or self.complementary_time is None:
            self.complementary_angle = float(acc_angle_deg)
            self.complementary_time = float(time_sec)
            return float(acc_angle_deg)

        dt = max(0.0, float(time_sec) - self.complementary_time)
        if dt <= EPSILON:
            return self.complementary_angle

        acc_weight = clamp(self.config.complementary_acc_weight, 0.0, 1.0)
        predicted = self.complementary_angle + gyro_deg_s * dt
        self.complementary_angle = (1.0 - acc_weight) * predicted + acc_weight * acc_angle_deg
        self.complementary_time = float(time_sec)
        return self.complementary_angle

    def _update_recent_swing_window(self, time_sec: float, angle_deg: float) -> float:
        window_sec = max(0.0, float(getattr(self.config, "motion_window_sec", 0.0)))
        now = float(time_sec)
        self.recent_angle_window.append((now, float(angle_deg)))
        if window_sec <= EPSILON:
            self.recent_angle_window = self.recent_angle_window[-1:]
            return 0.0

        cutoff = now - window_sec
        while self.recent_angle_window and self.recent_angle_window[0][0] < cutoff:
            self.recent_angle_window.pop(0)
        if not self.recent_angle_window:
            return 0.0

        angles = [angle for _time_sec, angle in self.recent_angle_window]
        return max(angles) - min(angles)

    def _update_cycle_extrema(self, angle_deg: float) -> None:
        if self.cycle_min_angle is None or angle_deg < self.cycle_min_angle:
            self.cycle_min_angle = angle_deg
        if self.candidate_peak_angle is None or angle_deg > self.candidate_peak_angle:
            self.candidate_peak_angle = angle_deg

    def _detect_gyro_zero_crossing(
        self,
        time_sec: float,
        angle_deg: float,
        gyro_deg_s: float,
    ) -> bool:
        if gyro_deg_s > self.max_positive_gyro_since_zero:
            self.max_positive_gyro_since_zero = gyro_deg_s
        if self.previous_sample is None:
            return False

        previous_time, _previous_angle, previous_gyro = self.previous_sample
        positive_to_negative = previous_gyro > 0.0 and gyro_deg_s <= 0.0
        enough_velocity = (
            self.max_positive_gyro_since_zero >= self.config.gyro_zero_threshold_deg_s
        )
        cycle_min = self.cycle_min_angle if self.cycle_min_angle is not None else angle_deg
        cycle_max = self.candidate_peak_angle if self.candidate_peak_angle is not None else angle_deg
        swing_range = max(0.0, cycle_max - cycle_min)
        enough_swing = swing_range >= self.config.min_swing_range_deg
        peak_ratio = clamp(float(getattr(self.config, "phase_peak_ratio", 0.60)), 0.0, 0.95)
        angle_near_peak = angle_deg >= cycle_min + peak_ratio * swing_range
        if not (
            positive_to_negative
            and enough_velocity
            and enough_swing
            and angle_near_peak
        ):
            return False

        denominator = previous_gyro - gyro_deg_s
        ratio = previous_gyro / denominator if abs(denominator) > EPSILON else 0.0
        cross_time = previous_time + clamp(ratio, 0.0, 1.0) * (time_sec - previous_time)
        enough_gap = (
            self.last_zero_time is None
            or cross_time - self.last_zero_time >= self.config.min_cycle_period_sec
        )
        if not enough_gap:
            return False

        if self.last_zero_time is not None:
            observed_period = cross_time - self.last_zero_time
            if (
                self.config.min_cycle_period_sec
                <= observed_period
                <= self.config.max_cycle_period_sec
            ):
                self.previous_cycle_period_sec = observed_period

        self.last_zero_time = cross_time
        self.current_cycle_start_time = cross_time
        self.zero_event_count += 1
        self.max_positive_gyro_since_zero = max(0.0, gyro_deg_s)
        self.candidate_peak_angle = angle_deg
        self.cycle_min_angle = angle_deg
        return True
