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


def _axis_index(axis: str | int, *, name: str) -> int:
    if isinstance(axis, int):
        value = int(axis)
        if 0 <= value <= 2:
            return value
        raise ValueError(f"{name} must be 0, 1, 2 or x, y, z")

    cleaned = str(axis or "").strip().lower()
    axis_map = {"x": 0, "0": 0, "y": 1, "1": 1, "z": 2, "2": 2}
    if cleaned not in axis_map:
        raise ValueError(f"{name} must be x, y, z, 0, 1, or 2")
    return axis_map[cleaned]


@dataclass
class ImuPhaseConfig:
    angle_sign: float = 1.0
    gyro_sign: float = 1.0
    gyro_unit: str = "deg"
    angle_source: str = "acc"
    acc_angle_numerator_axis: str | int = "z"
    acc_angle_denominator_axis: str | int = "x"
    gyro_axis: str | int = "y"
    lowpass_cutoff_hz: float = 6.0
    initial_cycle_period_sec: float = 1.0
    min_cycle_period_sec: float = 0.70
    max_cycle_period_sec: float = 3.0
    gyro_zero_threshold_deg_s: float = 10.0
    min_swing_range_deg: float = 25.0
    complementary_acc_weight: float = 0.02
    phase_offset: float = 0.0
    clamp_phase: bool = True
    motion_timeout_sec: float = 0.45
    motion_window_sec: float = 0.5
    end_phase_stop_threshold: float = 0.98
    motion_swing_range_ratio: float = 0.25


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


class ThighImuPhaseEstimator:
    """Estimate 0-1 gait phase from thigh sagittal angle and angular velocity.

    By default the event detector follows the earlier thigh IMU convention:
    angle = atan2(Acc_Z, Acc_X), angular velocity = Gyr_Y, phase zero is the
    positive-to-negative Gyr_Y zero crossing after sufficient swing amplitude.
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
        self.gyro_index = 3 + _axis_index(self.config.gyro_axis, name="gyro_axis")
        self.angle_filter = FirstOrderLowpass(self.config.lowpass_cutoff_hz)
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

    def update_swing_threshold(self, threshold_deg: float) -> None:
        self.config.min_swing_range_deg = max(0.0, float(threshold_deg))

    def process_6d(self, sample_6d, sample_time: float) -> ImuPhaseOutput:
        """Process [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, ...]."""
        angle_raw, gyro_raw = self.extract_raw_signal(sample_6d)
        return self.process_signal(angle_raw, gyro_raw, sample_time)

    def extract_raw_signal(self, sample_6d) -> tuple[float, float]:
        """Return configured raw sagittal angle and angular velocity in deg/deg-s."""
        values = [float(value) for value in sample_6d]
        if len(values) < 6:
            raise ValueError(f"IMU phase sample needs at least 6 values, got {len(values)}")
        acc_numerator = values[self.acc_angle_numerator_index]
        acc_denominator = values[self.acc_angle_denominator_index]
        gyro_axis_value = values[self.gyro_index]

        angle_acc_deg = math.degrees(math.atan2(acc_numerator, acc_denominator))
        angle_source = str(getattr(self.config, "angle_source", "acc")).strip().lower()
        angle_selected_deg = angle_acc_deg
        if angle_source.startswith("quat") and len(values) >= 10:
            euler_x_deg, euler_y_deg, euler_z_deg = quaternion_to_euler_xyz_degrees(
                values[6],
                values[7],
                values[8],
                values[9],
            )
            if angle_source == "quat_x":
                angle_selected_deg = euler_x_deg
            elif angle_source == "quat_y":
                angle_selected_deg = euler_y_deg
            elif angle_source == "quat_z":
                angle_selected_deg = euler_z_deg
        angle_raw = self.config.angle_sign * angle_selected_deg
        gyro_raw = self.config.gyro_sign * gyro_axis_value
        if str(self.config.gyro_unit).strip().lower().startswith("rad"):
            gyro_raw = math.degrees(gyro_raw)
        return float(angle_raw), float(gyro_raw)

    def process_signal(
        self,
        angle_raw_deg: float,
        gyro_raw_deg_s: float,
        sample_time: float,
    ) -> ImuPhaseOutput:
        """Process an already-computed sagittal angle signal."""
        angle_raw = float(angle_raw_deg)
        gyro_raw = float(gyro_raw_deg_s)
        if self.current_cycle_start_time is None:
            self.current_cycle_start_time = float(sample_time)

        angle_acc = self.angle_filter.update(angle_raw, sample_time)
        gyro = self.gyro_filter.update(gyro_raw, sample_time)
        angle = self._update_complementary_angle(angle_acc, gyro, sample_time)
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

        self.previous_sample = (float(sample_time), angle, gyro)

        return ImuPhaseOutput(
            angle_raw_deg=angle_raw,
            angle_deg=angle,
            angular_velocity_raw_deg_s=gyro_raw,
            angular_velocity_deg_s=gyro,
            phase_0_to_1=phase,
            phase_rad=phase * 2.0 * math.pi,
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
        )

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
        enough_swing = cycle_max - cycle_min >= self.config.min_swing_range_deg
        if not (positive_to_negative and enough_velocity and enough_swing):
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
