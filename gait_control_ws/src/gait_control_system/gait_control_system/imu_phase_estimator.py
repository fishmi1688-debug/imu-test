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


@dataclass
class ImuPhaseConfig:
    angle_sign: float = 1.0
    gyro_sign: float = 1.0
    gyro_unit: str = "deg"
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

    The event detector follows the standalone imu_gait tool:
    angle = atan2(Acc_Z, Acc_X), angular velocity = Gyr_Y, phase zero is the
    positive-to-negative Gyr_Y zero crossing after sufficient swing amplitude.
    During a cycle, phase advances using the previous observed cycle period.
    """

    def __init__(self, config: Optional[ImuPhaseConfig] = None):
        self.config = config or ImuPhaseConfig()
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

    def update_swing_threshold(self, threshold_deg: float) -> None:
        self.config.min_swing_range_deg = max(0.0, float(threshold_deg))

    def process_6d(self, sample_6d, sample_time: float) -> ImuPhaseOutput:
        """Process [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]."""
        acc_x = float(sample_6d[0])
        acc_z = float(sample_6d[2])
        gyro_y = float(sample_6d[4])

        if self.current_cycle_start_time is None:
            self.current_cycle_start_time = float(sample_time)

        angle_raw = self.config.angle_sign * math.degrees(math.atan2(acc_z, acc_x))
        gyro_raw = self.config.gyro_sign * gyro_y
        if str(self.config.gyro_unit).strip().lower().startswith("rad"):
            gyro_raw = math.degrees(gyro_raw)

        angle_acc = self.angle_filter.update(angle_raw, sample_time)
        gyro = self.gyro_filter.update(gyro_raw, sample_time)
        angle = self._update_complementary_angle(angle_acc, gyro, sample_time)
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
        if zero_event or (abs(gyro) >= motion_velocity_threshold and has_meaningful_swing):
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
