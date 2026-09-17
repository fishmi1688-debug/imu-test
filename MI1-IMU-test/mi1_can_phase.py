#!/usr/bin/env python3
"""Generate left/right gait phase from two Molex MI1 CAN IMUs.

This file is intentionally self-contained.  It does not import helpers from
DF-IMU-test or from any other project directory.

Supported MI1/HiPNUC CAN layouts from the manual:

* J1939 extended frames:
  0xFF34 acceleration, 0xFF37 angular velocity, 0xFF46 quaternion.
  The source address byte is the IMU node ID.
* CANopen standard TPDO frames:
  0x180+node acceleration, 0x280+node angular velocity, 0x480+node quaternion.

The default setup matches the requested bench wiring:
left node 0x01, right node 0x02, SocketCAN interface can0, 50 Hz output.
"""

from __future__ import annotations

import argparse
import csv
import errno
import math
import os
import select
import signal
import socket
import struct
import sys
import tempfile
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional


DEFAULT_CAN_INTERFACE = "can0"
DEFAULT_LEFT_NODE_ID = 0x01
DEFAULT_RIGHT_NODE_ID = 0x02
DEFAULT_OUTPUT_RATE_HZ = 50.0
DEFAULT_MAX_AGE_SEC = 0.15
DEFAULT_STARTUP_TIMEOUT_SEC = 10.0
DEFAULT_PLOT_WINDOW_SEC = 10.0
DEFAULT_PLOT_RATE_HZ = 15.0

DEFAULT_LOW_PASS_CUTOFF_HZ = 6.0
DEFAULT_INITIAL_CYCLE_PERIOD_SEC = 1.0
DEFAULT_MIN_CYCLE_PERIOD_SEC = 0.70
DEFAULT_MAX_CYCLE_PERIOD_SEC = 3.0
DEFAULT_PEAK_PROMINENCE_DEG = 6.0
DEFAULT_PEAK_DROP_DEG = 2.0
DEFAULT_GYRO_ZERO_THRESHOLD_DEG_S = 10.0
DEFAULT_MIN_PHASE_ANGLE_RANGE_DEG = 25.0
DEFAULT_COMPLEMENTARY_ACC_WEIGHT = 0.02
DEFAULT_ANGLE_SIGN = -1.0
DEFAULT_GYRO_SIGN = 1.0
DEFAULT_ANGLE_SOURCE = "quat_y"
DEFAULT_ACC_LONG_AXIS = "x"
DEFAULT_ACC_SAGITTAL_AXIS = "z"
DEFAULT_GYRO_AXIS = "y"
DEFAULT_PHASE_SOURCE = "gyro_zero"

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

J1939_PGN_ACCEL = 0xFF34
J1939_PGN_GYRO = 0xFF37
J1939_PGN_QUAT = 0xFF46

CANOPEN_TPDO_ACCEL_BASE = 0x180
CANOPEN_TPDO_GYRO_BASE = 0x280
CANOPEN_TPDO_QUAT_BASE = 0x480

J1939_ACCEL_SCALE_G = 0.00048828
J1939_GYRO_SCALE_DPS = 0.061035
CANOPEN_ACCEL_SCALE_G = 0.001
CANOPEN_GYRO_SCALE_DPS = 0.1
QUAT_SCALE = 0.0001
EPSILON = 1e-9

STOP = False


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quat:
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class DecodedFrame:
    protocol: str
    node_id: int
    signal_name: str
    value: Vec3 | Quat


@dataclass
class ImuState:
    node_id: int
    protocol: Optional[str] = None
    accel_g: Optional[Vec3] = None
    gyro_dps: Optional[Vec3] = None
    quat: Optional[Quat] = None
    updated_at: dict[str, float] = field(default_factory=dict)
    frame_counts: dict[str, int] = field(default_factory=lambda: {"accel": 0, "gyro": 0, "quat": 0})

    def update(self, decoded: DecodedFrame, now: float) -> None:
        if self.protocol is None:
            self.protocol = decoded.protocol
        elif self.protocol != decoded.protocol:
            self.protocol = "mixed"

        if decoded.signal_name == "accel":
            self.accel_g = decoded.value  # type: ignore[assignment]
        elif decoded.signal_name == "gyro":
            self.gyro_dps = decoded.value  # type: ignore[assignment]
        elif decoded.signal_name == "quat":
            self.quat = decoded.value  # type: ignore[assignment]
        else:
            raise ValueError(f"unknown decoded signal: {decoded.signal_name}")

        self.updated_at[decoded.signal_name] = now
        self.frame_counts[decoded.signal_name] = self.frame_counts.get(decoded.signal_name, 0) + 1

    def is_complete(self) -> bool:
        return self.accel_g is not None and self.gyro_dps is not None and self.quat is not None

    def is_fresh(self, now: float, max_age_sec: float) -> bool:
        if not self.is_complete():
            return False
        if max_age_sec <= 0.0:
            return True
        return all(now - self.updated_at.get(name, -math.inf) <= max_age_sec for name in ("accel", "gyro", "quat"))

    def available_fields_text(self) -> str:
        fields = [name for name in ("accel", "gyro", "quat") if self.frame_counts.get(name, 0) > 0]
        return "+".join(fields) if fields else "none"

    def to_sample(self, role: str, now: float) -> "ImuSample":
        if self.accel_g is None or self.gyro_dps is None or self.quat is None:
            raise ValueError("cannot build sample from incomplete IMU state")
        return ImuSample(
            role=role,
            node_id=self.node_id,
            arrival_time=now,
            quat_w=self.quat.w,
            quat_x=self.quat.x,
            quat_y=self.quat.y,
            quat_z=self.quat.z,
            acc_x=self.accel_g.x,
            acc_y=self.accel_g.y,
            acc_z=self.accel_g.z,
            gyro_x=self.gyro_dps.x,
            gyro_y=self.gyro_dps.y,
            gyro_z=self.gyro_dps.z,
        )


@dataclass(frozen=True)
class ImuSample:
    role: str
    node_id: int
    arrival_time: float
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass
class PhaseEstimatorConfig:
    angle_sign: float = DEFAULT_ANGLE_SIGN
    gyro_sign: float = DEFAULT_GYRO_SIGN
    angle_source: str = DEFAULT_ANGLE_SOURCE
    acc_long_axis: str = DEFAULT_ACC_LONG_AXIS
    acc_sagittal_axis: str = DEFAULT_ACC_SAGITTAL_AXIS
    gyro_axis: str = DEFAULT_GYRO_AXIS
    lowpass_cutoff_hz: float = DEFAULT_LOW_PASS_CUTOFF_HZ
    initial_cycle_period_sec: float = DEFAULT_INITIAL_CYCLE_PERIOD_SEC
    min_cycle_period_sec: float = DEFAULT_MIN_CYCLE_PERIOD_SEC
    max_cycle_period_sec: float = DEFAULT_MAX_CYCLE_PERIOD_SEC
    peak_prominence_deg: float = DEFAULT_PEAK_PROMINENCE_DEG
    peak_drop_deg: float = DEFAULT_PEAK_DROP_DEG
    phase_source: str = DEFAULT_PHASE_SOURCE
    gyro_zero_threshold_deg_s: float = DEFAULT_GYRO_ZERO_THRESHOLD_DEG_S
    min_phase_angle_range_deg: float = DEFAULT_MIN_PHASE_ANGLE_RANGE_DEG
    complementary_acc_weight: float = DEFAULT_COMPLEMENTARY_ACC_WEIGHT
    phase_offset: float = 0.0
    clamp_phase: bool = True


@dataclass(frozen=True)
class ProcessedSample:
    time_sec: float
    angle_raw_deg: float
    angle_deg: float
    angular_velocity_raw_deg_s: float
    angular_velocity_deg_s: float
    angle_acc_deg: float
    angle_quat_x_deg: float
    angle_quat_y_deg: float
    angle_quat_z_deg: float
    angle_used_deg: float
    gyro_used_deg_s: float
    phase_0_to_1: float
    phase_percent: float
    previous_cycle_period_sec: float
    previous_cycle_frequency_hz: float
    zero_event: bool
    zero_event_count: int


@dataclass(frozen=True)
class PhaseInputs:
    angle_acc_deg: float
    angle_quat_x_deg: float
    angle_quat_y_deg: float
    angle_quat_z_deg: float
    angle_used_deg: float
    gyro_used_deg_s: float


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

        dt = max(0.0, time_sec - self.time_sec)
        if dt <= EPSILON:
            return self.value

        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (tau + dt)
        self.value = self.value + alpha * (float(value) - self.value)
        self.time_sec = float(time_sec)
        return self.value


class ThighPhaseEstimator:
    """Sagittal thigh phase estimator.

    Matches the imu_gait_UART phase pipeline: the caller selects the angle
    source and gyroscope axis first, then this estimator filters the selected
    angle/velocity and turns repeated zero events into gait phase.

    Default MI1 mounting convention:
    X points upward along the thigh, Y points to the user's left side, and
    Z points forward. With the default config this uses quaternion Euler Y and
    +gyro_y. The acceleration fallback is -atan2(acc_z, acc_x), so standing is
    close to 0 deg when gravity is aligned with the thigh X axis.
    """

    def __init__(self, config: PhaseEstimatorConfig):
        self.config = config
        self.angle_filter = FirstOrderLowpass(config.lowpass_cutoff_hz)
        self.gyro_filter = FirstOrderLowpass(config.lowpass_cutoff_hz)
        self.start_time: Optional[float] = None
        self.current_cycle_start_time: Optional[float] = None
        self.last_zero_time: Optional[float] = None
        self.previous_cycle_period_sec = clamp(
            config.initial_cycle_period_sec,
            config.min_cycle_period_sec,
            config.max_cycle_period_sec,
        )
        self.zero_event_count = 0
        self.previous_2: Optional[tuple[float, float, float]] = None
        self.previous_1: Optional[tuple[float, float, float]] = None
        self.recent_angles: deque[tuple[float, float]] = deque()
        self.candidate_peak_time: Optional[float] = None
        self.candidate_peak_angle: Optional[float] = None
        self.cycle_min_angle: Optional[float] = None
        self.complementary_angle: Optional[float] = None
        self.complementary_time: Optional[float] = None
        self.max_positive_gyro_since_zero = 0.0

    def reset(self) -> None:
        self.angle_filter.reset()
        self.gyro_filter.reset()
        self.start_time = None
        self.current_cycle_start_time = None
        self.last_zero_time = None
        self.previous_cycle_period_sec = clamp(
            self.config.initial_cycle_period_sec,
            self.config.min_cycle_period_sec,
            self.config.max_cycle_period_sec,
        )
        self.zero_event_count = 0
        self.previous_2 = None
        self.previous_1 = None
        self.recent_angles.clear()
        self.candidate_peak_time = None
        self.candidate_peak_angle = None
        self.cycle_min_angle = None
        self.complementary_angle = None
        self.complementary_time = None
        self.max_positive_gyro_since_zero = 0.0

    def process(self, sample: ImuSample, phase_inputs: PhaseInputs) -> ProcessedSample:
        if self.start_time is None:
            self.start_time = sample.arrival_time
        if self.current_cycle_start_time is None:
            self.current_cycle_start_time = sample.arrival_time

        time_sec = sample.arrival_time
        angle_acc = self.angle_filter.update(phase_inputs.angle_used_deg, time_sec)
        gyro = self.gyro_filter.update(phase_inputs.gyro_used_deg_s, time_sec)
        angle = self._update_complementary_angle(angle_acc, gyro, time_sec)

        self._append_recent_angle(time_sec, angle)
        self._update_cycle_extrema(time_sec, angle)
        if self.config.phase_source == "angle_peak":
            zero_event = self._detect_highest_lift_point(time_sec, angle, gyro)
        else:
            zero_event = self._detect_gyro_zero_crossing(time_sec, angle, gyro)

        period = max(self.previous_cycle_period_sec, EPSILON)
        waiting_for_first_event = self.zero_event_count == 0 and not zero_event
        elapsed = (
            0.0
            if zero_event or waiting_for_first_event
            else max(0.0, time_sec - (self.current_cycle_start_time or time_sec))
        )
        phase_raw = elapsed / period + self.config.phase_offset
        phase = clamp(phase_raw, 0.0, 1.0) if self.config.clamp_phase else wrap_phase(phase_raw)

        self.previous_2 = self.previous_1
        self.previous_1 = (time_sec, angle, gyro)

        return ProcessedSample(
            time_sec=time_sec - self.start_time,
            angle_raw_deg=phase_inputs.angle_used_deg,
            angle_deg=angle,
            angular_velocity_raw_deg_s=phase_inputs.gyro_used_deg_s,
            angular_velocity_deg_s=gyro,
            angle_acc_deg=phase_inputs.angle_acc_deg,
            angle_quat_x_deg=phase_inputs.angle_quat_x_deg,
            angle_quat_y_deg=phase_inputs.angle_quat_y_deg,
            angle_quat_z_deg=phase_inputs.angle_quat_z_deg,
            angle_used_deg=phase_inputs.angle_used_deg,
            gyro_used_deg_s=phase_inputs.gyro_used_deg_s,
            phase_0_to_1=phase,
            phase_percent=phase * 100.0,
            previous_cycle_period_sec=period,
            previous_cycle_frequency_hz=1.0 / period,
            zero_event=zero_event,
            zero_event_count=self.zero_event_count,
        )

    def _update_complementary_angle(self, acc_angle_deg: float, gyro_deg_s: float, time_sec: float) -> float:
        if self.complementary_angle is None or self.complementary_time is None:
            self.complementary_angle = acc_angle_deg
            self.complementary_time = time_sec
            return acc_angle_deg

        dt = max(0.0, time_sec - self.complementary_time)
        if dt <= EPSILON:
            return self.complementary_angle

        acc_weight = clamp(self.config.complementary_acc_weight, 0.0, 1.0)
        predicted = self.complementary_angle + gyro_deg_s * dt
        self.complementary_angle = (1.0 - acc_weight) * predicted + acc_weight * acc_angle_deg
        self.complementary_time = time_sec
        return self.complementary_angle

    def _append_recent_angle(self, time_sec: float, angle_deg: float) -> None:
        self.recent_angles.append((time_sec, angle_deg))
        cutoff = time_sec - self.config.max_cycle_period_sec
        while self.recent_angles and self.recent_angles[0][0] < cutoff:
            self.recent_angles.popleft()

    def _update_cycle_extrema(self, time_sec: float, angle_deg: float) -> None:
        if self.cycle_min_angle is None or angle_deg < self.cycle_min_angle:
            self.cycle_min_angle = angle_deg
        if self.candidate_peak_angle is None or angle_deg > self.candidate_peak_angle:
            self.candidate_peak_time = time_sec
            self.candidate_peak_angle = angle_deg

    def _detect_highest_lift_point(self, time_sec: float, angle_deg: float, gyro_deg_s: float) -> bool:
        del gyro_deg_s
        self._update_cycle_extrema(time_sec, angle_deg)
        if self.candidate_peak_time is None or self.candidate_peak_angle is None:
            return False

        peak_time = self.candidate_peak_time
        peak_angle = self.candidate_peak_angle
        cycle_min = self.cycle_min_angle if self.cycle_min_angle is not None else peak_angle
        enough_gap = self.last_zero_time is None or peak_time - self.last_zero_time >= self.config.min_cycle_period_sec
        enough_prominence = peak_angle - cycle_min >= self.config.peak_prominence_deg
        confirmed_fall = angle_deg <= peak_angle - self.config.peak_drop_deg
        peak_is_past = time_sec > peak_time + EPSILON

        if not (enough_gap and enough_prominence and confirmed_fall and peak_is_past):
            return False

        if self.last_zero_time is not None:
            observed_period = peak_time - self.last_zero_time
            if self.config.min_cycle_period_sec <= observed_period <= self.config.max_cycle_period_sec:
                self.previous_cycle_period_sec = observed_period

        self.last_zero_time = peak_time
        self.current_cycle_start_time = peak_time
        self.zero_event_count += 1
        self.candidate_peak_time = time_sec
        self.candidate_peak_angle = angle_deg
        self.cycle_min_angle = angle_deg
        self.recent_angles.clear()
        self.recent_angles.append((time_sec, angle_deg))
        return True

    def _detect_gyro_zero_crossing(self, time_sec: float, angle_deg: float, gyro_deg_s: float) -> bool:
        if gyro_deg_s > self.max_positive_gyro_since_zero:
            self.max_positive_gyro_since_zero = gyro_deg_s
        if self.previous_1 is None:
            return False

        previous_time, _previous_angle, previous_gyro = self.previous_1
        positive_to_negative = previous_gyro > 0.0 and gyro_deg_s <= 0.0
        enough_velocity = self.max_positive_gyro_since_zero >= self.config.gyro_zero_threshold_deg_s
        cycle_min = self.cycle_min_angle if self.cycle_min_angle is not None else angle_deg
        cycle_max = self.candidate_peak_angle if self.candidate_peak_angle is not None else angle_deg
        enough_angle_range = cycle_max - cycle_min >= self.config.min_phase_angle_range_deg
        if not (positive_to_negative and enough_velocity and enough_angle_range):
            return False

        denominator = previous_gyro - gyro_deg_s
        ratio = previous_gyro / denominator if abs(denominator) > EPSILON else 0.0
        cross_time = previous_time + clamp(ratio, 0.0, 1.0) * (time_sec - previous_time)
        enough_gap = self.last_zero_time is None or cross_time - self.last_zero_time >= self.config.min_cycle_period_sec
        if not enough_gap:
            return False

        if self.last_zero_time is not None:
            observed_period = cross_time - self.last_zero_time
            if self.config.min_cycle_period_sec <= observed_period <= self.config.max_cycle_period_sec:
                self.previous_cycle_period_sec = observed_period

        self.last_zero_time = cross_time
        self.current_cycle_start_time = cross_time
        self.zero_event_count += 1
        self.max_positive_gyro_since_zero = max(0.0, gyro_deg_s)
        self.candidate_peak_time = time_sec
        self.candidate_peak_angle = angle_deg
        self.cycle_min_angle = angle_deg
        self.recent_angles.clear()
        self.recent_angles.append((time_sec, angle_deg))
        return True


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


def sample_acc_axis(sample: ImuSample, axis: str) -> float:
    axis_map = {
        "x": sample.acc_x,
        "y": sample.acc_y,
        "z": sample.acc_z,
    }
    return axis_map[axis.lower()]


def sample_gyro_axis(sample: ImuSample, axis: str) -> float:
    axis_map = {
        "x": sample.gyro_x,
        "y": sample.gyro_y,
        "z": sample.gyro_z,
    }
    return axis_map[axis.lower()]


def phase_inputs_from_sample(sample: ImuSample, config: PhaseEstimatorConfig) -> PhaseInputs:
    long_acc = sample_acc_axis(sample, config.acc_long_axis)
    sagittal_acc = sample_acc_axis(sample, config.acc_sagittal_axis)
    angle_acc_deg = math.degrees(math.atan2(sagittal_acc, long_acc))

    euler_x_deg, euler_y_deg, euler_z_deg = quaternion_to_euler_xyz_degrees(
        sample.quat_w,
        sample.quat_x,
        sample.quat_y,
        sample.quat_z,
    )
    if config.angle_source == "quat_x":
        angle_used_deg = euler_x_deg
    elif config.angle_source == "quat_y":
        angle_used_deg = euler_y_deg
    elif config.angle_source == "quat_z":
        angle_used_deg = euler_z_deg
    else:
        angle_used_deg = angle_acc_deg
    angle_used_deg *= config.angle_sign

    gyro_used_deg_s = sample_gyro_axis(sample, config.gyro_axis) * config.gyro_sign
    return PhaseInputs(
        angle_acc_deg=angle_acc_deg,
        angle_quat_x_deg=euler_x_deg,
        angle_quat_y_deg=euler_y_deg,
        angle_quat_z_deg=euler_z_deg,
        angle_used_deg=angle_used_deg,
        gyro_used_deg_s=gyro_used_deg_s,
    )


def parse_int_auto(text: str) -> int:
    return int(text, 0)


def validate_node_id(value: int) -> int:
    if not 1 <= value <= 126:
        raise argparse.ArgumentTypeError("MI1 CAN node ID must be in range 1..126")
    return value


def local_time_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read two MI1 CAN IMUs and generate left/right gait phase.")
    parser.add_argument("--interface", default=DEFAULT_CAN_INTERFACE, help="SocketCAN interface, default: can0")
    parser.add_argument("--left-id", type=parse_int_auto, default=DEFAULT_LEFT_NODE_ID, help="Left MI1 node ID")
    parser.add_argument("--right-id", type=parse_int_auto, default=DEFAULT_RIGHT_NODE_ID, help="Right MI1 node ID")
    parser.add_argument(
        "--protocol",
        choices=("auto", "j1939", "canopen"),
        default="auto",
        help="CAN protocol to decode, default: auto",
    )
    parser.add_argument("--rate", type=float, default=DEFAULT_OUTPUT_RATE_HZ, help="Output rate in Hz")
    parser.add_argument("--samples", type=int, default=-1, help="Stop after N rows; default runs forever")
    parser.add_argument("--max-age", type=float, default=DEFAULT_MAX_AGE_SEC, help="Max field age in seconds")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=DEFAULT_STARTUP_TIMEOUT_SEC,
        help="Seconds to wait for complete left/right data; 0 waits forever",
    )
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV log path")
    parser.add_argument("--no-header", action="store_true", help="Do not print/write the CSV header")
    parser.add_argument("--no-stdout", action="store_true", help="Do not print CSV header or rows to stdout")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show a live plot of estimated phase, sagittal angle and angular velocity",
    )
    parser.add_argument(
        "--plot-window",
        type=float,
        default=DEFAULT_PLOT_WINDOW_SEC,
        help=f"Visible live plot time window in seconds, default: {DEFAULT_PLOT_WINDOW_SEC:g}",
    )
    parser.add_argument(
        "--plot-rate",
        type=float,
        default=DEFAULT_PLOT_RATE_HZ,
        help=f"Live plot refresh rate in Hz, default: {DEFAULT_PLOT_RATE_HZ:g}",
    )

    parser.add_argument(
        "--phase-source",
        choices=("gyro_zero", "angle_peak"),
        default=DEFAULT_PHASE_SOURCE,
        help="Phase zero detector, default: gyro_zero",
    )
    parser.add_argument(
        "--angle-source",
        choices=("acc", "quat_x", "quat_y", "quat_z"),
        default=DEFAULT_ANGLE_SOURCE,
        help="Angle used for phase; acc uses atan2(acc_sagittal_axis, acc_long_axis), default: quat_y",
    )
    parser.add_argument(
        "--acc-long-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_ACC_LONG_AXIS,
        help="IMU accelerometer axis along the thigh; default x for MI1 mount",
    )
    parser.add_argument(
        "--acc-sagittal-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_ACC_SAGITTAL_AXIS,
        help="IMU accelerometer axis in the sagittal plane; default z for MI1 mount",
    )
    parser.add_argument(
        "--gyro-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_GYRO_AXIS,
        help="IMU gyroscope axis for sagittal angular velocity; default y for MI1 mount",
    )
    parser.add_argument("--angle-sign", type=float, default=DEFAULT_ANGLE_SIGN, help="Default sagittal angle sign for both IMUs")
    parser.add_argument("--gyro-sign", type=float, default=DEFAULT_GYRO_SIGN, help="Default selected gyro-axis sign for both IMUs")
    parser.add_argument("--left-angle-sign", type=float, default=None, help="Override left angle sign")
    parser.add_argument("--left-gyro-sign", type=float, default=None, help="Override left gyro sign")
    parser.add_argument("--right-angle-sign", type=float, default=None, help="Override right angle sign")
    parser.add_argument("--right-gyro-sign", type=float, default=None, help="Override right gyro sign")
    parser.add_argument("--lowpass-cutoff", type=float, default=DEFAULT_LOW_PASS_CUTOFF_HZ)
    parser.add_argument("--initial-cycle-period", type=float, default=DEFAULT_INITIAL_CYCLE_PERIOD_SEC)
    parser.add_argument("--min-cycle-period", type=float, default=DEFAULT_MIN_CYCLE_PERIOD_SEC)
    parser.add_argument("--max-cycle-period", type=float, default=DEFAULT_MAX_CYCLE_PERIOD_SEC)
    parser.add_argument("--peak-prominence", type=float, default=DEFAULT_PEAK_PROMINENCE_DEG)
    parser.add_argument("--peak-drop", type=float, default=DEFAULT_PEAK_DROP_DEG)
    parser.add_argument("--gyro-zero-threshold", type=float, default=DEFAULT_GYRO_ZERO_THRESHOLD_DEG_S)
    parser.add_argument("--min-phase-angle-range", type=float, default=DEFAULT_MIN_PHASE_ANGLE_RANGE_DEG)
    parser.add_argument("--complementary-acc-weight", type=float, default=DEFAULT_COMPLEMENTARY_ACC_WEIGHT)
    parser.add_argument("--phase-offset", type=float, default=0.0, help="Additive phase offset in cycles")
    parser.add_argument("--no-clamp-phase", action="store_true", help="Wrap phase instead of clamping to 0..1")
    return parser


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    try:
        args.left_id = validate_node_id(args.left_id)
        args.right_id = validate_node_id(args.right_id)
    except argparse.ArgumentTypeError as exc:
        parser.error(str(exc))

    if args.left_id == args.right_id:
        parser.error("--left-id and --right-id must be different")
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.samples == 0 or args.samples < -1:
        parser.error("--samples must be positive, or omitted for continuous output")
    if args.max_age < 0.0:
        parser.error("--max-age must be non-negative")
    if args.startup_timeout < 0.0:
        parser.error("--startup-timeout must be non-negative")
    if args.plot_window <= 0.0:
        parser.error("--plot-window must be positive")
    if args.plot_rate <= 0.0:
        parser.error("--plot-rate must be positive")
    if args.lowpass_cutoff < 0.0:
        parser.error("--lowpass-cutoff must be non-negative")
    if args.initial_cycle_period <= 0.0:
        parser.error("--initial-cycle-period must be positive")
    if args.min_cycle_period <= 0.0:
        parser.error("--min-cycle-period must be positive")
    if args.max_cycle_period < args.min_cycle_period:
        parser.error("--max-cycle-period must be >= --min-cycle-period")
    if args.peak_prominence < 0.0 or args.peak_drop < 0.0:
        parser.error("--peak-prominence and --peak-drop must be non-negative")
    if args.gyro_zero_threshold < 0.0:
        parser.error("--gyro-zero-threshold must be non-negative")
    if args.min_phase_angle_range < 0.0:
        parser.error("--min-phase-angle-range must be non-negative")
    if not 0.0 <= args.complementary_acc_weight <= 1.0:
        parser.error("--complementary-acc-weight must be in range 0..1")


def decode_socketcan_frame(frame: bytes) -> Optional[tuple[int, bool, bytes]]:
    if len(frame) < CAN_FRAME_SIZE:
        return None

    can_id_flags, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, frame[:CAN_FRAME_SIZE])
    if can_id_flags & (CAN_ERR_FLAG | CAN_RTR_FLAG):
        return None

    is_extended = bool(can_id_flags & CAN_EFF_FLAG)
    can_id = can_id_flags & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    return can_id, is_extended, payload[: min(dlc, 8)]


def decode_vec3_i16(payload: bytes, scale: float) -> Optional[Vec3]:
    if len(payload) < 6:
        return None
    x_raw, y_raw, z_raw = struct.unpack_from("<hhh", payload)
    return Vec3(x_raw * scale, y_raw * scale, z_raw * scale)


def decode_quat_i16(payload: bytes) -> Optional[Quat]:
    if len(payload) < 8:
        return None
    w_raw, x_raw, y_raw, z_raw = struct.unpack_from("<hhhh", payload)
    return Quat(w_raw * QUAT_SCALE, x_raw * QUAT_SCALE, y_raw * QUAT_SCALE, z_raw * QUAT_SCALE)


def decode_j1939(can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if not is_extended:
        return None

    data_page = (can_id >> 24) & 0x01
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    source_address = can_id & 0xFF
    if pf != 0xFF:
        return None

    pgn = (data_page << 16) | (pf << 8) | ps
    if pgn == J1939_PGN_ACCEL:
        value = decode_vec3_i16(payload, J1939_ACCEL_SCALE_G)
        return DecodedFrame("j1939", source_address, "accel", value) if value is not None else None
    if pgn == J1939_PGN_GYRO:
        value = decode_vec3_i16(payload, J1939_GYRO_SCALE_DPS)
        return DecodedFrame("j1939", source_address, "gyro", value) if value is not None else None
    if pgn == J1939_PGN_QUAT:
        value = decode_quat_i16(payload)
        return DecodedFrame("j1939", source_address, "quat", value) if value is not None else None
    return None


def decode_canopen(can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if is_extended:
        return None

    node_id = can_id - CANOPEN_TPDO_ACCEL_BASE
    if 1 <= node_id <= 127:
        value = decode_vec3_i16(payload, CANOPEN_ACCEL_SCALE_G)
        return DecodedFrame("canopen", node_id, "accel", value) if value is not None else None

    node_id = can_id - CANOPEN_TPDO_GYRO_BASE
    if 1 <= node_id <= 127:
        value = decode_vec3_i16(payload, CANOPEN_GYRO_SCALE_DPS)
        return DecodedFrame("canopen", node_id, "gyro", value) if value is not None else None

    node_id = can_id - CANOPEN_TPDO_QUAT_BASE
    if 1 <= node_id <= 127:
        value = decode_quat_i16(payload)
        return DecodedFrame("canopen", node_id, "quat", value) if value is not None else None

    return None


def decode_imu_frame(protocol: str, can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if protocol in ("auto", "j1939"):
        decoded = decode_j1939(can_id, is_extended, payload)
        if decoded is not None:
            return decoded
    if protocol in ("auto", "canopen"):
        return decode_canopen(can_id, is_extended, payload)
    return None


def open_can_socket(interface: str) -> socket.socket:
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((interface,))
    except OSError as exc:
        if exc.errno == errno.ENODEV:
            raise RuntimeError(f"CAN interface {interface!r} does not exist or is not up") from exc
        if exc.errno in (errno.EPERM, errno.EACCES):
            raise RuntimeError(f"permission denied opening {interface!r}; run with sudo") from exc
        raise
    sock.setblocking(False)
    return sock


def process_available_frames(
    sock: socket.socket,
    protocol: str,
    target_nodes: set[int],
    states: dict[int, ImuState],
) -> None:
    while True:
        try:
            ready, _, _ = select.select([sock], [], [], 0.0)
        except InterruptedError:
            return
        if not ready:
            return

        try:
            frame = sock.recv(CAN_FRAME_SIZE)
        except BlockingIOError:
            return

        decoded_frame = decode_socketcan_frame(frame)
        if decoded_frame is None:
            continue

        can_id, is_extended, payload = decoded_frame
        decoded = decode_imu_frame(protocol, can_id, is_extended, payload)
        if decoded is None or decoded.node_id not in target_nodes:
            continue

        now = time.monotonic()
        state = states.setdefault(decoded.node_id, ImuState(decoded.node_id))
        state.update(decoded, now)


def phase_config_for_role(args: argparse.Namespace, role: str) -> PhaseEstimatorConfig:
    if role == "left":
        angle_sign = args.left_angle_sign if args.left_angle_sign is not None else args.angle_sign
        gyro_sign = args.left_gyro_sign if args.left_gyro_sign is not None else args.gyro_sign
    elif role == "right":
        angle_sign = args.right_angle_sign if args.right_angle_sign is not None else args.angle_sign
        gyro_sign = args.right_gyro_sign if args.right_gyro_sign is not None else args.gyro_sign
    else:
        raise ValueError(f"unknown role: {role}")

    return PhaseEstimatorConfig(
        angle_sign=angle_sign,
        gyro_sign=gyro_sign,
        angle_source=args.angle_source,
        acc_long_axis=args.acc_long_axis,
        acc_sagittal_axis=args.acc_sagittal_axis,
        gyro_axis=args.gyro_axis,
        lowpass_cutoff_hz=args.lowpass_cutoff,
        initial_cycle_period_sec=args.initial_cycle_period,
        min_cycle_period_sec=args.min_cycle_period,
        max_cycle_period_sec=args.max_cycle_period,
        peak_prominence_deg=args.peak_prominence,
        peak_drop_deg=args.peak_drop,
        phase_source=args.phase_source,
        gyro_zero_threshold_deg_s=args.gyro_zero_threshold,
        min_phase_angle_range_deg=args.min_phase_angle_range,
        complementary_acc_weight=args.complementary_acc_weight,
        phase_offset=args.phase_offset,
        clamp_phase=not args.no_clamp_phase,
    )


def role_output_fields(prefix: str) -> list[str]:
    return [
        f"{prefix}_node",
        f"{prefix}_protocol",
        f"{prefix}_phase_0_to_1",
        f"{prefix}_phase_percent",
        f"{prefix}_zero_event",
        f"{prefix}_zero_event_count",
        f"{prefix}_angle_raw_deg",
        f"{prefix}_angle_deg",
        f"{prefix}_angular_velocity_raw_deg_s",
        f"{prefix}_angular_velocity_deg_s",
        f"{prefix}_angle_acc_deg",
        f"{prefix}_angle_quat_x_deg",
        f"{prefix}_angle_quat_y_deg",
        f"{prefix}_angle_quat_z_deg",
        f"{prefix}_angle_used_deg",
        f"{prefix}_gyro_used_deg_s",
        f"{prefix}_cycle_period_sec",
        f"{prefix}_cycle_frequency_hz",
        f"{prefix}_qw",
        f"{prefix}_qx",
        f"{prefix}_qy",
        f"{prefix}_qz",
        f"{prefix}_ax_g",
        f"{prefix}_ay_g",
        f"{prefix}_az_g",
        f"{prefix}_gx_dps",
        f"{prefix}_gy_dps",
        f"{prefix}_gz_dps",
    ]


def format_role_output(state: ImuState, sample: ImuSample, processed: ProcessedSample) -> list[str]:
    return [
        f"0x{sample.node_id:02X}",
        state.protocol or "unknown",
        f"{processed.phase_0_to_1:.9f}",
        f"{processed.phase_percent:.6f}",
        "1" if processed.zero_event else "0",
        str(processed.zero_event_count),
        f"{processed.angle_raw_deg:.6f}",
        f"{processed.angle_deg:.6f}",
        f"{processed.angular_velocity_raw_deg_s:.6f}",
        f"{processed.angular_velocity_deg_s:.6f}",
        f"{processed.angle_acc_deg:.6f}",
        f"{processed.angle_quat_x_deg:.6f}",
        f"{processed.angle_quat_y_deg:.6f}",
        f"{processed.angle_quat_z_deg:.6f}",
        f"{processed.angle_used_deg:.6f}",
        f"{processed.gyro_used_deg_s:.6f}",
        f"{processed.previous_cycle_period_sec:.6f}",
        f"{processed.previous_cycle_frequency_hz:.6f}",
        f"{sample.quat_w:.7f}",
        f"{sample.quat_x:.7f}",
        f"{sample.quat_y:.7f}",
        f"{sample.quat_z:.7f}",
        f"{sample.acc_x:.6f}",
        f"{sample.acc_y:.6f}",
        f"{sample.acc_z:.6f}",
        f"{sample.gyro_x:.6f}",
        f"{sample.gyro_y:.6f}",
        f"{sample.gyro_z:.6f}",
    ]


def status_text(states: dict[int, ImuState], role_nodes: list[tuple[str, int]]) -> str:
    details = []
    for role, node_id in role_nodes:
        state = states.get(node_id)
        if state is None:
            details.append(f"{role}=0x{node_id:02X}:none")
        else:
            protocol = state.protocol or "unknown"
            details.append(f"{role}=0x{node_id:02X}:{protocol}:{state.available_fields_text()}")
    return ", ".join(details)


def has_new_complete_sample(
    state: ImuState,
    now: float,
    max_age_sec: float,
    last_counts: dict[str, int],
) -> bool:
    if not state.is_fresh(now, max_age_sec):
        return False
    return all(state.frame_counts.get(name, 0) > last_counts.get(name, 0) for name in ("accel", "gyro", "quat"))


def mark_sample_emitted(state: ImuState, last_counts: dict[str, int]) -> None:
    for name in ("accel", "gyro", "quat"):
        last_counts[name] = state.frame_counts.get(name, 0)


def open_csv_writer(path: Optional[Path], header: list[str], write_header: bool):
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    if write_header:
        writer.writerow(header)
        csv_file.flush()
    return csv_file, writer


class LivePhasePlotter:
    def __init__(self, roles: list[str], window_sec: float, refresh_rate_hz: float):
        self.roles = roles
        self.window_sec = float(window_sec)
        self.refresh_interval_sec = 1.0 / max(float(refresh_rate_hz), EPSILON)
        self.last_draw_time = 0.0
        self.closed = False
        self.buffers = {
            role: {
                "time": deque(),
                "phase": deque(),
                "angle": deque(),
                "angular_velocity": deque(),
            }
            for role in self.roles
        }

        mpl_config_dir = Path(tempfile.gettempdir()) / "matplotlib"
        mpl_config_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(mpl_config_dir))

        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError("matplotlib is required for --plot; install python3-matplotlib") from exc

        backend = matplotlib.get_backend().lower()
        noninteractive_backends = {"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"}
        if backend in noninteractive_backends or backend.startswith("module://matplotlib_inline"):
            raise RuntimeError(
                "--plot needs an interactive Matplotlib backend, but the current backend is "
                f"{matplotlib.get_backend()!r}. Run from a desktop/X11 session or set MPLBACKEND=QtAgg/TkAgg."
            )

        self.plt = plt
        self.plt.ion()
        self.fig, self.axes = self.plt.subplots(3, 1, sharex=True, figsize=(10, 7), constrained_layout=True)
        manager = getattr(self.fig.canvas, "manager", None)
        if manager is not None and hasattr(manager, "set_window_title"):
            manager.set_window_title("MI1 gait phase live view")
        self.fig.suptitle("MI1 gait phase live view")
        self.fig.canvas.mpl_connect("close_event", self._handle_close)

        metrics = [
            ("phase", "Phase (%)"),
            ("angle", "Sagittal angle (deg)"),
            ("angular_velocity", "Angular velocity (deg/s)"),
        ]
        colors = {"left": "tab:blue", "right": "tab:orange"}
        self.lines = {role: {} for role in self.roles}
        for axis, (metric, ylabel) in zip(self.axes, metrics):
            for role in self.roles:
                (line,) = axis.plot([], [], label=role, linewidth=1.6, color=colors.get(role))
                self.lines[role][metric] = line
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper right")
        self.axes[0].set_ylim(-2.0, 102.0)
        self.axes[-1].set_xlabel("Time (s)")
        self.plt.show(block=False)

    def add_samples(self, processed_by_role: dict[str, ProcessedSample]) -> bool:
        if self.closed:
            return False

        for role, processed in processed_by_role.items():
            if role not in self.buffers:
                continue
            buffer = self.buffers[role]
            buffer["time"].append(processed.time_sec)
            buffer["phase"].append(processed.phase_percent)
            buffer["angle"].append(processed.angle_deg)
            buffer["angular_velocity"].append(processed.angular_velocity_deg_s)

        self._trim_buffers()
        self._draw_if_due()
        return not self.closed

    def close(self) -> None:
        if not self.closed:
            self.closed = True
            self.plt.close(self.fig)

    def _handle_close(self, _event: object) -> None:
        self.closed = True

    def _trim_buffers(self) -> None:
        latest_time = self._latest_time()
        if latest_time is None:
            return
        cutoff = latest_time - self.window_sec
        for role in self.roles:
            buffer = self.buffers[role]
            while buffer["time"] and buffer["time"][0] < cutoff:
                for values in buffer.values():
                    values.popleft()

    def _draw_if_due(self) -> None:
        now = time.monotonic()
        if now - self.last_draw_time < self.refresh_interval_sec:
            return

        latest_time = self._latest_time()
        if latest_time is None:
            return

        x_min = max(0.0, latest_time - self.window_sec)
        x_max = max(self.window_sec, latest_time)
        for axis in self.axes:
            axis.set_xlim(x_min, x_max)

        for role in self.roles:
            buffer = self.buffers[role]
            time_values = list(buffer["time"])
            for metric in ("phase", "angle", "angular_velocity"):
                self.lines[role][metric].set_data(time_values, list(buffer[metric]))

        for axis in self.axes[1:]:
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
            axis.margins(y=0.15)

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)
        self.last_draw_time = now

    def _latest_time(self) -> Optional[float]:
        latest_values = [buffer["time"][-1] for buffer in self.buffers.values() if buffer["time"]]
        return max(latest_values) if latest_values else None


def run_loop(args: argparse.Namespace, sock: socket.socket) -> int:
    role_nodes = [("left", args.left_id), ("right", args.right_id)]
    target_nodes = {node_id for _role, node_id in role_nodes}
    states = {node_id: ImuState(node_id) for _role, node_id in role_nodes}
    phase_configs = {role: phase_config_for_role(args, role) for role, _node in role_nodes}
    estimators = {role: ThighPhaseEstimator(phase_configs[role]) for role, _node in role_nodes}
    last_emitted_counts = {node_id: {"accel": 0, "gyro": 0, "quat": 0} for _role, node_id in role_nodes}

    header = ["local_time", "t_ns", "hz"]
    for role, _node_id in role_nodes:
        header.extend(role_output_fields(role))

    csv_file = None
    csv_writer = None
    live_plotter = None
    try:
        csv_file, csv_writer = open_csv_writer(args.csv, header, not args.no_header)
        if args.plot:
            live_plotter = LivePhasePlotter([role for role, _node_id in role_nodes], args.plot_window, args.plot_rate)

        if not args.no_stdout and not args.no_header:
            print(",".join(header), flush=True)

        period = 1.0 / args.rate
        next_output = time.monotonic() + period
        start_wait = time.monotonic()
        last_status = start_wait
        last_output: Optional[float] = None
        output_count = 0

        while not STOP and (args.samples < 0 or output_count < args.samples):
            now = time.monotonic()
            timeout = max(0.0, next_output - now)
            try:
                ready, _, _ = select.select([sock], [], [], timeout)
            except InterruptedError:
                continue
            if ready:
                process_available_frames(sock, args.protocol, target_nodes, states)

            now = time.monotonic()
            process_available_frames(sock, args.protocol, target_nodes, states)
            if now < next_output:
                continue

            have_new_data = all(
                has_new_complete_sample(states[node_id], now, args.max_age, last_emitted_counts[node_id])
                for _role, node_id in role_nodes
            )
            if have_new_data:
                measured_hz = 0.0 if last_output is None else 1.0 / max(now - last_output, EPSILON)
                row = [local_time_text(), str(time.monotonic_ns()), f"{measured_hz:.2f}"]
                processed_by_role = {}
                for role, node_id in role_nodes:
                    state = states[node_id]
                    sample = state.to_sample(role, now)
                    phase_inputs = phase_inputs_from_sample(sample, phase_configs[role])
                    processed = estimators[role].process(sample, phase_inputs)
                    processed_by_role[role] = processed
                    row.extend(format_role_output(state, sample, processed))
                    mark_sample_emitted(state, last_emitted_counts[node_id])
                if not args.no_stdout:
                    print(",".join(row), flush=True)
                if csv_writer is not None:
                    csv_writer.writerow(row)
                    if output_count % 50 == 0 and csv_file is not None:
                        csv_file.flush()
                if live_plotter is not None and not live_plotter.add_samples(processed_by_role):
                    break
                last_output = now
                output_count += 1
            elif now - last_status >= 1.0:
                print(
                    f"Waiting for new complete MI1 accel/gyro/quat data: {status_text(states, role_nodes)}",
                    file=sys.stderr,
                    flush=True,
                )
                last_status = now

            if output_count == 0 and args.startup_timeout > 0.0 and now - start_wait >= args.startup_timeout:
                raise RuntimeError(
                    "timed out waiting for left/right MI1 acceleration, gyroscope and quaternion frames. "
                    f"Decoded state: {status_text(states, role_nodes)}. "
                    "Check CAN_H/CAN_L, termination, slcand can0, 1 Mbit/s baud, protocol, and node IDs 0x01/0x02."
                )

            next_output += period
            if next_output < now - period:
                next_output = now + period

        return 0 if not STOP else 130
    finally:
        if live_plotter is not None:
            live_plotter.close()
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()


def handle_signal(signum: int, frame: object) -> None:
    del signum, frame
    global STOP
    STOP = True


def main(argv: Optional[list[str]] = None) -> int:
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    parser = build_arg_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)

    sock = open_can_socket(args.interface)
    try:
        print(
            "MI1 CAN phase reader: "
            f"interface={args.interface}, protocol={args.protocol}, rate={args.rate:g} Hz, "
            f"left=0x{args.left_id:02X}, right=0x{args.right_id:02X}, "
            f"angle_source={args.angle_source}, "
            f"acc_angle=atan2(Acc_{args.acc_sagittal_axis.upper()}, Acc_{args.acc_long_axis.upper()}), "
            f"gyro_axis={args.gyro_axis.upper()}, phase_source={args.phase_source}",
            file=sys.stderr,
            flush=True,
        )
        return run_loop(args, sock)
    finally:
        sock.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
