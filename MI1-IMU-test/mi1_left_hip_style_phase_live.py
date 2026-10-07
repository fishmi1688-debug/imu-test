#!/usr/bin/env python3
"""Live MI1 left-IMU phase test using the gait_control imu_left_phase logic.

This script connects to one MI1/HiPNUC CAN IMU, keeps the same configurable IMU
axis mapping used by gait_control, and feeds the extracted angle/gyro into the
hip-controller-style preprocessing and phase generator.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import tempfile
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
GAIT_PACKAGE_SRC = REPO_ROOT / "gait_control_ws" / "src" / "gait_control_system"
if str(GAIT_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(GAIT_PACKAGE_SRC))

from gait_control_system.hip_style_imu_phase_estimator import HipStyleImuPhaseEstimator
from gait_control_system.imu_phase_estimator import ImuPhaseConfig, ThighImuPhaseEstimator
from gait_control_system.mi1_imu_phase_source import (
    DEFAULT_CAN_INTERFACE,
    DEFAULT_DATA_TIMEOUT_SEC,
    DEFAULT_EXPECTED_RATE_HZ,
    DEFAULT_LEFT_NODE_ID,
    WiredMi1CanImuPhaseSource,
    format_node_id,
)


STOP = False

DEFAULT_ANGLE_SIGN = -1.0
DEFAULT_GYRO_SIGN = 1.0
DEFAULT_ANGLE_SOURCE = "quat_sagittal"
DEFAULT_ACC_ANGLE_NUM_AXIS = "z"
DEFAULT_ACC_ANGLE_DEN_AXIS = "x"
DEFAULT_ACC_ANGLE_NUM_SIGN = -1.0
DEFAULT_ACC_ANGLE_DEN_SIGN = -1.0
DEFAULT_GYRO_AXIS = "y"
DEFAULT_QUAT_THIGH_AXIS = "x"
DEFAULT_QUAT_FORWARD_AXIS = "x"
DEFAULT_QUAT_VERTICAL_AXIS = "z"
DEFAULT_PLOT_WINDOW_SEC = 12.0
DEFAULT_PLOT_RATE_HZ = 20.0


def parse_int_auto(text: str) -> int:
    return int(str(text), 0)


def local_time_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def signed_axis_text(sign: float, axis: str) -> str:
    return f"{'-' if float(sign) < 0.0 else ''}{str(axis).upper()}"


def handle_signal(_signum, _frame) -> None:
    global STOP
    STOP = True


def sample_fields(sample: tuple[float, ...]) -> dict[str, float]:
    values = list(float(v) for v in sample)
    while len(values) < 10:
        values.append(float("nan"))
    return {
        "acc_x": values[0],
        "acc_y": values[1],
        "acc_z": values[2],
        "gyro_x": values[3],
        "gyro_y": values[4],
        "gyro_z": values[5],
        "quat_w": values[6],
        "quat_x": values[7],
        "quat_y": values[8],
        "quat_z": values[9],
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Connect to one MI1 CAN IMU and plot raw/processed data plus the "
            "hip-controller-style gait phase used by imu_left_phase."
        )
    )
    parser.add_argument("--interface", default=DEFAULT_CAN_INTERFACE, help="SocketCAN interface, default: can0")
    parser.add_argument("--node-id", type=parse_int_auto, default=DEFAULT_LEFT_NODE_ID, help="MI1 node ID, default: 0x01")
    parser.add_argument("--side", choices=("left", "right"), default="left", help="MI1 source side label")
    parser.add_argument("--protocol", choices=("auto", "j1939", "canopen"), default="auto", help="CAN protocol")
    parser.add_argument("--rate", type=float, default=DEFAULT_EXPECTED_RATE_HZ, help="Expected IMU rate in Hz")
    parser.add_argument("--data-timeout", type=float, default=DEFAULT_DATA_TIMEOUT_SEC, help="Fresh sample timeout in seconds")
    parser.add_argument("--max-field-age", type=float, default=None, help="Max age for acc/gyro/quat fields")
    parser.add_argument("--require-quaternion", action="store_true", help="Require quaternion fields in emitted samples")

    parser.add_argument(
        "--angle-source",
        choices=("acc", "quat_x", "quat_y", "quat_z", "quat_sagittal"),
        default=DEFAULT_ANGLE_SOURCE,
        help="Angle source. Default projects the quaternion-rotated thigh axis into the sagittal plane.",
    )
    parser.add_argument("--acc-angle-num-axis", choices=("x", "y", "z"), default=DEFAULT_ACC_ANGLE_NUM_AXIS)
    parser.add_argument("--acc-angle-den-axis", choices=("x", "y", "z"), default=DEFAULT_ACC_ANGLE_DEN_AXIS)
    parser.add_argument("--acc-angle-num-sign", type=float, default=DEFAULT_ACC_ANGLE_NUM_SIGN)
    parser.add_argument("--acc-angle-den-sign", type=float, default=DEFAULT_ACC_ANGLE_DEN_SIGN)
    parser.add_argument("--gyro-axis", choices=("x", "y", "z"), default=DEFAULT_GYRO_AXIS)
    parser.add_argument(
        "--quat-thigh-axis",
        choices=("x", "+x", "-x", "y", "+y", "-y", "z", "+z", "-z"),
        default=DEFAULT_QUAT_THIGH_AXIS,
        help="Body-frame axis along the thigh for quat_sagittal. Current mounting uses x.",
    )
    parser.add_argument(
        "--quat-forward-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_QUAT_FORWARD_AXIS,
        help="Fallback reference-frame forward axis for quat_sagittal when heading is degenerate.",
    )
    parser.add_argument(
        "--quat-vertical-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_QUAT_VERTICAL_AXIS,
        help="Reference-frame vertical axis used by quat_sagittal.",
    )
    parser.add_argument("--angle-sign", type=float, default=DEFAULT_ANGLE_SIGN)
    parser.add_argument("--gyro-sign", type=float, default=DEFAULT_GYRO_SIGN)
    parser.add_argument("--swing-threshold", type=float, default=25.0, help="Motion swing threshold in degrees")
    parser.add_argument("--motion-timeout", type=float, default=0.45, help="No-motion timeout in seconds")
    parser.add_argument(
        "--startup-warmup-strides",
        type=int,
        default=0,
        help="Suppress hip-style phase output until this many startup stride events have been observed.",
    )
    parser.add_argument(
        "--hip-phase-offset",
        "--phase-offset",
        dest="hip_phase_offset",
        type=float,
        default=0.0,
        help="Extra hip-style phase offset in cycles. Event-reference phase stays unshifted.",
    )
    parser.add_argument("--disable-startup-seed", action="store_true", help="Disable startup temporary phase seed")
    parser.add_argument("--startup-seed-min-window", type=float, default=0.10, help="Minimum gyro decision window in seconds")
    parser.add_argument("--startup-seed-max-window", type=float, default=0.20, help="Maximum gyro decision window in seconds")
    parser.add_argument("--startup-seed-gyro-threshold", type=float, default=15.0, help="Mean raw gyro threshold for startup seed in deg/s")
    parser.add_argument("--startup-seed-angle-threshold", type=float, default=3.0, help="Fallback angle delta threshold from stop baseline in degrees")
    parser.add_argument("--startup-seed-positive-phase", type=float, default=0.0, help="Temporary phase for positive startup gyro direction, radians")
    parser.add_argument("--startup-seed-negative-phase", type=float, default=-math.pi, help="Temporary phase for negative startup gyro direction, radians")
    parser.add_argument("--startup-seed-ramp", type=float, default=0.30, help="Assist ramp duration after startup seed in seconds")
    parser.add_argument("--stop-window", type=float, default=0.30, help="Quiet window for motion-to-stop detection in seconds")
    parser.add_argument("--stop-hold", type=float, default=0.12, help="Extra quiet hold before declaring stopped in seconds")
    parser.add_argument("--stop-gyro-rms-threshold", type=float, default=8.0, help="Stop gyro RMS threshold in deg/s")
    parser.add_argument("--stop-angle-range-threshold", type=float, default=3.0, help="Stop angle range threshold in degrees")
    parser.add_argument("--stop-phase-rate-limit", type=float, default=4.0, help="Max allowed real phase advance during stop transition in Hz; 0 disables")
    parser.add_argument("--stop-unstable-hold", type=float, default=0.25, help="Freeze phase and suppress motion this long after an unstable phase jump")

    parser.add_argument("--plot-window", type=float, default=DEFAULT_PLOT_WINDOW_SEC, help="Visible plot window in seconds")
    parser.add_argument("--plot-rate", type=float, default=DEFAULT_PLOT_RATE_HZ, help="Plot refresh rate in Hz")
    parser.add_argument("--no-plot", action="store_true", help="Disable live plot")
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV log path")
    parser.add_argument("--samples", type=int, default=-1, help="Stop after N processed samples; default runs forever")
    parser.add_argument("--print-every", type=int, default=10, help="Print every N processed samples; 0 disables stdout rows")
    return parser


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if not 1 <= int(args.node_id) <= 126:
        parser.error("--node-id must be in range 1..126")
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    if args.data_timeout <= 0.0:
        parser.error("--data-timeout must be positive")
    if args.max_field_age is not None and args.max_field_age < 0.0:
        parser.error("--max-field-age must be non-negative")
    if args.swing_threshold < 0.0:
        parser.error("--swing-threshold must be non-negative")
    if args.motion_timeout <= 0.0:
        parser.error("--motion-timeout must be positive")
    if args.startup_warmup_strides < 0:
        parser.error("--startup-warmup-strides must be non-negative")
    if args.startup_seed_min_window < 0.0:
        parser.error("--startup-seed-min-window must be non-negative")
    if args.startup_seed_max_window < args.startup_seed_min_window:
        parser.error("--startup-seed-max-window must be >= --startup-seed-min-window")
    if args.startup_seed_gyro_threshold < 0.0:
        parser.error("--startup-seed-gyro-threshold must be non-negative")
    if args.startup_seed_angle_threshold < 0.0:
        parser.error("--startup-seed-angle-threshold must be non-negative")
    if args.startup_seed_ramp < 0.0:
        parser.error("--startup-seed-ramp must be non-negative")
    if args.stop_window <= 0.0:
        parser.error("--stop-window must be positive")
    if args.stop_hold < 0.0:
        parser.error("--stop-hold must be non-negative")
    if args.stop_gyro_rms_threshold < 0.0:
        parser.error("--stop-gyro-rms-threshold must be non-negative")
    if args.stop_angle_range_threshold < 0.0:
        parser.error("--stop-angle-range-threshold must be non-negative")
    if args.stop_phase_rate_limit < 0.0:
        parser.error("--stop-phase-rate-limit must be non-negative")
    if args.stop_unstable_hold < 0.0:
        parser.error("--stop-unstable-hold must be non-negative")
    if args.plot_window <= 0.0:
        parser.error("--plot-window must be positive")
    if args.plot_rate <= 0.0:
        parser.error("--plot-rate must be positive")
    if args.samples == 0 or args.samples < -1:
        parser.error("--samples must be positive, or -1 for continuous")
    if args.print_every < 0:
        parser.error("--print-every must be non-negative")


def make_phase_config(args: argparse.Namespace, *, phase_offset: float = 0.0) -> ImuPhaseConfig:
    return ImuPhaseConfig(
        angle_sign=float(args.angle_sign),
        gyro_sign=float(args.gyro_sign),
        gyro_unit="deg",
        angle_source=str(args.angle_source),
        acc_angle_numerator_axis=str(args.acc_angle_num_axis),
        acc_angle_denominator_axis=str(args.acc_angle_den_axis),
        acc_angle_numerator_sign=float(args.acc_angle_num_sign),
        acc_angle_denominator_sign=float(args.acc_angle_den_sign),
        gyro_axis=str(args.gyro_axis),
        quaternion_thigh_axis=str(args.quat_thigh_axis),
        quaternion_sagittal_forward_axis=str(args.quat_forward_axis),
        quaternion_sagittal_vertical_axis=str(args.quat_vertical_axis),
        min_swing_range_deg=float(args.swing_threshold),
        motion_timeout_sec=float(args.motion_timeout),
        phase_offset=float(phase_offset),
        startup_warmup_strides=int(args.startup_warmup_strides),
        startup_seed_enabled=not bool(args.disable_startup_seed),
        startup_seed_min_window_sec=float(args.startup_seed_min_window),
        startup_seed_max_window_sec=float(args.startup_seed_max_window),
        startup_seed_gyro_threshold_deg_s=float(args.startup_seed_gyro_threshold),
        startup_seed_angle_threshold_deg=float(args.startup_seed_angle_threshold),
        startup_seed_positive_gyro_phase_rad=float(args.startup_seed_positive_phase),
        startup_seed_negative_gyro_phase_rad=float(args.startup_seed_negative_phase),
        startup_seed_ramp_sec=float(args.startup_seed_ramp),
        stop_detection_window_sec=float(args.stop_window),
        stop_detection_hold_sec=float(args.stop_hold),
        stop_gyro_rms_threshold_deg_s=float(args.stop_gyro_rms_threshold),
        stop_angle_range_threshold_deg=float(args.stop_angle_range_threshold),
        stop_phase_rate_limit_hz=float(args.stop_phase_rate_limit),
        stop_phase_unstable_hold_sec=float(args.stop_unstable_hold),
    )


def output_header() -> list[str]:
    return [
        "local_time",
        "sample_time",
        "relative_time",
        "seq",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "quat_w",
        "quat_x",
        "quat_y",
        "quat_z",
        "angle_raw_deg",
        "angle_drift_removed_deg",
        "angle_processed_deg",
        "gyro_raw_deg_s",
        "gyro_processed_deg_s",
        "phase_rad",
        "phase_percent",
        "zero_event",
        "zero_event_count",
        "motion_active",
        "phase_valid",
        "temporary_phase",
        "walking_mode",
        "stopped_event",
        "phase_limited",
        "startup_ramp_scale",
        "cycle_frequency_hz",
        "recent_swing_range_deg",
        "event_angle_processed_deg",
        "event_gyro_processed_deg_s",
        "event_phase_rad",
        "event_phase_percent",
        "event_zero_event",
        "event_zero_event_count",
        "event_motion_active",
        "event_phase_valid",
        "event_cycle_frequency_hz",
        "event_recent_swing_range_deg",
    ]


def output_row(
    sample: tuple[float, ...],
    sample_time: float,
    relative_time: float,
    seq: int,
    output,
    event_output,
) -> list[object]:
    fields = sample_fields(sample)
    temporary_phase = bool(getattr(output, "temporary_phase_active", False))
    phase_valid = bool((output.zero_event_count > 0 or temporary_phase) and not output.sample_rejected)
    event_phase_valid = bool(event_output.zero_event_count > 0 and not event_output.sample_rejected)
    return [
        local_time_text(),
        f"{sample_time:.6f}",
        f"{relative_time:.6f}",
        int(seq),
        f"{fields['acc_x']:.9f}",
        f"{fields['acc_y']:.9f}",
        f"{fields['acc_z']:.9f}",
        f"{fields['gyro_x']:.9f}",
        f"{fields['gyro_y']:.9f}",
        f"{fields['gyro_z']:.9f}",
        f"{fields['quat_w']:.9f}",
        f"{fields['quat_x']:.9f}",
        f"{fields['quat_y']:.9f}",
        f"{fields['quat_z']:.9f}",
        f"{output.angle_raw_deg:.6f}",
        f"{getattr(output, 'angle_drift_removed_deg', output.angle_deg):.6f}",
        f"{output.angle_deg:.6f}",
        f"{output.angular_velocity_raw_deg_s:.6f}",
        f"{output.angular_velocity_deg_s:.6f}",
        f"{output.phase_rad:.9f}",
        f"{output.phase_0_to_1 * 100.0:.6f}",
        int(bool(output.zero_event)),
        int(output.zero_event_count),
        int(bool(output.motion_active)),
        int(phase_valid),
        int(temporary_phase),
        int(bool(getattr(output, "walking_mode", True))),
        int(bool(getattr(output, "stopped_event", False))),
        int(bool(getattr(output, "phase_limited", False))),
        f"{float(getattr(output, 'startup_ramp_scale', 1.0)):.6f}",
        f"{output.previous_cycle_frequency_hz:.6f}",
        f"{output.recent_swing_range_deg:.6f}",
        f"{event_output.angle_deg:.6f}",
        f"{event_output.angular_velocity_deg_s:.6f}",
        f"{event_output.phase_rad:.9f}",
        f"{event_output.phase_0_to_1 * 100.0:.6f}",
        int(bool(event_output.zero_event)),
        int(event_output.zero_event_count),
        int(bool(event_output.motion_active)),
        int(event_phase_valid),
        f"{event_output.previous_cycle_frequency_hz:.6f}",
        f"{event_output.recent_swing_range_deg:.6f}",
    ]


class LivePlot:
    def __init__(self, window_sec: float, refresh_hz: float):
        self.window_sec = float(window_sec)
        self.refresh_period = 1.0 / max(float(refresh_hz), 1e-6)
        self.last_draw = 0.0
        self.history: dict[str, deque[float]] = {
            "t": deque(),
            "angle_raw": deque(),
            "angle_drift_removed": deque(),
            "angle_processed": deque(),
            "gyro_raw": deque(),
            "gyro_processed": deque(),
            "phase": deque(),
            "event_phase": deque(),
            "zero_t": deque(),
            "zero_phase": deque(),
            "event_zero_t": deque(),
            "event_zero_phase": deque(),
        }

        mpl_config_dir = Path(tempfile.gettempdir()) / "matplotlib"
        mpl_config_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(mpl_config_dir))
        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except Exception as exc:
            raise RuntimeError("matplotlib is required for live plotting") from exc

        backend = matplotlib.get_backend().lower()
        noninteractive = {"agg", "pdf", "ps", "svg", "template", "cairo"}
        if backend in noninteractive or backend.startswith("module://matplotlib_inline"):
            raise RuntimeError(
                "An interactive Matplotlib backend is required. Try MPLBACKEND=TkAgg or QtAgg."
            )

        self.plt = plt
        self.plt.ion()
        self.fig, self.axes = self.plt.subplots(
            3,
            1,
            sharex=True,
            figsize=(11, 8),
            constrained_layout=True,
        )
        try:
            self.fig.canvas.manager.set_window_title("MI1 left hip-style phase live")
        except Exception:
            pass
        self.fig.suptitle("MI1 left IMU hip-style phase")

        ax_angle, ax_gyro, ax_phase = self.axes
        (self.angle_raw_line,) = ax_angle.plot([], [], color="0.65", lw=1.0, label="angle raw")
        (self.angle_drift_line,) = ax_angle.plot(
            [],
            [],
            color="#17becf",
            lw=1.1,
            ls="--",
            label="drift-removed angle",
        )
        (self.angle_proc_line,) = ax_angle.plot([], [], color="#1f77b4", lw=1.5, label="SOGI angle")
        (self.gyro_raw_line,) = ax_gyro.plot([], [], color="0.65", lw=1.0, label="gyro raw")
        (self.gyro_proc_line,) = ax_gyro.plot([], [], color="#ff7f0e", lw=1.5, label="SOGI quadrature")
        (self.phase_line,) = ax_phase.plot([], [], color="#2ca02c", lw=1.6, label="hip-style phase")
        (self.event_phase_line,) = ax_phase.plot(
            [],
            [],
            color="#9467bd",
            lw=1.2,
            ls="--",
            label="event reference",
        )
        self.zero_points = ax_phase.scatter([], [], color="#d62728", s=24, label="hip zero", zorder=4)
        self.event_zero_points = ax_phase.scatter(
            [],
            [],
            color="#9467bd",
            s=18,
            marker="x",
            label="event zero",
            zorder=4,
        )
        self.status_text = ax_angle.text(
            0.01,
            0.98,
            "",
            transform=ax_angle.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.8"},
        )

        ax_angle.set_ylabel("Angle (deg)")
        ax_gyro.set_ylabel("Gyro (deg/s)")
        ax_phase.set_ylabel("Phase (%)")
        ax_phase.set_xlabel("Time (s)")
        ax_phase.set_ylim(-5.0, 105.0)
        for ax in self.axes:
            ax.grid(True, alpha=0.25)
            ax.legend(loc="upper right")
        self.plt.show(block=False)

    def add(self, t: float, output, event_output) -> None:
        self.history["t"].append(float(t))
        self.history["angle_raw"].append(float(output.angle_raw_deg))
        self.history["angle_drift_removed"].append(
            float(getattr(output, "angle_drift_removed_deg", output.angle_deg))
        )
        self.history["angle_processed"].append(float(output.angle_deg))
        self.history["gyro_raw"].append(float(output.angular_velocity_raw_deg_s))
        self.history["gyro_processed"].append(float(output.angular_velocity_deg_s))
        self.history["phase"].append(float(output.phase_0_to_1) * 100.0)
        self.history["event_phase"].append(float(event_output.phase_0_to_1) * 100.0)
        if bool(output.zero_event):
            self.history["zero_t"].append(float(t))
            self.history["zero_phase"].append(float(output.phase_0_to_1) * 100.0)
        if bool(event_output.zero_event):
            self.history["event_zero_t"].append(float(t))
            self.history["event_zero_phase"].append(float(event_output.phase_0_to_1) * 100.0)
        self._trim(float(t))

    def _trim(self, latest_t: float) -> None:
        cutoff = latest_t - self.window_sec
        while self.history["t"] and self.history["t"][0] < cutoff:
            for key in (
                "t",
                "angle_raw",
                "angle_drift_removed",
                "angle_processed",
                "gyro_raw",
                "gyro_processed",
                "phase",
                "event_phase",
            ):
                self.history[key].popleft()
        while self.history["zero_t"] and self.history["zero_t"][0] < cutoff:
            self.history["zero_t"].popleft()
            self.history["zero_phase"].popleft()
        while self.history["event_zero_t"] and self.history["event_zero_t"][0] < cutoff:
            self.history["event_zero_t"].popleft()
            self.history["event_zero_phase"].popleft()

    def update(self, output, event_output, source: WiredMi1CanImuPhaseSource) -> None:
        now = time.monotonic()
        if now - self.last_draw < self.refresh_period:
            return
        self.last_draw = now
        t = list(self.history["t"])
        if not t:
            self.plt.pause(0.001)
            return

        self.angle_raw_line.set_data(t, list(self.history["angle_raw"]))
        self.angle_drift_line.set_data(t, list(self.history["angle_drift_removed"]))
        self.angle_proc_line.set_data(t, list(self.history["angle_processed"]))
        self.gyro_raw_line.set_data(t, list(self.history["gyro_raw"]))
        self.gyro_proc_line.set_data(t, list(self.history["gyro_processed"]))
        self.phase_line.set_data(t, list(self.history["phase"]))
        self.event_phase_line.set_data(t, list(self.history["event_phase"]))
        zero_offsets = (
            list(zip(self.history["zero_t"], self.history["zero_phase"]))
            if self.history["zero_t"]
            else [(float("nan"), float("nan"))]
        )
        self.zero_points.set_offsets(zero_offsets)
        event_zero_offsets = (
            list(zip(self.history["event_zero_t"], self.history["event_zero_phase"]))
            if self.history["event_zero_t"]
            else [(float("nan"), float("nan"))]
        )
        self.event_zero_points.set_offsets(event_zero_offsets)

        xmin = max(0.0, t[-1] - self.window_sec)
        xmax = max(self.window_sec, t[-1] + 0.05)
        for ax in self.axes:
            ax.set_xlim(xmin, xmax)
            if ax is not self.axes[2]:
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
        temporary_phase = bool(getattr(output, "temporary_phase_active", False))
        valid = bool((output.zero_event_count > 0 or temporary_phase) and not output.sample_rejected)
        event_valid = bool(event_output.zero_event_count > 0 and not event_output.sample_rejected)
        status = (
            f"hip={output.phase_0_to_1 * 100.0:5.1f}% v={int(valid)} e={output.zero_event_count} "
            f"f={output.previous_cycle_frequency_hz:.2f}Hz  "
            f"tmp={int(temporary_phase)} walk={int(bool(getattr(output, 'walking_mode', True)))} "
            f"stop={int(bool(getattr(output, 'stopped_event', False)))} "
            f"lim={int(bool(getattr(output, 'phase_limited', False)))} "
            f"ramp={float(getattr(output, 'startup_ramp_scale', 1.0)):.2f}  "
            f"event={event_output.phase_0_to_1 * 100.0:5.1f}% v={int(event_valid)} "
            f"e={event_output.zero_event_count} f={event_output.previous_cycle_frequency_hz:.2f}Hz  "
            f"motion={int(output.motion_active)}  "
            f"fields={source.available_fields_text()}  "
            f"err={source.last_error or '-'}"
        )
        self.status_text.set_text(status)
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def close(self) -> None:
        try:
            self.plt.close(self.fig)
        except Exception:
            pass


def open_csv(path: Optional[Path]):
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    fp = path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(fp)
    writer.writerow(output_header())
    fp.flush()
    return fp, writer


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    validate_args(args, parser)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    require_quat = bool(args.require_quaternion or str(args.angle_source).startswith("quat"))
    source = WiredMi1CanImuPhaseSource(
        side=args.side,
        interface=args.interface,
        protocol=args.protocol,
        node_id=int(args.node_id),
        data_timeout_sec=float(args.data_timeout),
        max_field_age_sec=args.max_field_age,
        require_quaternion=require_quat,
    )
    source.set_measurement_enabled(True)
    if not source.start():
        print(f"error: failed to start MI1 source: {source.last_error}", file=sys.stderr)
        return 2

    event_config = make_phase_config(args, phase_offset=0.0)
    hip_config = make_phase_config(args, phase_offset=float(args.hip_phase_offset))
    estimator = HipStyleImuPhaseEstimator(hip_config, sample_rate_hz=float(args.rate))
    event_estimator = ThighImuPhaseEstimator(event_config)
    plot = None
    if not args.no_plot:
        try:
            plot = LivePlot(args.plot_window, args.plot_rate)
        except Exception as exc:
            source.stop()
            print(f"error: {exc}", file=sys.stderr)
            return 3

    csv_fp, csv_writer = open_csv(args.csv)
    header = output_header()
    if args.print_every:
        print(",".join(header))

    print(
        "MI1 left hip-style phase live test: "
        f"interface={args.interface}, node={format_node_id(args.node_id)}, "
        f"protocol={args.protocol}, rate={args.rate:.1f}Hz, "
        f"angle_source={args.angle_source}"
        f"{' (heading-free)' if args.angle_source == 'quat_sagittal' else ''}, "
        f"acc=atan2({signed_axis_text(args.acc_angle_num_sign, args.acc_angle_num_axis)},"
        f"{signed_axis_text(args.acc_angle_den_sign, args.acc_angle_den_axis)}), "
        f"quat_axes=thigh:{args.quat_thigh_axis},forward:{args.quat_forward_axis},"
        f"vertical:{args.quat_vertical_axis}, "
        f"gyro_axis={args.gyro_axis}, signs=({args.angle_sign:g},{args.gyro_sign:g}), "
        f"hip_phase_offset={args.hip_phase_offset:g}, "
        f"startup_warmup_strides={args.startup_warmup_strides}, "
        f"startup_seed={'off' if args.disable_startup_seed else 'on'} "
        f"win={args.startup_seed_min_window:.2f}-{args.startup_seed_max_window:.2f}s "
        f"gyro_thr={args.startup_seed_gyro_threshold:.1f}deg/s "
        f"ramp={args.startup_seed_ramp:.2f}s, "
        f"stop_window={args.stop_window:.2f}s stop_hold={args.stop_hold:.2f}s "
        f"stop_gyro_rms={args.stop_gyro_rms_threshold:.1f}deg/s "
        f"stop_angle_range={args.stop_angle_range_threshold:.1f}deg "
        f"phase_rate_limit={args.stop_phase_rate_limit:.2f}Hz "
        f"unstable_hold={args.stop_unstable_hold:.2f}s",
        file=sys.stderr,
    )

    first_sample_time: Optional[float] = None
    last_seq = 0
    processed_count = 0
    last_status_time = 0.0
    last_output = None

    try:
        while not STOP:
            latest = source.get_latest_sample_6d()
            if latest is None:
                now = time.monotonic()
                if now - last_status_time > 1.0:
                    last_status_time = now
                    print(
                        f"waiting for MI1 sample: fields={source.available_fields_text()} "
                        f"err={source.last_error or '-'}",
                        file=sys.stderr,
                    )
                time.sleep(0.01)
                continue

            sample, sample_time, seq = latest
            if int(seq) == int(last_seq):
                if plot is not None and last_output is not None:
                    plot.update(last_output[0], last_output[1], source)
                time.sleep(0.002)
                continue

            last_seq = int(seq)
            if first_sample_time is None:
                first_sample_time = float(sample_time)
            relative_time = float(sample_time) - float(first_sample_time)
            output = estimator.process_6d(sample, float(sample_time))
            event_output = event_estimator.process_6d(sample, float(sample_time))
            last_output = (output, event_output)
            processed_count += 1

            row = output_row(
                sample,
                float(sample_time),
                relative_time,
                int(seq),
                output,
                event_output,
            )
            if csv_writer is not None:
                csv_writer.writerow(row)
                if processed_count % 25 == 0:
                    csv_fp.flush()
            if args.print_every and processed_count % int(args.print_every) == 0:
                print(",".join(str(value) for value in row))

            if plot is not None:
                plot.add(relative_time, output, event_output)
                plot.update(output, event_output, source)

            if args.samples > 0 and processed_count >= args.samples:
                break

    finally:
        if csv_fp is not None:
            csv_fp.flush()
            csv_fp.close()
        if plot is not None:
            plot.close()
        source.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
