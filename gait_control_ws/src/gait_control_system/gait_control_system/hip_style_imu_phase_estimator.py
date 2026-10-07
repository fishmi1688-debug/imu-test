#!/usr/bin/env python3

"""Hip-controller style IMU-to-phase estimator for the left-IMU phase mode.

The estimator keeps the existing raw IMU axis/sign extraction from
``ThighImuPhaseEstimator`` and replaces only the phase-generation path with the
same structure used by ``hip-controller-main``:

raw SensorSignal -> pause gate -> drift removal -> SOGI-FLL -> gait controller
-> atan2 phase plane.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .imu_phase_estimator import (
    EPSILON,
    ImuPhaseConfig,
    ImuPhaseOutput,
    ThighImuPhaseEstimator,
    angular_delta_degrees,
    clamp,
)


PAUSE_DETECT_ENVELOPE_ALPHA = 0.10
PAUSE_ENTER_THRESHOLD_RAD_S = 0.2
PAUSE_EXIT_THRESHOLD_RAD_S = 0.5
MAX_PLAUSIBLE_TIME_DIFFERENCE_S = 1.0
NOMINAL_TIME_DIFFERENCE_S = 0.01
STRIDE_EVENT_COUNTER_TIME = 0.3099
STRIDE_EVENT_HIT_CROSSING_OFFSET = -0.1
STATE_CHANGE_TMIN_SEC = 0.0
STATE_CHANGE_TMAX_SEC = 0.6
POST_RESUME_FLL_COOLDOWN_TICKS = 80
POST_MODE_SWITCH_FLL_COOLDOWN_TICKS = 50
DEFAULT_STARTUP_WARMUP_STRIDES = 0
STARTUP_SEED_BASELINE_ALPHA = 0.08


def _hit_crossing_falling(curr: float, prev: float, offset: float = 0.0) -> bool:
    return prev >= offset > curr


def _hit_crossing_rising(curr: float, prev: float, offset: float = 0.0) -> bool:
    return prev <= offset and curr > offset


def _wrap_to_2pi(value: float) -> float:
    try:
        phase = float(value)
    except Exception:
        return 0.0
    if not math.isfinite(phase):
        return 0.0
    return phase % (2.0 * math.pi)


@dataclass
class _SensorSignal:
    timestamp: Optional[float]
    angle_rad: float = 0.0
    velocity_rad_per_sec: float = 0.0


@dataclass
class _SogiFllConfig:
    lower_cadence_bound: float = 0.3
    upper_cadence_bound: float = 1.8
    sogi_adaptation_gain: float = 1.0
    fll_adaptation_gain: float = 5.0
    lower_energy_threshold: float = 1e-4
    upper_energy_threshold: float = 1e-1
    frequency_estimate_smoother_bandwidth: float = 0.8
    lock_state_smoother_bandwidth: float = 1.50
    initial_frequency_guess: float = 0.7
    decay_not_walking: float = 0.999
    numerical_safety_floor: float = 1e-9


class _SecondOrderLowpass:
    def __init__(self, cutoff_rad_s: float, damping_ratio: float = 1.0):
        self.cutoff_rad_s = float(cutoff_rad_s)
        self.damping_ratio = float(damping_ratio)
        self.feedback_state = 0.0
        self.filtered_output = 0.0

    def reset(self) -> None:
        self.feedback_state = 0.0
        self.filtered_output = 0.0

    def _derivatives(self, feedback_state: float, filtered_output: float, x: float):
        error = float(x) - filtered_output
        corrected_error = error - 2.0 * self.damping_ratio * feedback_state
        return (
            self.cutoff_rad_s * corrected_error,
            self.cutoff_rad_s * feedback_state,
        )

    def step(self, x: float, dt: float) -> float:
        dt = max(0.0, float(dt))
        if dt <= EPSILON or self.cutoff_rad_s <= 0.0:
            return self.filtered_output

        q0 = self.feedback_state
        y0 = self.filtered_output
        k1q, k1y = self._derivatives(q0, y0, x)
        k2q, k2y = self._derivatives(q0 + 0.5 * dt * k1q, y0 + 0.5 * dt * k1y, x)
        k3q, k3y = self._derivatives(q0 + 0.5 * dt * k2q, y0 + 0.5 * dt * k2y, x)
        k4q, k4y = self._derivatives(q0 + dt * k3q, y0 + dt * k3y, x)
        self.feedback_state = q0 + (dt / 6.0) * (k1q + 2.0 * k2q + 2.0 * k3q + k4q)
        self.filtered_output = y0 + (dt / 6.0) * (k1y + 2.0 * k2y + 2.0 * k3y + k4y)
        return self.filtered_output


class _SogiFllFilter:
    def __init__(self, config: Optional[_SogiFllConfig] = None):
        self._config = config or _SogiFllConfig()
        self.reset()

    def reset(self) -> None:
        self._walking = True
        self._inphase = 0.0
        self._quadrature = 0.0
        self._omega_est = 2.0 * math.pi * self._config.initial_frequency_guess
        self._frequency_estimate = self._config.initial_frequency_guess
        self._confidence_state = 0.0
        self._fll_cooldown_ticks = 0

    def stop_walking(self) -> None:
        self._walking = False

    def start_walking(self) -> None:
        self._walking = True
        self._confidence_state = 0.0
        self._fll_cooldown_ticks = POST_RESUME_FLL_COOLDOWN_TICKS

    def set_config(self, config: _SogiFllConfig) -> None:
        self._config = config
        self._fll_cooldown_ticks = max(
            self._fll_cooldown_ticks,
            POST_MODE_SWITCH_FLL_COOLDOWN_TICKS,
        )

    def clear_state_keep_frequency(self) -> None:
        self._inphase = 0.0
        self._quadrature = 0.0
        self._confidence_state = 0.0

    @property
    def estimated_frequency_hz(self) -> float:
        return float(self._frequency_estimate)

    @property
    def estimated_omega_rad_s(self) -> float:
        return float(2.0 * math.pi * self._frequency_estimate)

    def seed_state(self, inphase: float, quadrature: float) -> None:
        if math.isfinite(inphase):
            self._inphase = float(inphase)
        if math.isfinite(quadrature):
            self._quadrature = float(quadrature)
        self._confidence_state = 0.0

    def filter(self, raw_theta_rad: float, dt: float) -> tuple[float, float]:
        dt = max(0.0, float(dt))
        w_min = 2.0 * math.pi * self._config.lower_cadence_bound
        w_max = 2.0 * math.pi * self._config.upper_cadence_bound
        omega_clipped = clamp(self._omega_est, w_min, w_max)

        if dt <= EPSILON:
            return self._inphase, self._quadrature

        alpha_frequency = math.exp(
            -2.0 * math.pi * self._config.frequency_estimate_smoother_bandwidth * dt
        )
        alpha_confidence = math.exp(
            -2.0 * math.pi * self._config.lock_state_smoother_bandwidth * dt
        )

        if self._walking:
            phase_error = float(raw_theta_rad) - self._inphase
            self._inphase += dt * (
                omega_clipped * self._quadrature
                + self._config.sogi_adaptation_gain * omega_clipped * phase_error
            )
            self._quadrature += dt * (-omega_clipped * self._inphase)
        else:
            self._inphase *= self._config.decay_not_walking
            self._quadrature *= self._config.decay_not_walking
            phase_error = 0.0

        energy = self._inphase * self._inphase + self._quadrature * self._quadrature
        self._confidence_state = (
            alpha_confidence * self._confidence_state
            + (1.0 - alpha_confidence) * energy
        )
        confidence = clamp(
            (self._confidence_state - self._config.lower_energy_threshold)
            / (
                self._config.upper_energy_threshold
                - self._config.lower_energy_threshold
                + self._config.numerical_safety_floor
            ),
            0.0,
            1.0,
        )

        if self._walking and self._fll_cooldown_ticks == 0:
            adaptation_error = (phase_error * self._quadrature) / (
                energy + self._config.numerical_safety_floor
            )
            self._omega_est = (
                omega_clipped
                + dt
                * self._config.fll_adaptation_gain
                * omega_clipped
                * adaptation_error
                * confidence
            )
            self._omega_est = clamp(self._omega_est, w_min, w_max)
            raw_frequency = self._omega_est / (2.0 * math.pi)
            self._frequency_estimate = (
                alpha_frequency * self._frequency_estimate
                + (1.0 - alpha_frequency) * raw_frequency
            )
            self._omega_est = 2.0 * math.pi * clamp(
                self._frequency_estimate,
                self._config.lower_cadence_bound,
                self._config.upper_cadence_bound,
            )
        elif self._walking:
            self._fll_cooldown_ticks -= 1

        return self._inphase, self._quadrature


class _HipStylePreprocessor:
    def __init__(self):
        self._prev_timestamp: Optional[float] = None
        self._drift_lpf = _SecondOrderLowpass(1.25, damping_ratio=1.0)
        self._velocity_drift_lpf = _SecondOrderLowpass(2.0 * math.pi * 0.1, damping_ratio=1.0)
        self._sogi = _SogiFllFilter(_SogiFllConfig())
        self.last_drift_removed_angle_rad: Optional[float] = None
        self.last_velocity_surrogate_rad_per_sec: Optional[float] = None
        self._seed_sogi_on_next_filter = False

    def reset(self) -> None:
        self._prev_timestamp = None
        self._drift_lpf.reset()
        self._velocity_drift_lpf.reset()
        self._sogi.reset()
        self.last_drift_removed_angle_rad = None
        self.last_velocity_surrogate_rad_per_sec = None
        self._seed_sogi_on_next_filter = False

    @property
    def estimated_frequency_hz(self) -> float:
        return self._sogi.estimated_frequency_hz

    def set_walking_mode(self, walking: bool) -> None:
        if walking:
            self._sogi.start_walking()
            self._seed_sogi_on_next_filter = True
        else:
            self._sogi.stop_walking()
            self._seed_sogi_on_next_filter = False

    def filter(self, raw_signal: _SensorSignal) -> _SensorSignal:
        if self._prev_timestamp is None or raw_signal.timestamp is None:
            self._prev_timestamp = raw_signal.timestamp
            return _SensorSignal(
                timestamp=raw_signal.timestamp,
                angle_rad=raw_signal.angle_rad,
                velocity_rad_per_sec=raw_signal.velocity_rad_per_sec,
            )

        dt = float(raw_signal.timestamp) - float(self._prev_timestamp)
        if dt <= 0.0:
            dt = NOMINAL_TIME_DIFFERENCE_S
        elif dt > MAX_PLAUSIBLE_TIME_DIFFERENCE_S:
            self._drift_lpf.reset()
            self._velocity_drift_lpf.reset()
            dt = NOMINAL_TIME_DIFFERENCE_S

        self._prev_timestamp = raw_signal.timestamp
        drift_estimate = self._drift_lpf.step(raw_signal.angle_rad, dt)
        angle_no_drift = float(raw_signal.angle_rad) - drift_estimate
        self.last_drift_removed_angle_rad = angle_no_drift
        if self._seed_sogi_on_next_filter:
            omega_rad_s = max(abs(self._sogi.estimated_omega_rad_s), EPSILON)
            self._sogi.seed_state(
                angle_no_drift,
                float(raw_signal.velocity_rad_per_sec) / omega_rad_s,
            )
            self._seed_sogi_on_next_filter = False
        angle_out, quadrature = self._sogi.filter(angle_no_drift, dt)
        self.last_velocity_surrogate_rad_per_sec = quadrature
        velocity_drift = self._velocity_drift_lpf.step(quadrature, dt)
        velocity_out = quadrature - velocity_drift
        return _SensorSignal(
            timestamp=raw_signal.timestamp,
            angle_rad=angle_out,
            velocity_rad_per_sec=velocity_out,
        )


class _MotionState:
    INITIAL = "initial"
    VELOCITY_MAX = "velocity_max"
    ANGLE_MAX = "angle_max"
    VELOCITY_MIN = "velocity_min"
    ANGLE_MIN = "angle_min"


class _MotionStateMachine:
    def __init__(self):
        self.state = _MotionState.INITIAL
        self.last_time_state_change_sec: Optional[float] = None

    def reset(self) -> None:
        self.state = _MotionState.INITIAL
        self.last_time_state_change_sec = None

    def update_motion_state(self, prev: _SensorSignal, curr: _SensorSignal) -> Optional[str]:
        if self._is_timeout(curr.timestamp):
            return None

        vel_max = (
            _hit_crossing_rising(curr.angle_rad, prev.angle_rad)
            and curr.velocity_rad_per_sec > 0.0
        )
        angle_max = (
            _hit_crossing_falling(curr.velocity_rad_per_sec, prev.velocity_rad_per_sec)
            and curr.angle_rad > 0.0
        )
        vel_min = (
            _hit_crossing_falling(curr.angle_rad, prev.angle_rad)
            and curr.velocity_rad_per_sec < 0.0
        )
        angle_min = (
            _hit_crossing_rising(curr.velocity_rad_per_sec, prev.velocity_rad_per_sec)
            and curr.angle_rad < 0.0
        )

        new_state = None
        if self.state == _MotionState.INITIAL:
            if vel_max:
                new_state = _MotionState.VELOCITY_MAX
            elif angle_max:
                new_state = _MotionState.ANGLE_MAX
            elif vel_min:
                new_state = _MotionState.VELOCITY_MIN
            elif angle_min:
                new_state = _MotionState.ANGLE_MIN
        elif self.state == _MotionState.ANGLE_MAX and vel_min:
            new_state = _MotionState.VELOCITY_MIN
        elif self.state == _MotionState.ANGLE_MIN and vel_max:
            new_state = _MotionState.VELOCITY_MAX
        elif self.state == _MotionState.VELOCITY_MAX and angle_max:
            new_state = _MotionState.ANGLE_MAX
        elif self.state == _MotionState.VELOCITY_MIN and angle_min:
            new_state = _MotionState.ANGLE_MIN

        if new_state is not None:
            self.state = new_state
            self.last_time_state_change_sec = curr.timestamp
        return new_state

    def _is_timeout(self, timestamp: Optional[float]) -> bool:
        if self.state == _MotionState.INITIAL:
            return False
        if self.last_time_state_change_sec is None or timestamp is None:
            return False
        dt = float(timestamp) - float(self.last_time_state_change_sec)
        if dt < STATE_CHANGE_TMIN_SEC:
            return True
        if dt >= STATE_CHANGE_TMAX_SEC:
            self.reset()
            return True
        return False


class _SteadyStateTracker:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self._angle_max = 0.0
        self._angle_min = 0.0
        self._velocity_max = 0.0
        self._velocity_min = 0.0
        self._center_vel = 0.0
        self._center_ang = 0.0
        self._scale_factor = 1.0
        self.vel_steady_state = 0.0
        self.ang_steady_state = 0.0

    def update_extrema(self, state: Optional[str], curr_signal: _SensorSignal) -> None:
        if state == _MotionState.ANGLE_MAX:
            self._angle_max = curr_signal.angle_rad
        elif state == _MotionState.ANGLE_MIN:
            self._angle_min = curr_signal.angle_rad
        elif state == _MotionState.VELOCITY_MAX:
            self._velocity_max = curr_signal.velocity_rad_per_sec
        elif state == _MotionState.VELOCITY_MIN:
            self._velocity_min = curr_signal.velocity_rad_per_sec

    def recenter(self) -> None:
        self._center_ang = (self._angle_max + self._angle_min) / 2.0
        self._center_vel = (self._velocity_max + self._velocity_min) / 2.0
        angle_range = abs(self._angle_max - self._angle_min)
        if angle_range <= 0.0:
            self._scale_factor = 1.0
        else:
            self._scale_factor = abs(self._velocity_max - self._velocity_min) / angle_range

    def update_steady_state(self, curr_signal: _SensorSignal) -> None:
        self.vel_steady_state = curr_signal.velocity_rad_per_sec - self._center_vel
        self.ang_steady_state = -(
            (curr_signal.angle_rad - self._center_ang) * self._scale_factor
        )


class _StrideEventDetector:
    def __init__(self):
        self.valid_stride = False
        self._enable_detector = False
        self._detector_count = True
        self._detector_counter_time = 0.01
        self._enable_trigger = True
        self._trigger_counter_time = 0.0

    def reset(self) -> None:
        self.valid_stride = False
        self._enable_detector = False
        self._detector_count = True
        self._detector_counter_time = 0.01
        self._enable_trigger = True
        self._trigger_counter_time = 0.0

    def is_valid_stride_event(
        self,
        time_difference: float,
        valid_ang_max: bool,
        prev_vel: float,
        curr_vel: float,
    ) -> bool:
        self._detect_new_stride(time_difference, prev_vel, curr_vel)
        self._detect_new_gait(time_difference, valid_ang_max)
        return self.valid_stride and self._enable_trigger

    def _detect_new_stride(self, dt: float, prev_vel: float, curr_vel: float) -> None:
        if self._detector_count and self._countdown_detected(dt):
            self._reset_detector_window()
        elif self._enable_detector and _hit_crossing_falling(
            curr_vel,
            prev_vel,
            STRIDE_EVENT_HIT_CROSSING_OFFSET,
        ):
            self.valid_stride = True
            self._detector_count = True

    def _detect_new_gait(self, dt: float, valid_ang_max: bool) -> None:
        if valid_ang_max:
            self._enable_trigger = True
        elif self._enable_trigger and self._countdown_triggered(dt):
            self._enable_trigger = False
            self._trigger_counter_time = 0.0

    def _countdown_triggered(self, dt: float) -> bool:
        self._trigger_counter_time += float(dt)
        self._enable_trigger = True
        return self._trigger_counter_time >= STRIDE_EVENT_COUNTER_TIME

    def _countdown_detected(self, dt: float) -> bool:
        self._detector_counter_time += float(dt)
        self._enable_detector = False
        return self._detector_counter_time >= STRIDE_EVENT_COUNTER_TIME

    def _reset_detector_window(self) -> None:
        self._detector_counter_time = 0.0
        self._detector_count = False
        self.valid_stride = False
        self._enable_detector = True


class _HipStyleGaitController:
    def __init__(self):
        self.prev_signal = _SensorSignal(timestamp=None)
        self.curr_signal = _SensorSignal(timestamp=None)
        self.state_machine = _MotionStateMachine()
        self.steady_state_tracker = _SteadyStateTracker()
        self.stride_event_detector = _StrideEventDetector()
        self.controller_initialized = False
        self.last_detection = False
        self.last_stride_event = False

    def reset(self) -> None:
        self.prev_signal = _SensorSignal(timestamp=None)
        self.curr_signal = _SensorSignal(timestamp=None)
        self.state_machine.reset()
        self.steady_state_tracker.reset()
        self.stride_event_detector.reset()
        self.controller_initialized = False
        self.last_detection = False
        self.last_stride_event = False

    def update_and_compute(self, curr_signal: _SensorSignal) -> float:
        self.last_stride_event = False
        self.prev_signal = self.curr_signal
        self.curr_signal = curr_signal
        if self.prev_signal.timestamp is None or self.curr_signal.timestamp is None:
            return 0.0

        state = self.state_machine.update_motion_state(self.prev_signal, self.curr_signal)
        if state is not None:
            self.steady_state_tracker.update_extrema(state, self.curr_signal)

        dt = float(self.curr_signal.timestamp) - float(self.prev_signal.timestamp)
        if dt <= 0.0:
            dt = NOMINAL_TIME_DIFFERENCE_S
        detected = self.stride_event_detector.is_valid_stride_event(
            time_difference=dt,
            valid_ang_max=(state == _MotionState.ANGLE_MAX),
            prev_vel=self.prev_signal.velocity_rad_per_sec,
            curr_vel=self.curr_signal.velocity_rad_per_sec,
        )
        self.last_stride_event = bool(not self.last_detection and detected)
        if self.last_stride_event:
            self.steady_state_tracker.recenter()
        self.last_detection = detected

        self.steady_state_tracker.update_steady_state(self.curr_signal)
        if self.stride_event_detector.valid_stride:
            self.controller_initialized = True
        if not self.controller_initialized:
            return 0.0
        return math.atan2(
            self.steady_state_tracker.vel_steady_state,
            self.steady_state_tracker.ang_steady_state,
        )


class HipStyleImuPhaseEstimator:
    """Estimate gait phase with the hip-controller-main processing sequence."""

    def __init__(
        self,
        config: Optional[ImuPhaseConfig] = None,
        *,
        sample_rate_hz: float = 50.0,
    ):
        self.config = config or ImuPhaseConfig()
        self.sample_rate_hz = max(1.0, float(sample_rate_hz))
        self._raw_extractor = ThighImuPhaseEstimator(self.config)
        self._preprocessor = _HipStylePreprocessor()
        self._gait_controller = _HipStyleGaitController()
        self.reset()

    def reset(self) -> None:
        self._raw_extractor.reset()
        self._preprocessor.reset()
        self._raw_velocity_envelope = 0.0
        self._is_walking_mode = True
        self._reset_phase_tracking()
        self._stop_baseline_angle_rad: Optional[float] = None
        self._startup_seed_pending = False
        self._startup_seed_active = False
        self._startup_seed_start_time: Optional[float] = None
        self._startup_seed_last_time: Optional[float] = None
        self._startup_seed_gyro_area_deg = 0.0
        self._startup_seed_phase_rad: Optional[float] = None
        self._startup_seed_decision_time: Optional[float] = None
        self._startup_ramp_start_time: Optional[float] = None
        self._stop_detection_window: list[tuple[float, float, float]] = []
        self._stop_candidate_start_time: Optional[float] = None
        self._stop_window_gyro_rms_rad_s = 0.0
        self._stop_window_angle_range_rad = 0.0
        self._phase_unstable_until: Optional[float] = None
        self.last_raw_angle_deg: Optional[float] = None
        self.last_raw_gyro_deg_s: Optional[float] = None
        self.last_raw_time_sec: Optional[float] = None
        self.last_output: Optional[ImuPhaseOutput] = None

    def _reset_phase_tracking(self) -> None:
        self._gait_controller.reset()
        self._phase_locked = False
        self._startup_stride_count = 0
        self._phase_alignment_rad = math.pi
        self._startup_seed_pending = False
        self._startup_seed_active = False
        self._startup_seed_start_time = None
        self._startup_seed_last_time = None
        self._startup_seed_gyro_area_deg = 0.0
        self._startup_seed_phase_rad = None
        self._startup_seed_decision_time = None
        self._startup_ramp_start_time = None
        self._stop_detection_window = []
        self._stop_candidate_start_time = None
        self._stop_window_gyro_rms_rad_s = 0.0
        self._stop_window_angle_range_rad = 0.0
        self._phase_unstable_until = None
        self.zero_event_count = 0
        self.previous_cycle_period_sec = clamp(
            self.config.initial_cycle_period_sec,
            self.config.min_cycle_period_sec,
            self.config.max_cycle_period_sec,
        )
        self.last_zero_time: Optional[float] = None
        self.last_motion_time: Optional[float] = None
        self.last_phase_rad: Optional[float] = None
        self.last_phase_time_sec: Optional[float] = None
        self.recent_angle_window: list[tuple[float, float]] = []

    def _startup_warmup_strides(self) -> int:
        try:
            return max(
                0,
                int(round(float(getattr(
                    self.config,
                    "startup_warmup_strides",
                    DEFAULT_STARTUP_WARMUP_STRIDES,
                )))),
            )
        except Exception:
            return DEFAULT_STARTUP_WARMUP_STRIDES

    def update_swing_threshold(self, threshold_deg: float) -> None:
        self.config.min_swing_range_deg = max(0.0, float(threshold_deg))

    def process_6d(
        self,
        sample_6d,
        sample_time: float,
        *,
        angle_source_override: Optional[str] = None,
    ) -> ImuPhaseOutput:
        angle_raw, gyro_raw = self._raw_extractor.extract_raw_signal(
            sample_6d,
            angle_source_override=angle_source_override,
        )
        return self.process_signal(angle_raw, gyro_raw, sample_time)

    def process_signal(
        self,
        angle_raw_deg: float,
        gyro_raw_deg_s: float,
        sample_time: float,
    ) -> ImuPhaseOutput:
        angle_raw_wrapped = float(angle_raw_deg)
        angle_raw = angle_raw_wrapped
        if self.last_raw_angle_deg is not None:
            angle_raw = self.last_raw_angle_deg + angular_delta_degrees(
                angle_raw_wrapped,
                self.last_raw_angle_deg,
            )
        gyro_raw = float(gyro_raw_deg_s)
        sample_time = float(sample_time)
        reject_reason = self._abnormal_sample_reason(angle_raw, gyro_raw, sample_time)
        if reject_reason:
            return self._make_rejected_output(angle_raw, gyro_raw, reject_reason)

        angle_rad = math.radians(angle_raw)
        gyro_rad_s = math.radians(gyro_raw)
        self._update_stop_detection_window(sample_time, angle_rad, gyro_rad_s)
        if (
            not self._is_walking_mode
            and not self._startup_seed_pending
            and not self._startup_seed_active
            and bool(getattr(self.config, "startup_seed_enabled", True))
            and abs(gyro_rad_s) >= PAUSE_EXIT_THRESHOLD_RAD_S
        ):
            self._begin_startup_seed(sample_time)
        started_walking, stopped_walking = self._update_pause_detector(gyro_rad_s)
        if stopped_walking:
            self._cancel_startup_seed(clear_ramp=True)
        if (
            self._is_walking_mode
            and started_walking
            and not self._startup_seed_pending
            and not self._startup_seed_active
        ):
            self._begin_startup_seed(sample_time)
        if self._startup_seed_pending:
            self._update_startup_seed(angle_rad, gyro_raw, sample_time)
        elif not self._is_walking_mode:
            self._update_stop_baseline(angle_rad)

        raw_signal = _SensorSignal(
            timestamp=sample_time,
            angle_rad=angle_rad,
            velocity_rad_per_sec=gyro_rad_s,
        )
        filtered_signal = self._preprocessor.filter(raw_signal)
        phase_plane_rad = self._gait_controller.update_and_compute(filtered_signal)

        phase_offset_rad = float(getattr(self.config, "phase_offset", 0.0)) * 2.0 * math.pi
        raw_zero_event = bool(self._gait_controller.last_stride_event)
        phase_limited = False
        accepted_raw_zero_event = bool(raw_zero_event)
        if accepted_raw_zero_event:
            # A real hip-zero legitimately wraps phase from late-cycle values
            # back to 0%. Do not run the non-zero phase-rate guard across this
            # discontinuity, otherwise normal strides look like jump faults.
            self._phase_unstable_until = None

        zero_event = False
        if accepted_raw_zero_event:
            if self.last_zero_time is not None:
                period = sample_time - float(self.last_zero_time)
                self.previous_cycle_period_sec = clamp(
                    period,
                    self.config.min_cycle_period_sec,
                    self.config.max_cycle_period_sec,
                )
            self.last_zero_time = sample_time
            self._startup_stride_count += 1
            if self._startup_stride_count >= self._startup_warmup_strides():
                self._phase_locked = True
                zero_event = True
                self.zero_event_count += 1
                self._cancel_startup_seed(clear_ramp=False)

        if zero_event:
            self._phase_alignment_rad = -phase_plane_rad

        period = max(self.previous_cycle_period_sec, EPSILON)
        temporary_phase_active = False
        if self._gait_controller.controller_initialized and self._phase_locked and self._is_walking_mode:
            # hip-controller-main phase is [-pi, pi] and reaches pi at the
            # positive-to-negative velocity event. Current gait_control modes
            # use that event as phase zero. Lock alignment on accepted stride
            # events so start-up transients do not leave a persistent offset.
            phase_rad = _wrap_to_2pi(
                phase_plane_rad + self._phase_alignment_rad + phase_offset_rad
            )
        elif self._startup_seed_active and self._startup_seed_phase_rad is not None:
            seed_time = (
                self._startup_seed_decision_time
                if self._startup_seed_decision_time is not None
                else self._startup_seed_start_time
            )
            elapsed = 0.0 if seed_time is None else max(0.0, sample_time - float(seed_time))
            phase_rad = _wrap_to_2pi(
                float(self._startup_seed_phase_rad)
                + phase_offset_rad
                + (elapsed / period) * 2.0 * math.pi
            )
            temporary_phase_active = True
        elif not self._is_walking_mode and self.last_phase_rad is not None:
            phase_rad = float(self.last_phase_rad)
        else:
            phase_rad = 0.0

        if self._phase_unstable_active(sample_time) and self.last_phase_rad is not None:
            phase_rad = float(self.last_phase_rad)
            phase_limited = True
        elif (
            self._phase_locked
            and self.last_phase_rad is not None
            and not temporary_phase_active
            and not zero_event
        ):
            phase_rad, limited_now = self._limit_phase_rate(phase_rad, sample_time)
            if limited_now:
                phase_limited = True
                self._mark_phase_unstable(sample_time)
                if self.last_phase_rad is not None:
                    phase_rad = float(self.last_phase_rad)

        filtered_angle_deg = math.degrees(filtered_signal.angle_rad)
        filtered_velocity_deg_s = math.degrees(filtered_signal.velocity_rad_per_sec)
        drift_removed_angle_deg = (
            math.degrees(float(self._preprocessor.last_drift_removed_angle_rad))
            if self._preprocessor.last_drift_removed_angle_rad is not None
            else filtered_angle_deg
        )
        recent_swing_range = self._update_recent_swing_window(
            sample_time,
            filtered_angle_deg,
        )
        motion_velocity_threshold = max(2.0, self.config.gyro_zero_threshold_deg_s * 0.35)
        motion_swing_range_threshold = max(
            2.0,
            self.config.min_swing_range_deg
            * clamp(self.config.motion_swing_range_ratio, 0.0, 1.0),
        )
        recent_swing_active = recent_swing_range >= motion_swing_range_threshold
        has_velocity_motion = abs(gyro_raw) >= motion_velocity_threshold
        if accepted_raw_zero_event or recent_swing_active or has_velocity_motion:
            self.last_motion_time = sample_time

        time_since_last_zero = (
            None if self.last_zero_time is None else max(0.0, sample_time - self.last_zero_time)
        )
        time_since_last_motion = (
            None
            if self.last_motion_time is None
            else max(0.0, sample_time - self.last_motion_time)
        )
        stale_cycle = (
            time_since_last_zero is None
            or time_since_last_zero > self.config.max_cycle_period_sec
        )
        low_motion_timeout = (
            time_since_last_motion is None
            or time_since_last_motion > self.config.motion_timeout_sec
        )
        locked_motion = bool(
            self.zero_event_count > 0
            and self._phase_locked
            and self._is_walking_mode
            and not low_motion_timeout
            and not self._phase_unstable_active(sample_time)
            and not phase_limited
            and (recent_swing_active or has_velocity_motion or accepted_raw_zero_event)
        )
        startup_seed_motion = bool(
            temporary_phase_active
            and (self._is_walking_mode or has_velocity_motion)
            and not low_motion_timeout
            and not self._phase_unstable_active(sample_time)
            and not phase_limited
            and (recent_swing_active or has_velocity_motion)
        )
        motion_active = bool(
            startup_seed_motion
            or (
                locked_motion
                and (
                    not stale_cycle
                    or has_velocity_motion
                    or recent_swing_active
                    or accepted_raw_zero_event
                )
            )
        )

        freq_hz = 1.0 / period
        if self.zero_event_count == 0:
            freq_hz = max(0.0, float(self._preprocessor.estimated_frequency_hz))
            period = 1.0 / max(freq_hz, 1e-6) if freq_hz > 0.0 else period
        startup_ramp_scale = self._startup_ramp_scale(sample_time)

        self.last_raw_angle_deg = angle_raw
        self.last_raw_gyro_deg_s = gyro_raw
        self.last_raw_time_sec = sample_time
        self.last_phase_rad = phase_rad
        self.last_phase_time_sec = sample_time

        output = ImuPhaseOutput(
            angle_raw_deg=angle_raw,
            angle_deg=filtered_angle_deg,
            angular_velocity_raw_deg_s=gyro_raw,
            angular_velocity_deg_s=filtered_velocity_deg_s,
            phase_0_to_1=float((phase_rad / (2.0 * math.pi)) % 1.0),
            phase_rad=float(phase_rad),
            previous_cycle_period_sec=float(period),
            previous_cycle_frequency_hz=float(freq_hz),
            zero_event=zero_event,
            zero_event_count=int(self.zero_event_count),
            motion_active=motion_active,
            current_swing_range_deg=float(recent_swing_range),
            time_since_last_zero_sec=time_since_last_zero,
            time_since_last_motion_sec=time_since_last_motion,
            recent_swing_range_deg=float(recent_swing_range),
            recent_swing_active=recent_swing_active,
            phase_limited=phase_limited,
            angle_drift_removed_deg=float(drift_removed_angle_deg),
            temporary_phase_active=temporary_phase_active,
            startup_ramp_scale=float(startup_ramp_scale),
            startup_seed_phase_rad=self._startup_seed_phase_rad,
            stopped_event=stopped_walking,
            walking_mode=bool(self._is_walking_mode),
        )
        self.last_output = output
        return output

    def _update_pause_detector(self, raw_velocity_rad_s: float) -> tuple[bool, bool]:
        was_walking = bool(self._is_walking_mode)
        self._raw_velocity_envelope = (
            (1.0 - PAUSE_DETECT_ENVELOPE_ALPHA) * self._raw_velocity_envelope
            + PAUSE_DETECT_ENVELOPE_ALPHA * abs(float(raw_velocity_rad_s))
        )
        if self._is_walking_mode and self._stop_confirmed():
            self._is_walking_mode = False
            self._preprocessor.set_walking_mode(False)
        elif (
            not self._is_walking_mode
            and self._raw_velocity_envelope > PAUSE_EXIT_THRESHOLD_RAD_S
        ):
            self._is_walking_mode = True
            self._stop_candidate_start_time = None
            self._preprocessor.set_walking_mode(True)
        return (not was_walking and self._is_walking_mode), (was_walking and not self._is_walking_mode)

    def _update_stop_detection_window(
        self,
        sample_time: float,
        angle_rad: float,
        raw_velocity_rad_s: float,
    ) -> None:
        now = float(sample_time)
        if not (math.isfinite(angle_rad) and math.isfinite(raw_velocity_rad_s)):
            self._stop_detection_window = []
            self._stop_candidate_start_time = None
            self._stop_window_gyro_rms_rad_s = 0.0
            self._stop_window_angle_range_rad = 0.0
            return

        self._stop_detection_window.append((now, float(angle_rad), float(raw_velocity_rad_s)))
        window_sec = max(
            0.05,
            float(getattr(self.config, "stop_detection_window_sec", 0.30)),
        )
        cutoff = now - window_sec
        while self._stop_detection_window and self._stop_detection_window[0][0] < cutoff:
            self._stop_detection_window.pop(0)

        if not self._stop_detection_window:
            self._stop_window_gyro_rms_rad_s = 0.0
            self._stop_window_angle_range_rad = 0.0
            return

        gyros = [gyro for _time_sec, _angle, gyro in self._stop_detection_window]
        angles = [angle for _time_sec, angle, _gyro in self._stop_detection_window]
        self._stop_window_gyro_rms_rad_s = math.sqrt(
            sum(gyro * gyro for gyro in gyros) / max(1, len(gyros))
        )
        self._stop_window_angle_range_rad = max(angles) - min(angles)

    def _stop_confirmed(self) -> bool:
        if not self._stop_detection_window:
            self._stop_candidate_start_time = None
            return False

        now = self._stop_detection_window[-1][0]
        span = self._stop_detection_window[-1][0] - self._stop_detection_window[0][0]
        window_sec = max(
            0.05,
            float(getattr(self.config, "stop_detection_window_sec", 0.30)),
        )
        enough_window = span >= min(window_sec * 0.75, 0.20)
        gyro_rms_limit = math.radians(
            max(0.0, float(getattr(self.config, "stop_gyro_rms_threshold_deg_s", 8.0)))
        )
        angle_range_limit = math.radians(
            max(0.0, float(getattr(self.config, "stop_angle_range_threshold_deg", 3.0)))
        )
        quiet_window = bool(
            enough_window
            and self._stop_window_gyro_rms_rad_s <= gyro_rms_limit
            and self._stop_window_angle_range_rad <= angle_range_limit
        )
        if not quiet_window:
            self._stop_candidate_start_time = None
            return False

        if self._stop_candidate_start_time is None:
            self._stop_candidate_start_time = now
        hold_sec = max(0.0, float(getattr(self.config, "stop_detection_hold_sec", 0.12)))
        return (now - float(self._stop_candidate_start_time)) >= hold_sec

    def _phase_delta_exceeds_limit(self, phase_rad: float, sample_time: float) -> bool:
        max_rate_hz = max(
            0.0,
            float(getattr(self.config, "stop_phase_rate_limit_hz", 0.0)),
        )
        if max_rate_hz <= EPSILON or self.last_phase_rad is None or self.last_phase_time_sec is None:
            return False

        dt = float(sample_time) - float(self.last_phase_time_sec)
        if dt <= EPSILON or dt > 0.25:
            return False

        max_delta = 2.0 * math.pi * max_rate_hz * dt
        signed_delta = math.atan2(
            math.sin(float(phase_rad) - float(self.last_phase_rad)),
            math.cos(float(phase_rad) - float(self.last_phase_rad)),
        )
        return signed_delta > max_delta

    def _limit_phase_rate(self, phase_rad: float, sample_time: float) -> tuple[float, bool]:
        if not self._phase_delta_exceeds_limit(phase_rad, sample_time):
            return _wrap_to_2pi(phase_rad), False
        previous = 0.0 if self.last_phase_rad is None else float(self.last_phase_rad)
        return _wrap_to_2pi(previous), True

    def _mark_phase_unstable(self, sample_time: float) -> None:
        hold_sec = max(
            0.0,
            float(getattr(self.config, "stop_phase_unstable_hold_sec", 0.25)),
        )
        if hold_sec <= EPSILON:
            self._phase_unstable_until = None
            return
        until = float(sample_time) + hold_sec
        if self._phase_unstable_until is None:
            self._phase_unstable_until = until
        else:
            self._phase_unstable_until = max(float(self._phase_unstable_until), until)

    def _phase_unstable_active(self, sample_time: float) -> bool:
        if self._phase_unstable_until is None:
            return False
        if float(sample_time) <= float(self._phase_unstable_until):
            return True
        self._phase_unstable_until = None
        return False

    def _update_stop_baseline(self, angle_rad: float) -> None:
        if not math.isfinite(angle_rad):
            return
        if self._stop_baseline_angle_rad is None:
            self._stop_baseline_angle_rad = float(angle_rad)
        else:
            alpha = clamp(STARTUP_SEED_BASELINE_ALPHA, 0.0, 1.0)
            self._stop_baseline_angle_rad += alpha * (
                float(angle_rad) - self._stop_baseline_angle_rad
            )

    def _startup_seed_window(self) -> tuple[float, float]:
        min_window = max(
            0.0,
            float(getattr(self.config, "startup_seed_min_window_sec", 0.10)),
        )
        max_window = max(
            min_window,
            float(getattr(self.config, "startup_seed_max_window_sec", 0.20)),
        )
        return min_window, max_window

    def _begin_startup_seed(self, sample_time: float) -> None:
        if not bool(getattr(self.config, "startup_seed_enabled", True)):
            return
        self._startup_seed_pending = True
        self._startup_seed_active = False
        self._startup_seed_start_time = float(sample_time)
        self._startup_seed_last_time = float(sample_time)
        self._startup_seed_gyro_area_deg = 0.0
        self._startup_seed_phase_rad = None
        self._startup_seed_decision_time = None

    def _cancel_startup_seed(self, *, clear_ramp: bool) -> None:
        self._startup_seed_pending = False
        self._startup_seed_active = False
        self._startup_seed_start_time = None
        self._startup_seed_last_time = None
        self._startup_seed_gyro_area_deg = 0.0
        self._startup_seed_phase_rad = None
        self._startup_seed_decision_time = None
        if clear_ramp:
            self._startup_ramp_start_time = None

    def _update_startup_seed(
        self,
        angle_rad: float,
        gyro_raw_deg_s: float,
        sample_time: float,
    ) -> None:
        if (
            not self._startup_seed_pending
            or self._startup_seed_active
            or self.zero_event_count > 0
        ):
            return

        now = float(sample_time)
        start_time = self._startup_seed_start_time
        if start_time is None:
            self._begin_startup_seed(now)
            start_time = self._startup_seed_start_time
        if start_time is None:
            return

        last_time = self._startup_seed_last_time
        dt = NOMINAL_TIME_DIFFERENCE_S if last_time is None else now - float(last_time)
        if dt <= 0.0 or dt > MAX_PLAUSIBLE_TIME_DIFFERENCE_S:
            dt = NOMINAL_TIME_DIFFERENCE_S
        self._startup_seed_last_time = now
        self._startup_seed_gyro_area_deg += float(gyro_raw_deg_s) * dt

        elapsed = max(0.0, now - float(start_time))
        min_window, max_window = self._startup_seed_window()
        if elapsed < min_window:
            return

        threshold = max(
            0.0,
            float(getattr(self.config, "startup_seed_gyro_threshold_deg_s", 15.0)),
        )
        mean_gyro = self._startup_seed_gyro_area_deg / max(elapsed, EPSILON)
        direction = 0
        if threshold <= 0.0 or abs(mean_gyro) >= threshold:
            direction = 1 if mean_gyro >= 0.0 else -1
        elif elapsed >= max_window and abs(mean_gyro) >= threshold * 0.5:
            direction = 1 if mean_gyro >= 0.0 else -1
        elif elapsed >= max_window and self._stop_baseline_angle_rad is not None:
            angle_threshold = max(
                0.0,
                float(getattr(self.config, "startup_seed_angle_threshold_deg", 3.0)),
            )
            angle_delta_deg = math.degrees(float(angle_rad) - self._stop_baseline_angle_rad)
            if angle_threshold <= 0.0 or abs(angle_delta_deg) >= angle_threshold:
                direction = 1 if angle_delta_deg >= 0.0 else -1

        if direction == 0:
            if elapsed >= max_window:
                self._startup_seed_pending = False
            return

        seed_phase = (
            float(getattr(self.config, "startup_seed_positive_gyro_phase_rad", 0.0))
            if direction > 0
            else float(getattr(self.config, "startup_seed_negative_gyro_phase_rad", -math.pi))
        )
        self._startup_seed_phase_rad = seed_phase
        self._startup_seed_decision_time = now
        self._startup_seed_pending = False
        self._startup_seed_active = True
        self._startup_ramp_start_time = now

    def _startup_ramp_scale(self, sample_time: float) -> float:
        start_time = self._startup_ramp_start_time
        ramp_sec = max(0.0, float(getattr(self.config, "startup_seed_ramp_sec", 0.30)))
        if start_time is None or ramp_sec <= EPSILON:
            return 1.0
        scale = clamp((float(sample_time) - float(start_time)) / ramp_sec, 0.0, 1.0)
        if scale >= 1.0:
            self._startup_ramp_start_time = None
        return scale

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
                phase_limited=bool(getattr(previous, "phase_limited", False)),
                temporary_phase_active=bool(getattr(previous, "temporary_phase_active", False)),
                startup_ramp_scale=float(getattr(previous, "startup_ramp_scale", 1.0)),
                startup_seed_phase_rad=getattr(previous, "startup_seed_phase_rad", None),
                stopped_event=False,
                walking_mode=bool(getattr(previous, "walking_mode", self._is_walking_mode)),
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
            phase_limited=False,
            temporary_phase_active=False,
            startup_ramp_scale=1.0,
            startup_seed_phase_rad=self._startup_seed_phase_rad,
            stopped_event=False,
            walking_mode=bool(self._is_walking_mode),
        )
