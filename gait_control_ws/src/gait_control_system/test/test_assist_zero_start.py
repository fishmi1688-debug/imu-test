"""Exercise motion restart gating against the actual configured torque curves."""

from collections import deque
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from gait_control_system.gait_control_system.motion_mode_defaults import get_default_motion_modes
from gait_control_system.gait_control_system.real_time_gait_analysis import RealTimeGaitAnalysis


@pytest.fixture
def analysis(monkeypatch):
    node = object.__new__(RealTimeGaitAnalysis)
    monkeypatch.setattr(node, 'get_logger', Mock(return_value=Mock()))
    node.motion_modes = get_default_motion_modes()
    node.current_motion_mode = 'imu_phase'
    node.buffer_size = 100
    node.debug_mode = False
    node.log_counter = 0
    node.gait_state = 1
    node.assist_enable = True
    node.phase_active = True
    node.imu_phase_motion_active = True
    node.imu_phase_abnormal_sample = False
    node.stairs_down_manual_assist_enabled = True
    node.stairs_down_manual_toggle_count = 0
    node.left_hipAss = deque(maxlen=100)
    node.right_hipAss = deque(maxlen=100)
    node.hipAss = deque(maxlen=100)
    node._reset_assist_curve_start()
    return node


def step(node, left_phase, right_phase=None):
    """Run the real torque-profile calculation with normalized IMU phases."""
    node.current_left_phase = left_phase * 2 * np.pi
    node.current_right_phase = (left_phase + 0.5 if right_phase is None else right_phase) * 2 * np.pi
    node.calculate_hip_torque_profile()
    return node.left_hipAss[-1], node.right_hipAss[-1]


def curve(node, phase):
    return float(node._calculate_hip_assist_quintic(phase % 1.0, node.get_current_assist_parameters()))


@pytest.mark.parametrize('mode', list(get_default_motion_modes()))
def test_all_registered_modes_start_at_zero_and_then_follow_their_curve(analysis, mode):
    analysis.current_motion_mode = mode
    assert abs(curve(analysis, 0.2)) > 0.01 or abs(curve(analysis, 0.7)) > 0.01
    assert step(analysis, 0.2) == (0.0, 0.0)
    assert analysis.assist_wait_next_zero
    for phase in np.arange(0.22, 1.22, 0.02):
        step(analysis, float(phase))
        if all(analysis._assist_curve_start_ready.values()):
            break
    assert all(analysis._assist_curve_start_ready.values())
    next_phase = float(phase) + 0.02
    assert step(analysis, next_phase) == pytest.approx((
        curve(analysis, next_phase), curve(analysis, next_phase + 0.5),
    ))
    assert not analysis.assist_wait_next_zero


def test_waits_for_nearest_curve_zero_instead_of_phase_wrap(analysis):
    assert step(analysis, 0.2) == (0.0, 0.0)
    for phase in (0.22, 0.26, 0.29):
        assert step(analysis, phase) == (0.0, 0.0)
        assert not any(analysis._assist_curve_start_ready.values())
    assert step(analysis, 0.31) == (0.0, 0.0)
    assert all(analysis._assist_curve_start_ready.values())
    assert analysis.current_left_phase == pytest.approx(0.31 * 2 * np.pi)
    assert step(analysis, 0.52) == pytest.approx((curve(analysis, 0.52), curve(analysis, 1.02)))


def test_sides_wait_for_their_own_nearest_zero(analysis):
    assert step(analysis, 0.2, 0.25) == (0.0, 0.0)
    assert step(analysis, 0.31, 0.27) == (0.0, 0.0)
    assert analysis._assist_curve_start_ready == {'left': True, 'right': False}
    left, right = step(analysis, 0.52, 0.28)
    assert left == pytest.approx(curve(analysis, 0.52))
    assert abs(left) > 0.001
    assert right == 0.0
    assert analysis.assist_wait_next_zero


def test_zero_gate_uses_phase_bias(analysis):
    analysis.motion_modes['imu_phase']['phase_bias'] = 0.17
    assert abs(curve(analysis, 0.05)) > 1.0
    assert step(analysis, 0.05) == (0.0, 0.0)
    assert step(analysis, 0.1) == (0.0, 0.0)
    assert not any(analysis._assist_curve_start_ready.values())
    assert step(analysis, 0.14) == (0.0, 0.0)
    assert all(analysis._assist_curve_start_ready.values())
    assert step(analysis, 0.35) == pytest.approx((curve(analysis, 0.35), curve(analysis, 0.85)))


def test_nonzero_curve_at_phase_zero_does_not_release_gate(analysis):
    params = analysis.motion_modes['imu_phase']
    params.update(ext_t0=0.8, ext_tf=0.2, ext_p=0.5, flex_Tmax=0.0)
    assert curve(analysis, 0.0) == pytest.approx(5.0)
    assert step(analysis, 0.95)[0] == 0.0
    assert step(analysis, 0.99)[0] == 0.0
    assert step(analysis, 0.01)[0] == 0.0
    assert not analysis._assist_curve_start_ready['left']
    assert step(analysis, 0.21)[0] == 0.0
    assert analysis._assist_curve_start_ready['left']


def test_zero_interval_between_samples_is_detected_and_release_frame_stays_zero(analysis):
    params = analysis.motion_modes['imu_phase']
    params.update(ext_tf=0.3, flex_t0=0.3005, flex_tf=0.9, flex_p=0.5)
    assert step(analysis, 0.29)[0] == 0.0
    assert abs(curve(analysis, 0.4)) > 0.1
    assert step(analysis, 0.4)[0] == 0.0
    assert analysis._assist_curve_start_ready['left']
    assert analysis.assist_curve_start_hold['left']
    assert step(analysis, 0.42)[0] == pytest.approx(curve(analysis, 0.42))
    assert not analysis.assist_curve_start_hold['left']


def test_overlapping_windows_use_net_torque_zero(analysis):
    params = analysis.motion_modes['imu_phase']
    params.update(ext_t0=0.0, ext_tf=0.6, ext_p=0.4,
                  flex_t0=0.1, flex_tf=0.7, flex_p=0.6)
    assert curve(analysis, 0.25) > 0.0
    assert curve(analysis, 0.42) < 0.0
    assert step(analysis, 0.25)[0] == 0.0
    assert step(analysis, 0.42)[0] == 0.0
    assert analysis._assist_curve_start_ready['left']
    assert step(analysis, 0.44)[0] == pytest.approx(curve(analysis, 0.44))


def test_window_boundary_is_not_zero_when_other_window_still_outputs(analysis):
    params = analysis.motion_modes['imu_phase']
    params.update(ext_t0=0.0, ext_tf=0.6, ext_p=0.4,
                  flex_t0=0.1, flex_tf=0.7, flex_p=0.6)
    assert abs(curve(analysis, 0.6)) > 0.1
    assert step(analysis, 0.55)[0] == 0.0
    assert step(analysis, 0.61)[0] == 0.0
    assert not analysis._assist_curve_start_ready['left']
    assert step(analysis, 0.71)[0] == 0.0
    assert analysis._assist_curve_start_ready['left']


@pytest.mark.parametrize('stop_flag', [
    'assist_enable', 'phase_active', 'imu_phase_motion_active', 'imu_phase_abnormal_sample',
])
def test_stop_or_invalid_input_rearms_zero_gate(analysis, stop_flag):
    step(analysis, 0.31)
    assert all(analysis._assist_curve_start_ready.values())
    step(analysis, 0.6)
    stopped_value = stop_flag == 'imu_phase_abnormal_sample'
    setattr(analysis, stop_flag, stopped_value)
    assert step(analysis, 0.65) == (0.0, 0.0)
    assert not any(analysis._assist_curve_start_ready.values())
    setattr(analysis, stop_flag, not stopped_value)
    assert step(analysis, 0.7) == (0.0, 0.0)
    assert not any(analysis._assist_curve_start_ready.values())
    assert step(analysis, 0.81) == (0.0, 0.0)
    assert all(analysis._assist_curve_start_ready.values())


def test_manual_stop_and_restart_between_samples_rearms_gate(analysis):
    step(analysis, 0.31)
    assert all(analysis._assist_curve_start_ready.values())
    analysis.set_stairs_down_manual_assist(enabled=False)
    analysis.set_stairs_down_manual_assist(enabled=True)
    assert not any(analysis._assist_curve_start_ready.values())
    assert step(analysis, 0.2) == (0.0, 0.0)


def test_backwards_phase_jitter_does_not_fake_a_full_cycle_zero_crossing(analysis):
    step(analysis, 0.2)
    assert step(analysis, 0.19) == (0.0, 0.0)
    assert not any(analysis._assist_curve_start_ready.values())
