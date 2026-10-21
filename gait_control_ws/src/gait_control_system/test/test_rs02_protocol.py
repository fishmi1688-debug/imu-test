"""Check RS02 manual wire examples without opening a hardware CAN interface."""

import csv
import os
from pathlib import Path
import runpy
import sys
from unittest.mock import Mock

import can
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from gait_control_system.gait_control_system import motor_controller as motor_module


@pytest.fixture
def controller(monkeypatch):
    """Use the real message encoder/decoder with an in-process CAN bus."""
    node = object.__new__(motor_module.MotorController)
    node.debug_mode = False
    node.log_counter = 0
    node.master_can_id = 0xFD
    node.active_report_target_hz = 50
    node.feedback_function_ids = {0x02, 0x18}
    node.feedback_timeout_sec = 0.15
    node.left_motor_timestamp = None
    node.right_motor_timestamp = None
    node.feedback_log_handle = None
    node.feedback_log_last_row_time = 0.0
    node.feedback_monitor_last_log_time = 0.0
    logger = Mock()
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    monkeypatch.setattr(motor_module, 'RS02_STARTUP_LIMITS_ENABLED', True)
    monkeypatch.setattr(motor_module, 'RS02_STARTUP_LIMIT_TORQUE_NM', 17.0)
    monkeypatch.setattr(motor_module, 'RS02_STARTUP_LIMIT_CUR_A', 23.0)
    node.bus = can.Bus(
        interface='virtual', channel=object(),
        receive_own_messages=True, preserve_timestamps=True,
    )
    yield node
    node.bus.shutdown()


def take_frame(controller):
    """Read one encoded frame and enforce the manual's frame format."""
    frame = controller.bus.recv(timeout=0)
    assert frame is not None
    assert frame.is_extended_id
    assert not frame.is_fd
    assert frame.dlc == 8
    return frame.arbitration_id, bytes(frame.data).hex()


def test_power_on_uses_preset_zero_without_sending_zero_frames(controller, monkeypatch, tmp_path):
    """A fresh controller starts ready and preserves the configured encoder zero."""
    controller_class = motor_module.MotorController
    monkeypatch.setattr(motor_module.Node, '__init__', lambda *args, **kwargs: None)
    monkeypatch.setattr(controller_class, 'get_logger', Mock(return_value=Mock()))
    monkeypatch.setattr(controller_class, 'create_publisher', Mock())
    monkeypatch.setattr(controller_class, 'create_subscription', Mock())
    monkeypatch.setattr(motor_module, 'setup_can_interface', Mock())
    monkeypatch.setattr(motor_module.can.interface, 'Bus', Mock(return_value=controller.bus))
    monkeypatch.setattr(motor_module, 'WORKSPACE_ROOT', str(tmp_path))
    zero_command = Mock(side_effect=AssertionError('power-on must preserve the fixed zero'))
    monkeypatch.setattr(controller_class, 'send_mechanical_zero_command', zero_command)

    node = controller_class()
    try:
        assert node.mechanical_zeroed
        assert node.left_motor_zeroed
        assert node.right_motor_zeroed
        assert node.motion_confirmed_after_zero
        assert node.left_motor_timestamp is None
        assert node.right_motor_timestamp is None
        zero_command.assert_not_called()
        assert [take_frame(controller) for _ in range(4)] == [
            (0x1200FD01, '0b70000000008841'),
            (0x1200FD01, '187000000000b841'),
            (0x1200FD02, '0b70000000008841'),
            (0x1200FD02, '187000000000b841'),
        ]
        assert controller.bus.recv(timeout=0) is None
        assert node.send_mit_torque_command(0x01, -15.0)
        assert node.send_mit_torque_command(0x02, 15.0)
        for motor_id, data in ((0x01, 'ffff000000000258'), (0x02, 'ffff0000ffff0258')):
            controller.bus.send(can.Message(
                arbitration_id=0x028000FD | (motor_id << 8),
                is_extended_id=True, data=bytes.fromhex(data), timestamp=1000.0,
            ))
        assert node.monitor_feedback(now=1000.0, expected=True) == 2
        assert node.monitor_feedback(now=1000.01, expected=True) == 0
        assert node.monitor_feedback(now=1000.2, expected=True) == 0
    finally:
        node.close_calibration_log()
    with open(node.feedback_log_file, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 2  # The 10 ms call does not add a duplicate 50 Hz record.
    for side in ('left', 'right'):
        assert float(rows[0][f'{side}_command_torque_nm']) == 15.0
        assert float(rows[0][f'{side}_feedback_torque_nm']) == 17.0
        assert rows[0][f'{side}_mode'] == '2'
        assert rows[0][f'{side}_fresh'] == 'True'
        assert rows[1][f'{side}_feedback_torque_nm'] == ''
        assert rows[1][f'{side}_fault'] == ''
        assert rows[1][f'{side}_fresh'] == 'False'
    assert node.feedback_log_handle is None


@pytest.mark.parametrize('motor_id', [0x01, 0x02])
def test_enable_configures_limits_and_50hz(controller, motor_id):
    """Write official RS02 limits, the 20 ms period, report, then enable."""
    assert controller.enable_motor(motor_id)
    assert [take_frame(controller) for _ in range(5)] == [
        (0x1200FD00 | motor_id, '0b70000000008841'),  # 0x700B: 17.0 Nm
        (0x1200FD00 | motor_id, '187000000000b841'),  # 0x7018: 23.0 A
        (0x1200FD00 | motor_id, '2670000003000000'),  # 0x7026: uint16 3
        (0x1800FD00 | motor_id, '0102030405060100'),
        (0x0300FD00 | motor_id, '0000000000000000'),
    ]
    assert controller.bus.recv(timeout=0) is None


@pytest.mark.parametrize('motor_id', [0x01, 0x02])
def test_zero_stop_and_report_off(controller, motor_id):
    """Pages 34, 36: preserve mechanical zero, stop and report commands."""
    assert controller.send_mechanical_zero_command(motor_id)
    assert controller.disable_motor(motor_id)
    assert controller.set_active_report(motor_id, enabled=False)
    assert [take_frame(controller) for _ in range(3)] == [
        (0x0600FD00 | motor_id, '0100000000000000'),
        (0x0400FD00 | motor_id, '0000000000000000'),
        (0x1800FD00 | motor_id, '0102030405060000'),
    ]


@pytest.mark.parametrize('motor_id', [0x01, 0x02])
@pytest.mark.parametrize('torque,torque_field', [
    (-17.0, 0x0000), (0.0, 0x7FFF), (8.5, 0xBFFF), (15.0, 0xF0F0), (17.0, 0xFFFF),
])
def test_torque_frame_keeps_17nm_wire_scale(controller, motor_id, torque, torque_field):
    """Page 33: preserve ±17 Nm scaling, including commands above 14 Nm."""
    assert controller.send_mit_torque_command(motor_id, torque)
    assert take_frame(controller) == (
        0x01000000 | (torque_field << 8) | motor_id, '7fff7fff00000000',
    )


@pytest.mark.parametrize('command,expected', [
    ((12.57, 44.0, 500.0, 5.0, 17.0), (0x01FFFF01, 'ffffffffffffffff')),
    ((-12.57, -44.0, 0.0, 0.0, -17.0), (0x01000001, '0000000000000000')),
])
def test_control_frame_full_ranges(controller, command, expected):
    """Page 33: all four payload fields are big-endian uint16."""
    assert controller.send_mit_command(0x01, *command)
    assert take_frame(controller) == expected


def test_control_frame_mixed_fields_keep_byte_order(controller):
    """Distinct fields catch swapped payload words and wrong endianness."""
    assert controller.send_mit_command(0x02, 6.285, -22.0, 125.0, 3.75, 8.5)
    assert take_frame(controller) == (0x01BFFF02, 'bfff3fff3fffbfff')


@pytest.mark.parametrize('function_id', [0x02, 0x18])
@pytest.mark.parametrize('motor_id,side,sign', [(0x01, 'left', -1), (0x02, 'right', 1)])
@pytest.mark.parametrize('payload,angle,velocity,torque,temperature', [
    ('ffff0000ffff0258', 12.57, -44.0, 17.0, 60.0),
    ('0000ffff00000190', -12.57, 44.0, -17.0, 40.0),
])
def test_feedback_ranges_faults_and_side_signs(
    controller, function_id, motor_id, side, sign, payload, angle, velocity, torque, temperature,
):
    """Pages 33, 37: decode both feedback types and retain the mounting signs."""
    controller.bus.send(can.Message(
        arbitration_id=(function_id << 24) | 0x850000 | (motor_id << 8) | 0xFD,
        is_extended_id=True,
        data=bytes.fromhex(payload),
        timestamp=1234.5,
    ))
    assert controller.poll_feedback() == 1
    assert getattr(controller, f'{side}_motor_angle') == pytest.approx(sign * angle)
    assert getattr(controller, f'{side}_motor_velocity') == pytest.approx(sign * velocity)
    assert getattr(controller, f'{side}_motor_current') == pytest.approx(sign * torque)
    assert getattr(controller, f'{side}_motor_temperature') == temperature
    assert getattr(controller, f'{side}_motor_error') == 5
    assert getattr(controller, f'{side}_motor_mode') == 2
    assert getattr(controller, f'{side}_motor_timestamp') == 1234.5


@pytest.mark.parametrize('arbitration_id,extended,payload', [
    (0x181, False, '0000000000000000'),  # MI1 standard CAN frame
    (0x110001FD, True, '0b70000000008841'),  # Parameter read response
    (0x020003FD, True, '0000000000000000'),  # Unconfigured motor ID
    (0x020001FD, True, '00000000000000'),  # Truncated status
])
def test_other_can_traffic_does_not_update_motor_feedback(
    controller, arbitration_id, extended, payload,
):
    """Ignore unrelated IMU/parameter traffic and reject short status data."""
    controller.left_motor_timestamp = None
    controller.right_motor_timestamp = None
    controller.bus.send(can.Message(
        arbitration_id=arbitration_id, is_extended_id=extended,
        data=bytes.fromhex(payload), timestamp=1234.5,
    ))
    assert controller.poll_feedback() == 0
    assert controller.left_motor_timestamp is None
    assert controller.right_motor_timestamp is None


@pytest.mark.parametrize('mode', [0, 1, 2])
def test_monitor_reads_command_responses_and_decodes_run_state(controller, monkeypatch, mode):
    """Monitoring consumes replies even when periodic reports are disabled."""
    recv = Mock(wraps=controller.bus.recv)
    monkeypatch.setattr(controller.bus, 'recv', recv)
    controller.bus.send(can.Message(
        arbitration_id=0x020002FD | (mode << 22), is_extended_id=True,
        data=bytes.fromhex('ffff0000ffff0258'), timestamp=100.0,
    ))
    assert controller.monitor_feedback(now=100.0) == 1
    status = controller.get_motor_feedback_status(0x02, now=100.0)
    assert status['fresh']
    assert status['mode'] == mode
    assert status['feedback_torque_nm'] == 17.0
    assert status['temperature_c'] == 60.0
    assert status['fault'] == 0
    assert all(call.kwargs['timeout'] == 0.0 for call in recv.call_args_list)
    assert controller.bus.recv(timeout=0) is None  # Monitoring transmits no commands.


def test_missing_and_stale_feedback_are_not_reported_as_zero_torque(controller):
    """No response or an expired packet must remain explicitly invalid."""
    missing = controller.get_motor_feedback_status(0x01, now=100.0)
    assert not missing['fresh']
    assert missing['feedback_torque_nm'] is None
    assert missing['mode'] is None
    assert missing['fault'] is None
    assert missing['age_ms'] is None
    controller.bus.send(can.Message(
        arbitration_id=0x028102FD, is_extended_id=True,
        data=bytes.fromhex('ffff0000ffff0258'), timestamp=100.0,
    ))
    controller.monitor_feedback(now=100.0)
    fresh = controller.get_motor_feedback_status(0x02, now=100.0)
    assert fresh['fault'] == 1
    stale = controller.get_motor_feedback_status(0x02, now=100.2)
    assert not stale['fresh']
    assert stale['feedback_torque_nm'] is None
    assert stale['temperature_c'] is None
    assert stale['mode'] is None
    assert stale['fault'] is None
    assert stale['age_ms'] == pytest.approx(200.0)


def test_sent_target_and_feedback_are_separate_and_failed_send_keeps_target(controller, monkeypatch):
    """The target records the last successful send in the same signed coordinates."""
    assert controller.send_mit_torque_command(0x01, -15.0)
    controller.bus.send(can.Message(
        arbitration_id=0x028001FD, is_extended_id=True,
        data=bytes.fromhex('ffff000000000258'), timestamp=100.0,
    ))
    controller.monitor_feedback(now=100.0)
    status = controller.get_motor_feedback_status(0x01, now=100.0)
    assert status['command_torque_nm'] == 15.0
    assert status['feedback_torque_nm'] == 17.0
    monkeypatch.setattr(controller.bus, 'send', Mock(side_effect=can.CanError('send failed')))
    assert not controller.send_mit_torque_command(0x01, -5.0)
    assert controller.get_motor_feedback_status(0x01, now=100.0)['command_torque_nm'] == 15.0


def test_missing_expected_feedback_warning_is_limited_to_once_a_second(controller):
    """Lack of response is visible even without debug logging."""
    logger = controller.get_logger()
    controller.monitor_feedback(now=100.0, expected=True)
    assert logger.warn.call_count == 1
    assert '尚无反馈' in logger.warn.call_args.args[0]
    controller.monitor_feedback(now=100.02, expected=True)
    assert logger.warn.call_count == 1
    controller.monitor_feedback(now=101.0, expected=True)
    assert logger.warn.call_count == 2


@pytest.mark.parametrize('torque,current,expected', [
    (30.0, 40.0, ['0b70000000008841', '187000000000b841']),
    (15.0, 20.0, ['0b70000000007041', '187000000000a041']),
    (-1.0, -1.0, ['0b70000000000000', '1870000000000000']),
])
def test_startup_parameters_stay_in_rs02_ranges(controller, monkeypatch, torque, current, expected):
    """Allow settings above 14 Nm and clamp to the official RS02 limits."""
    monkeypatch.setattr(motor_module, 'RS02_STARTUP_LIMIT_TORQUE_NM', torque)
    monkeypatch.setattr(motor_module, 'RS02_STARTUP_LIMIT_CUR_A', current)
    assert controller.configure_startup_torque_limits(0x01)
    assert [take_frame(controller)[1] for _ in range(2)] == expected


@pytest.mark.parametrize('use_legacy,use_rs02,expected', [
    (False, False, (0xFD, 50, 17.0, 23.0, True)),
    (True, False, (0xFE, 50, 10.0, 12.0, False)),
    (True, True, (0xFC, 50, 11.0, 13.0, True)),
])
def test_defaults_and_environment_compatibility(monkeypatch, use_legacy, use_rs02, expected):
    """Keep IDs and 50 Hz defaults, with RS02 settings taking precedence."""
    for name in list(os.environ):
        if name.startswith(('GAIT_RS01_', 'GAIT_RS02_')):
            monkeypatch.delenv(name)
    if use_legacy:
        for suffix, value in zip(
            ('MASTER_ID', 'REPORT_HZ', 'LIMIT_TORQUE_NM', 'LIMIT_CUR_A', 'STARTUP_LIMITS_ENABLE'),
            ('0xFE', '50', '10', '12', '0'),
        ):
            monkeypatch.setenv(f'GAIT_RS01_{suffix}', value)
    if use_rs02:
        for suffix, value in zip(
            ('MASTER_ID', 'REPORT_HZ', 'LIMIT_TORQUE_NM', 'LIMIT_CUR_A', 'STARTUP_LIMITS_ENABLE'),
            ('0xFC', '50', '11', '13', '1'),
        ):
            monkeypatch.setenv(f'GAIT_RS02_{suffix}', value)
    constants = runpy.run_path(str(Path(motor_module.__file__).with_name('gait_constants.py')))
    assert (constants['LEFT_MOTOR_ID'], constants['RIGHT_MOTOR_ID']) == (0x01, 0x02)
    assert tuple(constants[name] for name in (
        'RS02_MASTER_ID', 'RS02_ACTIVE_REPORT_HZ', 'RS02_STARTUP_LIMIT_TORQUE_NM',
        'RS02_STARTUP_LIMIT_CUR_A', 'RS02_STARTUP_LIMITS_ENABLED',
    )) == expected
