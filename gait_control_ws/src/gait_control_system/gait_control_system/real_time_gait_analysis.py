import csv
import math
import os
import time
from collections import deque
from dataclasses import replace
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32

from .adaptive_oscillator_estimator import AdaptiveOscillatorEstimator
from .gait_model_phase_estimator import (
    DEFAULT_MODEL_PHASE_MODEL_PATH,
    DEFAULT_MODEL_PHASE_SCALER_PATH,
    MODEL_PHASE_SAMPLE_RATE_HZ,
    MODEL_PHASE_WINDOW_N,
    RealtimeOnnxGaitPhaseEstimator,
)
from .imu_phase_estimator import ImuPhaseConfig, ImuPhaseOutput, ThighImuPhaseEstimator
from .gait_constants import AO_CONFIG, CAN_INTERFACE
from .mi1_imu_phase_source import (
    DEFAULT_CAN_INTERFACE as DEFAULT_MI1_IMU_PHASE_CAN_INTERFACE,
    DEFAULT_EXPECTED_RATE_HZ as DEFAULT_MI1_IMU_PHASE_RATE_HZ,
    WiredMi1CanImuPhaseSource,
    format_node_id as format_mi1_node_id,
)
from .motion_mode_defaults import get_default_motion_modes
from .phase_bias_fitting import phase_bias_with_smoothing
from .sen0694_imu_phase_source import (
    DEFAULT_I2C_ADAPTER_NAME as DEFAULT_IMU_PHASE_I2C_ADAPTER_NAME,
    DEFAULT_SEN0694_RATE_HZ,
    WiredSen0694ImuPhaseSource,
    format_i2c_addr,
)
from .wired_imu_phase_source import (
    DEFAULT_CAN_INTERFACE as DEFAULT_IMU_PHASE_CAN_INTERFACE,
    WiredCanImuPhaseSource,
    format_can_id,
)
from .test_mode_phase_helper import (
    TEST_MODE_PHASE_MAX_CYCLE_SEC,
    TEST_MODE_PHASE_MIN_CYCLE_SEC,
    TEST_MODE_PEAK_SLOPE_EPS,
    TestModePhaseTracker,
)

CYCLING_TOGGLE_MODES = ("cycling", "uphill")
TEST_PEAK_PHASE_MODES = ("test", "walking_test")
DIFF_PEAK_PHASE_MODES = ("walking_diff_test",)
MOTOR_PEAK_PHASE_MODES = (*TEST_PEAK_PHASE_MODES, *DIFF_PEAK_PHASE_MODES)
IMU_AO_PHASE_MODES = ("imu_left_ao_phase", "imu_ao_phase")
DUAL_IMU_PHASE_MODES = ("imu_phase", "imu_ao_phase")
SINGLE_IMU_PHASE_MODES = ("imu_left_phase", "imu_left_ao_phase")
MODEL_PHASE_MODES = ("model_phase",)
IMU_PHASE_MODES = (*DUAL_IMU_PHASE_MODES, *SINGLE_IMU_PHASE_MODES, *MODEL_PHASE_MODES)
STAIRS_DOWN_MANUAL_MODES = ("stairs_down", *MOTOR_PEAK_PHASE_MODES, *IMU_PHASE_MODES)
DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG = 25.0
DEFAULT_IMU_PHASE_LEFT_MAC = "D4:22:CD:00:84:61"
DEFAULT_IMU_PHASE_RIGHT_MAC = "D4:22:CD:00:83:98"
DEFAULT_WALKING_IMU_MAC = "D4:22:CD:00:8A:5A"
DEFAULT_CYCLING_SLOT_IMU_MAC = "D4:22:CD:00:8A:5B"
SEN0694_IMU_PHASE_SOURCE_NAMES = (
    "sen0694",
    "sen0694_i2c",
    "i2c",
    "wired_i2c",
    "dfrobot",
    "dfrobot_sen0694",
)
CAN_IMU_PHASE_SOURCE_NAMES = ("can", "wired_can", "socketcan")
MI1_IMU_PHASE_SOURCE_NAMES = (
    "mi1",
    "mi1_can",
    "molex_mi1",
    "hipnuc",
    "hipnuc_can",
    "wired",
    *CAN_IMU_PHASE_SOURCE_NAMES,
)
LEGACY_CAN_IMU_PHASE_SOURCE_NAMES = ("legacy_can", "raw_can", "wired_can_legacy")
WIRED_IMU_PHASE_SOURCE_NAMES = (
    *SEN0694_IMU_PHASE_SOURCE_NAMES,
    *MI1_IMU_PHASE_SOURCE_NAMES,
    *LEGACY_CAN_IMU_PHASE_SOURCE_NAMES,
)
DEFAULT_IMU_PHASE_SOURCE_KIND = "mi1_can"
VALID_IMU_PHASE_AXES = {"x": 0, "y": 1, "z": 2, "0": 0, "1": 1, "2": 2}
VALID_IMU_PHASE_ANGLE_SOURCES = {"acc", "quat_x", "quat_y", "quat_z"}
DEFAULT_EVENT_PROB_THRESHOLD = 0.8

# 相位输入/停止态预览门控（便于集中调参）
PHASE_PREVIEW_WINDOW_SEC = 0.5       # 门控RMS统计窗口时长（秒）
PHASE_PREVIEW_START_ANGLE_RMS = 0.05 # 门控开启角度RMS阈值（rad）
PHASE_PREVIEW_STOP_ANGLE_RMS = 0.03  # 门控关闭角度RMS阈值（rad，滞回）
PHASE_PREVIEW_START_DQ_RMS = 0.15    # 门控开启角速度差RMS阈值（rad/s）
PHASE_PREVIEW_STOP_DQ_RMS = 0.08     # 门控关闭角速度差RMS阈值（rad/s，滞回）
PHASE_INPUT_DEADZONE_ANGLE = 0.1     # 相位输入角度死区：|angle_diff|<阈值时置0（rad）
MOTOR_ANGLE_LPF_CUTOFF_HZ = 4.0      # 所有电机编码器角度一阶低通截止频率
IMU_PHASE_ANGLE_LPF_CUTOFF_HZ = 6.0  # 大腿矢状面角度/角速度一阶低通截止频率
DIFF_TEST_MOTOR_ANGLE_LPF_CUTOFF_HZ = 4.0  # 差分测试模式电机角度一阶低通截止频率
IMU_PHASE_DISPLAY_MAX_ANGLE_JUMP_DEG = 45.0  # 仅用于APP/日志显示的角度跳变保持阈值
IMU_PHASE_DISPLAY_MAX_GYRO_JUMP_DEG_S = 900.0  # 仅用于APP/日志显示的角速度跳变保持阈值

# 停止->运动切换时的起步初始相位参数（便于现场快速调参）
START_PHASE_INIT_LEFT_FIRST = 0.0    # 左脚先迈时，左腿相位初始值（rad）
START_PHASE_INIT_RIGHT_FIRST = np.pi # 右脚先迈时，左腿相位初始值（rad）
START_LEG_DIFF_THRESHOLD_RAD = 0.05  # 左-右电机角度差相对停止基线变化超过该值时判定先迈脚
START_LEG_DIFF_THRESHOLD_DEG = 3.0   # 左-右IMU角度差相对停止基线变化超过该值时判定先迈脚
START_LEG_DIFF_GYRO_THRESHOLD_DEG_S = 15.0  # IMU差分角速度超过该值时也可判定先迈脚


def _read_env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except Exception:
        return float(default)


def _env_enabled(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return raw.strip().lower() not in ("0", "false", "no", "off")


def _read_env_axis(name: str, default: str) -> str:
    raw = os.environ.get(name)
    cleaned = str(raw if raw is not None else default).strip().lower()
    if cleaned not in VALID_IMU_PHASE_AXES:
        return str(default).strip().lower()
    return cleaned


def _read_env_angle_source(name: str, default: str) -> str:
    raw = os.environ.get(name)
    cleaned = str(raw if raw is not None else default).strip().lower()
    if cleaned not in VALID_IMU_PHASE_ANGLE_SOURCES:
        return str(default).strip().lower()
    return cleaned


class RealTimeGaitAnalysis(Node):
    """实时步态分析节点，支持多种运动模式和参数调整"""
    
    def __init__(self, dt=0.033, buffer_size=100, debug_mode=False):  # 根据真实30Hz采样频率设置
        super().__init__('gait_analysis_node')
        self.dt = dt                  # 固定采样周期（秒）约30Hz
        self.base_dt = dt             # 记录原始设定的传感器采样周期
        self.base_dt_locked = False   # 是否已依据真实数据校准采样周期
        self.buffer_size = buffer_size
        self.debug_mode = debug_mode  # 调试模式开关（默认关闭以提升性能）
        self.log_counter = 0  # 日志计数器，用于降频输出
        self.motor_controller_ref = None  # 对MotorController的引用，将在主函数中设置
        
        self.rhip_buffer = deque(maxlen=buffer_size)  # 右髋的角度缓存
        self.lhip_buffer = deque(maxlen=buffer_size)  # 左髋的角度缓存
        self.dq_buffer = deque(maxlen=buffer_size)    # 左右关节的相对角速度缓存
        self.gflag = deque([0], maxlen=buffer_size)   # 步态标志缓存
        self.footp = {'p': deque(maxlen=buffer_size), 'pha': deque(maxlen=buffer_size)}  # 步态相位和标志
        
        # 分别存储左右电机的助力力矩
        self.left_hipAss = deque(maxlen=buffer_size)   # 左侧电机助力力矩缓存
        self.right_hipAss = deque(maxlen=buffer_size)  # 右侧电机助力力矩缓存
        # 保持兼容性
        self.hipAss = deque(maxlen=buffer_size)        # 助力力矩缓存（保持向后兼容）

        # 当前运动模式
        self.current_motion_mode = "walking"  # 默认为行走模式
        
        # 定义不同运动模式的五次多项式助力曲线参数
        # 统一从 motion_mode_defaults 中加载，方便全局修改默认值
        self.motion_modes = get_default_motion_modes()
        self.dynamic_phase_bias = self.motion_modes.get("walking", {}).get("phase_bias", 0.0)
        self.phase_bias_pub = self.create_publisher(Float32, "/phase_bias/adjusted", 10)
        
        # 自适应振荡器（与 analyze_motor_angles.py 对齐）
        self.ao_config = AO_CONFIG.copy()
        self.init_frequency = 1.0  # Hz，提高初始频率估计
        self.adaptive_oscillator = AdaptiveOscillatorEstimator(self.dt, self.ao_config)
        self.imu_phase_left_adaptive_oscillator = AdaptiveOscillatorEstimator(
            self.dt,
            self.ao_config.copy(),
        )
        self.imu_phase_right_adaptive_oscillator = AdaptiveOscillatorEstimator(
            self.dt,
            self.ao_config.copy(),
        )
        self.imu_phase_diff_adaptive_oscillator = AdaptiveOscillatorEstimator(
            self.dt,
            self.ao_config.copy(),
        )
        self.imu_phase_left_ao_last_time = None
        self.imu_phase_right_ao_last_time = None
        self.imu_phase_diff_ao_last_time = None
        self.imu_phase_left_ao_zero_event_count = 0
        self.imu_phase_right_ao_zero_event_count = 0
        self.imu_phase_diff_ao_zero_event_count = 0
        self.phase_active = False
        self.phi_L = 0.0
        self.current_left_phase = 0.0
        self.current_right_phase = np.pi
        self._phase_unwrap_prev = None
        self.phase_rate_freq = None
        self._phase_rate_prev_time = None
        self.phase_peak_freq = None
        self._phase_peak_prev_phase = None
        self._phase_peak_prev_time = None
        self._bias_cycle_prev_phase = None
        self._bias_cycle_update_done = False

        # 停止态相位预览门控（冻结输出 + 滞回）
        preview_window_size = max(5, int(round(PHASE_PREVIEW_WINDOW_SEC / max(self.dt, 1e-3))))
        self.phase_preview_gate_open = False
        self.phase_preview_gate_prev = False
        self.phase_preview_angle_sq = deque(maxlen=preview_window_size)
        self.phase_preview_dq_sq = deque(maxlen=preview_window_size)
        self.phase_preview_start_angle_rms = PHASE_PREVIEW_START_ANGLE_RMS  # rad
        self.phase_preview_stop_angle_rms = PHASE_PREVIEW_STOP_ANGLE_RMS   # rad
        self.phase_preview_start_dq_rms = PHASE_PREVIEW_START_DQ_RMS     # rad/s
        self.phase_preview_stop_dq_rms = PHASE_PREVIEW_STOP_DQ_RMS      # rad/s
        self.phase_input_deadzone_angle = PHASE_INPUT_DEADZONE_ANGLE  # rad, |angle_diff|小于该值视为0
        
        # 人体步频检测和自适应同步参数
        self.step_frequency_history = deque(maxlen=20)  # 步频历史记录，保存最近20个周期
        self.last_step_detection_time = time.time()  # 上次步频检测时间
        self.estimated_human_frequency = 1.0  # 估计的人体步频 (Hz)
        self.frequency_sync_gain = 0.05  # 频率同步增益（较小值确保平滑调整）
        self.min_sync_samples = 5  # 最少样本数才开始同步
        self.frequency_update_interval = 0.5  # 频率更新间隔（秒）
        self.last_frequency_update_time = time.time()
        
        # 步态周期检测变量
        self.last_peak_time_L = 0.0  # 左腿上次峰值时间
        self.last_peak_time_R = 0.0  # 右腿上次峰值时间
        self.peak_threshold = 0.5  # 峰值检测阈值 (rad)
        self.last_angle_L = 0.0  # 上次左腿角度
        self.last_angle_R = 0.0  # 上次右腿角度
        self.peak_state_L = 'rising'  # 左腿峰值检测状态 ('rising', 'falling')
        self.peak_state_R = 'rising'  # 右腿峰值检测状态

        # test模式：不使用AO，基于左右髋角峰值独立估计助力时序
        self.test_mode_phase_min_cycle = TEST_MODE_PHASE_MIN_CYCLE_SEC
        self.test_mode_phase_max_cycle = TEST_MODE_PHASE_MAX_CYCLE_SEC
        self.test_mode_peak_slope_eps = TEST_MODE_PEAK_SLOPE_EPS
        self.test_mode_phase_tracker = TestModePhaseTracker(
            min_cycle_sec=self.test_mode_phase_min_cycle,
            max_cycle_sec=self.test_mode_phase_max_cycle,
            peak_slope_eps=self.test_mode_peak_slope_eps,
            peak_threshold=self.peak_threshold,
            estimated_human_frequency=self.estimated_human_frequency,
        )
        self.test_mode_left_phase_valid = False
        self.test_mode_right_phase_valid = False
        self.test_mode_left_assist_ready = False
        self.test_mode_right_assist_ready = False
        self.motor_angle_lpf_cutoff_hz = max(
            0.0,
            _read_env_float("GAIT_MOTOR_ANGLE_LPF_CUTOFF_HZ", MOTOR_ANGLE_LPF_CUTOFF_HZ),
        )
        self.left_motor_angle_filtered = None
        self.right_motor_angle_filtered = None
        self.current_left_motor_angle_raw = 0.0
        self.current_right_motor_angle_raw = 0.0
        self.current_left_motor_angle = 0.0
        self.current_right_motor_angle = 0.0
        self.diff_test_motor_angle_lpf_cutoff_hz = max(
            0.0,
            _read_env_float(
                "GAIT_DIFF_TEST_MOTOR_ANGLE_LPF_CUTOFF_HZ",
                DIFF_TEST_MOTOR_ANGLE_LPF_CUTOFF_HZ,
            ),
        )
        self.diff_test_left_motor_angle_filtered = None
        self.diff_test_right_motor_angle_filtered = None

        # IMU相位模式：默认使用 MI1 CAN 输出作为左/右大腿相位源。
        self.imu_phase_source_kind = str(
            os.environ.get("GAIT_IMU_PHASE_SOURCE", DEFAULT_IMU_PHASE_SOURCE_KIND)
        ).strip().lower()
        self.imu_phase_uses_sen0694 = (
            self.imu_phase_source_kind in SEN0694_IMU_PHASE_SOURCE_NAMES
        )
        self.imu_phase_uses_mi1 = self.imu_phase_source_kind in MI1_IMU_PHASE_SOURCE_NAMES
        self.imu_phase_uses_legacy_can = (
            self.imu_phase_source_kind in LEGACY_CAN_IMU_PHASE_SOURCE_NAMES
        )
        self.imu_phase_uses_can = bool(
            self.imu_phase_uses_mi1 or self.imu_phase_uses_legacy_can
        )
        self.imu_phase_is_wired = self.imu_phase_source_kind in WIRED_IMU_PHASE_SOURCE_NAMES
        if self.imu_phase_uses_sen0694:
            default_phase_angle_sign = 1.0
            default_phase_gyro_sign = 1.0
            default_angle_source = "acc"
            default_angle_numerator_axis = "z"
            default_angle_denominator_axis = "y"
            default_gyro_axis = "x"
            self.imu_phase_source_label = "有线SEN0694 I2C IMU"
            self.imu_phase_source_short_label = "SEN0694 I2C"
            self.imu_phase_source_check_hint = "请检查SEN0694供电、I2C总线、0x4A/0x4B地址和i2c-gpio-h2适配器"
        elif self.imu_phase_uses_mi1:
            default_phase_angle_sign = -1.0
            default_phase_gyro_sign = 1.0
            default_angle_source = "quat_y"
            default_angle_numerator_axis = "z"
            default_angle_denominator_axis = "x"
            default_gyro_axis = "y"
            self.imu_phase_source_label = "MI1有线CAN IMU"
            self.imu_phase_source_short_label = "MI1 CAN"
            self.imu_phase_source_check_hint = "请检查MI1供电、CAN连接、终端电阻、can0状态、1Mbit/s波特率、协议和节点ID"
        elif self.imu_phase_uses_legacy_can:
            default_phase_angle_sign = -1.0
            default_phase_gyro_sign = -1.0
            default_angle_source = "acc"
            default_angle_numerator_axis = "z"
            default_angle_denominator_axis = "x"
            default_gyro_axis = "y"
            self.imu_phase_source_label = "旧有线CAN IMU"
            self.imu_phase_source_short_label = "Legacy CAN"
            self.imu_phase_source_check_hint = "请检查IMU设备供电、CAN连接、终端电阻、can0状态和帧ID配置"
        else:
            default_phase_angle_sign = 1.0
            default_phase_gyro_sign = 1.0
            default_angle_source = "acc"
            default_angle_numerator_axis = "z"
            default_angle_denominator_axis = "x"
            default_gyro_axis = "y"
            self.imu_phase_source_label = "大腿IMU"
            self.imu_phase_source_short_label = "BLE"
            self.imu_phase_source_check_hint = "请检查IMU连接"
        self.imu_phase_swing_threshold = float(
            self.motion_modes.get("imu_phase", {}).get(
                "swing_threshold",
                DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG,
            )
        )
        gyro_unit = os.environ.get("GAIT_IMU_PHASE_GYRO_UNIT", "deg")
        common_phase_offset = _read_env_float("GAIT_IMU_PHASE_OFFSET", 0.0)
        common_angle_source = _read_env_angle_source(
            "GAIT_IMU_PHASE_ANGLE_SOURCE",
            default_angle_source,
        )
        common_angle_numerator_axis = _read_env_axis(
            "GAIT_IMU_PHASE_ACC_ANGLE_NUM_AXIS",
            default_angle_numerator_axis,
        )
        common_angle_denominator_axis = _read_env_axis(
            "GAIT_IMU_PHASE_ACC_ANGLE_DEN_AXIS",
            default_angle_denominator_axis,
        )
        common_gyro_axis = _read_env_axis("GAIT_IMU_PHASE_GYRO_AXIS", default_gyro_axis)
        common_imu_phase_angle_lpf_cutoff_hz = max(
            0.0,
            _read_env_float(
                "GAIT_IMU_PHASE_ANGLE_LPF_CUTOFF_HZ",
                IMU_PHASE_ANGLE_LPF_CUTOFF_HZ,
            ),
        )
        imu_phase_motion_timeout_sec = max(
            0.05,
            _read_env_float("GAIT_IMU_PHASE_MOTION_TIMEOUT_SEC", 0.45),
        )
        imu_phase_motion_window_sec = max(
            0.05,
            _read_env_float("GAIT_IMU_PHASE_MOTION_WINDOW_SEC", 0.5),
        )
        imu_phase_end_stop_threshold = max(
            0.0,
            min(1.0, _read_env_float("GAIT_IMU_PHASE_END_STOP_THRESHOLD", 0.98)),
        )
        imu_phase_motion_swing_range_ratio = max(
            0.0,
            min(1.0, _read_env_float("GAIT_IMU_PHASE_MOTION_SWING_RANGE_RATIO", 0.25)),
        )
        imu_phase_reject_abnormal_samples = _env_enabled(
            "GAIT_IMU_PHASE_REJECT_ABNORMAL_SAMPLES",
            False,
        )
        imu_phase_max_abs_angle_deg = max(
            0.0,
            _read_env_float("GAIT_IMU_PHASE_MAX_ABS_ANGLE_DEG", 170.0),
        )
        imu_phase_max_abs_gyro_deg_s = max(
            0.0,
            _read_env_float("GAIT_IMU_PHASE_MAX_ABS_GYRO_DEG_S", 600.0),
        )
        imu_phase_max_angle_jump_deg = max(
            0.0,
            _read_env_float("GAIT_IMU_PHASE_MAX_ANGLE_JUMP_DEG", 30.0),
        )
        imu_phase_max_gyro_jump_deg_s = max(
            0.0,
            _read_env_float("GAIT_IMU_PHASE_MAX_GYRO_JUMP_DEG_S", 600.0),
        )
        imu_phase_max_phase_rate_hz = max(
            0.0,
            _read_env_float("GAIT_IMU_PHASE_MAX_PHASE_RATE_HZ", 0.0),
        )
        self.imu_phase_left_estimator = ThighImuPhaseEstimator(
            ImuPhaseConfig(
                angle_sign=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_ANGLE_SIGN",
                    default_phase_angle_sign,
                ),
                gyro_sign=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_GYRO_SIGN",
                    default_phase_gyro_sign,
                ),
                gyro_unit=gyro_unit,
                angle_source=_read_env_angle_source(
                    "GAIT_IMU_PHASE_LEFT_ANGLE_SOURCE",
                    common_angle_source,
                ),
                acc_angle_numerator_axis=_read_env_axis(
                    "GAIT_IMU_PHASE_LEFT_ACC_ANGLE_NUM_AXIS",
                    common_angle_numerator_axis,
                ),
                acc_angle_denominator_axis=_read_env_axis(
                    "GAIT_IMU_PHASE_LEFT_ACC_ANGLE_DEN_AXIS",
                    common_angle_denominator_axis,
                ),
                gyro_axis=_read_env_axis("GAIT_IMU_PHASE_LEFT_GYRO_AXIS", common_gyro_axis),
                lowpass_cutoff_hz=max(
                    0.0,
                    _read_env_float(
                        "GAIT_IMU_PHASE_LEFT_ANGLE_LPF_CUTOFF_HZ",
                        common_imu_phase_angle_lpf_cutoff_hz,
                    ),
                ),
                phase_offset=_read_env_float("GAIT_IMU_PHASE_LEFT_OFFSET", common_phase_offset),
                min_swing_range_deg=self.imu_phase_swing_threshold,
                motion_timeout_sec=imu_phase_motion_timeout_sec,
                motion_window_sec=imu_phase_motion_window_sec,
                end_phase_stop_threshold=imu_phase_end_stop_threshold,
                motion_swing_range_ratio=imu_phase_motion_swing_range_ratio,
                reject_abnormal_samples=imu_phase_reject_abnormal_samples,
                max_abs_angle_deg=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_MAX_ABS_ANGLE_DEG",
                    imu_phase_max_abs_angle_deg,
                ),
                max_abs_gyro_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_MAX_ABS_GYRO_DEG_S",
                    imu_phase_max_abs_gyro_deg_s,
                ),
                max_angle_jump_deg=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_MAX_ANGLE_JUMP_DEG",
                    imu_phase_max_angle_jump_deg,
                ),
                max_gyro_jump_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_LEFT_MAX_GYRO_JUMP_DEG_S",
                    imu_phase_max_gyro_jump_deg_s,
                ),
                max_phase_rate_hz=imu_phase_max_phase_rate_hz,
            )
        )
        # imu_left_phase alone uses the quaternion/vector sagittal-angle path
        # from the dual-IMU improvement note. Other modes keep the existing
        # left estimator so their phase inputs remain unchanged.
        self.imu_phase_left_quaternion_estimator = ThighImuPhaseEstimator(
            replace(
                self.imu_phase_left_estimator.config,
                angle_source="quat_sagittal",
                gyro_source="quat_sagittal",
                quaternion_thigh_axis=(1.0, 0.0, 0.0),
                quaternion_sagittal_forward_axis="x",
                quaternion_sagittal_vertical_axis="z",
                quaternion_gyro_sagittal_axis="y",
                reject_abnormal_samples=True,
                max_phase_rate_hz=0.0,
                max_abs_angle_deg=max(
                    360.0,
                    float(self.imu_phase_left_estimator.config.max_abs_angle_deg),
                ),
            )
        )
        self.imu_phase_right_estimator = ThighImuPhaseEstimator(
            ImuPhaseConfig(
                angle_sign=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_ANGLE_SIGN",
                    default_phase_angle_sign,
                ),
                gyro_sign=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_GYRO_SIGN",
                    default_phase_gyro_sign,
                ),
                gyro_unit=gyro_unit,
                angle_source=_read_env_angle_source(
                    "GAIT_IMU_PHASE_RIGHT_ANGLE_SOURCE",
                    common_angle_source,
                ),
                acc_angle_numerator_axis=_read_env_axis(
                    "GAIT_IMU_PHASE_RIGHT_ACC_ANGLE_NUM_AXIS",
                    common_angle_numerator_axis,
                ),
                acc_angle_denominator_axis=_read_env_axis(
                    "GAIT_IMU_PHASE_RIGHT_ACC_ANGLE_DEN_AXIS",
                    common_angle_denominator_axis,
                ),
                gyro_axis=_read_env_axis("GAIT_IMU_PHASE_RIGHT_GYRO_AXIS", common_gyro_axis),
                lowpass_cutoff_hz=max(
                    0.0,
                    _read_env_float(
                        "GAIT_IMU_PHASE_RIGHT_ANGLE_LPF_CUTOFF_HZ",
                        common_imu_phase_angle_lpf_cutoff_hz,
                    ),
                ),
                phase_offset=_read_env_float("GAIT_IMU_PHASE_RIGHT_OFFSET", common_phase_offset),
                min_swing_range_deg=self.imu_phase_swing_threshold,
                motion_timeout_sec=imu_phase_motion_timeout_sec,
                motion_window_sec=imu_phase_motion_window_sec,
                end_phase_stop_threshold=imu_phase_end_stop_threshold,
                motion_swing_range_ratio=imu_phase_motion_swing_range_ratio,
                reject_abnormal_samples=imu_phase_reject_abnormal_samples,
                max_abs_angle_deg=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_MAX_ABS_ANGLE_DEG",
                    imu_phase_max_abs_angle_deg,
                ),
                max_abs_gyro_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_MAX_ABS_GYRO_DEG_S",
                    imu_phase_max_abs_gyro_deg_s,
                ),
                max_angle_jump_deg=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_MAX_ANGLE_JUMP_DEG",
                    imu_phase_max_angle_jump_deg,
                ),
                max_gyro_jump_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_RIGHT_MAX_GYRO_JUMP_DEG_S",
                    imu_phase_max_gyro_jump_deg_s,
                ),
                max_phase_rate_hz=imu_phase_max_phase_rate_hz,
            )
        )
        self.imu_phase_diff_estimator = ThighImuPhaseEstimator(
            ImuPhaseConfig(
                angle_sign=1.0,
                gyro_sign=1.0,
                gyro_unit="deg",
                lowpass_cutoff_hz=max(
                    0.0,
                    _read_env_float(
                        "GAIT_IMU_PHASE_DIFF_ANGLE_LPF_CUTOFF_HZ",
                        common_imu_phase_angle_lpf_cutoff_hz,
                    ),
                ),
                phase_offset=_read_env_float(
                    "GAIT_IMU_PHASE_DIFF_OFFSET",
                    common_phase_offset,
                ),
                min_swing_range_deg=max(
                    0.0,
                    _read_env_float(
                        "GAIT_IMU_PHASE_DIFF_SWING_THRESHOLD_DEG",
                        self.imu_phase_swing_threshold,
                    ),
                ),
                motion_timeout_sec=imu_phase_motion_timeout_sec,
                motion_window_sec=imu_phase_motion_window_sec,
                end_phase_stop_threshold=imu_phase_end_stop_threshold,
                motion_swing_range_ratio=imu_phase_motion_swing_range_ratio,
                reject_abnormal_samples=imu_phase_reject_abnormal_samples,
                max_abs_angle_deg=_read_env_float(
                    "GAIT_IMU_PHASE_DIFF_MAX_ABS_ANGLE_DEG",
                    max(220.0, imu_phase_max_abs_angle_deg),
                ),
                max_abs_gyro_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_DIFF_MAX_ABS_GYRO_DEG_S",
                    imu_phase_max_abs_gyro_deg_s * 2.0,
                ),
                max_angle_jump_deg=_read_env_float(
                    "GAIT_IMU_PHASE_DIFF_MAX_ANGLE_JUMP_DEG",
                    max(60.0, imu_phase_max_angle_jump_deg * 2.0),
                ),
                max_gyro_jump_deg_s=_read_env_float(
                    "GAIT_IMU_PHASE_DIFF_MAX_GYRO_JUMP_DEG_S",
                    imu_phase_max_gyro_jump_deg_s * 2.0,
                ),
                max_phase_rate_hz=imu_phase_max_phase_rate_hz,
            )
        )
        self.imu_phase_last_left_seq = 0
        self.imu_phase_last_right_seq = 0
        self.imu_phase_last_diff_left_seq = 0
        self.imu_phase_last_diff_right_seq = 0
        self.imu_phase_left_output = None
        self.imu_phase_right_output = None
        self.imu_phase_diff_output = None
        self.imu_phase_left_sample_time = None
        self.imu_phase_right_sample_time = None
        self.imu_phase_diff_sample_time = None
        self.imu_phase_left_zero_event = False
        self.imu_phase_right_zero_event = False
        self.imu_phase_left_valid = False
        self.imu_phase_right_valid = False
        self.imu_phase_valid = False
        self.imu_phase_left_motion_active = False
        self.imu_phase_right_motion_active = False
        self.imu_phase_motion_active = False
        self.imu_phase_left_angle_deg = 0.0
        self.imu_phase_right_angle_deg = 0.0
        self.imu_phase_left_angular_velocity_deg_s = 0.0
        self.imu_phase_right_angular_velocity_deg_s = 0.0
        self.imu_phase_diff_angle_deg = 0.0
        self.imu_phase_diff_angular_velocity_deg_s = 0.0
        self.imu_phase_left_safe_angle = 0.0
        self.imu_phase_right_safe_angle = 0.0
        self.imu_phase_diff_safe_angle = 0.0
        self.imu_phase_left_safe_gyro = 0.0
        self.imu_phase_right_safe_gyro = 0.0
        self.imu_phase_diff_safe_gyro = 0.0
        self.imu_phase_left_safe_angle_initialized = False
        self.imu_phase_right_safe_angle_initialized = False
        self.imu_phase_diff_safe_angle_initialized = False
        self.imu_phase_left_safe_gyro_initialized = False
        self.imu_phase_right_safe_gyro_initialized = False
        self.imu_phase_diff_safe_gyro_initialized = False
        self.imu_phase_display_max_angle_jump_deg = max(
            0.0,
            _read_env_float(
                "GAIT_IMU_PHASE_DISPLAY_MAX_ANGLE_JUMP_DEG",
                IMU_PHASE_DISPLAY_MAX_ANGLE_JUMP_DEG,
            ),
        )
        self.imu_phase_display_max_gyro_jump_deg_s = max(
            0.0,
            _read_env_float(
                "GAIT_IMU_PHASE_DISPLAY_MAX_GYRO_JUMP_DEG_S",
                IMU_PHASE_DISPLAY_MAX_GYRO_JUMP_DEG_S,
            ),
        )
        self.imu_phase_left_swing_range_deg = 0.0
        self.imu_phase_right_swing_range_deg = 0.0
        self.imu_phase_diff_swing_range_deg = 0.0
        self.imu_phase_left_recent_swing_range_deg = 0.0
        self.imu_phase_right_recent_swing_range_deg = 0.0
        self.imu_phase_diff_recent_swing_range_deg = 0.0
        self.imu_phase_left_frequency_hz = 0.0
        self.imu_phase_right_frequency_hz = 0.0
        self.imu_phase_abnormal_sample = False
        self.imu_phase_abnormal_reason = ""

        self.model_phase_estimator = None
        self.model_phase_output = None
        self.model_phase_init_error = ""
        self.model_phase_valid = False
        self.model_phase_raw_phase = 0.0
        self.model_phase_post_phase = 0.0
        self.model_phase_stride_rate_hz = 0.0
        self.model_phase_last_left_seq = 0
        self.model_phase_last_right_seq = 0
        self.model_phase_model_path = os.environ.get(
            "GAIT_MODEL_PHASE_MODEL_PATH",
            str(DEFAULT_MODEL_PHASE_MODEL_PATH),
        )
        self.model_phase_scaler_path = os.environ.get(
            "GAIT_MODEL_PHASE_SCALER_PATH",
            str(DEFAULT_MODEL_PHASE_SCALER_PATH),
        )
        self.model_phase_sample_rate_hz = max(
            1.0,
            _read_env_float(
                "GAIT_MODEL_PHASE_SAMPLE_RATE_HZ",
                MODEL_PHASE_SAMPLE_RATE_HZ,
            ),
        )
        self.model_phase_angle_lpf_cutoff_hz = max(
            0.0,
            _read_env_float(
                "GAIT_MODEL_PHASE_ANGLE_LPF_CUTOFF_HZ",
                common_imu_phase_angle_lpf_cutoff_hz,
            ),
        )
        try:
            self.model_phase_window_size = max(
                2,
                int(os.environ.get("GAIT_MODEL_PHASE_WINDOW_N", str(MODEL_PHASE_WINDOW_N))),
            )
        except Exception:
            self.model_phase_window_size = MODEL_PHASE_WINDOW_N
        try:
            self.model_phase_estimator = RealtimeOnnxGaitPhaseEstimator(
                model_path=self.model_phase_model_path,
                scaler_path=self.model_phase_scaler_path,
                sample_rate_hz=self.model_phase_sample_rate_hz,
                window_size=self.model_phase_window_size,
                angle_lowpass_cutoff_hz=self.model_phase_angle_lpf_cutoff_hz,
            )
            self.get_logger().info(
                "模型相位估计器已就绪: "
                f"model={self.model_phase_model_path}, "
                f"scaler={self.model_phase_scaler_path}, "
                f"rate={self.model_phase_sample_rate_hz:.1f}Hz, "
                f"window={self.model_phase_window_size}, "
                f"angle_lpf={self.model_phase_angle_lpf_cutoff_hz:.2f}Hz"
            )
        except Exception as exc:
            self.model_phase_init_error = str(exc)
            self.model_phase_estimator = None
            self.get_logger().warn(f"模型相位估计器初始化失败: {exc}")
        
        # 步态开始/结束检测参数（仅使用IMU模型）
        self.timer_r = 0.0  # 仅用于调试输出
        self.r_value = 0.0
        self.gait_state = 0  # 步态状态: 0=停止, 1=行走
        self.assist_enable = False  # 助力使能标志，基于步态状态控制
        self.assist_output_active = False  # 当前周期实际是否存在非零助力输出
        self.actual_left_torque = 0.0
        self.actual_right_torque = 0.0
        self.assist_wait_next_zero = False  # 运动判定后等待下一个相位零点再启动助力
        self._assist_zero_prev_phase = None  # 零点检测用上一时刻相位
        self._start_phase_init_pending = False
        self._start_phase_init_value = START_PHASE_INIT_LEFT_FIRST
        self._start_phase_init_side = "left"
        self._motor_diff_start_baseline = None
        self._motor_diff_start_seeded = False
        self._imu_diff_start_baseline = None
        self._imu_diff_start_seeded = False
        self._imu_diff_start_phase_rad = START_PHASE_INIT_LEFT_FIRST
        self._imu_diff_start_time = None
        self.stairs_down_manual_assist_enabled = False  # 下楼梯模式手动启停状态（按钮控制）
        self.stairs_down_manual_toggle_count = 0
        self.assist_startup_delay = 2.0  # 启动延迟时间（秒），等待系统稳定
        self.startup_time = time.time()  # 记录启动时间
        self.last_process_time = None  # 用于实时估计采样间隔
        self.dt_filtered = dt  # 固定采样时间（保持与 base_dt 一致）
        # walking/cycling 的 BLE 启停模型不是默认有线 IMU 相位助力所必需的，
        # 延后到 APP 连接或真正进入对应模式时再导入，避免占用开机启动路径。
        self.imu_model_info = {}
        self.imu_model_init_error = ""
        self.imu_model_detector = None
        self.imu_model_path = os.environ.get("GAIT_IMU_MODEL_PATH", "")
        self.imu_model_mac = os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC)
        self.imu_model_info = {
            "connected": False,
            "mac_address": self.imu_model_mac,
            "predicted_label": "停止",
            "motion_probability": 0.0,
            "ready": False,
            "last_error": "",
        }
        self.get_logger().info(
            f"🧠 walking启停IMU模型延后初始化: mac={self.imu_model_mac}，"
            "等待APP连接或切换到步行模式"
        )

        self.cycling_toggle_info = {}
        self.cycling_toggle_detector = None
        self.cycling_model_path = os.environ.get("GAIT_CYCLING_IMU_MODEL_PATH", "")
        self.cycling_imu_mac = os.environ.get(
            "GAIT_CYCLING_IMU_MAC",
            DEFAULT_CYCLING_SLOT_IMU_MAC,
        )
        try:
            cycling_event_consecutive = max(
                1, int(os.environ.get("GAIT_CYCLING_EVENT_CONSECUTIVE", "2"))
            )
        except Exception:
            cycling_event_consecutive = 2
        try:
            cycling_release_consecutive = max(
                1, int(os.environ.get("GAIT_CYCLING_RELEASE_CONSECUTIVE", "2"))
            )
        except Exception:
            cycling_release_consecutive = 2
        try:
            cycling_release_hysteresis = float(
                np.clip(
                    float(os.environ.get("GAIT_CYCLING_RELEASE_HYSTERESIS", "0.10")),
                    0.0,
                    1.0,
                )
            )
        except Exception:
            cycling_release_hysteresis = 0.10
        self.cycling_event_consecutive = cycling_event_consecutive
        self.cycling_release_consecutive = cycling_release_consecutive
        self.cycling_release_hysteresis = cycling_release_hysteresis
        self.cycling_toggle_info = {
            "connected": False,
            "measuring": False,
            "ready": False,
            "stale": True,
            "mac_address": self.cycling_imu_mac,
            "predicted_label": "停止",
            "event_probability": 0.0,
            "last_error": "",
        }
        self.get_logger().info(
            f"🚴 cycling/uphill启停IMU模型延后初始化: mac={self.cycling_imu_mac}，"
            "切换到 cycling/uphill 或 APP 连接时加载"
        )

        self.imu_phase_left_info = {}
        self.imu_phase_right_info = {}
        self.imu_phase_left_detector = None
        self.imu_phase_right_detector = None
        if self.imu_phase_uses_sen0694:
            imu_phase_data_timeout = max(
                0.05,
                _read_env_float("GAIT_IMU_PHASE_DATA_TIMEOUT_SEC", 0.5),
            )
            imu_phase_i2c_rate_hz = max(
                1.0,
                _read_env_float("GAIT_IMU_PHASE_I2C_RATE_HZ", DEFAULT_SEN0694_RATE_HZ),
            )
            for side, attr_name, info_attr, label in (
                ("left", "imu_phase_left_detector", "imu_phase_left_info", "左腿"),
                ("right", "imu_phase_right_detector", "imu_phase_right_info", "右腿"),
            ):
                try:
                    detector = WiredSen0694ImuPhaseSource(
                        side=side,
                        data_timeout_sec=imu_phase_data_timeout,
                        sample_rate_hz=imu_phase_i2c_rate_hz,
                    )
                    setattr(self, attr_name, detector)
                    setattr(
                        self,
                        info_attr,
                        {
                            "connected": False,
                            "measuring": False,
                            "ready": False,
                            "stale": True,
                            "mac_address": detector.mac_address,
                            "predicted_label": f"SEN0694{label}",
                            "last_error": "",
                        },
                    )
                    adapter_hint = os.environ.get(
                        "GAIT_IMU_PHASE_I2C_ADAPTER_NAME",
                        DEFAULT_IMU_PHASE_I2C_ADAPTER_NAME,
                    )
                    self.get_logger().info(
                        f"🦵 IMU相位{label}SEN0694 I2C源已就绪(未连接): "
                        f"addr={format_i2c_addr(detector.address)}, "
                        f"rate={imu_phase_i2c_rate_hz:.1f}Hz, "
                        f"adapter={adapter_hint}"
                    )
                except Exception as exc:
                    getattr(self, info_attr).update({"last_error": str(exc)})
                    self.get_logger().warn(f"⚠️ IMU相位{label}SEN0694 I2C源初始化失败: {exc}")
                    setattr(self, attr_name, None)
        elif self.imu_phase_uses_mi1:
            imu_phase_can_interface = os.environ.get(
                "GAIT_IMU_PHASE_CAN_INTERFACE",
                DEFAULT_MI1_IMU_PHASE_CAN_INTERFACE or CAN_INTERFACE,
            ).strip() or DEFAULT_MI1_IMU_PHASE_CAN_INTERFACE or CAN_INTERFACE
            imu_phase_data_timeout = max(
                0.05,
                _read_env_float("GAIT_IMU_PHASE_DATA_TIMEOUT_SEC", 0.5),
            )
            for side, attr_name, info_attr, label in (
                ("left", "imu_phase_left_detector", "imu_phase_left_info", "左腿"),
                ("right", "imu_phase_right_detector", "imu_phase_right_info", "右腿"),
            ):
                try:
                    detector = WiredMi1CanImuPhaseSource(
                        side=side,
                        interface=imu_phase_can_interface,
                        data_timeout_sec=imu_phase_data_timeout,
                    )
                    setattr(self, attr_name, detector)
                    setattr(
                        self,
                        info_attr,
                        {
                            "connected": False,
                            "measuring": False,
                            "ready": False,
                            "stale": True,
                            "mac_address": detector.mac_address,
                            "predicted_label": f"MI1{label}",
                            "last_error": "",
                        },
                    )
                    self.get_logger().info(
                        f"🦵 IMU相位{label}MI1 CAN源已就绪(未连接): "
                        f"interface={imu_phase_can_interface}, "
                        f"node={format_mi1_node_id(detector.node_id)}, "
                        f"protocol={detector.protocol}, "
                        f"rate={float(getattr(detector, 'sample_rate_hz', DEFAULT_MI1_IMU_PHASE_RATE_HZ)):.1f}Hz, "
                        "angle=quat_y, acc=atan2(Acc_Z, Acc_X), gyro=Gyr_Y"
                    )
                except Exception as exc:
                    getattr(self, info_attr).update({"last_error": str(exc)})
                    self.get_logger().warn(f"⚠️ IMU相位{label}MI1 CAN源初始化失败: {exc}")
                    setattr(self, attr_name, None)
        elif self.imu_phase_uses_legacy_can:
            imu_phase_can_interface = os.environ.get(
                "GAIT_IMU_PHASE_CAN_INTERFACE",
                DEFAULT_IMU_PHASE_CAN_INTERFACE,
            ).strip() or DEFAULT_IMU_PHASE_CAN_INTERFACE
            imu_phase_data_timeout = max(
                0.05,
                _read_env_float("GAIT_IMU_PHASE_DATA_TIMEOUT_SEC", 0.5),
            )
            for side, attr_name, info_attr, label in (
                ("left", "imu_phase_left_detector", "imu_phase_left_info", "左腿"),
                ("right", "imu_phase_right_detector", "imu_phase_right_info", "右腿"),
            ):
                try:
                    detector = WiredCanImuPhaseSource(
                        side=side,
                        interface=imu_phase_can_interface,
                        data_timeout_sec=imu_phase_data_timeout,
                    )
                    setattr(self, attr_name, detector)
                    ids = ", ".join(
                        f"{name}={format_can_id(can_id)}"
                        for name, can_id in detector.frame_ids.items()
                    )
                    setattr(
                        self,
                        info_attr,
                        {
                            "connected": False,
                            "measuring": False,
                            "ready": False,
                            "stale": True,
                            "mac_address": detector.mac_address,
                            "predicted_label": f"旧CAN{label}",
                            "last_error": "",
                        },
                    )
                    self.get_logger().info(
                        f"🦵 IMU相位{label}旧有线CAN源已就绪(未连接): "
                        f"interface={imu_phase_can_interface}, {ids}"
                    )
                except Exception as exc:
                    getattr(self, info_attr).update({"last_error": str(exc)})
                    self.get_logger().warn(f"⚠️ IMU相位{label}旧有线CAN源初始化失败: {exc}")
                    setattr(self, attr_name, None)
        else:
            imu_phase_left_mac = os.environ.get(
                "GAIT_IMU_PHASE_LEFT_MAC",
                DEFAULT_IMU_PHASE_LEFT_MAC,
            )
            imu_phase_right_mac = os.environ.get(
                "GAIT_IMU_PHASE_RIGHT_MAC",
                DEFAULT_IMU_PHASE_RIGHT_MAC,
            )
            imu_phase_connect_timeout = max(
                1.0,
                _read_env_float("GAIT_IMU_PHASE_CONNECT_TIMEOUT_SEC", 15.0),
            )
            imu_phase_reconnect_interval = max(
                0.5,
                _read_env_float("GAIT_IMU_PHASE_RECONNECT_INTERVAL_SEC", 5.0),
            )
            imu_phase_scan_before_connect = _env_enabled(
                "GAIT_IMU_PHASE_SCAN_BEFORE_CONNECT",
                True,
            )
            from .imu_model_start_stop_detector import IMUModelStartStopDetector

            try:
                self.imu_phase_left_detector = IMUModelStartStopDetector(
                    model_path=None,
                    mac_address=imu_phase_left_mac,
                    sample_rate_hz=30.0,
                    connect_timeout_sec=imu_phase_connect_timeout,
                    reconnect_interval_sec=imu_phase_reconnect_interval,
                    require_model=False,
                    scan_before_connect=imu_phase_scan_before_connect,
                )
                init_err = str(self.imu_phase_left_detector.last_error or "").strip()
                if init_err:
                    self.imu_phase_left_info.update({"last_error": init_err})
                    self.get_logger().warn(
                        f"⚠️ IMU相位左腿IMU不可用: {self.imu_phase_left_detector.last_error}"
                    )
                    self.imu_phase_left_detector = None
                else:
                    self.get_logger().info(
                        f"🦵 IMU相位左腿IMU已就绪(未连接): mac={imu_phase_left_mac}"
                    )
            except Exception as exc:
                self.imu_phase_left_info.update({"last_error": str(exc)})
                self.get_logger().warn(f"⚠️ IMU相位左腿IMU初始化失败: {exc}")
                self.imu_phase_left_detector = None

            try:
                self.imu_phase_right_detector = IMUModelStartStopDetector(
                    model_path=None,
                    mac_address=imu_phase_right_mac,
                    sample_rate_hz=30.0,
                    connect_timeout_sec=imu_phase_connect_timeout,
                    reconnect_interval_sec=imu_phase_reconnect_interval,
                    require_model=False,
                    scan_before_connect=imu_phase_scan_before_connect,
                )
                init_err = str(self.imu_phase_right_detector.last_error or "").strip()
                if init_err:
                    self.imu_phase_right_info.update({"last_error": init_err})
                    self.get_logger().warn(
                        f"⚠️ IMU相位右腿IMU不可用: {self.imu_phase_right_detector.last_error}"
                    )
                    self.imu_phase_right_detector = None
                else:
                    self.get_logger().info(
                        f"🦵 IMU相位右腿IMU已就绪(未连接): mac={imu_phase_right_mac}"
                    )
            except Exception as exc:
                self.imu_phase_right_info.update({"last_error": str(exc)})
                self.get_logger().warn(f"⚠️ IMU相位右腿IMU初始化失败: {exc}")
                self.imu_phase_right_detector = None
        
        # 数据记录配置
        self.data_logging_enabled = True  # 启用数据记录
        self.log_file_path = ""  # 记录文件路径，稍后设置
        self.log_file = None  # CSV文件对象
        self.csv_writer = None  # CSV写入器
        self.log_interval = 1  # 记录间隔（每N次循环记录一次，1表示每次都记录）
        self.log_counter_data = 0  # 数据记录计数器
        self.log_flush_interval_rows = max(1, int(os.environ.get("GAIT_CSV_FLUSH_ROWS", "50")))
        self.log_flush_interval_sec = max(0.1, float(os.environ.get("GAIT_CSV_FLUSH_SEC", "1.0")))
        self._last_log_flush_time = time.monotonic()
        
        # 本地实时绘图已禁用：所有数据通过蓝牙下发给APP端绘图
        self.plot_enabled = False
        
        # 声明ROS2参数（在所有属性定义之后）
        self._declare_motion_mode_parameters()
        
        # 添加参数变化回调
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info("步态分析节点初始化完成，支持实时参数调整")
        print("🦾 关节助力系统已激活")
        print("可用运动模式:")
        for mode, params in self.motion_modes.items():
            print(f"  {mode}: {params['name']} - {params['description']}")
            print(f"     伸展助力: {params['ext_Tmax']:.2f} Nm ({params['ext_t0']:.2f} - {params['ext_tf']:.2f})")
            print(f"     屈曲助力: {params['flex_Tmax']:.2f} Nm ({params['flex_t0']:.2f} - {params['flex_tf']:.2f})")
            print(f"     相位偏置: {params['phase_bias']:.3f}")
        print(f"当前模式: {self.current_motion_mode} ({self.motion_modes[self.current_motion_mode]['name']})")
    
    def _declare_motion_mode_parameters(self):
        """声明所有运动模式的五次多项式助力曲线参数"""
        for mode_key, mode_info in self.motion_modes.items():
            # 伸展助力起始时刻参数 (归一化相位 0-1)
            ext_t0_param_name = f"{mode_key}.ext_t0"
            ext_t0_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的伸展助力起始时刻 (归一化相位 0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(ext_t0_param_name, mode_info['ext_t0'], ext_t0_descriptor)
            
            # 伸展助力结束时刻参数 (归一化相位 0-1)
            ext_tf_param_name = f"{mode_key}.ext_tf"
            ext_tf_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的伸展助力结束时刻 (归一化相位 0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(ext_tf_param_name, mode_info['ext_tf'], ext_tf_descriptor)
            
            # 伸展助力峰值位置参数 (0-1, 相对于t0-tf区间)
            ext_p_param_name = f"{mode_key}.ext_p"
            ext_p_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的伸展助力峰值位置 (0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(ext_p_param_name, mode_info['ext_p'], ext_p_descriptor)
            
            # 伸展助力最大力矩参数 (Nm)
            ext_Tmax_param_name = f"{mode_key}.ext_Tmax"
            ext_Tmax_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的伸展助力最大力矩 (Nm)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(ext_Tmax_param_name, mode_info['ext_Tmax'], ext_Tmax_descriptor)
            
            # 屈曲助力起始时刻参数 (归一化相位 0-1)
            flex_t0_param_name = f"{mode_key}.flex_t0"
            flex_t0_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的屈曲助力起始时刻 (归一化相位 0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(flex_t0_param_name, mode_info['flex_t0'], flex_t0_descriptor)
            
            # 屈曲助力结束时刻参数 (归一化相位 0-1)
            flex_tf_param_name = f"{mode_key}.flex_tf"
            flex_tf_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的屈曲助力结束时刻 (归一化相位 0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(flex_tf_param_name, mode_info['flex_tf'], flex_tf_descriptor)
            
            # 屈曲助力峰值位置参数 (0-1, 相对于t0-tf区间)
            flex_p_param_name = f"{mode_key}.flex_p"
            flex_p_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的屈曲助力峰值位置 (0-1)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(flex_p_param_name, mode_info['flex_p'], flex_p_descriptor)
            
            # 屈曲助力最大力矩参数 (Nm)
            flex_Tmax_param_name = f"{mode_key}.flex_Tmax"
            flex_Tmax_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的屈曲助力最大力矩 (Nm)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(flex_Tmax_param_name, mode_info['flex_Tmax'], flex_Tmax_descriptor)
            
            # 偏置相位参数 (-1到1, 正值助力提前，负值助力迟后)
            phase_bias_param_name = f"{mode_key}.phase_bias"
            phase_bias_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的偏置相位 (-1到1, 负值使助力迟后)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(phase_bias_param_name, mode_info['phase_bias'], phase_bias_descriptor)

            # 频率-相位偏置线性插值参数
            phase_bias_0p6_param_name = f"{mode_key}.phase_bias_at_0p6"
            phase_bias_0p6_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}在0.6Hz时的相位偏置",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(
                phase_bias_0p6_param_name,
                mode_info.get("phase_bias_at_0p6", 0.0),
                phase_bias_0p6_descriptor,
            )

            phase_bias_slope_param_name = f"{mode_key}.phase_bias_slope"
            phase_bias_slope_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}相位偏置线性插值斜率 (偏置/Hz)",
                type=ParameterType.PARAMETER_DOUBLE
            )
            self.declare_parameter(
                phase_bias_slope_param_name,
                mode_info.get("phase_bias_slope", 0.0),
                phase_bias_slope_descriptor,
            )

            event_prob_threshold_default = mode_info.get("event_prob_threshold")
            if event_prob_threshold_default is None:
                if mode_key == "cycling" and self.cycling_toggle_detector is not None:
                    event_prob_threshold_default = float(
                        getattr(
                            self.cycling_toggle_detector,
                            "event_prob_threshold",
                            DEFAULT_EVENT_PROB_THRESHOLD,
                        )
                    )
                else:
                    event_prob_threshold_default = DEFAULT_EVENT_PROB_THRESHOLD
            event_prob_threshold_default = float(
                np.clip(event_prob_threshold_default, 0.0, 1.0)
            )
            self.motion_modes[mode_key]["event_prob_threshold"] = event_prob_threshold_default
            if mode_key == "cycling" and self.cycling_toggle_detector is not None:
                self.cycling_toggle_detector.event_prob_threshold = event_prob_threshold_default

            event_prob_threshold_param_name = f"{mode_key}.event_prob_threshold"
            event_prob_threshold_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的启停事件概率阈值 (0到1, 仅cycling/uphill模式生效)",
                type=ParameterType.PARAMETER_DOUBLE,
            )
            self.declare_parameter(
                event_prob_threshold_param_name,
                event_prob_threshold_default,
                event_prob_threshold_descriptor,
            )

            swing_threshold_default = float(
                mode_info.get("swing_threshold", DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG)
            )
            self.motion_modes[mode_key]["swing_threshold"] = swing_threshold_default
            swing_threshold_param_name = f"{mode_key}.swing_threshold"
            swing_threshold_descriptor = ParameterDescriptor(
                description=f"{mode_info['name']}的IMU摆幅阈值 (deg，仅IMU相位模式生效)",
                type=ParameterType.PARAMETER_DOUBLE,
            )
            self.declare_parameter(
                swing_threshold_param_name,
                swing_threshold_default,
                swing_threshold_descriptor,
            )
        
        # 初始频率固定为1Hz，不再作为可调参数
    
    def init_data_logging(self, log_directory=None):
        """初始化数据记录系统"""
        if not self.data_logging_enabled:
            return
        
        try:
            # 使用独立的步态数据日志目录
            if log_directory is None:
                workspace_root = os.environ.get('GAIT_WORKSPACE', '/home/zhang/gait_control_ws')
                log_directory = os.path.join(workspace_root, "gait_logs")
            
            # 确保日志目录存在
            os.makedirs(log_directory, exist_ok=True)
            
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(log_directory, f"gait_data_{timestamp}.csv")
            
            # 创建CSV文件并写入表头
            self.log_file = open(self.log_file_path, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.log_file)
            
            # 写入CSV表头
            headers = [
                'timestamp',           # 时间戳
                'left_angle',         # 左角度，低通后 (rad)
                'right_angle',        # 右角度，低通后 (rad)
                'left_velocity',      # 左电机角速度 (rad/s)
                'right_velocity',     # 右电机角速度 (rad/s)
                'phase_left',         # 左腿相位 (rad)
                'phase_right',        # 右腿相位 (rad)
                'assist_left',        # 左侧助力 (Nm)
                'assist_right',       # 右侧助力 (Nm)
                'motion_detected',    # 运动检测状态
                'assist_enabled'      # 助力使能状态
            ]
            self.csv_writer.writerow(headers)
            self.log_file.flush()
            self._last_log_flush_time = time.monotonic()
            
            # 无论是否在调试模式，都输出日志文件路径（重要信息）
            self.get_logger().info(f"📊 步态数据记录已启用")
            self.get_logger().info(f"� 日志文件: {self.log_file_path}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 数据记录初始化失败: {e}")
            self.data_logging_enabled = False
    
    def log_data(self, left_angle, right_angle, left_velocity, right_velocity, 
                 phase_left, phase_right, assist_left, assist_right, 
                 motion_detected, assist_enabled):
        """记录一组数据到CSV文件"""
        if not self.data_logging_enabled or self.csv_writer is None:
            return
            
        # 检查记录间隔
        self.log_counter_data += 1
        if self.log_counter_data % self.log_interval != 0:
            return
            
        try:
            # 获取当前时间戳
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 毫秒精度
            
            # 写入数据行
            data_row = [
                timestamp,
                f"{left_angle:.6f}",
                f"{right_angle:.6f}",
                f"{left_velocity:.6f}",
                f"{right_velocity:.6f}",
                f"{phase_left:.6f}",
                f"{phase_right:.6f}",
                f"{assist_left:.6f}",
                f"{assist_right:.6f}",
                motion_detected,
                assist_enabled
            ]
            self.csv_writer.writerow(data_row)
            
            flush_due_by_rows = (
                self.log_counter_data % (self.log_interval * self.log_flush_interval_rows) == 0
            )
            flush_due_by_time = (
                time.monotonic() - self._last_log_flush_time >= self.log_flush_interval_sec
            )
            if flush_due_by_rows or flush_due_by_time:
                self.log_file.flush()
                self._last_log_flush_time = time.monotonic()
                
            # 每100次记录输出一次统计信息
            if self.debug_mode and self.log_counter_data % 100 == 0:
                self.get_logger().info(f"📝 已记录 {self.log_counter_data} 条数据到CSV")
                
        except Exception as e:
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ 数据记录写入失败: {e}")
    
    def close_data_logging(self):
        """关闭数据记录"""
        if self.log_file is not None:
            try:
                self.log_file.flush()
                self.log_file.close()
                # 无论是否在调试模式，都输出最终统计（重要信息）
                self.get_logger().info(f"📊 数据记录已关闭")
                self.get_logger().info(f"📁 共记录 {self.log_counter_data} 条数据")
                self.get_logger().info(f"💾 文件已保存: {self.log_file_path}")
            except Exception as e:
                self.get_logger().error(f"❌ 关闭数据记录文件失败: {e}")
            finally:
                self.log_file = None
                self.csv_writer = None

    def _ensure_walking_imu_detector(self) -> tuple[bool, str]:
        """Create the walking BLE IMU detector only when that path is used."""
        if self.imu_model_detector is not None:
            return True, ""
        imu_model_path = str(
            getattr(self, "imu_model_path", os.environ.get("GAIT_IMU_MODEL_PATH", ""))
        )
        imu_mac = str(
            getattr(
                self,
                "imu_model_mac",
                os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC),
            )
        )
        try:
            from .imu_model_start_stop_detector import IMUModelStartStopDetector

            detector = IMUModelStartStopDetector(
                model_path=imu_model_path or None,
                mac_address=imu_mac,
                sample_rate_hz=30.0,
            )
            init_err = str(detector.last_error or "").strip()
            if init_err:
                self.imu_model_init_error = init_err
                self.imu_model_info.update({"last_error": init_err, "ready": False})
                self.get_logger().warn(
                    f"⚠️ IMU模型判停不可用，将保持停止状态: {init_err}"
                )
                return False, init_err
            self.imu_model_detector = detector
            self.imu_model_init_error = ""
            self.imu_model_info.update(
                {
                    "connected": False,
                    "measuring": False,
                    "ready": False,
                    "stale": True,
                    "mac_address": imu_mac,
                    "predicted_label": "停止",
                    "motion_probability": 0.0,
                    "last_error": "",
                }
            )
            self.get_logger().info(
                f"🧠 启停检测IMU模型已就绪(未连接): "
                f"mac={imu_mac}, model={detector.model_path}"
            )
            return True, ""
        except Exception as exc:
            init_err = str(exc)
            self.imu_model_init_error = init_err
            self.imu_model_info.update({"last_error": init_err, "ready": False})
            self.get_logger().warn(f"⚠️ IMU模型判停初始化失败，将保持停止状态: {exc}")
            return False, init_err

    def _ensure_cycling_toggle_detector(self) -> tuple[bool, str]:
        """Create the cycling/uphill BLE detector only when that path is used."""
        if self.cycling_toggle_detector is not None:
            return True, ""
        cycling_model_path = str(getattr(self, "cycling_model_path", ""))
        cycling_imu_mac = str(
            getattr(
                self,
                "cycling_imu_mac",
                os.environ.get("GAIT_CYCLING_IMU_MAC", DEFAULT_CYCLING_SLOT_IMU_MAC),
            )
        )
        threshold = float(
            np.clip(
                self.motion_modes.get("cycling", {}).get(
                    "event_prob_threshold",
                    DEFAULT_EVENT_PROB_THRESHOLD,
                ),
                0.0,
                1.0,
            )
        )
        try:
            from .imu_cycling_toggle_detector import CyclingToggleIMUDetector

            detector = CyclingToggleIMUDetector(
                model_path=cycling_model_path or None,
                mac_address=cycling_imu_mac,
                sample_rate_hz=30.0,
                event_prob_threshold=threshold,
                event_consecutive=int(getattr(self, "cycling_event_consecutive", 2)),
                release_consecutive=int(getattr(self, "cycling_release_consecutive", 2)),
                release_prob_hysteresis=float(
                    getattr(self, "cycling_release_hysteresis", 0.10)
                ),
            )
            init_err = str(detector.last_error or "").strip()
            if init_err:
                self.cycling_toggle_info.update({"last_error": init_err, "ready": False})
                self.get_logger().warn(
                    f"⚠️ cycling/uphill启停模型不可用，将保持停止状态: {init_err}"
                )
                return False, init_err
            detector.reset_toggle_state(assist_enabled=False)
            self.cycling_toggle_detector = detector
            self.cycling_toggle_info.update(
                {
                    "connected": False,
                    "measuring": False,
                    "ready": False,
                    "stale": True,
                    "mac_address": cycling_imu_mac,
                    "predicted_label": "停止",
                    "event_probability": 0.0,
                    "last_error": "",
                }
            )
            self.get_logger().info(
                f"🚴 cycling/uphill启停IMU模型已就绪(未连接): "
                f"mac={cycling_imu_mac}, model={detector.model_path}"
            )
            self.get_logger().info(
                f"🚴 cycling启停稳态参数: "
                f"event_consecutive={getattr(self, 'cycling_event_consecutive', 2)}, "
                f"release_consecutive={getattr(self, 'cycling_release_consecutive', 2)}, "
                f"release_hysteresis={float(getattr(self, 'cycling_release_hysteresis', 0.10)):.2f}"
            )
            return True, ""
        except Exception as exc:
            init_err = str(exc)
            self.cycling_toggle_info.update({"last_error": init_err, "ready": False})
            self.get_logger().warn(f"⚠️ cycling/uphill启停模型初始化失败，将保持停止状态: {exc}")
            return False, init_err

    def _ensure_imu_detector_for_slot(self, slot_key: str) -> tuple[bool, str]:
        slot_key = str(slot_key).strip().lower()
        if slot_key == "walking":
            return self._ensure_walking_imu_detector()
        if slot_key == "cycling":
            return self._ensure_cycling_toggle_detector()
        detector_attr = self._imu_detector_attr_for_slot(slot_key)
        if detector_attr and getattr(self, detector_attr, None) is not None:
            return True, ""
        return False, "detector_unavailable"

    def close_imu_model_detector(self):
        """关闭所有gait侧IMU连接器。"""
        detector_specs = (
            ("imu_model_detector", "walking IMU模型判停"),
            ("cycling_toggle_detector", "cycling IMU模型判停"),
            ("imu_phase_left_detector", "IMU相位左腿IMU"),
            ("imu_phase_right_detector", "IMU相位右腿IMU"),
        )
        for attr_name, label in detector_specs:
            detector = getattr(self, attr_name, None)
            if detector is None:
                continue
            try:
                detector.stop()
                self.get_logger().info(f"📴 {label}已关闭")
            except Exception as exc:
                self.get_logger().warn(f"⚠️ 关闭{label}失败: {exc}")
            finally:
                setattr(self, attr_name, None)

    def _is_valid_mac(self, value: str) -> bool:
        parts = value.strip().split(":")
        if len(parts) != 6:
            return False
        try:
            return all(len(p) == 2 and int(p, 16) >= 0 for p in parts)
        except Exception:
            return False

    def _collect_detector_status(self, detector, fallback_info: dict, fallback_mac: str, init_err: str = ""):
        info = dict(fallback_info or {})
        if detector is not None:
            # 必须以检测器实时状态为准，不能用 fallback 的旧值覆盖连接状态。
            connected_now = bool(getattr(detector, "_connected", False))
            measuring_now = bool(getattr(detector, "_measuring", False))
            info["connected"] = connected_now
            info["measuring"] = measuring_now
            info["mac_address"] = str(getattr(detector, "mac_address", fallback_mac))
            info["last_error"] = str(getattr(detector, "last_error", "") or "")
            latest_sample = (
                detector.get_latest_sample_6d()
                if hasattr(detector, "get_latest_sample_6d")
                else None
            )
            if not connected_now or not measuring_now or latest_sample is None:
                info["ready"] = False
                info["stale"] = True
            else:
                _sample, sample_time, _seq = latest_sample
                timeout_sec = float(getattr(detector, "data_timeout_sec", 1.0))
                sample_stale = (time.time() - float(sample_time)) > max(timeout_sec, 0.1)
                info["ready"] = not sample_stale
                info["stale"] = sample_stale
        return {
            "connected": bool(info.get("connected", False)),
            "measuring": bool(info.get("measuring", False)),
            "ready": bool(info.get("ready", False)),
            "stale": bool(info.get("stale", True)),
            "mac": str(info.get("mac_address", fallback_mac)),
            "ble_adapter": str(info.get("ble_adapter", "")),
            "label": str(info.get("predicted_label", "")),
            "last_error": str(info.get("last_error", init_err or "")),
        }

    def get_imu_connection_status(self):
        walking_mac = os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC)
        cycling_mac = os.environ.get("GAIT_CYCLING_IMU_MAC", DEFAULT_CYCLING_SLOT_IMU_MAC)
        if getattr(self, "imu_phase_is_wired", False):
            left_detector = getattr(self, "imu_phase_left_detector", None)
            right_detector = getattr(self, "imu_phase_right_detector", None)
            imu_phase_left_mac = str(
                getattr(left_detector, "mac_address", "wired:imu:left") or "wired:imu:left"
            )
            imu_phase_right_mac = str(
                getattr(right_detector, "mac_address", "wired:imu:right") or "wired:imu:right"
            )
        else:
            imu_phase_left_mac = os.environ.get(
                "GAIT_IMU_PHASE_LEFT_MAC",
                DEFAULT_IMU_PHASE_LEFT_MAC,
            )
            imu_phase_right_mac = os.environ.get(
                "GAIT_IMU_PHASE_RIGHT_MAC",
                DEFAULT_IMU_PHASE_RIGHT_MAC,
            )
        walking = self._collect_detector_status(
            detector=getattr(self, "imu_model_detector", None),
            fallback_info=getattr(self, "imu_model_info", {}) or {},
            fallback_mac=walking_mac,
            init_err=getattr(self, "imu_model_init_error", ""),
        )
        cycling = self._collect_detector_status(
            detector=getattr(self, "cycling_toggle_detector", None),
            fallback_info=getattr(self, "cycling_toggle_info", {}) or {},
            fallback_mac=cycling_mac,
            init_err="",
        )
        imu_phase_left = self._collect_detector_status(
            detector=getattr(self, "imu_phase_left_detector", None),
            fallback_info=getattr(self, "imu_phase_left_info", {}) or {},
            fallback_mac=imu_phase_left_mac,
            init_err="",
        )
        imu_phase_right = self._collect_detector_status(
            detector=getattr(self, "imu_phase_right_detector", None),
            fallback_info=getattr(self, "imu_phase_right_info", {}) or {},
            fallback_mac=imu_phase_right_mac,
            init_err="",
        )
        return {
            "walking": walking,
            "cycling": cycling,
            "imu_phase_left": imu_phase_left,
            "imu_phase_right": imu_phase_right,
        }

    def set_imu_measurement_enabled(self, enabled: bool, slot: str | None = None):
        target = bool(enabled)
        slot_key = str(slot).strip().lower() if slot is not None else ""
        selected_slots = ("walking", "cycling", "imu_phase_left", "imu_phase_right")
        if slot is not None:
            if slot_key not in selected_slots:
                return {"ok": False, "reason": "invalid_slot", "slot": slot_key}
            selected_slots = (slot_key,)

        for current_slot in selected_slots:
            detector_attr = self._imu_detector_attr_for_slot(current_slot)
            detector = getattr(self, detector_attr, None)
            if detector is None and target:
                ok, reason = self._ensure_imu_detector_for_slot(current_slot)
                if not ok:
                    return {"ok": False, "reason": reason, "slot": current_slot}
                detector = getattr(self, detector_attr, None)
            if detector is None:
                continue
            try:
                detector.set_measurement_enabled(target)
            except Exception as exc:
                return {"ok": False, "reason": str(exc), "slot": current_slot}
        return {"ok": True, "enabled": target, "slot": slot_key or "all"}

    def should_measure_imu_slot(self, slot: str) -> bool:
        """Return whether a slot should stream notifications in the current motion mode."""
        slot_key = str(slot).strip().lower()
        mode = str(getattr(self, "current_motion_mode", "walking"))
        phase_keep_default = not bool(getattr(self, "imu_phase_is_wired", False))
        if mode in SINGLE_IMU_PHASE_MODES:
            return slot_key == "imu_phase_left"
        if mode in IMU_PHASE_MODES:
            return slot_key in ("imu_phase_left", "imu_phase_right")
        if (
            slot_key == "imu_phase_left"
            and _env_enabled("GAIT_IMU_PHASE_KEEP_STREAMING", phase_keep_default)
        ):
            return True
        if mode in CYCLING_TOGGLE_MODES:
            return slot_key == "cycling"
        if mode in STAIRS_DOWN_MANUAL_MODES:
            return False
        return slot_key == "walking"

    def _imu_detector_attr_for_slot(self, slot_key: str) -> str:
        return {
            "walking": "imu_model_detector",
            "cycling": "cycling_toggle_detector",
            "imu_phase_left": "imu_phase_left_detector",
            "imu_phase_right": "imu_phase_right_detector",
        }.get(str(slot_key).strip().lower(), "")

    def _default_mac_for_imu_slot(self, slot_key: str) -> str:
        slot_key = str(slot_key).strip().lower()
        if slot_key == "walking":
            return os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC).strip().upper()
        if slot_key == "cycling":
            return os.environ.get(
                "GAIT_CYCLING_IMU_MAC",
                DEFAULT_CYCLING_SLOT_IMU_MAC,
            ).strip().upper()
        if slot_key == "imu_phase_left":
            if getattr(self, "imu_phase_is_wired", False):
                return ""
            return os.environ.get(
                "GAIT_IMU_PHASE_LEFT_MAC",
                DEFAULT_IMU_PHASE_LEFT_MAC,
            ).strip().upper()
        if slot_key == "imu_phase_right":
            if getattr(self, "imu_phase_is_wired", False):
                return ""
            return os.environ.get(
                "GAIT_IMU_PHASE_RIGHT_MAC",
                DEFAULT_IMU_PHASE_RIGHT_MAC,
            ).strip().upper()
        return ""

    def control_imu_connection(self, slot: str, connect: bool, mac: str = ""):
        slot_key = str(slot).strip().lower()
        valid_slots = ("walking", "cycling", "imu_phase_left", "imu_phase_right")
        if slot_key not in valid_slots:
            return {"ok": False, "slot": slot_key, "reason": "invalid_slot"}

        detector_attr = self._imu_detector_attr_for_slot(slot_key)
        detector = getattr(self, detector_attr, None)
        if detector is None:
            if connect:
                ok, reason = self._ensure_imu_detector_for_slot(slot_key)
                if not ok:
                    return {"ok": False, "slot": slot_key, "reason": reason}
                detector = getattr(self, detector_attr, None)
            else:
                status = self.get_imu_connection_status().get(slot_key, {})
                return {
                    "ok": True,
                    "slot": slot_key,
                    "connected": False,
                    "measuring": False,
                    "ready": False,
                    "stale": True,
                    "mac": str(status.get("mac", "")),
                    "label": str(status.get("label", "")),
                    "reason": str(status.get("last_error", "")),
                }
        if detector is None:
            return {"ok": False, "slot": slot_key, "reason": "detector_unavailable"}

        normalized_mac = (mac or "").strip().upper()
        phase_slot_uses_wired = (
            slot_key in ("imu_phase_left", "imu_phase_right")
            and getattr(self, "imu_phase_is_wired", False)
        )
        if phase_slot_uses_wired:
            normalized_mac = ""
        elif slot_key in ("imu_phase_left", "imu_phase_right") or not normalized_mac:
            normalized_mac = self._default_mac_for_imu_slot(slot_key)
        if normalized_mac:
            if not self._is_valid_mac(normalized_mac):
                return {"ok": False, "slot": slot_key, "reason": "invalid_mac"}
            if str(getattr(detector, "mac_address", "")).upper() != normalized_mac:
                try:
                    detector.stop()
                except Exception:
                    pass
                detector.mac_address = normalized_mac

        try:
            if connect:
                # 连接命令只负责让主控连接 IMU。是否进入测量流由当前模式决定，
                # 不依赖电机标零状态，避免 APP 在助力启动前连接 IMU 时被置为不可测量。
                should_measure = self.should_measure_imu_slot(slot_key)
                detector.set_measurement_enabled(should_measure)
                started = bool(detector.start())
                if not started:
                    reason = str(getattr(detector, "last_error", "") or "start_failed")
                    return {"ok": False, "slot": slot_key, "reason": reason}
            else:
                detector.set_measurement_enabled(False)
                detector.stop()
                if slot_key == "cycling":
                    try:
                        detector.reset_toggle_state(assist_enabled=False)
                    except Exception:
                        pass
        except Exception as exc:
            return {"ok": False, "slot": slot_key, "reason": str(exc)}

        status = self.get_imu_connection_status().get(slot_key, {})
        return {
            "ok": True,
            "slot": slot_key,
            "connected": bool(status.get("connected", False)),
            "measuring": bool(status.get("measuring", False)),
            "ready": bool(status.get("ready", False)),
            "stale": bool(status.get("stale", True)),
            "mac": str(status.get("mac", normalized_mac)),
            "label": str(status.get("label", "")),
            "reason": str(status.get("last_error", "")),
        }

    def destroy_node(self):
        """节点销毁时同步释放IMU判停资源。"""
        self.close_imu_model_detector()
        return super().destroy_node()
    
    def init_real_time_plot(self):
        """实时绘图已禁用（保留兼容接口）。"""
        self.plot_enabled = False

    def update_plot_data(self, left_angle, right_angle, phase_left, phase_right, assist_left, assist_right):
        """实时绘图已禁用（保留兼容接口）。"""
        return

    def close_real_time_plot(self):
        """实时绘图已禁用（保留兼容接口）。"""
        self.plot_enabled = False
    
    def parameters_callback(self, params):
        """参数变化回调函数"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            param_name = param.name
            param_value = param.value
            
            # 解析参数名称 (格式: mode_key.parameter_type)
            if '.' in param_name:
                mode_key, param_type = param_name.split('.', 1)
                
                if mode_key in self.motion_modes:
                    # 五次多项式参数列表
                    if param_type in ['ext_t0', 'ext_tf', 'ext_p', 'ext_Tmax', 
                                     'flex_t0', 'flex_tf', 'flex_p', 'flex_Tmax',
                                     'phase_bias', 'phase_bias_at_0p6', 'phase_bias_slope',
                                     'event_prob_threshold', 'swing_threshold']:
                        # 验证参数值范围
                        if param_type in ['ext_t0', 'ext_tf', 'flex_t0', 'flex_tf']:
                            # 时刻参数: 归一化相位 0-1
                            if 0.0 <= param_value <= 1.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"{param_type} = {param_value:.3f}"
                                )
                            else:
                                self.get_logger().warn(
                                    f"时刻参数值超出范围 (0.0-1.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False, 
                                                         reason="时刻参数值超出允许范围 (0.0-1.0)")
                        elif param_type in ['ext_p', 'flex_p']:
                            # 峰值位置参数: 0-1
                            if 0.0 <= param_value <= 1.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"{param_type} = {param_value:.3f}"
                                )
                            else:
                                self.get_logger().warn(
                                    f"峰值位置参数值超出范围 (0.0-1.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False, 
                                                         reason="峰值位置参数值超出允许范围 (0.0-1.0)")
                        elif param_type in ['ext_Tmax', 'flex_Tmax']:
                            # 最大力矩参数: 0-17 Nm
                            if 0.0 <= param_value <= 17.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"{param_type} = {param_value:.2f} Nm"
                                )
                            else:
                                self.get_logger().warn(
                                    f"最大力矩参数值超出范围 (0.0-17.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False, 
                                                         reason="最大力矩参数值超出允许范围 (0.0-17.0 Nm)")
                        elif param_type == 'phase_bias':
                            # 偏置相位参数: -1 到 1 (负值使助力迟后，正值使助力提前)
                            if -1.0 <= param_value <= 1.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                bias_desc = "迟后" if param_value < 0 else ("提前" if param_value > 0 else "无偏置")
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"phase_bias = {param_value:.3f} ({bias_desc})"
                                )
                            else:
                                self.get_logger().warn(
                                    f"偏置相位参数值超出范围 (-1.0到1.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False, 
                                                         reason="偏置相位参数值超出允许范围 (-1.0到1.0)")
                        elif param_type == 'phase_bias_at_0p6':
                            # 0.6Hz 相位偏置参数: -1 到 1
                            if -1.0 <= param_value <= 1.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"phase_bias_at_0p6 = {param_value:.3f}"
                                )
                            else:
                                self.get_logger().warn(
                                    f"0.6Hz相位偏置超出范围 (-1.0到1.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False,
                                                         reason="0.6Hz相位偏置超出允许范围 (-1.0到1.0)")
                        elif param_type == 'phase_bias_slope':
                            # 线性插值斜率: -10 到 10 (偏置/Hz)
                            if -10.0 <= param_value <= 10.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"phase_bias_slope = {param_value:.3f}"
                                )
                            else:
                                self.get_logger().warn(
                                    f"线性插值斜率超出范围 (-10.0到10.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False,
                                                         reason="线性插值斜率超出允许范围 (-10.0到10.0)")
                        elif param_type == 'event_prob_threshold':
                            # cycling/uphill 启停事件概率阈值: 0 到 1
                            if 0.0 <= param_value <= 1.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                detector = getattr(self, "cycling_toggle_detector", None)
                                if (
                                    mode_key in CYCLING_TOGGLE_MODES
                                    and detector is not None
                                    and self.current_motion_mode == mode_key
                                ):
                                    detector.event_prob_threshold = float(param_value)
                                    self.get_logger().info(
                                        f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                        f"event_prob_threshold = {param_value:.3f} (当前模式已生效)"
                                    )
                                elif mode_key in CYCLING_TOGGLE_MODES:
                                    self.get_logger().info(
                                        f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                        f"event_prob_threshold = {param_value:.3f} (切换到该模式后生效)"
                                    )
                                else:
                                    self.get_logger().info(
                                        f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                        f"event_prob_threshold = {param_value:.3f} (当前模式不使用)"
                                    )
                            else:
                                self.get_logger().warn(
                                    f"启停事件概率阈值超出范围 (0.0到1.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False,
                                                         reason="启停事件概率阈值超出允许范围 (0.0到1.0)")
                        elif param_type == 'swing_threshold':
                            # IMU相位摆幅阈值: 0 到 90 deg
                            if 0.0 <= param_value <= 90.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                if mode_key in (*DUAL_IMU_PHASE_MODES, *SINGLE_IMU_PHASE_MODES):
                                    self.imu_phase_swing_threshold = float(param_value)
                                    self.imu_phase_left_estimator.update_swing_threshold(param_value)
                                    self.imu_phase_left_quaternion_estimator.update_swing_threshold(param_value)
                                    self.imu_phase_right_estimator.update_swing_threshold(param_value)
                                    self.imu_phase_diff_estimator.update_swing_threshold(param_value)
                                    effect = "当前模式已生效" if self.current_motion_mode == mode_key else "切换到该模式后生效"
                                elif mode_key in MODEL_PHASE_MODES:
                                    effect = "模型相位模式不使用"
                                else:
                                    effect = "当前模式不使用"
                                self.get_logger().info(
                                    f"参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"swing_threshold = {param_value:.2f} deg ({effect})"
                                )
                            else:
                                self.get_logger().warn(
                                    f"IMU摆幅阈值超出范围 (0.0到90.0 deg): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False,
                                                         reason="IMU摆幅阈值超出允许范围 (0.0到90.0 deg)")
                    else:
                        self.get_logger().warn(f"未知参数类型: {param_type}")
                        return SetParametersResult(successful=False, 
                                                 reason="未知参数类型")
                else:
                    self.get_logger().warn(f"未知运动模式: {mode_key}")
                    return SetParametersResult(successful=False, 
                                             reason="未知运动模式")
        return SetParametersResult(successful=True)
    
    def set_motion_mode(self, mode):
        """设置运动模式"""
        if mode in self.motion_modes:
            self.current_motion_mode = mode
            print(f"🔄 运动模式切换为: {mode} ({self.motion_modes[mode]['name']})")
            current_params = self.motion_modes[mode]
            print(f"   - 伸展起始相位: {current_params['ext_t0']:.2f}")
            print(f"   - 伸展结束相位: {current_params['ext_tf']:.2f}")
            print(f"   - 伸展最大力矩: {current_params['ext_Tmax']:.2f} Nm")
            print(f"   - 屈曲起始相位: {current_params['flex_t0']:.2f}")
            print(f"   - 屈曲结束相位: {current_params['flex_tf']:.2f}")
            print(f"   - 屈曲最大力矩: {current_params['flex_Tmax']:.2f} Nm")
            print(f"   - 偏置相位: {current_params['phase_bias']:.3f}")
            print(
                f"   - 启停事件阈值: "
                f"{float(current_params.get('event_prob_threshold', DEFAULT_EVENT_PROB_THRESHOLD)):.3f}"
            )
            if mode not in STAIRS_DOWN_MANUAL_MODES:
                self.stairs_down_manual_assist_enabled = False

            if mode in CYCLING_TOGGLE_MODES:
                self._ensure_imu_detector_for_slot("cycling")
                detector = getattr(self, "cycling_toggle_detector", None)
                threshold = float(
                    np.clip(
                        current_params.get("event_prob_threshold", DEFAULT_EVENT_PROB_THRESHOLD),
                        0.0,
                        1.0,
                    )
                )
                if detector is not None:
                    detector.event_prob_threshold = threshold
                    detector.reset_toggle_state(assist_enabled=False)
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self.gait_state = 0
                self.reset_phase_estimator()
                print(f"   - {mode}启停状态: 静默（等待脚部IMU识别到“启停”事件）")
                print(f"   - {mode}模型阈值: {threshold:.3f}")
                return True

            if mode in MODEL_PHASE_MODES:
                self.stairs_down_manual_assist_enabled = False
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self.gait_state = 0
                self.reset_phase_estimator()
                source_text = str(getattr(self, "imu_phase_source_label", "MI1有线CAN IMU"))
                print(f"   - 相位来源: 左右{source_text} + ONNX步态相位模型")
                print(f"   - 模型文件: {self.model_phase_model_path}")
                print(f"   - Scaler文件: {self.model_phase_scaler_path}")
                print(
                    f"   - 模型采样率/窗口: "
                    f"{self.model_phase_sample_rate_hz:.1f}Hz / {self.model_phase_window_size}点"
                )
                if self.model_phase_estimator is None:
                    print(f"   - 模型相位状态: 不可用 ({self.model_phase_init_error})")
                else:
                    print("   - 模型相位状态: 使用后处理相位计算助力")
                print(f"   - {mode}启停状态: 手动按钮控制（默认关闭）")
                return True

            if mode in IMU_PHASE_MODES:
                self.stairs_down_manual_assist_enabled = False
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self.gait_state = 0
                self.reset_phase_estimator()
                threshold = float(
                    current_params.get("swing_threshold", self.imu_phase_swing_threshold)
                )
                self.imu_phase_swing_threshold = threshold
                self.imu_phase_left_estimator.update_swing_threshold(threshold)
                self.imu_phase_left_quaternion_estimator.update_swing_threshold(threshold)
                self.imu_phase_right_estimator.update_swing_threshold(threshold)
                self.imu_phase_diff_estimator.update_swing_threshold(threshold)
                source_text = str(getattr(self, "imu_phase_source_label", "大腿IMU"))
                if mode in IMU_AO_PHASE_MODES and mode in SINGLE_IMU_PHASE_MODES:
                    print(
                        f"   - 相位来源: 左{source_text}矢状面角度/角速度 + 自适应振荡器，"
                        "右腿相位=左腿相位+pi"
                    )
                elif mode in IMU_AO_PHASE_MODES:
                    print(
                        f"   - 相位来源: 左{source_text}-右{source_text}矢状面角度/角速度差值 "
                        "+ 自适应振荡器，右腿相位=左腿相位+pi"
                    )
                elif mode in SINGLE_IMU_PHASE_MODES:
                    print(f"   - 相位来源: 左{source_text}，右腿相位=左腿相位+pi")
                else:
                    print(
                        f"   - 相位来源: 左{source_text}-右{source_text}矢状面角度/角速度差值，"
                        "右腿相位=左腿相位+pi"
                    )
                print(f"   - IMU摆幅阈值: {threshold:.2f} deg")
                print(f"   - {mode}启停状态: 手动按钮控制（默认关闭）")
                return True
            if mode in STAIRS_DOWN_MANUAL_MODES:
                self.stairs_down_manual_assist_enabled = False
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self._clear_start_phase_init()
                self.gait_state = 0
                self.reset_phase_estimator()
                print(f"   - {mode}启停状态: 手动按钮控制（默认关闭）")
                return True
            self._ensure_imu_detector_for_slot("walking")
            return True
        else:
            print(f"❌ 错误: 未知运动模式 '{mode}'")
            print(f"可用模式: {list(self.motion_modes.keys())}")
            return False

    def set_stairs_down_manual_assist(self, enabled: bool | None = None):
        """设置/切换手动启停模式的助力状态（按钮控制）。"""
        if self.current_motion_mode not in STAIRS_DOWN_MANUAL_MODES:
            return {
                "ok": False,
                "reason": "mode_not_manual_assist",
                "mode": self.current_motion_mode,
                "enabled": bool(getattr(self, "stairs_down_manual_assist_enabled", False)),
            }

        mode_name = self.motion_modes.get(self.current_motion_mode, {}).get(
            "name", self.current_motion_mode
        )
        current = bool(getattr(self, "stairs_down_manual_assist_enabled", False))
        target = (not current) if enabled is None else bool(enabled)
        changed = target != current
        self.stairs_down_manual_assist_enabled = target
        self.assist_wait_next_zero = False
        self._assist_zero_prev_phase = None

        if target:
            if self.current_motion_mode in MOTOR_PEAK_PHASE_MODES:
                self.reset_phase_estimator()
            self.gait_state = 1
            self.assist_enable = True
        else:
            self.gait_state = 0
            self.assist_enable = False
            if self.current_motion_mode in MOTOR_PEAK_PHASE_MODES:
                self.reset_phase_estimator()
            else:
                self._clear_start_phase_init()

        if changed:
            self.stairs_down_manual_toggle_count += 1
            action = "开启" if target else "关闭"
            if self.current_motion_mode in MOTOR_PEAK_PHASE_MODES and target:
                self.get_logger().info(
                    f"🪜 {mode_name}手动启停: {action}助力，先采第一周期，第二周期开始助力 "
                    f"(toggle_count={self.stairs_down_manual_toggle_count})"
                )
            else:
                self.get_logger().info(
                    f"🪜 {mode_name}手动启停: {action}助力 (toggle_count={self.stairs_down_manual_toggle_count})"
                )

        return {
            "ok": True,
            "mode": self.current_motion_mode,
            "enabled": bool(self.stairs_down_manual_assist_enabled),
            "changed": bool(changed),
            "toggle_count": int(self.stairs_down_manual_toggle_count),
        }
    
    def get_current_assist_parameters(self):
        """获取当前运动模式的五次多项式助力曲线参数"""
        mode_params = self.motion_modes[self.current_motion_mode]
        return {
            "ext_t0": mode_params["ext_t0"],
            "ext_tf": mode_params["ext_tf"],
            "ext_p": mode_params["ext_p"],
            "ext_Tmax": mode_params["ext_Tmax"],
            "flex_t0": mode_params["flex_t0"],
            "flex_tf": mode_params["flex_tf"],
            "flex_p": mode_params["flex_p"],
            "flex_Tmax": mode_params["flex_Tmax"],
            "phase_bias": mode_params["phase_bias"]
        }
    
    def _quintic_minjerk(self, s):
        """
        五次多项式（最小加加速度）函数
        参数：
            s: 归一化时间 (0-1)
        返回：
            0到1之间的平滑过渡值
        """
        return 10 * s**3 - 15 * s**4 + 6 * s**5
    
    def _quintic_window(self, phase, t0, tf, p):
        """
        五次多项式窗口函数
        参数：
            phase: 步态相位 (0-1归一化)
            t0: 窗口起始相位
            tf: 窗口结束相位
            p: 峰值位置 (0-1，在起始和结束之间的比例)
        返回：
            0到1之间的权重值
        """
        phase = np.asarray(phase)
        w = np.zeros_like(phase)
        eps = 1e-9
        
        t0 = t0 % 1.0
        tf = tf % 1.0
        p = np.clip(p, 1e-6, 1 - 1e-6)
        
        if tf >= t0:
            # 不跨越周期边界
            dur = tf - t0
            if dur <= eps:
                return w
            t_peak = t0 + p * dur
            
            # 上升阶段
            mask_r = (phase >= t0) & (phase <= t_peak)
            rise_denom = t_peak - t0
            if np.any(mask_r) and rise_denom > eps:
                s = (phase[mask_r] - t0) / rise_denom
                w[mask_r] = self._quintic_minjerk(s)
            
            # 下降阶段
            mask_f = (phase > t_peak) & (phase <= tf)
            fall_denom = tf - t_peak
            if np.any(mask_f) and fall_denom > eps:
                s2 = (phase[mask_f] - t_peak) / fall_denom
                w[mask_f] = 1 - self._quintic_minjerk(s2)
        else:
            # 跨越周期边界
            dur = (1 - t0) + tf
            if dur <= eps:
                return w
            dist = np.where(phase >= t0, phase - t0, (1 - t0) + phase)
            peak_dist = p * dur
            
            # 上升阶段
            mask_r = (dist >= 0) & (dist <= peak_dist)
            if np.any(mask_r) and peak_dist > eps:
                s = dist[mask_r] / peak_dist
                w[mask_r] = self._quintic_minjerk(s)
            
            # 下降阶段
            mask_f = (dist > peak_dist) & (dist <= dur)
            fall_denom = dur - peak_dist
            if np.any(mask_f) and fall_denom > eps:
                s2 = (dist[mask_f] - peak_dist) / fall_denom
                w[mask_f] = 1 - self._quintic_minjerk(s2)
        
        return w
    
    def _calculate_hip_assist_quintic(self, phase_normalized, params):
        """
        使用五次多项式计算完整助力力矩
        参数：
            phase_normalized: 归一化相位 (0-1)
            params: 助力参数字典
        返回：
            助力力矩值 (Nm)
        """
        # 应用相位偏置（支持负值：负值使助力迟后，正值使助力提前）
        # Python的%运算符对负数会正确处理，确保结果在[0,1)范围内
        phase = (phase_normalized + params["phase_bias"]) % 1.0
        
        # 计算伸展和屈曲窗口
        w_ext = self._quintic_window(phase, params["ext_t0"], params["ext_tf"], params["ext_p"])
        w_flex = self._quintic_window(phase, params["flex_t0"], params["flex_tf"], params["flex_p"])
        
        # 计算总力矩（伸展为正,屈曲为负）
        torque = params["ext_Tmax"] * w_ext - params["flex_Tmax"] * w_flex
        
        return torque
    
    def _compute_external_phase(self, theta, dtheta, theta_mean):
        """
        根据角度和角速度构造外部观测相位 psi。
        使用 phase portrait: atan2(dtheta, theta - theta_mean)
        """
        try:
            return math.atan2(dtheta, (theta - theta_mean) if theta_mean is not None else theta)
        except Exception:
            return 0.0

    def _wrap_to_pi(self, angle):
        """将角度环绕到 [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _wrap_to_2pi(self, angle):
        """将角度环绕到 [0, 2π)"""
        try:
            value = float(angle)
        except Exception:
            return 0.0
        if not math.isfinite(value):
            return 0.0
        return value % (2 * np.pi)

    def _set_current_phase_pair(self, left_phase, right_phase=None):
        """统一维护内部相位状态，避免 APP 或日志看到越界相位。"""
        left = self._wrap_to_2pi(left_phase)
        right_source = left + np.pi if right_phase is None else right_phase
        right = self._wrap_to_2pi(right_source)
        self.current_left_phase = left
        self.current_right_phase = right
        self.phi_L = left
        return left, right

    def _finite_or_default(self, value, default=0.0):
        try:
            result = float(value)
        except Exception:
            return float(default)
        if not math.isfinite(result):
            return float(default)
        return result

    def _imu_phase_display_limit(self, side: str, kind: str) -> float:
        estimator = getattr(self, f"imu_phase_{side}_estimator", None)
        config = getattr(estimator, "config", None)
        if kind == "gyro":
            default = 800.0 if side != "diff" else 1600.0
            return max(0.0, float(getattr(config, "max_abs_gyro_deg_s", default)))
        default = 170.0 if side != "diff" else 220.0
        return max(0.0, float(getattr(config, "max_abs_angle_deg", default)))

    def _sanitize_imu_phase_display_value(
        self,
        side: str,
        kind: str,
        value,
        *,
        hold_previous: bool = False,
    ) -> float:
        safe_attr = f"imu_phase_{side}_safe_{kind}"
        initialized_attr = f"{safe_attr}_initialized"
        fallback = self._finite_or_default(getattr(self, safe_attr, 0.0), 0.0)
        if hold_previous:
            return fallback
        result = self._finite_or_default(value, fallback)
        limit = self._imu_phase_display_limit(side, kind)
        if limit > 0.0 and abs(result) > limit:
            return fallback
        initialized = bool(getattr(self, initialized_attr, False))
        if kind == "gyro":
            jump_limit = float(
                getattr(
                    self,
                    "imu_phase_display_max_gyro_jump_deg_s",
                    IMU_PHASE_DISPLAY_MAX_GYRO_JUMP_DEG_S,
                )
            )
        else:
            jump_limit = float(
                getattr(
                    self,
                    "imu_phase_display_max_angle_jump_deg",
                    IMU_PHASE_DISPLAY_MAX_ANGLE_JUMP_DEG,
                )
            )
            if side == "diff":
                jump_limit *= 2.0
        if initialized and jump_limit > 0.0 and abs(result - fallback) > jump_limit:
            return fallback
        setattr(self, safe_attr, result)
        setattr(self, initialized_attr, True)
        return result

    def _clear_start_phase_init(self):
        """清除停止->行走起步相位初始化请求。"""
        self._start_phase_init_pending = False
        self._start_phase_init_value = START_PHASE_INIT_LEFT_FIRST
        self._start_phase_init_side = "left"

    def _prepare_start_phase_init(self, left_angle=None, right_angle=None):
        """
        根据角度差判定首迈脚，并设置停止->运动时的初始相位：
        左脚先迈 -> 0 rad；右脚先迈 -> pi rad。
        """
        if left_angle is None or right_angle is None:
            if len(self.lhip_buffer) >= 1 and len(self.rhip_buffer) >= 1:
                angle_diff = float(self.lhip_buffer[-1]) - float(self.rhip_buffer[-1])
            else:
                angle_diff = 0.0
        else:
            angle_diff = float(left_angle) - float(right_angle)

        if angle_diff >= 0.0:
            init_phase = START_PHASE_INIT_LEFT_FIRST
            start_side = "left"
        else:
            init_phase = START_PHASE_INIT_RIGHT_FIRST
            start_side = "right"

        self._start_phase_init_pending = True
        self._start_phase_init_value = init_phase
        self._start_phase_init_side = start_side
        self._phase_unwrap_prev = None
        self._phase_peak_prev_phase = None
        self._phase_peak_prev_time = None
        self._assist_zero_prev_phase = init_phase
        self._set_current_phase_pair(init_phase)

        side_cn = "左脚" if start_side == "left" else "右脚"
        self.get_logger().info(
            f"🎯 起步脚判别: angle_diff={angle_diff:.3f} rad, 先迈{side_cn}, 初始相位={init_phase:.3f} rad"
        )

    def _judge_start_leg_from_diff(
        self,
        diff_value: float,
        baseline: float,
        *,
        diff_threshold: float,
        diff_velocity: float | None = None,
        velocity_threshold: float | None = None,
    ):
        """根据左-右差值相对停止基线的变化方向判断先迈脚。"""
        delta = float(diff_value) - float(baseline)
        decision_signal = None
        if abs(delta) >= float(diff_threshold):
            decision_signal = delta
        elif (
            diff_velocity is not None
            and velocity_threshold is not None
            and abs(float(diff_velocity)) >= float(velocity_threshold)
        ):
            decision_signal = float(diff_velocity)

        if decision_signal is None:
            return None

        if decision_signal >= 0.0:
            return "left", START_PHASE_INIT_LEFT_FIRST, delta
        return "right", START_PHASE_INIT_RIGHT_FIRST, delta

    def _log_diff_start_leg_decision(self, source: str, side: str, init_phase: float, delta: float, unit: str):
        side_cn = "左脚" if side == "left" else "右脚"
        self.get_logger().info(
            f"🎯 {source}起步脚判别: delta={delta:.3f}{unit}, "
            f"先迈{side_cn}, 初始左相位={init_phase:.3f} rad"
        )

    def _get_pre_cycle_zero_start(self, params):
        """获取周期开始前的零助力区间起点；若不存在则返回 None。"""
        ext_t0 = params["ext_t0"] % 1.0
        ext_tf = params["ext_tf"] % 1.0
        flex_t0 = params["flex_t0"] % 1.0
        flex_tf = params["flex_tf"] % 1.0

        if ext_tf < ext_t0 or flex_tf < flex_t0:
            return None
        return max(ext_tf, flex_tf)
    
    def detect_step_frequency(self, left_angle, right_angle):
        """
        基于髋关节角度峰值检测人体步频
        参数：
            left_angle: 左髋角度 (rad)
            right_angle: 右髋角度 (rad)
        """
        current_time = time.time()
        
        # 左腿峰值检测
        if self._detect_peak(left_angle, self.last_angle_L, 'L'):
            if self.last_peak_time_L > 0:
                cycle_time = current_time - self.last_peak_time_L
                if 0.3 < cycle_time < 3.0:  # 合理的步态周期范围 (20-200 步/分钟)
                    step_freq = 1.0 / cycle_time
                    self.step_frequency_history.append(step_freq)
                    if self.debug_mode:
                        self.get_logger().info(f"🦵 左腿步频检测: {step_freq:.2f} Hz (周期: {cycle_time:.2f}s)")
            self.last_peak_time_L = current_time
        
        # 右腿峰值检测
        if self._detect_peak(right_angle, self.last_angle_R, 'R'):
            if self.last_peak_time_R > 0:
                cycle_time = current_time - self.last_peak_time_R
                if 0.3 < cycle_time < 3.0:  # 合理的步态周期范围
                    step_freq = 1.0 / cycle_time
                    self.step_frequency_history.append(step_freq)
                    if self.debug_mode:
                        self.get_logger().info(f"🦵 右腿步频检测: {step_freq:.2f} Hz (周期: {cycle_time:.2f}s)")
            self.last_peak_time_R = current_time
        
        # 更新上次角度
        self.last_angle_L = left_angle
        self.last_angle_R = right_angle
        
        # 更新估计的人体步频
        if len(self.step_frequency_history) >= self.min_sync_samples:
            # 使用中位数滤波，去除异常值
            frequencies = list(self.step_frequency_history)
            self.estimated_human_frequency = np.median(frequencies)

            if self.debug_mode and current_time - self.last_frequency_update_time > 2.0:
                self.get_logger().info(f"🎯 估计人体步频: {self.estimated_human_frequency:.2f} Hz")
                self.last_frequency_update_time = current_time
    
    def _detect_peak(self, current_angle, last_angle, leg_side):
        """
        检测角度峰值（用于步频计算）
        参数：
            current_angle: 当前角度
            last_angle: 上次角度
            leg_side: 'L' 或 'R'
        返回：
            bool: 是否检测到峰值
        """
        if leg_side == 'L':
            state = self.peak_state_L
        else:
            state = self.peak_state_R
        
        angle_diff = current_angle - last_angle
        
        # 检测上升到下降的转折点（正峰值）
        if state == 'rising' and angle_diff < -0.05 and current_angle > self.peak_threshold:
            if leg_side == 'L':
                self.peak_state_L = 'falling'
            else:
                self.peak_state_R = 'falling'
            return True
        
        # 检测下降到上升的转折点（负峰值）
        elif state == 'falling' and angle_diff > 0.05 and current_angle < -self.peak_threshold:
            if leg_side == 'L':
                self.peak_state_L = 'rising'
            else:
                self.peak_state_R = 'rising'
            return True
        
        return False

    def _update_phase_bias_auto(self):
        """walking/cycling 模式下，在周期开始前的零助力点更新 phase_bias。"""
        if self.current_motion_mode not in ("walking", "cycling"):
            return
        if not getattr(self, "phase_active", False):
            self._bias_cycle_prev_phase = None
            self._bias_cycle_update_done = False
            return
        phase = getattr(self, "current_left_phase", None)
        if phase is None:
            return
        prev_phase = self._bias_cycle_prev_phase
        if prev_phase is not None and phase - prev_phase < -np.pi:
            self._bias_cycle_update_done = False
        self._bias_cycle_prev_phase = phase
        if prev_phase is None or self._bias_cycle_update_done:
            return

        mode_key = self.current_motion_mode
        params = self.get_current_assist_parameters()
        zero_start = self._get_pre_cycle_zero_start(params)
        if zero_start is None:
            return
        phase_norm = (phase % (2 * np.pi)) / (2 * np.pi)
        phase_assist = (phase_norm + params["phase_bias"]) % 1.0
        if phase_assist < zero_start:
            return
        torque = self._calculate_hip_assist_quintic(phase_norm, params)
        if abs(float(torque)) > 1e-6:
            return

        freq = self._get_freq_for_bias()
        if freq is None:
            return
        prev_bias = self.motion_modes[mode_key].get("phase_bias", self.dynamic_phase_bias)
        last_freq = getattr(self, "_last_freq_for_bias", None)
        linear_bias_at_0p6 = self.motion_modes[mode_key].get("phase_bias_at_0p6")
        linear_slope = self.motion_modes[mode_key].get("phase_bias_slope")
        new_bias = phase_bias_with_smoothing(
            freq_hz=freq,
            prev_bias=prev_bias,
            dt=self.dt,
            mode_key=mode_key,
            linear_bias_at_0p6=linear_bias_at_0p6,
            linear_slope=linear_slope,
            last_freq_hz=last_freq,
        )
        if abs(new_bias - prev_bias) < 1e-4:
            self._last_freq_for_bias = freq
            self._bias_cycle_update_done = True
            return
        self._last_freq_for_bias = freq
        self._bias_cycle_update_done = True
        self.motion_modes[mode_key]["phase_bias"] = new_bias
        self.dynamic_phase_bias = new_bias
        if self.debug_mode:
            self.get_logger().info(
                f"🎚️ 相位偏置自适应: freq={freq:.2f}Hz prev={prev_bias:.3f} -> new={new_bias:.3f}"
            )
        try:
            msg = Float32()
            msg.data = float(new_bias)
            self.phase_bias_pub.publish(msg)
        except Exception:
            pass
        try:
            self.set_parameters(
                [Parameter(f"{self.current_motion_mode}.phase_bias", value=new_bias)]
            )
        except Exception:
            pass
    
    def sync_oscillator_frequency(self):
        """
        保留接口：当前振荡器实时自适应，无需额外同步
        """
        # 新的AO自动调节频率，此处保持兼容并确保频率在安全范围
        osc = getattr(self, "adaptive_oscillator", None)
        if osc is None:
            return
        try:
            osc.para[1, 1] = np.clip(
                osc.para[1, 1],
                self.ao_config["OSC_OMEGA_MIN"],
                self.ao_config["OSC_OMEGA_MAX"],
            )
        except Exception:
            return
    
    def detect_motion(self, left_velocity, right_velocity):
        """
        【已废弃】基于髋角速度检测运动状态
        该函数保留仅为兼容性，实际步态检测使用 gait_start_stop_detection()
        """
        pass

    def _gait_start_stop_detection_by_cycling_toggle(self, left_angle=None, right_angle=None):
        """cycling/uphill模式：检测到一次“启停”事件就翻转一次助力开关。"""
        detector = getattr(self, "cycling_toggle_detector", None)
        mode_key = self.current_motion_mode if self.current_motion_mode in CYCLING_TOGGLE_MODES else "cycling"
        mode_name = self.motion_modes.get(mode_key, {}).get("name", mode_key)
        if detector is None:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            self.gait_state = 0
            return False

        gait_state, state_changed, info = detector.detect()
        self.cycling_toggle_info = info
        self.imu_model_info = info
        self.gait_state = gait_state
        self.timer_r = 0.0
        self.r_value = float(info.get("event_probability", info.get("motion_probability", 0.0)))

        if self.gait_state == 0:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            if state_changed:
                # 论文策略：AO 在停止期间保持运行，不重置
                self.get_logger().info(
                    f"🛑 {mode_name}启停触发停止 - p_event={self.r_value:.3f}, "
                    f"toggle_count={info.get('toggle_count', 0)}（AO保持运行）"
                )
        else:
            if state_changed:
                # AO 在停止期间持续运行，此处仅恢复助力门控，
                # 等待下一个 phase=0 零点后再输出助力。
                self.phase_active = True
                self.assist_wait_next_zero = True
                self.assist_enable = False
                self._prepare_start_phase_init(left_angle, right_angle)
                self.get_logger().info(
                    f"🚴 {mode_name}启停触发开始 - p_event={self.r_value:.3f}, "
                    f"toggle_count={info.get('toggle_count', 0)}，等待零点后启动助力"
                )
            elif self.assist_wait_next_zero:
                self.assist_enable = False
            else:
                self.assist_enable = True

        if self.debug_mode and self.log_counter % 50 == 0:
            gait_status = "助力开" if self.gait_state == 1 else "助力关"
            conn_status = "在线" if info.get("connected", False) else "离线"
            self.get_logger().info(
                f"📊 {mode_name}启停 - 状态={gait_status}, conn={conn_status}, "
                f"label={info.get('predicted_label', '')}, p_event={self.r_value:.3f}, "
                f"latched={info.get('event_latched', False)}, toggle_count={info.get('toggle_count', 0)}"
            )
            if not info.get("connected", False) and info.get("last_error"):
                self.get_logger().warn(f"⚠️ {mode_name} IMU连接异常: {info.get('last_error')}")

        return self.assist_enable

    def _gait_start_stop_detection_by_stairs_down_manual(self):
        """手动启停模式：由按钮直接控制助力启停，不使用启停模型。"""
        mode_name = self.motion_modes.get(self.current_motion_mode, {}).get(
            "name", self.current_motion_mode
        )
        enabled = bool(getattr(self, "stairs_down_manual_assist_enabled", False))
        self.gait_state = 1 if enabled else 0
        self.assist_enable = enabled
        self.assist_wait_next_zero = False
        self.timer_r = 0.0
        self.r_value = 1.0 if enabled else 0.0
        self.imu_model_info = {
            "connected": False,
            "mac_address": "",
            "predicted_label": "手动开启" if enabled else "手动关闭",
            "motion_probability": float(self.r_value),
            "ready": True,
            "last_error": "",
            "stale": False,
            "motion_votes": 1 if enabled else 0,
            "stop_votes": 0 if enabled else 1,
        }
        if not enabled:
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()

        if self.debug_mode and self.log_counter % 50 == 0:
            status = "开启" if enabled else "关闭"
            self.get_logger().info(
                f"🪜 {mode_name}手动启停 - 助力{status}, toggle_count={self.stairs_down_manual_toggle_count}"
            )
        return self.assist_enable

    def _gait_start_stop_detection_by_imu_model(self, left_angle=None, right_angle=None):
        """优先使用左大腿IMU模型进行启停判定。"""
        detector = getattr(self, "imu_model_detector", None)
        if detector is None:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            self.gait_state = 0
            err = str(getattr(self, "imu_model_init_error", "") or "imu_detector_unavailable")
            self.imu_model_info = {
                "connected": False,
                "mac_address": os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC),
                "predicted_label": "停止",
                "motion_probability": 0.0,
                "ready": False,
                "last_error": err,
                "stale": True,
                "motion_votes": 0,
                "stop_votes": 0,
            }
            return False

        gait_state, state_changed, info = detector.detect()
        self.imu_model_info = info
        self.gait_state = gait_state
        self.timer_r = 0.0
        self.r_value = float(info.get("motion_probability", 0.0))

        if self.gait_state == 0:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            if state_changed:
                # 论文 (Li et al. TMRB-2022) 策略：停止时不重置 AO。
                # AO 在 extract_foot_phase gait_state=0 分支持续运行 step()，
                # 振幅 alpha 随输入信号衰减自然趋近 0，频率 omega 保持上一步频，
                # 恢复行走后 AO 可在 1-2 步态周期内重新锁定相位与频率。
                reason = "IMU数据超时" if info.get("stale", False) else "模型判停"
                self.get_logger().info(
                    f"🛑 {reason} - label={info.get('predicted_label', '')}, "
                    f"p_motion={self.r_value:.3f}, 停止助力（AO保持运行）"
                )
        else:
            if state_changed:
                # 【Bug修复】原来在"判启"时也调用 reset_phase_estimator()，
                # 导致 AO 在判停期间的预热(preview)成果被丢弃，重新从0开始收敛。
                # 停止期间 extract_foot_phase 仍以 gait_state=0 分支持续运行 AO.step()，
                # AO 已在跟踪当前步频信号，不应二次重置，否则相位每次"判启"都回到初始相位，
                # 表现为"判断为停止之后再次启动时相位不再正常更新/收敛很慢"。
                # 只需重置助力逻辑门控，不触碰 AO 状态。
                self.phase_active = True   # 允许 extract_foot_phase 中的行走分支激活
                self.assist_wait_next_zero = True
                self.assist_enable = False
                self._prepare_start_phase_init(left_angle, right_angle)
                self.get_logger().info(
                    f"🚶 模型判启 - label={info.get('predicted_label', '')}, "
                    f"p_motion={self.r_value:.3f}, AO保持连续，等待下一个相位零点后启动助力"
                )
            elif self.assist_wait_next_zero:
                self.assist_enable = False
            else:
                self.assist_enable = True

        if self.debug_mode and self.log_counter % 50 == 0:
            gait_status = "行走" if self.gait_state == 1 else "停止"
            conn_status = "在线" if info.get("connected", False) else "离线"
            start_gate = "等待零点" if self.assist_wait_next_zero else "已开放"
            self.get_logger().info(
                f"📊 IMU模型判停 - 状态={gait_status}, conn={conn_status}, "
                f"label={info.get('predicted_label', '')}, p_motion={self.r_value:.3f}, "
                f"votes(m/s)={info.get('motion_votes', 0)}/{info.get('stop_votes', 0)}, gate={start_gate}"
            )
            if not info.get("connected", False) and info.get("last_error"):
                self.get_logger().warn(f"⚠️ IMU连接异常: {info.get('last_error')}")

        return self.assist_enable

    def gait_start_stop_detection(self, left_angle, right_angle, left_velocity, right_velocity):
        """
        步态开始/结束检测（仅使用左大腿IMU模型）
        
        返回：
            bool: True表示应该提供助力，False表示不提供助力
        """
        # 【优先级1】机械标零检查：必须完成机械标零才能进行步态检测
        if hasattr(self, 'motor_controller_ref') and self.motor_controller_ref:
            if not self.motor_controller_ref.mechanical_zeroed:
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self.gait_state = 0
                self.timer_r = 0.0
                self.reset_phase_estimator()
                if self.debug_mode and self.log_counter % 100 == 0:
                    self.get_logger().info("🔒 机械标零未完成，步态检测暂停，助力禁用")
                return False
        
        # 【优先级2】启动延迟保护：系统启动后等待一段时间再开始判断
        if (time.time() - self.startup_time) < self.assist_startup_delay:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            self.gait_state = 0
            self.reset_phase_estimator()
            if self.debug_mode and self.log_counter % 50 == 0:
                remaining = self.assist_startup_delay - (time.time() - self.startup_time)
                self.get_logger().info(f"⏳ 启动延迟保护中，剩余 {remaining:.1f} 秒")
            return False
        
        # 【优先级3】运动确认检查：标零后需要确认运动才能启用助力
        if hasattr(self, 'motor_controller_ref') and self.motor_controller_ref:
            motor_controller = self.motor_controller_ref
            if motor_controller.mechanical_zeroed and not motor_controller.motion_confirmed_after_zero:
                motion_confirm_threshold = 0.4  # 运动确认阈值（rad/s）
                sustained_motion_time = 1.5  # 需要持续运动的时间（秒）
                if self.current_motion_mode in IMU_PHASE_MODES:
                    max_velocity = float(
                        np.deg2rad(
                            max(
                                abs(getattr(self, "imu_phase_left_angular_velocity_deg_s", 0.0)),
                                abs(getattr(self, "imu_phase_right_angular_velocity_deg_s", 0.0)),
                            )
                        )
                    )
                else:
                    max_velocity = max(abs(left_velocity), abs(right_velocity))
                
                if max_velocity >= motion_confirm_threshold:
                    current_time = time.time()
                    time_since_zero = current_time - motor_controller.zero_completion_time
                    
                    if time_since_zero >= sustained_motion_time:
                        motor_controller.motion_confirmed_after_zero = True
                        motor_controller.get_logger().info(
                            f"✅ 调零后运动状态已确认，开始启用步态检测"
                            f"（持续运动时间: {time_since_zero:.1f}秒，最大角速度: {max_velocity:.3f} rad/s）")
                    else:
                        self.assist_enable = False
                        self.assist_wait_next_zero = False
                        self._assist_zero_prev_phase = None
                        self.gait_state = 0
                        if self.debug_mode and self.log_counter % 20 == 0:
                            self.get_logger().info(
                                f"⏳ 等待运动确认 - 已持续: {time_since_zero:.1f}s/{sustained_motion_time}s，"
                                f"最大角速度: {max_velocity:.3f} rad/s")
                        return False
                else:
                    self.assist_enable = False
                    self.assist_wait_next_zero = False
                    self._assist_zero_prev_phase = None
                    self.gait_state = 0
                    if self.debug_mode and self.log_counter % 50 == 0:
                        self.get_logger().info(
                            f"⏳ 调零后等待明确运动 - 最大角速度: {max_velocity:.3f} rad/s "
                            f"< 确认阈值: {motion_confirm_threshold} rad/s")
                    return False

        if self.current_motion_mode in STAIRS_DOWN_MANUAL_MODES:
            return self._gait_start_stop_detection_by_stairs_down_manual()

        if self.current_motion_mode in CYCLING_TOGGLE_MODES:
            if getattr(self, "cycling_toggle_detector", None) is None:
                self._ensure_imu_detector_for_slot("cycling")
            if getattr(self, "cycling_toggle_detector", None) is None:
                self.assist_enable = False
                self.assist_wait_next_zero = False
                self._assist_zero_prev_phase = None
                self.gait_state = 0
                self.timer_r = 0.0
                self.r_value = 0.0
                if self.debug_mode and self.log_counter % 100 == 0:
                    mode_name = self.motion_modes.get(
                        self.current_motion_mode, {}
                    ).get("name", self.current_motion_mode)
                    self.get_logger().warn(f"⚠️ {mode_name}启停模型不可用，助力保持关闭")
                return False
            return self._gait_start_stop_detection_by_cycling_toggle(left_angle, right_angle)

        if getattr(self, "imu_model_detector", None) is None:
            self._ensure_imu_detector_for_slot("walking")
        if getattr(self, "imu_model_detector", None) is None:
            self.assist_enable = False
            self.assist_wait_next_zero = False
            self._assist_zero_prev_phase = None
            self._clear_start_phase_init()
            self.gait_state = 0
            self.timer_r = 0.0
            self.r_value = 0.0
            err = str(getattr(self, "imu_model_init_error", "") or "imu_detector_unavailable")
            self.imu_model_info = {
                "connected": False,
                "mac_address": os.environ.get("GAIT_IMU_MAC", DEFAULT_WALKING_IMU_MAC),
                "predicted_label": "停止",
                "motion_probability": 0.0,
                "ready": False,
                "last_error": err,
                "stale": True,
                "motion_votes": 0,
                "stop_votes": 0,
            }
            if self.debug_mode and self.log_counter % 100 == 0:
                self.get_logger().warn("⚠️ walking启停模型不可用，助力保持关闭")
            return False

        return self._gait_start_stop_detection_by_imu_model(left_angle, right_angle)

    def comprehensive_assist_decision(self, left_angle, right_angle, left_velocity, right_velocity):
        """
        【已废弃】综合电机角度和角速度的助力判别
        该函数保留仅为兼容性，实际步态检测使用 gait_start_stop_detection()
        
        参数：
            left_angle: 左髋角度 (rad)
            right_angle: 右髋角度 (rad)
            left_velocity: 左髋角速度 (rad/s)
            right_velocity: 右髋角速度 (rad/s)
        
        返回：
            bool: True表示应该提供助力，False表示不提供助力
        """
        # 直接调用新的步态检测函数
        return self.gait_start_stop_detection(left_angle, right_angle, left_velocity, right_velocity)

    def process_input_data(self, rhip_angle, lhip_angle, rhip_velocity, lhip_velocity, sensor_timestamp=None):
        """处理输入数据并进行步态分析"""
        # 使用传感器时间戳更新采样周期
        now = sensor_timestamp if sensor_timestamp is not None else time.time()
        if self.last_process_time is not None:
            actual_dt = now - self.last_process_time
            # 仅在启动初期进行一次性校准，期望30Hz左右（0.025-0.05s范围）
            if not self.base_dt_locked and 0.020 < actual_dt < 0.060:  # 16.7-50Hz范围
                self.base_dt = actual_dt
                self.dt_filtered = actual_dt
                self.dt = actual_dt
                self.base_dt_locked = True
                if self.debug_mode:
                    self.get_logger().info(f"dt已校准为: {self.base_dt:.4f}s ({1/self.base_dt:.1f}Hz)")
                for osc_attr in (
                    "adaptive_oscillator",
                    "imu_phase_left_adaptive_oscillator",
                    "imu_phase_right_adaptive_oscillator",
                    "imu_phase_diff_adaptive_oscillator",
                ):
                    osc = getattr(self, osc_attr, None)
                    if osc is not None:
                        osc.update_dt(self.base_dt)
        self.last_process_time = now
        # 保持后续处理使用校准后的固定 dt
        self.dt = self.base_dt
        self.dt_filtered = self.base_dt
        for osc_attr in (
            "adaptive_oscillator",
            "imu_phase_left_adaptive_oscillator",
            "imu_phase_right_adaptive_oscillator",
            "imu_phase_diff_adaptive_oscillator",
        ):
            osc = getattr(self, osc_attr, None)
            if osc is not None:
                osc.update_dt(self.base_dt)

        raw_lhip_angle = float(lhip_angle)
        raw_rhip_angle = float(rhip_angle)
        lhip_angle, rhip_angle, motor_angle_filter_alpha = self._filter_motor_angles(
            raw_lhip_angle,
            raw_rhip_angle,
        )
        self.current_motor_angle_filter_alpha = motor_angle_filter_alpha

        self.rhip_buffer.append(rhip_angle)
        self.lhip_buffer.append(lhip_angle)
        
        # 保存最近速度，用于 AO 的 psi 构造
        self.last_lhip_velocity = lhip_velocity if lhip_velocity is not None else self.last_lhip_velocity
        self.last_rhip_velocity = rhip_velocity if rhip_velocity is not None else self.last_rhip_velocity
        
        # 使用IMU模型进行步态开始/结束检测
        self.assist_enable = self.gait_start_stop_detection(
            lhip_angle, rhip_angle, lhip_velocity, rhip_velocity
        )

        # 仅在步行状态下更新步频，用于相位偏置自适应
        if self.gait_state == 1:
            self.detect_step_frequency(lhip_angle, rhip_angle)
        
        # 在调试模式下输出传入步态检测的参数
        if self.debug_mode and self.log_counter % 20 == 0:
            self.get_logger().info(
                f"🔍 步态检测输入 - 左角度: {lhip_angle:.4f} rad "
                f"(raw={raw_lhip_angle:.4f}), 右角度: {rhip_angle:.4f} rad "
                f"(raw={raw_rhip_angle:.4f}), lpf_alpha={motor_angle_filter_alpha:.3f}"
            )
            self.get_logger().info(f"🔍 步态检测输入 - 左速度: {lhip_velocity:.4f} rad/s, 右速度: {rhip_velocity:.4f} rad/s")
            self.get_logger().info(f"🎯 步态检测结果 - assist_enable: {self.assist_enable}, gait_state: {self.gait_state}")
        
        # 计算左右关节的相对角速度
        if len(self.rhip_buffer) >= 1:
            dq = lhip_velocity - rhip_velocity  # 相对角速度计算
            self.dq_buffer.append(dq)
        
        # 后续步态分析流程（现在 extract_foot_phase 使用 AO）
        self.calculate_gait_flag()
        self.extract_foot_phase()
        self.calculate_hip_torque_profile()
        if self.gait_state == 1 and getattr(self, "phase_active", False):
            self._update_phase_bias_auto()

    def calculate_gait_flag(self):
        """
        更新步态标志（仅用于记录与可视化）
        """
        current_flag = 1 if self.gait_state == 1 else 0
        self.gflag.append(current_flag)
        self.footp["pha"] = list(self.gflag)
        
        if self.debug_mode and self.log_counter % 100 == 0:
            self.get_logger().info(
                f"🚶 步态标志更新 - 状态: {current_flag}, p_motion={getattr(self, 'r_value', 0.0):.4f}"
            )

    def reset_phase_estimator(self):
        """将相位估计重置为初始未使用状态，并重置振荡器参数"""
        self.phase_active = False
        self.phi_L = 0.0
        self.current_left_phase = 0.0
        self.current_right_phase = np.pi
        self._phase_unwrap_prev = None
        self.phase_rate_freq = None
        self._phase_rate_prev_time = None
        self.phase_peak_freq = None
        self._phase_peak_prev_phase = None
        self._phase_peak_prev_time = None
        self._bias_cycle_prev_phase = None
        self._bias_cycle_update_done = False
        self._assist_zero_prev_phase = None
        self._clear_start_phase_init()
        self._motor_diff_start_baseline = None
        self._motor_diff_start_seeded = False
        self._imu_diff_start_baseline = None
        self._imu_diff_start_seeded = False
        self._imu_diff_start_phase_rad = START_PHASE_INIT_LEFT_FIRST
        self._imu_diff_start_time = None
        self.phase_preview_gate_open = False
        self.phase_preview_gate_prev = False
        if hasattr(self, "phase_preview_angle_sq"):
            self.phase_preview_angle_sq.clear()
        if hasattr(self, "phase_preview_dq_sq"):
            self.phase_preview_dq_sq.clear()
        if hasattr(self, "test_mode_phase_tracker"):
            self.test_mode_phase_tracker.reset(
                estimated_human_frequency=float(getattr(self, "estimated_human_frequency", 1.0)),
                keep_period=False,
            )
        self.test_mode_left_phase_valid = False
        self.test_mode_right_phase_valid = False
        self.test_mode_left_assist_ready = False
        self.test_mode_right_assist_ready = False
        self._reset_motor_angle_filter()
        self._reset_diff_test_motor_angle_filter()
        if hasattr(self, "imu_phase_left_estimator"):
            self.imu_phase_left_estimator.reset()
        if hasattr(self, "imu_phase_left_quaternion_estimator"):
            self.imu_phase_left_quaternion_estimator.reset()
        if hasattr(self, "imu_phase_right_estimator"):
            self.imu_phase_right_estimator.reset()
        if hasattr(self, "imu_phase_diff_estimator"):
            self.imu_phase_diff_estimator.reset()
        self.imu_phase_last_left_seq = 0
        self.imu_phase_last_right_seq = 0
        self.imu_phase_last_diff_left_seq = 0
        self.imu_phase_last_diff_right_seq = 0
        self.imu_phase_left_output = None
        self.imu_phase_right_output = None
        self.imu_phase_diff_output = None
        self.imu_phase_left_sample_time = None
        self.imu_phase_right_sample_time = None
        self.imu_phase_diff_sample_time = None
        self.imu_phase_left_zero_event = False
        self.imu_phase_right_zero_event = False
        self.imu_phase_left_valid = False
        self.imu_phase_right_valid = False
        self.imu_phase_valid = False
        self.imu_phase_left_motion_active = False
        self.imu_phase_right_motion_active = False
        self.imu_phase_motion_active = False
        self.imu_phase_left_angle_deg = 0.0
        self.imu_phase_right_angle_deg = 0.0
        self.imu_phase_left_angular_velocity_deg_s = 0.0
        self.imu_phase_right_angular_velocity_deg_s = 0.0
        self.imu_phase_diff_angle_deg = 0.0
        self.imu_phase_diff_angular_velocity_deg_s = 0.0
        self.imu_phase_left_safe_angle = 0.0
        self.imu_phase_right_safe_angle = 0.0
        self.imu_phase_diff_safe_angle = 0.0
        self.imu_phase_left_safe_gyro = 0.0
        self.imu_phase_right_safe_gyro = 0.0
        self.imu_phase_diff_safe_gyro = 0.0
        self.imu_phase_left_safe_angle_initialized = False
        self.imu_phase_right_safe_angle_initialized = False
        self.imu_phase_diff_safe_angle_initialized = False
        self.imu_phase_left_safe_gyro_initialized = False
        self.imu_phase_right_safe_gyro_initialized = False
        self.imu_phase_diff_safe_gyro_initialized = False
        self.imu_phase_left_swing_range_deg = 0.0
        self.imu_phase_right_swing_range_deg = 0.0
        self.imu_phase_diff_swing_range_deg = 0.0
        self.imu_phase_left_recent_swing_range_deg = 0.0
        self.imu_phase_right_recent_swing_range_deg = 0.0
        self.imu_phase_diff_recent_swing_range_deg = 0.0
        self.imu_phase_left_frequency_hz = 0.0
        self.imu_phase_right_frequency_hz = 0.0
        self.imu_phase_abnormal_sample = False
        self.imu_phase_abnormal_reason = ""
        self.imu_phase_left_ao_last_time = None
        self.imu_phase_right_ao_last_time = None
        self.imu_phase_diff_ao_last_time = None
        self.imu_phase_left_ao_last_phase_rad = None
        self.imu_phase_right_ao_last_phase_rad = None
        self.imu_phase_diff_ao_last_phase_rad = None
        self.imu_phase_left_ao_last_phase_time = None
        self.imu_phase_right_ao_last_phase_time = None
        self.imu_phase_diff_ao_last_phase_time = None
        self.imu_phase_left_ao_zero_event_count = 0
        self.imu_phase_right_ao_zero_event_count = 0
        self.imu_phase_diff_ao_zero_event_count = 0
        if hasattr(self, "model_phase_estimator") and self.model_phase_estimator is not None:
            self.model_phase_estimator.reset()
        self.model_phase_output = None
        self.model_phase_valid = False
        self.model_phase_raw_phase = 0.0
        self.model_phase_post_phase = 0.0
        self.model_phase_stride_rate_hz = 0.0
        self.model_phase_last_left_seq = 0
        self.model_phase_last_right_seq = 0
        self.assist_output_active = False
        self.actual_left_torque = 0.0
        self.actual_right_torque = 0.0

        # 强制重置自适应振荡器，确保频率恢复到初始设定值 (OSC_OMEGA_INIT)
        # 这可以防止振荡器在停止后保留错误的频率状态（如谐波锁定）
        if hasattr(self, "adaptive_oscillator"):
            # 不重置滤波器，保持其对信号直流分量的适应，防止启动时的瞬态冲击
            self.adaptive_oscillator.reset(reset_filter=False)
        for osc_attr in (
            "imu_phase_left_adaptive_oscillator",
            "imu_phase_right_adaptive_oscillator",
            "imu_phase_diff_adaptive_oscillator",
        ):
            osc = getattr(self, osc_attr, None)
            if osc is not None:
                osc.reset(reset_filter=True)

        # 清空缓存，回到初始状态
        if hasattr(self, "left_phase_buffer"):
            self.left_phase_buffer.clear()
        if hasattr(self, "right_phase_buffer"):
            self.right_phase_buffer.clear()
        self.footp["p"] = [0]
        if self.debug_mode:
            self.get_logger().info("🔄 相位估计器已重置到初始状态，振荡器参数已恢复默认")

    def _get_ao_frequency_hz(self):
        """获取当前自适应振荡器频率 (Hz)，内部存储为 rad/s"""
        try:
            return self.adaptive_oscillator.para[1, 1] / (2 * np.pi)
        except Exception:
            return 0.0

    def _update_phase_rate_frequency(self, phase):
        """根据相位变化率更新频率估计 (Hz)。"""
        if not getattr(self, "phase_active", False) or phase is None:
            self._phase_unwrap_prev = None
            self.phase_rate_freq = None
            self._phase_rate_prev_time = None
            return
        prev_phase = getattr(self, "_phase_unwrap_prev", None)
        if prev_phase is None:
            self._phase_unwrap_prev = phase
            self._phase_rate_prev_time = getattr(self, "last_process_time", None)
            return
        delta = phase - prev_phase
        if delta < -np.pi:
            delta += 2 * np.pi
        elif delta > np.pi:
            delta -= 2 * np.pi
        self._phase_unwrap_prev = phase
        current_time = getattr(self, "last_process_time", None)
        prev_time = getattr(self, "_phase_rate_prev_time", None)
        dt_local = None
        if current_time is not None and prev_time is not None:
            dt_local = current_time - prev_time
        if current_time is not None:
            self._phase_rate_prev_time = current_time
        dt_used = dt_local if dt_local is not None and dt_local > 0 else self.dt
        if dt_used <= 0:
            return
        freq = delta / (2 * np.pi * dt_used)
        if freq <= 0:
            return
        self.phase_rate_freq = freq

    def _update_phase_peak_frequency(self, phase):
        """根据相位峰值间隔更新频率估计 (Hz)。"""
        if not getattr(self, "phase_active", False) or phase is None:
            self._phase_peak_prev_phase = None
            self._phase_peak_prev_time = None
            self.phase_peak_freq = None
            return
        prev_phase = getattr(self, "_phase_peak_prev_phase", None)
        self._phase_peak_prev_phase = phase
        if prev_phase is None:
            self._phase_peak_prev_time = getattr(self, "last_process_time", None)
            return
        delta = phase - prev_phase
        if delta >= -np.pi:
            return
        current_time = getattr(self, "last_process_time", None)
        prev_time = getattr(self, "_phase_peak_prev_time", None)
        self._phase_peak_prev_time = current_time
        if current_time is None or prev_time is None:
            return
        period = current_time - prev_time
        if period <= 0:
            return
        self.phase_peak_freq = 1.0 / period

    def _get_phase_rate_frequency_hz(self):
        """读取相位变化率频率 (Hz)。"""
        return self.phase_rate_freq

    def _get_phase_peak_frequency_hz(self):
        """读取相位峰值频率 (Hz)。"""
        return self.phase_peak_freq

    def _get_freq_for_bias(self):
        """相位偏置优先使用RAO频率，失败时退回到相位峰值频率。"""
        freq = self._get_ao_frequency_hz()
        if freq and freq > 0:
            return freq
        freq = self._get_phase_peak_frequency_hz()
        return freq if freq and freq > 0 else None

    def _update_preview_gate(self, angle_diff, dq):
        """停止态相位预览滞回门控：返回是否允许更新 preview 相位。"""
        self.phase_preview_angle_sq.append(float(angle_diff) ** 2)
        self.phase_preview_dq_sq.append(float(dq) ** 2)

        angle_rms = math.sqrt(sum(self.phase_preview_angle_sq) / len(self.phase_preview_angle_sq))
        dq_rms = math.sqrt(sum(self.phase_preview_dq_sq) / len(self.phase_preview_dq_sq))

        if self.phase_preview_gate_open:
            if (
                angle_rms <= self.phase_preview_stop_angle_rms
                and dq_rms <= self.phase_preview_stop_dq_rms
            ):
                self.phase_preview_gate_open = False
        else:
            if (
                angle_rms >= self.phase_preview_start_angle_rms
                or dq_rms >= self.phase_preview_start_dq_rms
            ):
                self.phase_preview_gate_open = True

        if self.debug_mode and self.phase_preview_gate_open != self.phase_preview_gate_prev:
            state = "开放" if self.phase_preview_gate_open else "冻结"
            self.get_logger().info(
                f"🧊 停止态相位预览门控切换: {state} "
                f"(angle_rms={angle_rms:.4f}, dq_rms={dq_rms:.4f})"
            )
        self.phase_preview_gate_prev = self.phase_preview_gate_open
        return self.phase_preview_gate_open

    def _filter_alpha_for_cutoff(self, cutoff_hz: float) -> float:
        cutoff = max(0.0, float(cutoff_hz))
        if cutoff <= 0.0:
            return 1.0
        dt_used = max(1e-4, float(getattr(self, "base_dt", self.dt) or self.dt))
        return max(0.0, min(1.0, 1.0 - math.exp(-2.0 * math.pi * cutoff * dt_used)))

    def _reset_motor_angle_filter(self):
        """重置全局电机编码器角度低通滤波器。"""
        self.left_motor_angle_filtered = None
        self.right_motor_angle_filtered = None

    def _filter_motor_angles(self, left_angle: float, right_angle: float) -> tuple[float, float, float]:
        """对所有模式使用的左右电机编码器角度做一阶低通。"""
        left_raw = float(left_angle)
        right_raw = float(right_angle)
        self.current_left_motor_angle_raw = left_raw
        self.current_right_motor_angle_raw = right_raw

        cutoff_hz = float(getattr(self, "motor_angle_lpf_cutoff_hz", MOTOR_ANGLE_LPF_CUTOFF_HZ))
        if self.current_motion_mode in DIFF_PEAK_PHASE_MODES:
            cutoff_hz = float(
                getattr(
                    self,
                    "diff_test_motor_angle_lpf_cutoff_hz",
                    DIFF_TEST_MOTOR_ANGLE_LPF_CUTOFF_HZ,
                )
            )
        alpha = self._filter_alpha_for_cutoff(cutoff_hz)

        prev_left = getattr(self, "left_motor_angle_filtered", None)
        prev_right = getattr(self, "right_motor_angle_filtered", None)
        if prev_left is None or prev_right is None:
            filtered_left = left_raw
            filtered_right = right_raw
        else:
            filtered_left = float(prev_left) + alpha * (left_raw - float(prev_left))
            filtered_right = float(prev_right) + alpha * (right_raw - float(prev_right))

        self.left_motor_angle_filtered = filtered_left
        self.right_motor_angle_filtered = filtered_right
        self.current_left_motor_angle = filtered_left
        self.current_right_motor_angle = filtered_right
        return filtered_left, filtered_right, alpha

    def _reset_diff_test_motor_angle_filter(self):
        """保留兼容状态；差分模式现在复用全局电机角度滤波器。"""
        self.diff_test_left_motor_angle_filtered = None
        self.diff_test_right_motor_angle_filtered = None

    def _filter_diff_test_motor_angles(self, left_angle: float, right_angle: float) -> tuple[float, float, float]:
        """差分模式读取的 buffer 已经是全局滤波后的电机角度。"""
        return float(left_angle), float(right_angle), 1.0

    def _extract_diff_test_mode_phase(self):
        """差分步行测试相位：用滤波后的左-右电机角度差估计单一相位。"""
        if len(self.lhip_buffer) < 1 or len(self.rhip_buffer) < 1:
            self.reset_phase_estimator()
            return

        current_time = float(getattr(self, "last_process_time", None) or time.time())
        filtered_left_angle = float(self.lhip_buffer[-1])
        filtered_right_angle = float(self.rhip_buffer[-1])
        raw_left_angle = float(
            getattr(self, "current_left_motor_angle_raw", filtered_left_angle)
        )
        raw_right_angle = float(
            getattr(self, "current_right_motor_angle_raw", filtered_right_angle)
        )
        left_angle, right_angle, _filter_alpha = self._filter_diff_test_motor_angles(
            filtered_left_angle,
            filtered_right_angle,
        )
        filter_alpha = float(getattr(self, "current_motor_angle_filter_alpha", _filter_alpha))
        angle_diff = left_angle - right_angle

        tracker = getattr(self, "test_mode_phase_tracker", None)
        if tracker is None:
            tracker = TestModePhaseTracker(
                min_cycle_sec=self.test_mode_phase_min_cycle,
                max_cycle_sec=self.test_mode_phase_max_cycle,
                peak_slope_eps=self.test_mode_peak_slope_eps,
                peak_threshold=self.peak_threshold,
                estimated_human_frequency=self.estimated_human_frequency,
            )
            self.test_mode_phase_tracker = tracker
        tracker.peak_threshold = float(self.peak_threshold)

        if int(self.gait_state) != 1:
            self._motor_diff_start_baseline = float(angle_diff)
            self._motor_diff_start_seeded = False
        elif self._motor_diff_start_baseline is None:
            self._motor_diff_start_baseline = float(angle_diff)
        elif not self._motor_diff_start_seeded:
            decision = self._judge_start_leg_from_diff(
                angle_diff,
                self._motor_diff_start_baseline,
                diff_threshold=START_LEG_DIFF_THRESHOLD_RAD,
            )
            if decision is not None:
                start_side, init_phase, delta = decision
                tracker.seed_single_signal_phase(
                    angle=angle_diff,
                    current_time=current_time,
                    phase_norm=(init_phase / (2.0 * np.pi)) % 1.0,
                    estimated_human_frequency=self.estimated_human_frequency,
                    assist_ready=False,
                )
                self._motor_diff_start_seeded = True
                self._log_diff_start_leg_decision(
                    "电机差分",
                    start_side,
                    init_phase,
                    delta,
                    "rad",
                )

        result = tracker.update_single_signal(
            angle=angle_diff,
            current_time=current_time,
            gait_state=int(self.gait_state),
        )

        self.test_mode_left_phase_valid = bool(result.get("left_valid", False))
        self.test_mode_right_phase_valid = bool(result.get("right_valid", False))
        self.test_mode_left_assist_ready = bool(result.get("left_assist_ready", False))
        self.test_mode_right_assist_ready = bool(result.get("right_assist_ready", False))
        self._set_current_phase_pair(
            result.get("left_phase_rad", 0.0),
            result.get("right_phase_rad", np.pi),
        )
        self.phase_active = bool(result.get("phase_active", False))
        self._update_phase_rate_frequency(self.current_left_phase)
        self._update_phase_peak_frequency(self.current_left_phase)
        self._assist_zero_prev_phase = self.current_left_phase

        if not hasattr(self, 'left_phase_buffer'):
            self.left_phase_buffer = deque(maxlen=self.buffer_size)
            self.right_phase_buffer = deque(maxlen=self.buffer_size)
        self.left_phase_buffer.append(self.current_left_phase)
        self.right_phase_buffer.append(self.current_right_phase)
        self.footp["p"] = list(self.left_phase_buffer)
        self.footp["pha"] = list(self.gflag)

        if self.debug_mode and bool(result.get("left_peak", False)):
            period = float(result.get("left_period_sec", 0.0))
            self.get_logger().info(
                f"🧪 差分test峰值触发: raw_diff={raw_left_angle - raw_right_angle:.3f} rad, "
                f"filtered_diff={angle_diff:.3f} rad, alpha={filter_alpha:.3f}, "
                f"period={period:.3f}s, ready={self.test_mode_left_assist_ready}"
            )

    def _extract_test_mode_phase(self):
        """test模式相位提取：左右腿独立峰值触发，不使用AO。"""
        if len(self.lhip_buffer) < 1 or len(self.rhip_buffer) < 1:
            self.reset_phase_estimator()
            return

        current_time = float(getattr(self, "last_process_time", None) or time.time())
        left_angle = float(self.lhip_buffer[-1])
        right_angle = float(self.rhip_buffer[-1])
        tracker = getattr(self, "test_mode_phase_tracker", None)
        if tracker is None:
            tracker = TestModePhaseTracker(
                min_cycle_sec=self.test_mode_phase_min_cycle,
                max_cycle_sec=self.test_mode_phase_max_cycle,
                peak_slope_eps=self.test_mode_peak_slope_eps,
                peak_threshold=self.peak_threshold,
                estimated_human_frequency=self.estimated_human_frequency,
            )
            self.test_mode_phase_tracker = tracker
        tracker.peak_threshold = float(self.peak_threshold)
        result = tracker.update(
            left_angle=left_angle,
            right_angle=right_angle,
            current_time=current_time,
            gait_state=int(self.gait_state),
        )

        self.test_mode_left_phase_valid = bool(result.get("left_valid", False))
        self.test_mode_right_phase_valid = bool(result.get("right_valid", False))
        self.test_mode_left_assist_ready = bool(result.get("left_assist_ready", False))
        self.test_mode_right_assist_ready = bool(result.get("right_assist_ready", False))
        self._set_current_phase_pair(
            result.get("left_phase_rad", 0.0),
            result.get("right_phase_rad", 0.0),
        )
        self.phase_active = bool(result.get("phase_active", False))
        self._update_phase_rate_frequency(self.current_left_phase)
        self._update_phase_peak_frequency(self.current_left_phase)
        self._assist_zero_prev_phase = self.current_left_phase

        if not hasattr(self, 'left_phase_buffer'):
            self.left_phase_buffer = deque(maxlen=self.buffer_size)
            self.right_phase_buffer = deque(maxlen=self.buffer_size)
        self.left_phase_buffer.append(self.current_left_phase)
        self.right_phase_buffer.append(self.current_right_phase)
        self.footp["p"] = list(self.left_phase_buffer)
        self.footp["pha"] = list(self.gflag)

        if self.debug_mode and (bool(result.get("left_peak", False)) or bool(result.get("right_peak", False))):
            left_period = float(result.get("left_period_sec", 0.0))
            right_period = float(result.get("right_period_sec", 0.0))
            self.get_logger().info(
                f"🧪 test峰值触发: L_peak={bool(result.get('left_peak', False))}, "
                f"R_peak={bool(result.get('right_peak', False))}, "
                f"L_period={left_period:.3f}s, R_period={right_period:.3f}s, "
                f"L_ready={self.test_mode_left_assist_ready}, R_ready={self.test_mode_right_assist_ready}"
            )

    def _process_imu_adaptive_oscillator_side(
        self,
        side: str,
        signal_output: ImuPhaseOutput,
        sample_time: float,
        estimator: ThighImuPhaseEstimator,
    ) -> ImuPhaseOutput:
        """Run one side's IMU sagittal-angle signal through its own RAO."""
        if bool(getattr(signal_output, "sample_rejected", False)):
            return signal_output

        osc = getattr(self, f"imu_phase_{side}_adaptive_oscillator", None)
        if osc is None:
            raise RuntimeError(f"IMU AO oscillator unavailable for {side}")

        last_time_attr = f"imu_phase_{side}_ao_last_time"
        last_time = getattr(self, last_time_attr, None)
        dt_local = float(self.dt)
        if last_time is not None:
            sample_dt = float(sample_time) - float(last_time)
            if 0.001 <= sample_dt <= 0.2:
                dt_local = sample_dt
        osc.update_dt(dt_local)
        setattr(self, last_time_attr, float(sample_time))

        angle_rad = float(np.deg2rad(signal_output.angle_deg))
        angular_velocity_rad_s = float(np.deg2rad(signal_output.angular_velocity_deg_s))
        phase_rad = osc.step(angle_rad, angular_velocity_rad_s)
        phase_offset = float(getattr(estimator.config, "phase_offset", 0.0))
        phase_rad = self._wrap_to_2pi(phase_rad + phase_offset * 2.0 * np.pi)
        zero_event = bool(getattr(osc, "last_zero_cross_event", False))
        phase_limited = False
        phase_rate_limit_hz = float(getattr(estimator.config, "max_phase_rate_hz", 0.0))
        last_phase_attr = f"imu_phase_{side}_ao_last_phase_rad"
        last_phase_time_attr = f"imu_phase_{side}_ao_last_phase_time"
        last_phase = getattr(self, last_phase_attr, None)
        last_phase_time = getattr(self, last_phase_time_attr, None)
        if (
            phase_rate_limit_hz > 0.0
            and last_phase is not None
            and last_phase_time is not None
            and not zero_event
        ):
            phase_dt = float(sample_time) - float(last_phase_time)
            if 0.0 < phase_dt <= 0.25:
                max_delta = 2.0 * np.pi * phase_rate_limit_hz * phase_dt
                forward_delta = (float(phase_rad) - float(last_phase)) % (2.0 * np.pi)
                if forward_delta > max_delta:
                    phase_rad = self._wrap_to_2pi(float(last_phase) + max_delta)
                    phase_limited = True
        setattr(self, last_phase_attr, float(phase_rad))
        setattr(self, last_phase_time_attr, float(sample_time))

        zero_count_attr = f"imu_phase_{side}_ao_zero_event_count"
        zero_count = int(getattr(self, zero_count_attr, 0))
        if zero_event:
            zero_count += 1
            setattr(self, zero_count_attr, zero_count)

        try:
            freq_hz = float(osc.para[1, 1]) / (2.0 * np.pi)
        except Exception:
            freq_hz = float(signal_output.previous_cycle_frequency_hz)
        if freq_hz <= 0.0:
            freq_hz = float(signal_output.previous_cycle_frequency_hz)
        period_sec = 1.0 / max(freq_hz, 1e-6)

        recent_swing_active = bool(getattr(signal_output, "recent_swing_active", False))
        motion_active = bool(
            not phase_limited
            and (signal_output.motion_active or (zero_count > 0 and recent_swing_active))
        )

        return ImuPhaseOutput(
            angle_raw_deg=signal_output.angle_raw_deg,
            angle_deg=signal_output.angle_deg,
            angular_velocity_raw_deg_s=signal_output.angular_velocity_raw_deg_s,
            angular_velocity_deg_s=signal_output.angular_velocity_deg_s,
            phase_0_to_1=float((phase_rad / (2.0 * np.pi)) % 1.0),
            phase_rad=float(phase_rad),
            previous_cycle_period_sec=float(period_sec),
            previous_cycle_frequency_hz=float(freq_hz),
            zero_event=zero_event,
            zero_event_count=zero_count,
            motion_active=motion_active,
            current_swing_range_deg=signal_output.current_swing_range_deg,
            time_since_last_zero_sec=signal_output.time_since_last_zero_sec,
            time_since_last_motion_sec=signal_output.time_since_last_motion_sec,
            recent_swing_range_deg=getattr(
                signal_output,
                "recent_swing_range_deg",
                signal_output.current_swing_range_deg,
            ),
            recent_swing_active=recent_swing_active,
            phase_limited=phase_limited,
        )

    def _apply_seeded_imu_diff_phase(self, signal_output: ImuPhaseOutput, sample_time: float) -> ImuPhaseOutput:
        """首个差分零点前，用起步脚判别得到的相位做临时推进。"""
        if not bool(getattr(self, "_imu_diff_start_seeded", False)):
            return signal_output
        if int(getattr(signal_output, "zero_event_count", 0)) > 0:
            return signal_output

        start_time = getattr(self, "_imu_diff_start_time", None)
        if start_time is None:
            start_time = float(sample_time)
            self._imu_diff_start_time = start_time
        elapsed = max(0.0, float(sample_time) - float(start_time))
        period = max(float(signal_output.previous_cycle_period_sec), 1e-6)
        phase_rad = self._wrap_to_2pi(
            float(getattr(self, "_imu_diff_start_phase_rad", START_PHASE_INIT_LEFT_FIRST))
            + (elapsed / period) * 2.0 * np.pi
        )
        signal_output.phase_rad = float(phase_rad)
        signal_output.phase_0_to_1 = float((phase_rad / (2.0 * np.pi)) % 1.0)
        return signal_output

    def _process_imu_phase_side(
        self,
        side: str,
        detector,
        estimator,
        use_adaptive_oscillator: bool = False,
    ):
        """Process one IMU phase side and return the latest estimator output."""
        if detector is None or not hasattr(detector, "get_latest_sample_6d"):
            return None
        latest = detector.get_latest_sample_6d()
        if latest is None:
            return None
        sample, sample_time, seq = latest
        timeout_sec = float(getattr(detector, "data_timeout_sec", 1.0))
        if time.time() - float(sample_time) > max(timeout_sec, 0.1):
            return None
        setattr(self, f"imu_phase_{side}_sample_time", float(sample_time))

        last_seq_attr = f"imu_phase_last_{side}_seq"
        output_attr = f"imu_phase_{side}_output"
        last_seq = int(getattr(self, last_seq_attr, 0))
        output = getattr(self, output_attr, None)
        is_new_sample = int(seq) != last_seq
        if is_new_sample:
            output = estimator.process_6d(sample, float(sample_time))
            if use_adaptive_oscillator:
                output = self._process_imu_adaptive_oscillator_side(
                    side,
                    output,
                    float(sample_time),
                    estimator,
                )
            setattr(self, last_seq_attr, int(seq))
            setattr(self, output_attr, output)

        if output is None:
            return None

        sample_rejected = bool(getattr(output, "sample_rejected", False))
        reject_reason = str(getattr(output, "reject_reason", "") or "")
        display_angle = self._sanitize_imu_phase_display_value(
            side,
            "angle",
            output.angle_deg,
            hold_previous=sample_rejected,
        )
        display_gyro = self._sanitize_imu_phase_display_value(
            side,
            "gyro",
            output.angular_velocity_deg_s,
            hold_previous=sample_rejected,
        )
        setattr(self, f"imu_phase_{side}_angle_deg", display_angle)
        setattr(
            self,
            f"imu_phase_{side}_angular_velocity_deg_s",
            display_gyro,
        )
        setattr(self, f"imu_phase_{side}_swing_range_deg", float(output.current_swing_range_deg))
        setattr(
            self,
            f"imu_phase_{side}_recent_swing_range_deg",
            float(getattr(output, "recent_swing_range_deg", output.current_swing_range_deg)),
        )
        setattr(self, f"imu_phase_{side}_frequency_hz", float(output.previous_cycle_frequency_hz))
        setattr(self, f"imu_phase_{side}_sample_rejected", sample_rejected)
        setattr(self, f"imu_phase_{side}_reject_reason", reject_reason)
        setattr(self, f"imu_phase_{side}_valid", bool(output.zero_event_count > 0 and not sample_rejected))
        setattr(self, f"imu_phase_{side}_motion_active", bool(output.motion_active and not sample_rejected))
        setattr(self, f"imu_phase_{side}_zero_event", bool(is_new_sample and output.zero_event and not sample_rejected))
        return output

    def _extract_imu_phase(self):
        """IMU相位模式：双IMU使用左右大腿差分估计相位，单左IMU由左相位推右相位。"""
        ao_phase_mode = self.current_motion_mode in IMU_AO_PHASE_MODES
        single_imu_mode = self.current_motion_mode in SINGLE_IMU_PHASE_MODES
        dual_diff_mode = self.current_motion_mode in DUAL_IMU_PHASE_MODES
        left_estimator = self.imu_phase_left_estimator
        if self.current_motion_mode == "imu_left_phase":
            left_estimator = self.imu_phase_left_quaternion_estimator
        left_output = self._process_imu_phase_side(
            "left",
            getattr(self, "imu_phase_left_detector", None),
            left_estimator,
            use_adaptive_oscillator=bool(ao_phase_mode and single_imu_mode),
        )
        right_output = None if single_imu_mode else self._process_imu_phase_side(
            "right",
            getattr(self, "imu_phase_right_detector", None),
            self.imu_phase_right_estimator,
            use_adaptive_oscillator=False,
        )
        left_rejected = bool(left_output is not None and getattr(left_output, "sample_rejected", False))
        right_rejected = bool(right_output is not None and getattr(right_output, "sample_rejected", False))
        abnormal_reasons = []
        if left_rejected:
            abnormal_reasons.append(f"L:{getattr(left_output, 'reject_reason', '')}")
        if right_rejected:
            abnormal_reasons.append(f"R:{getattr(right_output, 'reject_reason', '')}")

        diff_output = None
        diff_new_sample = False
        diff_angle_raw_deg = 0.0
        diff_gyro_raw_deg_s = 0.0
        if (
            dual_diff_mode
            and left_output is not None
            and right_output is not None
            and not left_rejected
            and not right_rejected
        ):
            left_seq = int(getattr(self, "imu_phase_last_left_seq", 0))
            right_seq = int(getattr(self, "imu_phase_last_right_seq", 0))
            last_diff_left_seq = int(getattr(self, "imu_phase_last_diff_left_seq", 0))
            last_diff_right_seq = int(getattr(self, "imu_phase_last_diff_right_seq", 0))
            diff_output = getattr(self, "imu_phase_diff_output", None)
            diff_angle_raw_deg = float(left_output.angle_raw_deg) - float(right_output.angle_raw_deg)
            diff_gyro_raw_deg_s = (
                float(left_output.angular_velocity_raw_deg_s)
                - float(right_output.angular_velocity_raw_deg_s)
            )
            if (
                diff_output is None
                or left_seq != last_diff_left_seq
                or right_seq != last_diff_right_seq
            ):
                left_time = getattr(self, "imu_phase_left_sample_time", None)
                right_time = getattr(self, "imu_phase_right_sample_time", None)
                fallback_time = float(getattr(self, "last_process_time", None) or time.time())
                diff_time = max(
                    float(left_time) if left_time is not None else fallback_time,
                    float(right_time) if right_time is not None else fallback_time,
                )
                last_diff_time = getattr(self, "imu_phase_diff_sample_time", None)
                if last_diff_time is not None and diff_time <= float(last_diff_time):
                    diff_time = float(last_diff_time) + max(0.001, min(float(self.dt), 0.05))

                if int(self.gait_state) != 1:
                    self._imu_diff_start_baseline = float(diff_angle_raw_deg)
                    self._imu_diff_start_seeded = False
                    self._imu_diff_start_phase_rad = START_PHASE_INIT_LEFT_FIRST
                    self._imu_diff_start_time = None
                elif self._imu_diff_start_baseline is None:
                    self._imu_diff_start_baseline = float(diff_angle_raw_deg)
                elif not self._imu_diff_start_seeded:
                    decision = self._judge_start_leg_from_diff(
                        diff_angle_raw_deg,
                        self._imu_diff_start_baseline,
                        diff_threshold=START_LEG_DIFF_THRESHOLD_DEG,
                        diff_velocity=diff_gyro_raw_deg_s,
                        velocity_threshold=START_LEG_DIFF_GYRO_THRESHOLD_DEG_S,
                    )
                    if decision is not None:
                        start_side, init_phase, delta = decision
                        self._imu_diff_start_seeded = True
                        self._imu_diff_start_phase_rad = init_phase
                        self._imu_diff_start_time = float(diff_time)
                        self._set_current_phase_pair(init_phase)
                        if ao_phase_mode:
                            osc = getattr(self, "imu_phase_diff_adaptive_oscillator", None)
                            if osc is not None and hasattr(osc, "prime_phase"):
                                osc.prime_phase(init_phase)
                        self._log_diff_start_leg_decision(
                            "IMU差分",
                            start_side,
                            init_phase,
                            delta,
                            "deg",
                        )

                diff_output = self.imu_phase_diff_estimator.process_signal(
                    diff_angle_raw_deg,
                    diff_gyro_raw_deg_s,
                    diff_time,
                )
                if not ao_phase_mode:
                    diff_output = self._apply_seeded_imu_diff_phase(diff_output, diff_time)
                if ao_phase_mode:
                    diff_output = self._process_imu_adaptive_oscillator_side(
                        "diff",
                        diff_output,
                        diff_time,
                        self.imu_phase_diff_estimator,
                    )
                self.imu_phase_last_diff_left_seq = left_seq
                self.imu_phase_last_diff_right_seq = right_seq
                self.imu_phase_diff_sample_time = float(diff_time)
                self.imu_phase_diff_output = diff_output
                diff_new_sample = True
                if bool(getattr(diff_output, "sample_rejected", False)):
                    abnormal_reasons.append(f"D:{getattr(diff_output, 'reject_reason', '')}")
                    self.imu_phase_diff_output = None
                    diff_output = None

        if bool(diff_output is not None and getattr(diff_output, "sample_rejected", False)):
            abnormal_reasons.append(f"D:{getattr(diff_output, 'reject_reason', '')}")
            diff_output = None

        if dual_diff_mode:
            diff_rejected = bool(abnormal_reasons)
            seeded_diff_valid = bool(
                diff_output is not None
                and self.gait_state == 1
                and not ao_phase_mode
                and not diff_rejected
                and bool(getattr(self, "_imu_diff_start_seeded", False))
                and int(getattr(diff_output, "zero_event_count", 0)) == 0
                and bool(getattr(diff_output, "recent_swing_active", False))
            )
            diff_valid = bool(
                diff_output is not None
                and not diff_rejected
                and (ao_phase_mode or diff_output.zero_event_count > 0 or seeded_diff_valid)
            )
            diff_motion_active = bool(diff_output is not None and diff_output.motion_active and not diff_rejected)
            self.imu_phase_left_valid = diff_valid
            self.imu_phase_right_valid = diff_valid
            self.imu_phase_valid = diff_valid
            self.imu_phase_left_motion_active = diff_motion_active
            self.imu_phase_right_motion_active = diff_motion_active
            self.imu_phase_motion_active = bool(diff_valid and diff_motion_active)
            self.imu_phase_left_zero_event = bool(
                diff_new_sample and diff_output is not None and diff_output.zero_event
            )
            self.imu_phase_right_zero_event = False
            if diff_output is not None:
                self.imu_phase_diff_angle_deg = self._sanitize_imu_phase_display_value(
                    "diff",
                    "angle",
                    diff_output.angle_deg,
                    hold_previous=diff_rejected,
                )
                self.imu_phase_diff_angular_velocity_deg_s = self._sanitize_imu_phase_display_value(
                    "diff",
                    "gyro",
                    diff_output.angular_velocity_deg_s,
                    hold_previous=diff_rejected,
                )
                self.imu_phase_diff_swing_range_deg = float(diff_output.current_swing_range_deg)
                self.imu_phase_diff_recent_swing_range_deg = float(
                    getattr(
                        diff_output,
                        "recent_swing_range_deg",
                        diff_output.current_swing_range_deg,
                    )
                )
                self._set_current_phase_pair(diff_output.phase_rad)
                diff_freq = float(diff_output.previous_cycle_frequency_hz)
                self.imu_phase_left_frequency_hz = diff_freq
                self.imu_phase_right_frequency_hz = diff_freq
            else:
                self._set_current_phase_pair(self.current_left_phase)
            left_valid = diff_valid
        else:
            left_valid = bool(
                left_output is not None
                and not left_rejected
                and (ao_phase_mode or left_output.zero_event_count > 0)
            )
            left_motion_active = bool(left_output is not None and left_output.motion_active and not left_rejected)
            self.imu_phase_left_valid = left_valid
            self.imu_phase_right_valid = bool(left_valid if single_imu_mode else False)
            self.imu_phase_valid = bool(left_valid)
            self.imu_phase_left_motion_active = left_motion_active
            self.imu_phase_right_motion_active = bool(left_motion_active if single_imu_mode else False)
            self.imu_phase_motion_active = bool(self.imu_phase_valid and left_motion_active)
            if single_imu_mode and left_output is not None:
                self.imu_phase_right_angle_deg = -float(self.imu_phase_left_angle_deg)
                self.imu_phase_right_angular_velocity_deg_s = -float(
                    self.imu_phase_left_angular_velocity_deg_s
                )
                self.imu_phase_right_swing_range_deg = float(self.imu_phase_left_swing_range_deg)
                self.imu_phase_right_recent_swing_range_deg = float(
                    self.imu_phase_left_recent_swing_range_deg
                )
                self.imu_phase_right_frequency_hz = float(self.imu_phase_left_frequency_hz)
                self.imu_phase_right_zero_event = False

            if left_output is not None and not left_rejected:
                self._set_current_phase_pair(left_output.phase_rad)
            else:
                self._set_current_phase_pair(self.current_left_phase)

        self.imu_phase_abnormal_sample = bool(abnormal_reasons)
        self.imu_phase_abnormal_reason = ";".join(
            reason for reason in abnormal_reasons if str(reason).strip()
        )
        if self.imu_phase_abnormal_sample:
            self.imu_phase_valid = False
            self.imu_phase_left_motion_active = False
            self.imu_phase_right_motion_active = False
            self.imu_phase_motion_active = False

        self.phase_active = bool(self.gait_state == 1 and self.imu_phase_valid)
        if self.phase_active:
            self._update_phase_rate_frequency(self.current_left_phase)
            self._update_phase_peak_frequency(self.current_left_phase)
        else:
            self._update_phase_rate_frequency(None)
            self._update_phase_peak_frequency(None)

        if (
            self.gait_state == 1
            and getattr(self, "assist_wait_next_zero", False)
            and bool(getattr(self, "imu_phase_left_zero_event", False))
            and self.imu_phase_valid
        ):
            self.assist_wait_next_zero = False
            self.assist_enable = True
            self.get_logger().info("IMU相位到达左腿零点，开始输出助力")

        if not hasattr(self, 'left_phase_buffer'):
            self.left_phase_buffer = deque(maxlen=self.buffer_size)
            self.right_phase_buffer = deque(maxlen=self.buffer_size)
        self.left_phase_buffer.append(self.current_left_phase)
        self.right_phase_buffer.append(self.current_right_phase)
        self.footp["p"] = list(self.left_phase_buffer)
        self.footp["pha"] = list(self.gflag)

        if self.debug_mode and self.log_counter % 30 == 0:
            if dual_diff_mode:
                phase_label = "IMU差分AO相位" if ao_phase_mode else "IMU差分相位"
                diff_debug = (
                    f"diff_raw={diff_angle_raw_deg:.2f}deg, "
                    f"diff_lpf={getattr(self, 'imu_phase_diff_angle_deg', 0.0):.2f}deg, "
                    f"diff_gyro={getattr(self, 'imu_phase_diff_angular_velocity_deg_s', 0.0):.2f}deg/s, "
                )
                right_valid_debug = left_valid
            else:
                phase_label = "IMU-AO相位" if ao_phase_mode else "IMU相位"
                diff_debug = ""
                right_valid_debug = self.imu_phase_right_valid
            self.get_logger().info(
                f"{phase_label} - "
                f"L_angle={self.imu_phase_left_angle_deg:.2f}deg, "
                f"R_angle={self.imu_phase_right_angle_deg:.2f}deg, "
                f"{diff_debug}"
                f"L_phase={self.current_left_phase:.2f}rad valid={left_valid}, "
                f"R_phase={self.current_right_phase:.2f}rad "
                f"valid={right_valid_debug}, "
                f"motion_active={self.imu_phase_motion_active}, "
                f"swing_range=L{self.imu_phase_left_swing_range_deg:.1f}/"
                f"R{self.imu_phase_right_swing_range_deg:.1f}deg, "
                f"window_swing=L{self.imu_phase_left_recent_swing_range_deg:.1f}/"
                f"R{self.imu_phase_right_recent_swing_range_deg:.1f}deg, "
                f"freq=L{self.imu_phase_left_frequency_hz:.2f}/"
                f"R{self.imu_phase_right_frequency_hz:.2f}Hz, "
                f"swing_threshold={self.imu_phase_swing_threshold:.1f}deg"
            )

    def _get_fresh_imu_phase_sample(self, detector):
        if detector is None or not hasattr(detector, "get_latest_sample_6d"):
            return None
        latest = detector.get_latest_sample_6d()
        if latest is None:
            return None
        sample, sample_time, seq = latest
        timeout_sec = float(getattr(detector, "data_timeout_sec", 1.0))
        if time.time() - float(sample_time) > max(timeout_sec, 0.1):
            return None
        return sample, float(sample_time), int(seq)

    def _extract_model_phase(self):
        """ONNX模型相位模式：双MI1 IMU组帧，使用后处理相位驱动助力。"""
        estimator = getattr(self, "model_phase_estimator", None)
        if estimator is None:
            self.model_phase_valid = False
            self.imu_phase_valid = False
            self.imu_phase_motion_active = False
            self.phase_active = False
            self._update_phase_rate_frequency(None)
            self._update_phase_peak_frequency(None)
            if self.debug_mode and self.log_counter % 60 == 0:
                self.get_logger().warn(
                    f"模型相位不可用: {getattr(self, 'model_phase_init_error', '')}"
                )
            return

        left_latest = self._get_fresh_imu_phase_sample(
            getattr(self, "imu_phase_left_detector", None)
        )
        right_latest = self._get_fresh_imu_phase_sample(
            getattr(self, "imu_phase_right_detector", None)
        )
        if left_latest is None or right_latest is None:
            self.model_phase_valid = False
            self.imu_phase_left_valid = False
            self.imu_phase_right_valid = False
            self.imu_phase_valid = False
            self.imu_phase_left_motion_active = False
            self.imu_phase_right_motion_active = False
            self.imu_phase_motion_active = False
            self.phase_active = False
            self._update_phase_rate_frequency(None)
            self._update_phase_peak_frequency(None)
            if self.debug_mode and self.log_counter % 60 == 0:
                self.get_logger().warn("模型相位等待左右IMU新鲜Acc/Gyr/Quat数据")
            return

        left_sample, left_time, left_seq = left_latest
        right_sample, right_time, right_seq = right_latest
        left_new = int(left_seq) != int(getattr(self, "model_phase_last_left_seq", 0))
        right_new = int(right_seq) != int(getattr(self, "model_phase_last_right_seq", 0))
        output = getattr(self, "model_phase_output", None)
        if left_new or right_new:
            try:
                output = estimator.process_samples(
                    left_sample,
                    left_time,
                    right_sample,
                    right_time,
                )
                self.model_phase_last_left_seq = int(left_seq)
                self.model_phase_last_right_seq = int(right_seq)
            except Exception as exc:
                err = str(exc)
                if err != getattr(self, "model_phase_init_error", ""):
                    self.get_logger().warn(f"模型相位推理失败: {err}")
                self.model_phase_init_error = err
                self.model_phase_valid = False
                self.imu_phase_valid = False
                self.imu_phase_motion_active = False
                self.phase_active = False
                self._update_phase_rate_frequency(None)
                self._update_phase_peak_frequency(None)
                return
            if output is not None:
                self.model_phase_output = output

        if output is None:
            self.model_phase_valid = False
            self.imu_phase_left_valid = False
            self.imu_phase_right_valid = False
            self.imu_phase_valid = False
            self.imu_phase_left_motion_active = False
            self.imu_phase_right_motion_active = False
            self.imu_phase_motion_active = False
            self.phase_active = False
            self._update_phase_rate_frequency(None)
            self._update_phase_peak_frequency(None)
            return

        self.model_phase_valid = True
        self.model_phase_raw_phase = float(output.raw_phase)
        self.model_phase_post_phase = float(output.phase_0_to_1)
        self.model_phase_stride_rate_hz = float(output.stride_rate_clamped_hz)

        self.imu_phase_left_angle_deg = self._sanitize_imu_phase_display_value(
            "left",
            "angle",
            output.left_euler_y_deg,
        )
        self.imu_phase_right_angle_deg = self._sanitize_imu_phase_display_value(
            "right",
            "angle",
            output.right_euler_y_deg,
        )
        self.imu_phase_left_angular_velocity_deg_s = self._sanitize_imu_phase_display_value(
            "left",
            "gyro",
            output.left_gyro_y_deg_s,
        )
        self.imu_phase_right_angular_velocity_deg_s = self._sanitize_imu_phase_display_value(
            "right",
            "gyro",
            output.right_gyro_y_deg_s,
        )
        self.imu_phase_left_swing_range_deg = 0.0
        self.imu_phase_right_swing_range_deg = 0.0
        self.imu_phase_left_recent_swing_range_deg = 0.0
        self.imu_phase_right_recent_swing_range_deg = 0.0
        self.imu_phase_left_frequency_hz = float(output.previous_cycle_frequency_hz)
        self.imu_phase_right_frequency_hz = float(output.previous_cycle_frequency_hz)
        self.imu_phase_left_valid = True
        self.imu_phase_right_valid = True
        self.imu_phase_valid = True
        self.imu_phase_left_motion_active = bool(output.motion_active)
        self.imu_phase_right_motion_active = bool(output.motion_active)
        self.imu_phase_motion_active = bool(output.motion_active)
        zero_event = bool(
            getattr(output, "is_new_prediction", True)
            and getattr(output, "zero_event", False)
        )
        self.imu_phase_left_zero_event = zero_event
        self.imu_phase_right_zero_event = False

        self._set_current_phase_pair(output.phase_rad)

        self.phase_active = bool(self.gait_state == 1 and self.imu_phase_valid)
        if self.phase_active and bool(getattr(output, "is_new_prediction", True)):
            self._update_phase_rate_frequency(self.current_left_phase)
            self._update_phase_peak_frequency(self.current_left_phase)
        elif not self.phase_active:
            self._update_phase_rate_frequency(None)
            self._update_phase_peak_frequency(None)

        if (
            self.gait_state == 1
            and getattr(self, "assist_wait_next_zero", False)
            and zero_event
            and self.imu_phase_valid
        ):
            self.assist_wait_next_zero = False
            self.assist_enable = True
            self.get_logger().info("模型相位到达左腿零点，开始输出助力")

        if not hasattr(self, 'left_phase_buffer'):
            self.left_phase_buffer = deque(maxlen=self.buffer_size)
            self.right_phase_buffer = deque(maxlen=self.buffer_size)
        self.left_phase_buffer.append(self.current_left_phase)
        self.right_phase_buffer.append(self.current_right_phase)
        self.footp["p"] = list(self.left_phase_buffer)
        self.footp["pha"] = list(self.gflag)

        if self.debug_mode and self.log_counter % 30 == 0:
            self.get_logger().info(
                "模型相位 - "
                f"L_EulerY={self.imu_phase_left_angle_deg:.2f}deg, "
                f"R_EulerY={self.imu_phase_right_angle_deg:.2f}deg, "
                f"raw={self.model_phase_raw_phase:.3f}, "
                f"post={self.model_phase_post_phase:.3f}, "
                f"L_phase={self.current_left_phase:.2f}rad, "
                f"R_phase={self.current_right_phase:.2f}rad, "
                f"r={self.model_phase_stride_rate_hz:.2f}Hz, "
                f"rows={int(getattr(output, 'total_rows', 0))}, "
                f"pred={int(getattr(output, 'total_predictions', 0))}"
            )

    def extract_foot_phase(self):
        """提取足部相位/频率（测试模式使用峰值法，其余模式使用RAO）。"""
        if self.current_motion_mode in DIFF_PEAK_PHASE_MODES:
            self._extract_diff_test_mode_phase()
            return

        if self.current_motion_mode in MOTOR_PEAK_PHASE_MODES:
            self._extract_test_mode_phase()
            return

        if self.current_motion_mode in MODEL_PHASE_MODES:
            self._extract_model_phase()
            return

        if self.current_motion_mode in IMU_PHASE_MODES:
            self._extract_imu_phase()
            return

        if len(self.lhip_buffer) < 1 or len(self.rhip_buffer) < 1:
            self.reset_phase_estimator()
            return

        # 项目统一定义：q = theta_l - theta_r
        raw_angle_diff = float(self.lhip_buffer[-1]) - float(self.rhip_buffer[-1])
        angle_deadzone = max(0.0, float(getattr(self, "phase_input_deadzone_angle", 0.0)))
        angle_diff = 0.0 if abs(raw_angle_diff) < angle_deadzone else raw_angle_diff

        # gait_state=0 时启用“冻结输出 + 滞回门控”：
        # 仅当角度/角速度波动超过启动阈值时才更新 preview 相位，
        # 否则保持最后相位值（只更新滤波器状态）。
        if self.gait_state == 0:
            dq = float(self.dq_buffer[-1]) if len(self.dq_buffer) > 0 else 0.0
            allow_preview_update = self._update_preview_gate(angle_diff, dq)
            self.phase_active = False
            if hasattr(self, "adaptive_oscillator"):
                try:
                    if allow_preview_update:
                        phase_preview = self.adaptive_oscillator.step(angle_diff)
                        self._set_current_phase_pair(phase_preview)
                    else:
                        self.adaptive_oscillator.run_filter_only(angle_diff)
                except Exception:
                    self.adaptive_oscillator.run_filter_only(angle_diff)
            if not hasattr(self, 'left_phase_buffer'):
                self.left_phase_buffer = deque(maxlen=self.buffer_size)
                self.right_phase_buffer = deque(maxlen=self.buffer_size)
            self.left_phase_buffer.append(getattr(self, "current_left_phase", 0.0))
            self.right_phase_buffer.append(getattr(self, "current_right_phase", np.pi))
            self.footp["p"] = list(self.left_phase_buffer)
            self.footp["pha"] = list(self.gflag)
            return
        
        # 使用自适应振荡器提取相位
        # 注意：step() 内部会调用 _filter()，所以这里不需要手动调用 run_filter_only
        if self._start_phase_init_pending:
            phase = float(self._start_phase_init_value)
            self._start_phase_init_pending = False
            try:
                if hasattr(self.adaptive_oscillator, "prime_phase"):
                    self.adaptive_oscillator.prime_phase(phase)
                self.adaptive_oscillator.run_filter_only(angle_diff)
            except Exception as exc:
                if self.debug_mode:
                    self.get_logger().warn(f"⚠️ 起步相位初始化失败，继续使用设定相位: {exc}")
        else:
            try:
                phase = self.adaptive_oscillator.step(angle_diff)
            except Exception as exc:
                if self.debug_mode:
                    self.get_logger().warn(f"⚠️ 自适应振荡器更新失败，重置相位: {exc}")
                self.reset_phase_estimator()
                return

        self._set_current_phase_pair(phase)
        self.phase_active = True
        self._update_phase_rate_frequency(self.current_left_phase)
        self._update_phase_peak_frequency(self.current_left_phase)

        # 助力启动门控：检测到运动后，必须等待下一次相位零点再开启助力
        prev_phase = getattr(self, "_assist_zero_prev_phase", None)
        crossed_zero = False
        if prev_phase is not None:
            crossed_zero = (
                prev_phase > np.pi
                and self.current_left_phase < np.pi
                and (self.current_left_phase - prev_phase) < -np.pi
            )
        self._assist_zero_prev_phase = self.current_left_phase
        if self.gait_state == 1 and getattr(self, "assist_wait_next_zero", False) and crossed_zero:
            self.assist_wait_next_zero = False
            self.assist_enable = True
            self.get_logger().info("🎯 到达下一相位零点，开始输出助力")

        if self.debug_mode and self.log_counter % 30 == 0:
            freq_hz = self._get_ao_frequency_hz()
            self.get_logger().info(f"🔄 振荡器输入 - 左右角度差: {angle_diff:.3f} rad")
            self.get_logger().info(f"🔄 振荡器状态 - 频率: {freq_hz:.2f} Hz, 左相位: {self.current_left_phase:.2f} rad, 右相位: {self.current_right_phase:.2f} rad")

        if not hasattr(self, 'left_phase_buffer'):
            self.left_phase_buffer = deque(maxlen=self.buffer_size)
            self.right_phase_buffer = deque(maxlen=self.buffer_size)
        self.left_phase_buffer.append(self.current_left_phase)
        self.right_phase_buffer.append(self.current_right_phase)

        self.footp["p"] = list(self.left_phase_buffer)
        self.footp["pha"] = list(self.gflag)

    def calculate_hip_torque_profile(self):
        """计算基于步态相位的髋关节力矩曲线（五次多项式）。"""
        # 获取当前运动模式的五次多项式助力曲线参数
        params = self.get_current_assist_parameters()
        
        # 检查相位是否已初始化
        if hasattr(self, 'current_left_phase') and hasattr(self, 'current_right_phase'):
            self.current_left_phase = self._wrap_to_2pi(self.current_left_phase)
            self.current_right_phase = self._wrap_to_2pi(self.current_right_phase)
            self.phi_L = self.current_left_phase
            # 将相位从 0-2π 归一化到 0-1
            left_phase_normalized = self.current_left_phase / (2 * np.pi)
            right_phase_normalized = self.current_right_phase / (2 * np.pi)
        else:
            # 如果相位还没初始化，使用默认值
            self.get_logger().warn("⚠️ 相位未初始化，使用默认相位值")
            left_phase_normalized = 0.0
            right_phase_normalized = 0.5
        
        # 使用五次多项式计算当前时刻的助力力矩
        if self.current_motion_mode in MOTOR_PEAK_PHASE_MODES:
            left_valid = bool(getattr(self, "test_mode_left_phase_valid", False))
            right_valid = bool(getattr(self, "test_mode_right_phase_valid", False))
            left_ready = bool(getattr(self, "test_mode_left_assist_ready", False))
            right_ready = bool(getattr(self, "test_mode_right_assist_ready", False))
            left_torque = (
                self._calculate_hip_assist_quintic(left_phase_normalized, params)
                if left_valid and left_ready
                else 0.0
            )
            right_torque = (
                self._calculate_hip_assist_quintic(right_phase_normalized, params)
                if right_valid and right_ready
                else 0.0
            )
        else:
            left_torque = self._calculate_hip_assist_quintic(left_phase_normalized, params)
            right_torque = self._calculate_hip_assist_quintic(right_phase_normalized, params)
        
        imu_phase_motion_inactive = (
            self.current_motion_mode in IMU_PHASE_MODES
            and not bool(getattr(self, "imu_phase_motion_active", False))
        )
        imu_phase_abnormal = (
            self.current_motion_mode in IMU_PHASE_MODES
            and bool(getattr(self, "imu_phase_abnormal_sample", False))
        )

        # 运动检测/相位有效性：相位被置零、助力关闭或IMU相位未检测到真实运动时，不输出助力
        if (
            not self.assist_enable
            or not getattr(self, "phase_active", False)
            or imu_phase_motion_inactive
            or imu_phase_abnormal
        ):
            left_torque = 0.0
            right_torque = 0.0
            if self.debug_mode and self.log_counter % 20 == 0:
                reason = (
                    "IMU相位异常样本"
                    if imu_phase_abnormal
                    else
                    "IMU相位静止/轻微摆动"
                    if imu_phase_motion_inactive
                    else "相位未激活或助力关闭"
                )
                self.get_logger().info(f"🛑 步态检测：{reason}，助力输出置零")
        
        # 业务层与协议层统一限制为 ±17 Nm。
        left_torque = max(-17.0, min(17.0, left_torque))
        right_torque = max(-17.0, min(17.0, right_torque))
        
        # 仅在调试模式下每20次循环输出一次详细日志
        if self.debug_mode and self.log_counter % 20 == 0:
            gait_phase_name = "行走" if self.gait_state == 1 else "停止"
            self.get_logger().info(f"🌊 五次多项式助力计算 - 步态状态: {gait_phase_name}")
            self.get_logger().info(f"📊 助力参数 - 伸展最大: {params['ext_Tmax']:.2f}, 屈曲最大: {params['flex_Tmax']:.2f} Nm")
            self.get_logger().info(f"⚡ 左相位: {left_phase_normalized:.3f}, 右相位: {right_phase_normalized:.3f}")
            self.get_logger().info(f"🔧 左侧助力: {left_torque:.3f} Nm, 右侧助力: {right_torque:.3f} Nm")
        
        # 更新助力缓存（保存当前值用于记录和可视化）
        if not hasattr(self, 'left_hipAss'):
            self.left_hipAss = deque(maxlen=self.buffer_size)
            self.right_hipAss = deque(maxlen=self.buffer_size)
            self.hipAss = deque(maxlen=self.buffer_size)
        
        self.left_hipAss.append(left_torque)
        self.right_hipAss.append(right_torque)
        self.hipAss.append(left_torque)  # 保持向后兼容性（使用左侧力矩作为默认）
        
        # 增加日志计数器
        self.log_counter += 1
