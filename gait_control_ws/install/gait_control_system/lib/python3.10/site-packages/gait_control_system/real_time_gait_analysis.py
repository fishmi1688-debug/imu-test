import csv
import math
import os
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32

from .adaptive_oscillator_estimator import AdaptiveOscillatorEstimator
from .imu_cycling_toggle_detector import CyclingToggleIMUDetector, DEFAULT_EVENT_PROB_THRESHOLD
from .gait_constants import AO_CONFIG
from .imu_model_start_stop_detector import IMUModelStartStopDetector
from .motion_mode_defaults import get_default_motion_modes
from .phase_bias_fitting import phase_bias_with_smoothing
from .test_mode_phase_helper import (
    TEST_MODE_PHASE_MAX_CYCLE_SEC,
    TEST_MODE_PHASE_MIN_CYCLE_SEC,
    TEST_MODE_PEAK_SLOPE_EPS,
    TestModePhaseTracker,
)

CYCLING_TOGGLE_MODES = ("cycling", "uphill")
TEST_PEAK_PHASE_MODES = ("test", "walking_test")
STAIRS_DOWN_MANUAL_MODES = ("stairs_down", *TEST_PEAK_PHASE_MODES)

# 相位输入/停止态预览门控（便于集中调参）
PHASE_PREVIEW_WINDOW_SEC = 0.5       # 门控RMS统计窗口时长（秒）
PHASE_PREVIEW_START_ANGLE_RMS = 0.05 # 门控开启角度RMS阈值（rad）
PHASE_PREVIEW_STOP_ANGLE_RMS = 0.03  # 门控关闭角度RMS阈值（rad，滞回）
PHASE_PREVIEW_START_DQ_RMS = 0.15    # 门控开启角速度差RMS阈值（rad/s）
PHASE_PREVIEW_STOP_DQ_RMS = 0.08     # 门控关闭角速度差RMS阈值（rad/s，滞回）
PHASE_INPUT_DEADZONE_ANGLE = 0.1     # 相位输入角度死区：|angle_diff|<阈值时置0（rad）

# 停止->运动切换时的起步初始相位参数（便于现场快速调参）
START_PHASE_INIT_LEFT_FIRST = 0.0    # 左脚先迈时，左腿相位初始值（rad）
START_PHASE_INIT_RIGHT_FIRST = np.pi # 右脚先迈时，左腿相位初始值（rad）
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
        self.stairs_down_manual_assist_enabled = False  # 下楼梯模式手动启停状态（按钮控制）
        self.stairs_down_manual_toggle_count = 0
        self.assist_startup_delay = 2.0  # 启动延迟时间（秒），等待系统稳定
        self.startup_time = time.time()  # 记录启动时间
        self.last_process_time = None  # 用于实时估计采样间隔
        self.dt_filtered = dt  # 固定采样时间（保持与 base_dt 一致）
        self.imu_model_info = {}
        self.imu_model_init_error = ""
        self.imu_model_detector = None
        imu_model_path = os.environ.get("GAIT_IMU_MODEL_PATH", "")
        imu_mac = os.environ.get("GAIT_IMU_MAC", "D4:22:CD:00:8A:5A")
        self.imu_model_info = {
            "connected": False,
            "mac_address": imu_mac,
            "predicted_label": "停止",
            "motion_probability": 0.0,
            "ready": False,
            "last_error": "",
        }
        try:
            self.imu_model_detector = IMUModelStartStopDetector(
                model_path=imu_model_path or None,
                mac_address=imu_mac,
                sample_rate_hz=30.0,
            )
            init_err = str(self.imu_model_detector.last_error or "").strip()
            if not init_err:
                self.imu_model_init_error = ""
                self.get_logger().info(
                    f"🧠 启停检测IMU模型已就绪(未连接): mac={imu_mac}, model={self.imu_model_detector.model_path}"
                )
                self.get_logger().info("📱 等待APP发送 imu_manage connect=true 后再连接 walking IMU")
            else:
                self.imu_model_init_error = init_err
                self.imu_model_info.update({"last_error": init_err})
                self.get_logger().warn(
                    f"⚠️ IMU模型判停不可用，将保持停止状态: {self.imu_model_detector.last_error}"
                )
                self.imu_model_detector = None
        except Exception as exc:
            init_err = str(exc)
            self.imu_model_init_error = init_err
            self.imu_model_info.update({"last_error": init_err})
            self.get_logger().warn(f"⚠️ IMU模型判停初始化失败，将保持停止状态: {exc}")
            self.imu_model_detector = None

        self.cycling_toggle_info = {}
        self.cycling_toggle_detector = None
        cycling_model_path = os.environ.get("GAIT_CYCLING_IMU_MODEL_PATH", "")
        cycling_imu_mac = os.environ.get("GAIT_CYCLING_IMU_MAC", "D4:22:CD:00:8A:5B")
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
        try:
            self.cycling_toggle_detector = CyclingToggleIMUDetector(
                model_path=cycling_model_path or None,
                mac_address=cycling_imu_mac,
                sample_rate_hz=30.0,
                event_consecutive=cycling_event_consecutive,
                release_consecutive=cycling_release_consecutive,
                release_prob_hysteresis=cycling_release_hysteresis,
            )
            init_err = str(self.cycling_toggle_detector.last_error or "").strip()
            if not init_err:
                self.cycling_toggle_detector.reset_toggle_state(assist_enabled=False)
                self.get_logger().info(
                    f"🚴 cycling/uphill启停IMU模型已就绪(未连接): mac={cycling_imu_mac}, model={self.cycling_toggle_detector.model_path}"
                )
                self.get_logger().info(
                    f"🚴 cycling启停稳态参数: event_consecutive={cycling_event_consecutive}, "
                    f"release_consecutive={cycling_release_consecutive}, "
                    f"release_hysteresis={cycling_release_hysteresis:.2f}"
                )
                self.get_logger().info("📱 等待APP发送 imu_manage connect=true 后再连接 cycling 槽位 IMU（cycling/uphill 共用）")
            else:
                self.get_logger().warn(
                    f"⚠️ cycling/uphill启停模型不可用，将保持停止状态: {self.cycling_toggle_detector.last_error}"
                )
                self.cycling_toggle_detector = None
        except Exception as exc:
            self.get_logger().warn(f"⚠️ cycling/uphill启停模型初始化失败，将保持停止状态: {exc}")
            self.cycling_toggle_detector = None
        
        # 数据记录配置
        self.data_logging_enabled = True  # 启用数据记录
        self.log_file_path = ""  # 记录文件路径，稍后设置
        self.log_file = None  # CSV文件对象
        self.csv_writer = None  # CSV写入器
        self.log_interval = 1  # 记录间隔（每N次循环记录一次，1表示每次都记录）
        self.log_counter_data = 0  # 数据记录计数器
        
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
                'left_angle',         # 左电机角度 (rad)
                'right_angle',        # 右电机角度 (rad)
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
            
            # 每5次记录刷新一次文件缓冲区，确保数据及时写入
            if self.log_counter_data % (self.log_interval * 5) == 0:
                self.log_file.flush()
                
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

    def close_imu_model_detector(self):
        """关闭IMU模型判停器（walking + cycling）。"""
        detector = getattr(self, "imu_model_detector", None)
        if detector is not None:
            try:
                detector.stop()
                self.get_logger().info("📴 walking IMU模型判停已关闭")
            except Exception as exc:
                self.get_logger().warn(f"⚠️ 关闭walking IMU模型判停失败: {exc}")
            finally:
                self.imu_model_detector = None

        cycling_detector = getattr(self, "cycling_toggle_detector", None)
        if cycling_detector is not None:
            try:
                cycling_detector.stop()
                self.get_logger().info("📴 cycling IMU模型判停已关闭")
            except Exception as exc:
                self.get_logger().warn(f"⚠️ 关闭cycling IMU模型判停失败: {exc}")
            finally:
                self.cycling_toggle_detector = None

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
            info["connected"] = bool(getattr(detector, "_connected", False))
            info["measuring"] = bool(getattr(detector, "_measuring", False))
            info["mac_address"] = str(getattr(detector, "mac_address", fallback_mac))
            info["last_error"] = str(getattr(detector, "last_error", "") or "")
        return {
            "connected": bool(info.get("connected", False)),
            "measuring": bool(info.get("measuring", False)),
            "ready": bool(info.get("ready", False)),
            "stale": bool(info.get("stale", True)),
            "mac": str(info.get("mac_address", fallback_mac)),
            "label": str(info.get("predicted_label", "")),
            "last_error": str(info.get("last_error", init_err or "")),
        }

    def get_imu_connection_status(self):
        walking_mac = os.environ.get("GAIT_IMU_MAC", "D4:22:CD:00:8A:5A")
        cycling_mac = os.environ.get("GAIT_CYCLING_IMU_MAC", "D4:22:CD:00:8A:5B")
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
        return {"walking": walking, "cycling": cycling}

    def set_imu_measurement_enabled(self, enabled: bool, slot: str | None = None):
        target = bool(enabled)
        slot_key = str(slot).strip().lower() if slot is not None else ""
        selected_slots = ("walking", "cycling")
        if slot is not None:
            if slot_key not in selected_slots:
                return {"ok": False, "reason": "invalid_slot", "slot": slot_key}
            selected_slots = (slot_key,)

        for current_slot in selected_slots:
            detector_attr = "imu_model_detector" if current_slot == "walking" else "cycling_toggle_detector"
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
        if mode in CYCLING_TOGGLE_MODES:
            return slot_key == "cycling"
        if mode in STAIRS_DOWN_MANUAL_MODES:
            return False
        return slot_key == "walking"

    def control_imu_connection(self, slot: str, connect: bool, mac: str = ""):
        slot_key = str(slot).strip().lower()
        if slot_key not in ("walking", "cycling"):
            return {"ok": False, "slot": slot_key, "reason": "invalid_slot"}

        detector_attr = "imu_model_detector" if slot_key == "walking" else "cycling_toggle_detector"
        detector = getattr(self, detector_attr, None)
        if detector is None:
            return {"ok": False, "slot": slot_key, "reason": "detector_unavailable"}

        normalized_mac = (mac or "").strip().upper()
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
                should_measure = bool(
                    getattr(getattr(self, "motor_controller_ref", None), "mechanical_zeroed", False)
                ) and self.should_measure_imu_slot(slot_key)
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
                                     'event_prob_threshold']:
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
                            # 最大力矩参数: 0-35 Nm
                            if 0.0 <= param_value <= 35.0:
                                self.motion_modes[mode_key][param_type] = param_value
                                self.get_logger().info(
                                    f"📊 参数更新: {self.motion_modes[mode_key]['name']} "
                                    f"{param_type} = {param_value:.2f} Nm"
                                )
                            else:
                                self.get_logger().warn(
                                    f"最大力矩参数值超出范围 (0.0-35.0): {param_name} = {param_value}"
                                )
                                return SetParametersResult(successful=False, 
                                                         reason="最大力矩参数值超出允许范围 (0.0-35.0 Nm)")
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
            return True
        else:
            print(f"❌ 错误: 未知运动模式 '{mode}'")
            print(f"可用模式: {list(self.motion_modes.keys())}")
            return False

    def set_stairs_down_manual_assist(self, enabled: bool | None = None):
        """设置/切换 stairs_down/test/walking_test 模式手动启停状态（按钮控制）。"""
        if self.current_motion_mode not in STAIRS_DOWN_MANUAL_MODES:
            return {
                "ok": False,
                "reason": "mode_not_stairs_down_or_test_family",
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
            if self.current_motion_mode in TEST_PEAK_PHASE_MODES:
                self.reset_phase_estimator()
            self.gait_state = 1
            self.assist_enable = True
        else:
            self.gait_state = 0
            self.assist_enable = False
            if self.current_motion_mode in TEST_PEAK_PHASE_MODES:
                self.reset_phase_estimator()
            else:
                self._clear_start_phase_init()

        if changed:
            self.stairs_down_manual_toggle_count += 1
            action = "开启" if target else "关闭"
            if self.current_motion_mode in TEST_PEAK_PHASE_MODES and target:
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
        return angle % (2 * np.pi)

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
        self.phi_L = init_phase
        self.current_left_phase = init_phase
        self.current_right_phase = self._wrap_to_2pi(init_phase + np.pi)

        side_cn = "左脚" if start_side == "left" else "右脚"
        self.get_logger().info(
            f"🎯 起步脚判别: angle_diff={angle_diff:.3f} rad, 先迈{side_cn}, 初始相位={init_phase:.3f} rad"
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
        """stairs_down/test模式：由按钮直接控制助力启停，不使用IMU模型。"""
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
                "mac_address": os.environ.get("GAIT_IMU_MAC", "D4:22:CD:00:8A:5A"),
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
                "mac_address": os.environ.get("GAIT_IMU_MAC", "D4:22:CD:00:8A:5A"),
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
                if hasattr(self, "adaptive_oscillator"):
                    self.adaptive_oscillator.update_dt(self.base_dt)
        self.last_process_time = now
        # 保持后续处理使用校准后的固定 dt
        self.dt = self.base_dt
        self.dt_filtered = self.base_dt
        if hasattr(self, "adaptive_oscillator"):
            self.adaptive_oscillator.update_dt(self.base_dt)
        
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
            self.get_logger().info(f"🔍 步态检测输入 - 左角度: {lhip_angle:.4f} rad, 右角度: {rhip_angle:.4f} rad")
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
        self.assist_output_active = False
        self.actual_left_torque = 0.0
        self.actual_right_torque = 0.0

        # 强制重置自适应振荡器，确保频率恢复到初始设定值 (OSC_OMEGA_INIT)
        # 这可以防止振荡器在停止后保留错误的频率状态（如谐波锁定）
        if hasattr(self, "adaptive_oscillator"):
            # 不重置滤波器，保持其对信号直流分量的适应，防止启动时的瞬态冲击
            self.adaptive_oscillator.reset(reset_filter=False)

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
        self.current_left_phase = float(result.get("left_phase_rad", 0.0))
        self.current_right_phase = float(result.get("right_phase_rad", 0.0))
        self.phi_L = self.current_left_phase
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

    def extract_foot_phase(self):
        """提取足部相位/频率（test/walking_test使用峰值法，其余模式使用RAO）。"""
        if self.current_motion_mode in TEST_PEAK_PHASE_MODES:
            self._extract_test_mode_phase()
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
                        self.phi_L = phase_preview
                        self.current_left_phase = phase_preview
                        self.current_right_phase = self._wrap_to_2pi(phase_preview + np.pi)
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

        self.phi_L = phase
        self.current_left_phase = phase
        self.current_right_phase = self._wrap_to_2pi(phase + np.pi)
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
            # 将相位从 0-2π 归一化到 0-1
            left_phase_normalized = self.current_left_phase / (2 * np.pi)
            right_phase_normalized = self.current_right_phase / (2 * np.pi)
        else:
            # 如果相位还没初始化，使用默认值
            self.get_logger().warn("⚠️ 相位未初始化，使用默认相位值")
            left_phase_normalized = 0.0
            right_phase_normalized = 0.5
        
        # 使用五次多项式计算当前时刻的助力力矩
        if self.current_motion_mode in TEST_PEAK_PHASE_MODES:
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
        
        # 运动检测/相位有效性：相位被置零或助力关闭时，不输出助力
        if not self.assist_enable or not getattr(self, "phase_active", False):
            left_torque = 0.0
            right_torque = 0.0
            if self.debug_mode and self.log_counter % 20 == 0:
                self.get_logger().info("🛑 步态检测：相位未激活或助力关闭，助力输出置零")
        
        # 确保业务层最终助力输出不超过 35 Nm；MIT 协议层的 ±54 Nm 限幅保持不变。
        left_torque = max(-35.0, min(35.0, left_torque))
        right_torque = max(-35.0, min(35.0, right_torque))
        
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
