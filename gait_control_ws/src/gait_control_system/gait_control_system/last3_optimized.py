#!/usr/bin/env python3

"""
优化的步态控制系统主程序
自动提供关节助力功能，配合移动端APP进行实时参数调节
移除了多余的ROS话题发布代码，专注于核心控制功能
"""

import json
import os
import signal
import struct
import subprocess
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String as RosString

from .bluetooth_server import build_default_server
from .can_utils import cleanup_can_interface, setup_can_interface
from .adaptive_oscillator_estimator import AdaptiveOscillatorEstimator
from .gait_constants import (
    AO_CONFIG,
    CAN_INTERFACE,
    LEFT_MOTOR_ID,
    RIGHT_MOTOR_ID,
)
from .motor_controller import MotorController
from .real_time_gait_analysis import RealTimeGaitAnalysis, STAIRS_DOWN_MANUAL_MODES


def _is_can_interface_up(interface: str = CAN_INTERFACE) -> bool:
    """Check whether socketcan interface is UP."""
    try:
        result = subprocess.run(
            ["ip", "link", "show", interface],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return False
        output = result.stdout or ""
        return ("state UP" in output) or ("<" in output and "UP" in output)
    except Exception:
        return False


def _wait_for_can_interface_up(
    interface: str = CAN_INTERFACE,
    timeout_sec: float = 3.0,
    poll_interval_sec: float = 0.1,
) -> bool:
    """Wait until CAN interface becomes available, return False on timeout."""
    timeout_sec = max(float(timeout_sec), 0.1)
    poll_interval_sec = max(float(poll_interval_sec), 0.05)
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if _is_can_interface_up(interface):
            return True
        time.sleep(poll_interval_sec)
    return _is_can_interface_up(interface)


def _wait_for_dual_motor_feedback(
    motor_controller: MotorController,
    timeout_sec: float = 1.0,
    poll_interval_sec: float = 0.05,
) -> bool:
    """Wait for both motors to report periodic RS01 feedback frames."""
    timeout_sec = max(float(timeout_sec), 0.05)
    poll_interval_sec = max(float(poll_interval_sec), 0.01)
    deadline = time.time() + timeout_sec
    left_ready = False
    right_ready = False

    while time.time() < deadline:
        motor_controller.poll_feedback(timeout=min(poll_interval_sec, 0.02), max_messages=50)
        left_ready = left_ready or motor_controller.has_fresh_feedback(
            LEFT_MOTOR_ID,
            timeout=timeout_sec,
        )
        right_ready = right_ready or motor_controller.has_fresh_feedback(
            RIGHT_MOTOR_ID,
            timeout=timeout_sec,
        )
        if left_ready and right_ready:
            return True
        time.sleep(poll_interval_sec)

    return left_ready and right_ready


def main():
    """主函数 - 自动启动关节助力系统"""
    print("🚀 启动关节助力系统...")
    print("💪 系统将自动提供关节助力，无需手动选择")
    
    # 检查是否启用调试模式（通过环境变量）
    debug_mode = os.environ.get('GAIT_DEBUG', '0') == '1'
    if debug_mode:
        print("� 调试模式已启用（详细日志输出）")
    else:
        print("⚡ 性能模式已启用（最小日志输出，适合RDK X5等嵌入式平台）")
        print("   提示：设置环境变量 GAIT_DEBUG=1 可启用调试模式")
    
    # 初始化CAN硬件
    setup_can_interface()
    can_ready_timeout = float(os.environ.get("GAIT_CAN_READY_TIMEOUT", "3.0"))
    if _wait_for_can_interface_up(interface=CAN_INTERFACE, timeout_sec=can_ready_timeout):
        print("✅ CAN接口就绪，继续启动")
    else:
        print("⚠️ CAN接口就绪检测超时，继续启动并由后续通信逻辑兜底")
    
    # 注册退出处理
    signal.signal(signal.SIGINT, lambda s, f: cleanup_can_interface())
    
    # ROS2节点初始化，传入调试模式参数
    rclpy.init()
    gait_analysis = RealTimeGaitAnalysis(debug_mode=debug_mode)
    motor_controller = MotorController(gait_analysis_ref=gait_analysis, debug_mode=debug_mode)
    
    # 设置互相引用
    gait_analysis.motor_controller_ref = motor_controller
    
    # 初始化数据记录系统
    gait_analysis.init_data_logging()

    try:
        motor_controller.get_logger().info("🚀 开始双电机伺服 CAN 初始化...")
        
        for i in range(3):
            motor_controller.get_logger().info(f"� 初始化尝试 ({i+1}/3)...")
            
            motor_controller.enable_motor(LEFT_MOTOR_ID)
            time.sleep(0.01)
            motor_controller.send_mit_torque_command(LEFT_MOTOR_ID, 0.0)
            
            motor_controller.enable_motor(RIGHT_MOTOR_ID)
            time.sleep(0.01)
            motor_controller.send_mit_torque_command(RIGHT_MOTOR_ID, 0.0)
            
            wait_timeout = 1.0 if i < 2 else 0.2
            if _wait_for_dual_motor_feedback(
                motor_controller,
                timeout_sec=wait_timeout,
                poll_interval_sec=0.02,
            ):
                motor_controller.get_logger().info("✅ 双电机 RS01 周期反馈已就绪，提前结束初始化等待")
                break
        
        motor_controller.get_logger().info("🤖 双电机初始化命令发送完毕")
        
        # 设置默认运动模式为步行
        gait_analysis.set_motion_mode('walking')
        motor_controller.get_logger().info("🚶 默认运动模式设置为: 步行模式")
        
        print("\n🦾 关节助力系统已启动")
        print("="*50)
        print("⚠️ 重要提示:")
        print("   1. 当前按 RS01 私有运控 CAN 协议通信")
        print("   2. 左电机 CAN ID=1，右电机 CAN ID=2")
        print("   3. APP 点击“开始传输并启动系统”后会重新使能电机并执行标零")
        print("   4. 电机状态帧采用 RS01 50Hz 主动上报（类型 2/24）")
        print("")
        print("📱 APP蓝牙控制命令示例:")
        print('   - 切换模式: {"t":3,"m":0}')
        print('   - 参数调整: {"t":4,"m":0,"u":[[4,3.2]]}')
        print('   - 紧急停止: {"t":5,"c":2}')
        print('   - 下楼梯/测试手动启停: {"t":5,"c":3,"v":1}')
        print("🖥️ 终端将实时显示来自APP的操作命令与执行结果")
        print("="*50)
        
    except Exception as e:
        motor_controller.get_logger().error(f"❌ 初始化失败: {e}")
        return

    # 添加数据处理相关的变量
    retry_count = 0
    max_retries = 5
    last_left_motor_torque = 0.0
    last_right_motor_torque = 0.0
    torque_filter_alpha = 0.3  # 电机目标力矩平滑系数
    last_state_signature = None
    last_state_send_time = 0.0
    last_imu_status_signature = None
    last_bt_client_connected = False
    last_processed_left_ts = None
    last_processed_right_ts = None
    bt_plot_batch_frames = []
    bt_plot_batch_size = max(1, int(os.environ.get("GAIT_BT_PLOT_BATCH_SIZE", "5")))
    bt_plot_every_n_frames = max(1, int(os.environ.get("GAIT_BT_PLOT_EVERY_N_FRAMES", "25")))
    bt_plot_mode = str(os.environ.get("GAIT_BT_PLOT_MODE", "batch")).strip().lower()
    bt_plot_format = str(os.environ.get("GAIT_BT_PLOT_FORMAT", "binary")).strip().lower()
    if bt_plot_format in ("json", "full", "legacy_json"):
        bt_plot_format = "legacy"
    elif bt_plot_format in ("compact_json", "c1"):
        bt_plot_format = "compact"
    elif bt_plot_format not in ("legacy", "compact", "binary"):
        bt_plot_format = "compact"
    bt_plot_enabled = os.environ.get("GAIT_BT_PLOT_ENABLE", "1") != "0"
    bt_plot_frame_counter = 0
    app_runtime_enabled = False
    try:
        bt_state_min_interval = max(
            0.0, float(os.environ.get("GAIT_BT_STATE_MIN_INTERVAL_SEC", "1.0"))
        )
    except Exception:
        bt_state_min_interval = 1.0
    try:
        bt_state_keepalive_interval = max(
            0.0, float(os.environ.get("GAIT_BT_STATE_KEEPALIVE_SEC", "2.0"))
        )
    except Exception:
        bt_state_keepalive_interval = 2.0
    bt_idle_state_push = os.environ.get("GAIT_BT_IDLE_STATE_PUSH", "0") != "0"
    # 为保证蓝牙协议数值化，状态包固定使用紧凑数值格式。
    bt_state_compact = True
    bt_state_compact_include_params = (
        os.environ.get("GAIT_BT_STATE_COMPACT_INCLUDE_PARAMS", "0") != "0"
    )
    bt_state_mode_order = (
        "walking",
        "stairs_up",
        "stairs_down",
        "test",
        "walking_test",
        "cycling",
        "uphill",
        "downhill",
    )
    bt_state_mode_to_code = {mode_key: idx for idx, mode_key in enumerate(bt_state_mode_order)}
    bt_msg_type_to_code = {
        "ping": 1,
        "get_state": 2,
        "set_mode": 3,
        "set_params": 4,
        "command": 5,
        "set_stream": 6,
        "imu_manage": 7,
        "state": 8,
        "plot": 9,
        "plot_batch": 10,
        "ack": 11,
        "error": 12,
        "pong": 13,
        "telemetry": 14,
        "imu_manage_ack": 15,
        "imu_manage_status": 16,
    }
    bt_msg_code_to_type = {code: name for name, code in bt_msg_type_to_code.items()}
    bt_command_code_to_name = {
        1: "start_assist",
        2: "emergency_stop",
        3: "stairs_down_toggle",
        4: "mechanical_zero",
        5: "motor_enable",
    }
    bt_slot_code_to_name = {
        0: "walking",
        1: "cycling",
    }
    bt_param_keys = (
        "ext_t0",
        "ext_tf",
        "ext_p",
        "ext_Tmax",
        "flex_t0",
        "flex_tf",
        "flex_p",
        "flex_Tmax",
        "phase_bias",
        "phase_bias_at_0p6",
        "phase_bias_slope",
        "event_prob_threshold",
    )
    bt_param_key_to_code = {name: idx + 1 for idx, name in enumerate(bt_param_keys)}
    bt_param_code_to_key = {code: name for name, code in bt_param_key_to_code.items()}
    bt_reason_code = {
        "ok": 0,
        "mechanical_zero_failed": 1,
        "motor_enable_failed": 2,
        "manual_prepare_failed": 3,
        "manual_mode_required": 4,
        "mode_not_stairs_down_or_test_family": 4,
        "unknown_command": 5,
        "unknown_mode": 6,
        "invalid_value": 7,
        "no_params": 8,
        "invalid_plot_format": 9,
        "invalid_plot_mode": 10,
        "invalid_plot_batch_size": 11,
        "invalid_plot_every_n_frames": 12,
        "mode_required": 13,
        "invalid_json": 14,
        "unknown_type": 15,
        "imu_control_failed": 16,
        "invalid_slot": 17,
    }
    bt_state_flag_bits = {
        "phase_active": 0,
        "assist_wait_next_zero": 1,
        "stairs_down_manual_assist": 2,
        "assist_enabled": 3,
        "assist_armed": 4,
        "assist_output_active": 5,
        "mechanical_zero_ready": 6,
        "motion_confirmed": 7,
        "run_enabled": 8,
        "test_left_phase_valid": 9,
        "test_right_phase_valid": 10,
        "test_left_assist_ready": 11,
        "test_right_assist_ready": 12,
        "imu_connected": 13,
        "imu_ready": 14,
        "imu_stale": 15,
        "imu_walk_connected": 16,
        "imu_walk_measuring": 17,
        "imu_walk_ready": 18,
        "imu_walk_stale": 19,
        "imu_cycle_connected": 20,
        "imu_cycle_measuring": 21,
        "imu_cycle_ready": 22,
        "imu_cycle_stale": 23,
    }
    
    # 性能监控
    loop_times = deque(maxlen=100)  # 记录最近100次循环时间
    last_perf_report_time = time.time()
    
    imu_measurement_enabled_by_slot = {"walking": None, "cycling": None}

    def _desired_imu_measurement_slots() -> dict[str, bool]:
        mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
        if mode in ("cycling", "uphill"):
            return {"walking": False, "cycling": True}
        if mode in ("stairs_down", "test", "walking_test"):
            return {"walking": False, "cycling": False}
        return {"walking": True, "cycling": False}

    def _sync_imu_measurement_for_mode(force: bool = False) -> None:
        nonlocal imu_measurement_enabled_by_slot
        targets = _desired_imu_measurement_slots()
        changed = False
        for slot_name, enabled in targets.items():
            if force or imu_measurement_enabled_by_slot.get(slot_name) != enabled:
                gait_analysis.set_imu_measurement_enabled(enabled, slot=slot_name)
                imu_measurement_enabled_by_slot[slot_name] = enabled
                changed = True
        if changed:
            mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
            walking_text = "开" if targets.get("walking") else "关"
            cycling_text = "开" if targets.get("cycling") else "关"
            motor_controller.get_logger().info(
                f"📶 IMU测量流按模式同步: mode={mode}, walking={walking_text}, cycling={cycling_text}"
            )

    bluetooth_server = None
    bt_enabled = os.environ.get("GAIT_BT_ENABLE", "1") != "0"
    if bt_enabled:
        motor_controller.get_logger().info("🔧 初始化蓝牙BLE Notify服务...")
        bt_plot_bin_magic = b"GBF1"
        bt_plot_bin_header_struct = struct.Struct("<4sBBH")
        bt_plot_bin_record_struct = struct.Struct("<Qhhhhh")

        def _bt_round(value: float, decimals: int = 4) -> float:
            return round(float(value), decimals)

        def _send_bt_payload(payload: dict) -> bool:
            if bluetooth_server is None:
                return False
            return bluetooth_server.send_line(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            )

        def _send_bt_binary(
            payload: bytes,
            *,
            payload_desc: str = "binary",
            soft_realtime: bool = True,
        ) -> bool:
            if bluetooth_server is None:
                return False
            if not hasattr(bluetooth_server, "send_bytes"):
                return False
            return bool(
                bluetooth_server.send_bytes(
                    payload,
                    payload_desc=payload_desc,
                    soft_realtime=soft_realtime,
                )
            )

        motor_controller.get_logger().info(
            "📡 蓝牙plot传输配置: "
            f"format={bt_plot_format}, mode={bt_plot_mode}, "
            f"batch_size={bt_plot_batch_size}, every_n={bt_plot_every_n_frames}"
        )

        def _log_imu_status_if_changed(imu_status: dict) -> None:
            nonlocal last_imu_status_signature
            status_parts = {}
            for slot_name in ("walking", "cycling"):
                slot_status = imu_status.get(slot_name, {}) or {}
                status_parts[slot_name] = {
                    "connected": bool(slot_status.get("connected", False)),
                    "measuring": bool(slot_status.get("measuring", False)),
                    "ready": bool(slot_status.get("ready", False)),
                    "stale": bool(slot_status.get("stale", True)),
                    "mac": str(slot_status.get("mac", "")),
                    "last_error": str(slot_status.get("last_error", "")),
                }
            signature = json.dumps(status_parts, ensure_ascii=False, sort_keys=True)
            if signature == last_imu_status_signature:
                return
            last_imu_status_signature = signature
            for slot_name, slot_status in status_parts.items():
                error = str(slot_status.get("last_error", "")).strip()
                connected = bool(slot_status.get("connected", False))
                measuring = bool(slot_status.get("measuring", False))
                ready = bool(slot_status.get("ready", False))
                stale = bool(slot_status.get("stale", True))
                mac = str(slot_status.get("mac", ""))
                message = (
                    f"📡 IMU状态 {slot_name}: connected={connected}, "
                    f"measuring={measuring}, ready={ready}, stale={stale}, mac={mac}"
                )
                if error and not connected:
                    motor_controller.get_logger().warn(f"{message}, error={error}")
                else:
                    motor_controller.get_logger().info(message)

        def _compact_imu_error_for_bt(error: str) -> str:
            text = str(error or "").strip()
            if not text:
                return ""
            transient_markers = (
                "等待IMU蓝牙连接通道",
                "正在扫描IMU",
                "正在连接IMU",
            )
            if any(marker in text for marker in transient_markers):
                return "connecting"
            if len(text) > 96:
                return text[:96]
            return text

        def _build_imu_status_fields() -> dict:
            imu_status = gait_analysis.get_imu_connection_status()
            _log_imu_status_if_changed(imu_status)
            walk_status = imu_status.get("walking", {}) or {}
            cycle_status = imu_status.get("cycling", {}) or {}
            walk_error = _compact_imu_error_for_bt(str(walk_status.get("last_error", "")))
            cycle_error = _compact_imu_error_for_bt(str(cycle_status.get("last_error", "")))
            return {
                # Backward-compatible walking IMU fields.
                "imu_connected": bool(walk_status.get("connected", False)),
                "imu_measuring": bool(walk_status.get("measuring", False)),
                "imu_ready": bool(walk_status.get("ready", False)),
                "imu_stale": bool(walk_status.get("stale", True)),
                "imu_last_error": walk_error,
                "imu_mac": str(walk_status.get("mac", "")),
                "imu_label": str(walk_status.get("label", "")),
                # Explicit slot status fields for APP IMU manager.
                "imu_walk_connected": bool(walk_status.get("connected", False)),
                "imu_walk_measuring": bool(walk_status.get("measuring", False)),
                "imu_walk_ready": bool(walk_status.get("ready", False)),
                "imu_walk_stale": bool(walk_status.get("stale", True)),
                "imu_walk_last_error": walk_error,
                "imu_walk_mac": str(walk_status.get("mac", "")),
                "imu_walk_label": str(walk_status.get("label", "")),
                "imu_cycle_connected": bool(cycle_status.get("connected", False)),
                "imu_cycle_measuring": bool(cycle_status.get("measuring", False)),
                "imu_cycle_ready": bool(cycle_status.get("ready", False)),
                "imu_cycle_stale": bool(cycle_status.get("stale", True)),
                "imu_cycle_last_error": cycle_error,
                "imu_cycle_mac": str(cycle_status.get("mac", "")),
                "imu_cycle_label": str(cycle_status.get("label", "")),
            }

        def _build_mode_params_payload(mode_key: str) -> dict:
            mode_params = gait_analysis.motion_modes.get(mode_key, {}) or {}
            return {
                "ext_t0": float(mode_params.get("ext_t0", 0.0)),
                "ext_tf": float(mode_params.get("ext_tf", 0.0)),
                "ext_p": float(mode_params.get("ext_p", 0.0)),
                "ext_Tmax": float(mode_params.get("ext_Tmax", 0.0)),
                "flex_t0": float(mode_params.get("flex_t0", 0.0)),
                "flex_tf": float(mode_params.get("flex_tf", 0.0)),
                "flex_p": float(mode_params.get("flex_p", 0.0)),
                "flex_Tmax": float(mode_params.get("flex_Tmax", 0.0)),
                "phase_bias": float(mode_params.get("phase_bias", 0.0)),
                "phase_bias_at_0p6": float(mode_params.get("phase_bias_at_0p6", 0.0)),
                "phase_bias_slope": float(mode_params.get("phase_bias_slope", 0.0)),
                "event_prob_threshold": float(mode_params.get("event_prob_threshold", 0.0)),
            }

        def _build_mode_param_pairs(mode_key: str) -> list[list[float]]:
            payload = _build_mode_params_payload(mode_key)
            pairs: list[list[float]] = []
            for key, value in payload.items():
                code = bt_param_key_to_code.get(key)
                if code is None:
                    continue
                pairs.append([int(code), float(value)])
            return pairs

        def _reason_to_code(reason: str | None) -> int:
            normalized = str(reason or "").strip().lower()
            if not normalized:
                return bt_reason_code["ok"]
            return int(bt_reason_code.get(normalized, bt_reason_code["unknown_type"]))

        def _decode_mode_from_payload(payload: dict, default_mode: str = "") -> str:
            mode = str(payload.get("mode", "")).strip()
            if mode:
                return mode
            mode_code_raw = payload.get("m", None)
            try:
                idx = int(mode_code_raw)
                if 0 <= idx < len(bt_state_mode_order):
                    return str(bt_state_mode_order[idx]).strip()
            except Exception:
                pass
            return default_mode

        def _build_gait_state_payload(
            mechanical_zero_ready: bool | None = None,
            motion_confirmed: bool | None = None,
        ) -> dict:
            current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
            mode_params = _build_mode_params_payload(current_mode)
            if mechanical_zero_ready is None:
                mechanical_zero_ready = bool(getattr(motor_controller, "mechanical_zeroed", False))
            if motion_confirmed is None:
                motion_confirmed = (
                    bool(getattr(motor_controller, "motion_confirmed_after_zero", False))
                    if mechanical_zero_ready
                    else True
                )
            assist_armed = bool(
                app_runtime_enabled
                and mechanical_zero_ready
                and motion_confirmed
                and gait_analysis.assist_enable
            )
            payload = {
                "type": "state",
                "t": bt_msg_type_to_code["state"],
                "motion_mode": current_mode,
                "mode_name": str(mode_params.get("name", current_mode)),
                "mode_description": str(mode_params.get("description", "")),
                "params": mode_params,
                "gait_state": int(getattr(gait_analysis, "gait_state", 0)),
                "phase_active": bool(getattr(gait_analysis, "phase_active", False)),
                "assist_wait_next_zero": bool(getattr(gait_analysis, "assist_wait_next_zero", False)),
                "stairs_down_manual_assist": bool(
                    getattr(gait_analysis, "stairs_down_manual_assist_enabled", False)
                ),
                "assist_enabled": assist_armed,
                "assist_armed": assist_armed,
                "assist_output_active": bool(getattr(gait_analysis, "assist_output_active", False)),
                "assist_left": float(getattr(gait_analysis, "actual_left_torque", 0.0)),
                "assist_right": float(getattr(gait_analysis, "actual_right_torque", 0.0)),
                "mechanical_zero_ready": bool(mechanical_zero_ready),
                "motion_confirmed": bool(motion_confirmed),
                "run_enabled": bool(app_runtime_enabled),
                "test_left_phase_valid": bool(
                    getattr(gait_analysis, "test_mode_left_phase_valid", False)
                ),
                "test_right_phase_valid": bool(
                    getattr(gait_analysis, "test_mode_right_phase_valid", False)
                ),
                "test_left_assist_ready": bool(
                    getattr(gait_analysis, "test_mode_left_assist_ready", False)
                ),
                "test_right_assist_ready": bool(
                    getattr(gait_analysis, "test_mode_right_assist_ready", False)
                ),
                "detection_score": float(getattr(gait_analysis, "r_value", 0.0)),
            }
            payload.update(_build_imu_status_fields())
            return payload

        def _encode_state_flags(flag_values: dict[str, bool]) -> int:
            flags = 0
            for key, value in flag_values.items():
                if not value:
                    continue
                bit = bt_state_flag_bits.get(key)
                if bit is None:
                    continue
                flags |= 1 << int(bit)
            return int(flags)

        def _build_compact_state_payload(
            mechanical_zero_ready: bool | None = None,
            motion_confirmed: bool | None = None,
        ) -> dict:
            current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
            if mechanical_zero_ready is None:
                mechanical_zero_ready = bool(getattr(motor_controller, "mechanical_zeroed", False))
            if motion_confirmed is None:
                motion_confirmed = (
                    bool(getattr(motor_controller, "motion_confirmed_after_zero", False))
                    if mechanical_zero_ready
                    else True
                )
            assist_armed = bool(
                app_runtime_enabled
                and mechanical_zero_ready
                and motion_confirmed
                and gait_analysis.assist_enable
            )
            imu_status = _build_imu_status_fields()
            flag_values = {
                "phase_active": bool(getattr(gait_analysis, "phase_active", False)),
                "assist_wait_next_zero": bool(getattr(gait_analysis, "assist_wait_next_zero", False)),
                "stairs_down_manual_assist": bool(
                    getattr(gait_analysis, "stairs_down_manual_assist_enabled", False)
                ),
                "assist_enabled": assist_armed,
                "assist_armed": assist_armed,
                "assist_output_active": bool(getattr(gait_analysis, "assist_output_active", False)),
                "mechanical_zero_ready": bool(mechanical_zero_ready),
                "motion_confirmed": bool(motion_confirmed),
                "run_enabled": bool(app_runtime_enabled),
                "test_left_phase_valid": bool(
                    getattr(gait_analysis, "test_mode_left_phase_valid", False)
                ),
                "test_right_phase_valid": bool(
                    getattr(gait_analysis, "test_mode_right_phase_valid", False)
                ),
                "test_left_assist_ready": bool(
                    getattr(gait_analysis, "test_mode_left_assist_ready", False)
                ),
                "test_right_assist_ready": bool(
                    getattr(gait_analysis, "test_mode_right_assist_ready", False)
                ),
                "imu_connected": bool(imu_status.get("imu_connected", False)),
                "imu_ready": bool(imu_status.get("imu_ready", False)),
                "imu_stale": bool(imu_status.get("imu_stale", True)),
                "imu_walk_connected": bool(imu_status.get("imu_walk_connected", False)),
                "imu_walk_measuring": bool(imu_status.get("imu_walk_measuring", False)),
                "imu_walk_ready": bool(imu_status.get("imu_walk_ready", False)),
                "imu_walk_stale": bool(imu_status.get("imu_walk_stale", True)),
                "imu_cycle_connected": bool(imu_status.get("imu_cycle_connected", False)),
                "imu_cycle_measuring": bool(imu_status.get("imu_cycle_measuring", False)),
                "imu_cycle_ready": bool(imu_status.get("imu_cycle_ready", False)),
                "imu_cycle_stale": bool(imu_status.get("imu_cycle_stale", True)),
            }
            mode_code = int(bt_state_mode_to_code.get(current_mode, -1))
            payload = {
                "t": bt_msg_type_to_code["state"],
                "v": 1,
                "mi": mode_code,
                "gs": int(getattr(gait_analysis, "gait_state", 0)),
                "f": _encode_state_flags(flag_values),
                "ds": _bt_round(float(getattr(gait_analysis, "r_value", 0.0)), 4),
            }
            if bt_state_compact_include_params:
                payload["u"] = _build_mode_param_pairs(current_mode)
            return payload

        def _build_state_signature(payload: dict) -> str:
            if "mi" in payload and "f" in payload:
                signature_payload = {
                    "t": payload.get("t"),
                    "v": payload.get("v"),
                    "mi": payload.get("mi"),
                    "gs": payload.get("gs"),
                    "f": payload.get("f"),
                    "u": payload.get("u"),
                }
                return json.dumps(
                    signature_payload,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                )
            # state 只用于同步离散状态位；连续变化的实时量放到 plot/plot_batch，
            # 否则电机运动时 assist/detection_score 每帧变化会把 state 也推成高频流。
            signature_payload = {
                "type": payload.get("type"),
                "motion_mode": payload.get("motion_mode"),
                "mode_name": payload.get("mode_name"),
                "mode_description": payload.get("mode_description"),
                "params": payload.get("params"),
                "gait_state": payload.get("gait_state"),
                "phase_active": payload.get("phase_active"),
                "assist_wait_next_zero": payload.get("assist_wait_next_zero"),
                "stairs_down_manual_assist": payload.get("stairs_down_manual_assist"),
                "assist_enabled": payload.get("assist_enabled"),
                "assist_armed": payload.get("assist_armed"),
                "assist_output_active": payload.get("assist_output_active"),
                "mechanical_zero_ready": payload.get("mechanical_zero_ready"),
                "motion_confirmed": payload.get("motion_confirmed"),
                "run_enabled": payload.get("run_enabled"),
                "test_left_phase_valid": payload.get("test_left_phase_valid"),
                "test_right_phase_valid": payload.get("test_right_phase_valid"),
                "test_left_assist_ready": payload.get("test_left_assist_ready"),
                "test_right_assist_ready": payload.get("test_right_assist_ready"),
                "imu_connected": payload.get("imu_connected"),
                "imu_measuring": payload.get("imu_measuring"),
                "imu_ready": payload.get("imu_ready"),
                "imu_stale": payload.get("imu_stale"),
                "imu_last_error": payload.get("imu_last_error"),
                "imu_mac": payload.get("imu_mac"),
                "imu_label": payload.get("imu_label"),
                "imu_walk_connected": payload.get("imu_walk_connected"),
                "imu_walk_measuring": payload.get("imu_walk_measuring"),
                "imu_walk_ready": payload.get("imu_walk_ready"),
                "imu_walk_stale": payload.get("imu_walk_stale"),
                "imu_walk_last_error": payload.get("imu_walk_last_error"),
                "imu_walk_mac": payload.get("imu_walk_mac"),
                "imu_walk_label": payload.get("imu_walk_label"),
                "imu_cycle_connected": payload.get("imu_cycle_connected"),
                "imu_cycle_measuring": payload.get("imu_cycle_measuring"),
                "imu_cycle_ready": payload.get("imu_cycle_ready"),
                "imu_cycle_stale": payload.get("imu_cycle_stale"),
                "imu_cycle_last_error": payload.get("imu_cycle_last_error"),
                "imu_cycle_mac": payload.get("imu_cycle_mac"),
                "imu_cycle_label": payload.get("imu_cycle_label"),
            }
            return json.dumps(
                signature_payload,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )

        def _is_test_peak_mode(mode_key: str) -> bool:
            return mode_key in ("test", "walking_test")

        def _select_plot_phase_and_assist(
            phase_left: float,
            phase_right: float,
            assist_left: float,
            assist_right: float,
        ) -> tuple[float, float]:
            current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
            if not _is_test_peak_mode(current_mode):
                return float(phase_left), float(assist_left)

            # test/walking_test 的实时绘图相位固定使用左腿相位，
            # 同时固定绘制左腿助力，避免左右腿混用导致图形跳变。
            return float(phase_left), float(assist_left)

        def _build_gait_plot_payload(
            *,
            ts: float,
            left_angle: float,
            right_angle: float,
            phase_left: float,
            phase_right: float,
            assist_left: float,
            assist_right: float,
        ) -> dict:
            phase, assist = _select_plot_phase_and_assist(
                phase_left=phase_left,
                phase_right=phase_right,
                assist_left=assist_left,
                assist_right=assist_right,
            )
            return {
                "x": _bt_round(ts, 3),
                "l": _bt_round(left_angle),
                "r": _bt_round(right_angle),
                # Keep telemetry consistent with RAO input definition: q = theta_l - theta_r.
                "d": _bt_round(float(left_angle) - float(right_angle)),
                "p": _bt_round(phase),
                "a": _bt_round(assist),
            }

        def _build_plot_compact_payload(plot_payload: dict) -> dict:
            return {
                "t": bt_msg_type_to_code["plot"],
                "v": 1,
                "x": float(plot_payload.get("x", 0.0)),
                "l": float(plot_payload.get("l", 0.0)),
                "r": float(plot_payload.get("r", 0.0)),
                "d": float(plot_payload.get("d", 0.0)),
                "p": float(plot_payload.get("p", 0.0)),
                "a": float(plot_payload.get("a", 0.0)),
            }

        def _build_plot_compact_tuple(plot_payload: dict) -> list[float]:
            return [
                float(plot_payload.get("x", 0.0)),
                float(plot_payload.get("l", 0.0)),
                float(plot_payload.get("r", 0.0)),
                float(plot_payload.get("d", 0.0)),
                float(plot_payload.get("p", 0.0)),
                float(plot_payload.get("a", 0.0)),
            ]

        def _clip_i16(value: float, scale: float) -> int:
            scaled = int(round(float(value) * float(scale)))
            if scaled > 32767:
                return 32767
            if scaled < -32768:
                return -32768
            return scaled

        def _pack_plot_binary_frames(
            frames: list[dict],
            *,
            force_batch: bool = False,
        ) -> bytes:
            if not frames:
                return b""
            kind = 2 if (force_batch or len(frames) > 1) else 1
            frame_count = len(frames)
            if frame_count > 255:
                frames = frames[:255]
                frame_count = len(frames)
            records = []
            for frame in frames:
                ts_ms = int(round(float(frame.get("x", 0.0)) * 1000.0))
                if ts_ms < 0:
                    ts_ms = 0
                ts_ms &= 0xFFFFFFFFFFFFFFFF
                records.append(
                    bt_plot_bin_record_struct.pack(
                        ts_ms,
                        _clip_i16(frame.get("l", 0.0), 100.0),
                        _clip_i16(frame.get("r", 0.0), 100.0),
                        _clip_i16(frame.get("d", 0.0), 100.0),
                        _clip_i16(frame.get("p", 0.0), 1000.0),
                        _clip_i16(frame.get("a", 0.0), 100.0),
                    )
                )
            payload = b"".join(records)
            return bt_plot_bin_header_struct.pack(
                bt_plot_bin_magic,
                kind,
                frame_count,
                len(payload),
            ) + payload

        def _send_state_payload_if_changed(
            *,
            force: bool = False,
            mechanical_zero_ready: bool | None = None,
            motion_confirmed: bool | None = None,
        ) -> bool:
            nonlocal last_state_signature, last_state_send_time
            if bluetooth_server is None:
                return False
            if not force and not app_runtime_enabled and not bt_idle_state_push:
                return False
            if bt_state_compact:
                payload = _build_compact_state_payload(
                    mechanical_zero_ready=mechanical_zero_ready,
                    motion_confirmed=motion_confirmed,
                )
            else:
                payload = _build_gait_state_payload(
                    mechanical_zero_ready=mechanical_zero_ready,
                    motion_confirmed=motion_confirmed,
                )
            signature = _build_state_signature(payload)
            now = time.time()
            if not force and signature == last_state_signature:
                if (
                    bt_state_keepalive_interval <= 0.0
                    or last_state_send_time <= 0.0
                    or (now - last_state_send_time) < bt_state_keepalive_interval
                ):
                    return False
            if (
                not force
                and last_state_send_time > 0.0
                and bt_state_min_interval > 0.0
                and (now - last_state_send_time) < bt_state_min_interval
            ):
                return False
            payload["ts"] = now
            sent = _send_bt_payload(payload)
            if sent:
                last_state_signature = signature
                last_state_send_time = now
            return sent

        def _flush_plot_batch() -> bool:
            nonlocal bt_plot_batch_frames
            if bluetooth_server is None or not bt_plot_batch_frames:
                return False
            if bt_plot_format == "binary":
                payload_bytes = _pack_plot_binary_frames(bt_plot_batch_frames, force_batch=True)
                sent = _send_bt_binary(
                    payload_bytes,
                    payload_desc=f"plot_bin[{len(bt_plot_batch_frames)}]",
                    soft_realtime=True,
                )
            elif bt_plot_format == "compact":
                payload = {
                    "t": bt_msg_type_to_code["plot_batch"],
                    "v": 1,
                    "f": [
                        _build_plot_compact_tuple(frame)
                        for frame in bt_plot_batch_frames
                    ],
                }
                sent = _send_bt_payload(payload)
            else:
                payload = {
                    "t": bt_msg_type_to_code["plot_batch"],
                    "f": [
                        _build_plot_compact_tuple(frame)
                        for frame in bt_plot_batch_frames
                    ],
                }
                sent = _send_bt_payload(payload)
            if sent:
                bt_plot_batch_frames = []
            return sent

        def _send_plot_payload(payload: dict) -> bool:
            if bt_plot_format == "binary":
                payload_bytes = _pack_plot_binary_frames([payload], force_batch=False)
                return _send_bt_binary(payload_bytes, payload_desc="plot_bin", soft_realtime=True)
            if bt_plot_format == "compact":
                return _send_bt_payload(_build_plot_compact_payload(payload))
            return _send_bt_payload(payload)

        def _handle_bt_message(line: str):
            nonlocal app_runtime_enabled
            nonlocal bt_plot_mode
            nonlocal bt_plot_format
            nonlocal bt_plot_batch_size
            nonlocal bt_plot_every_n_frames
            nonlocal bt_plot_batch_frames
            nonlocal bt_plot_frame_counter
            payload = None
            text = line.strip()
            if not text:
                return None
            motor_controller.get_logger().info(f"📱 APP命令 <- {text}")

            def _json_response(data: dict) -> str:
                return json.dumps(data, ensure_ascii=False, separators=(",", ":"))

            def _error_response(code_name: str, **extra: object) -> str:
                payload_err: dict[str, object] = {
                    "t": bt_msg_type_to_code["error"],
                    "o": 0,
                    "ec": int(bt_reason_code.get(code_name, bt_reason_code["unknown_type"])),
                }
                for key, value in extra.items():
                    payload_err[key] = value
                return _json_response(payload_err)

            def _mode_code(mode_name: str) -> int:
                return int(bt_state_mode_to_code.get(str(mode_name), -1))

            def _coerce_bool(value: object, default: bool = False) -> bool:
                if isinstance(value, bool):
                    return value
                if isinstance(value, (int, float)):
                    return int(value) != 0
                if isinstance(value, str):
                    token = value.strip().lower()
                    if token in ("1", "true", "on", "yes"):
                        return True
                    if token in ("0", "false", "off", "no"):
                        return False
                return default

            def _plot_format_to_code(name: str) -> int:
                return {"legacy": 0, "compact": 1, "binary": 2}.get(name, 1)

            def _plot_mode_to_code(name: str) -> int:
                return {"batch": 0, "sample": 1}.get(name, 0)

            def _mac_from_numeric(payload_obj: dict) -> str:
                raw_list = payload_obj.get("ma")
                if isinstance(raw_list, list):
                    bytes_out: list[int] = []
                    for item in raw_list:
                        try:
                            value = int(item)
                        except Exception:
                            return ""
                        if value < 0 or value > 255:
                            return ""
                        bytes_out.append(value)
                    if len(bytes_out) == 6:
                        return ":".join(f"{value:02X}" for value in bytes_out)
                return str(payload_obj.get("mac", "")).strip().upper()

            try:
                payload = json.loads(text)
            except Exception:
                if text.lower() == "ping":
                    motor_controller.get_logger().info("📱 APP命令解析: ping")
                    return _json_response({"t": bt_msg_type_to_code["pong"], "o": 1})
                motor_controller.get_logger().warn("⚠️ APP命令解析失败: invalid_json")
                return _error_response("invalid_json")

            msg_type = str(payload.get("type", "")).strip().lower()
            if not msg_type:
                raw_type_code = payload.get("t", None)
                try:
                    msg_type_code = int(raw_type_code)
                except Exception:
                    msg_type_code = None
                if msg_type_code is not None:
                    msg_type = str(bt_msg_code_to_type.get(msg_type_code, "")).strip().lower()
            if msg_type == "ping":
                motor_controller.get_logger().info("📱 APP命令解析: ping")
                return _json_response({"t": bt_msg_type_to_code["pong"], "o": 1})
            if msg_type == "get_state":
                motor_controller.get_logger().info("📱 APP操作: get_state")
                return _json_response(_build_compact_state_payload())
            if msg_type == "imu_manage":
                slot = str(payload.get("slot", "")).strip().lower()
                if not slot:
                    try:
                        slot_code_raw = payload.get("s", None)
                        slot = str(bt_slot_code_to_name.get(int(slot_code_raw), "")).strip().lower()
                    except Exception:
                        slot = ""
                if not slot:
                    return _error_response("invalid_slot")
                connect = _coerce_bool(payload.get("x", payload.get("connect", True)), default=True)
                mac = _mac_from_numeric(payload)
                result = gait_analysis.control_imu_connection(slot=slot, connect=connect, mac=mac)
                resolved_slot = str(result.get("slot", slot))
                slot_code_value = None
                for _code, _name in bt_slot_code_to_name.items():
                    if _name == resolved_slot:
                        slot_code_value = int(_code)
                        break
                ok = bool(result.get("ok", False))
                reason_code = _reason_to_code(str(result.get("reason", "")) if not ok else "ok")
                motor_controller.get_logger().info(
                    "📱 APP操作: imu_manage "
                    f"slot={slot}, connect={connect}, "
                    f"mac={str(result.get('mac', mac or '-')) or '-'}, "
                    f"ok={result.get('ok')}, reason={str(result.get('reason', '')) or '-'}"
                )
                return _json_response(
                    {
                        "t": bt_msg_type_to_code["imu_manage_ack"],
                        "o": 1 if ok else 0,
                        "s": slot_code_value,
                        "k": 1 if bool(result.get("connected", False)) else 0,
                        "q": 1 if bool(result.get("measuring", False)) else 0,
                        "d": 1 if bool(result.get("ready", False)) else 0,
                        "z": 1 if bool(result.get("stale", True)) else 0,
                        "r": reason_code,
                    }
                )
            if msg_type == "set_mode":
                mode = _decode_mode_from_payload(payload)
                if not mode:
                    motor_controller.get_logger().warn("⚠️ APP命令缺少字段: mode")
                    return _error_response("mode_required")
                ok = gait_analysis.set_motion_mode(mode)
                applied_mode = str(getattr(gait_analysis, "current_motion_mode", mode))
                applied_mode_code = bt_state_mode_to_code.get(applied_mode, -1)
                motor_controller.get_logger().info(
                    f"📱 APP操作: set_mode mode={mode}, applied={applied_mode}, ok={ok}"
                )
                response = {
                    "t": bt_msg_type_to_code["ack"],
                    "o": 1 if ok else 0,
                    "a": 4,
                    "m": applied_mode_code,
                }
                if ok:
                    response["u"] = _build_mode_param_pairs(applied_mode)
                else:
                    response["r"] = _reason_to_code("unknown_mode")
                return _json_response(response)
            if msg_type == "set_params":
                requested_mode = _decode_mode_from_payload(payload)
                current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
                allow_offmode = _coerce_bool(
                    payload.get("ao", payload.get("allow_offmode", False)),
                    default=False,
                )
                mode = requested_mode or current_mode
                mode_overridden = False
                if requested_mode and requested_mode != current_mode and not allow_offmode:
                    mode = current_mode
                    mode_overridden = True
                    motor_controller.get_logger().warn(
                        "⚠️ APP参数模式与当前模式不一致，已改为当前模式应用: "
                        f"requested={requested_mode}, current={current_mode}"
                    )
                if mode not in gait_analysis.motion_modes:
                    motor_controller.get_logger().warn(
                        f"⚠️ APP命令模式无效: requested={requested_mode or '-'}, target={mode}"
                    )
                    return _error_response("unknown_mode")
                params = payload.get("p", payload.get("params", {})) or {}
                updates = []
                if isinstance(params, dict):
                    for key, value in params.items():
                        resolved_key = str(key).strip()
                        if resolved_key.isdigit():
                            resolved_key = str(
                                bt_param_code_to_key.get(int(resolved_key), resolved_key)
                            )
                        if resolved_key not in bt_param_key_to_code:
                            continue
                        name = f"{mode}.{resolved_key}"
                        try:
                            updates.append(Parameter(name, value=float(value)))
                        except Exception:
                            motor_controller.get_logger().warn(
                                f"⚠️ APP参数非法: {key}={value}"
                            )
                            return _error_response("invalid_value")
                raw_updates = payload.get("u", None)
                if isinstance(raw_updates, list):
                    for item in raw_updates:
                        if not isinstance(item, (list, tuple)) or len(item) < 2:
                            continue
                        try:
                            code = int(item[0])
                            key = bt_param_code_to_key.get(code)
                            if key is None:
                                continue
                            value = float(item[1])
                        except Exception:
                            return _error_response("invalid_value")
                        updates.append(Parameter(f"{mode}.{key}", value=value))
                if not updates:
                    motor_controller.get_logger().warn("⚠️ APP命令缺少字段: params")
                    return _error_response("no_params")
                results = gait_analysis.set_parameters(updates)
                ok = all(getattr(r, "successful", False) for r in results)
                failure_reason = ""
                if not ok:
                    failed = next(
                        (r for r in results if not getattr(r, "successful", False)),
                        None,
                    )
                    failure_reason = str(getattr(failed, "reason", "") or "")
                    if failure_reason:
                        motor_controller.get_logger().warn(
                            f"⚠️ APP参数应用失败: {failure_reason}"
                        )
                reason_code = _reason_to_code(failure_reason if failure_reason else "invalid_value")
                if not ok and reason_code == bt_reason_code["unknown_type"]:
                    reason_code = bt_reason_code["invalid_value"]
                resolved_mode_code = bt_state_mode_to_code.get(mode, -1)
                motor_controller.get_logger().info(
                    "📱 APP操作: set_params "
                    f"requested={requested_mode or current_mode}, target={mode}, "
                    f"count={len(updates)}, ok={ok}, overridden={mode_overridden}"
                )
                return _json_response(
                    {
                        "t": bt_msg_type_to_code["ack"],
                        "o": 1 if ok else 0,
                        "a": 5,
                        "m": resolved_mode_code,
                        "n": len(updates),
                        "ov": 1 if mode_overridden else 0,
                        "r": reason_code if not ok else bt_reason_code["ok"],
                    }
                )
            if msg_type == "set_stream":
                plot_format_value = payload.get("plot_format", payload.get("pf", bt_plot_format))
                plot_format_raw = str(plot_format_value).strip().lower()
                if plot_format_raw.isdigit():
                    plot_format_raw = {
                        "0": "legacy",
                        "1": "compact",
                        "2": "binary",
                    }.get(plot_format_raw, plot_format_raw)
                if plot_format_raw in ("json", "full", "legacy_json"):
                    requested_plot_format = "legacy"
                elif plot_format_raw in ("compact_json", "c1"):
                    requested_plot_format = "compact"
                elif plot_format_raw in ("binary", "legacy", "compact"):
                    requested_plot_format = plot_format_raw
                else:
                    motor_controller.get_logger().warn(
                        f"⚠️ APP流配置非法: plot_format={plot_format_raw}"
                    )
                    return _error_response("invalid_plot_format")

                plot_mode_value = payload.get("plot_mode", payload.get("pm", bt_plot_mode))
                plot_mode_raw = str(plot_mode_value).strip().lower()
                if plot_mode_raw.isdigit():
                    plot_mode_raw = {
                        "0": "batch",
                        "1": "sample",
                    }.get(plot_mode_raw, plot_mode_raw)
                if plot_mode_raw in ("batch", "every_n", "sample"):
                    requested_plot_mode = "batch" if plot_mode_raw == "batch" else "sample"
                else:
                    motor_controller.get_logger().warn(
                        f"⚠️ APP流配置非法: plot_mode={plot_mode_raw}"
                    )
                    return _error_response("invalid_plot_mode")

                try:
                    requested_batch_size = max(
                        1,
                        int(payload.get("plot_batch_size", payload.get("pn", bt_plot_batch_size))),
                    )
                except Exception:
                    return _error_response("invalid_plot_batch_size")
                try:
                    requested_every_n = max(
                        1,
                        int(payload.get("plot_every_n_frames", payload.get("pe", bt_plot_every_n_frames))),
                    )
                except Exception:
                    return _error_response("invalid_plot_every_n_frames")

                bt_plot_format = requested_plot_format
                bt_plot_mode = requested_plot_mode
                bt_plot_batch_size = requested_batch_size
                bt_plot_every_n_frames = requested_every_n
                bt_plot_batch_frames = []
                bt_plot_frame_counter = 0

                motor_controller.get_logger().info(
                    "📱 APP操作: set_stream "
                    f"format={bt_plot_format}, mode={bt_plot_mode}, "
                    f"batch_size={bt_plot_batch_size}, every_n={bt_plot_every_n_frames}"
                )
                return _json_response(
                    {
                        "t": bt_msg_type_to_code["ack"],
                        "o": 1,
                        "a": 3,
                        "pf": _plot_format_to_code(bt_plot_format),
                        "pm": _plot_mode_to_code(bt_plot_mode),
                        "pn": bt_plot_batch_size,
                        "pe": bt_plot_every_n_frames,
                    }
                )
            if msg_type == "command":
                name = str(payload.get("name", "")).strip().lower()
                if not name:
                    cmd_code_raw = payload.get("c", None)
                    try:
                        name = str(bt_command_code_to_name.get(int(cmd_code_raw), "")).strip().lower()
                    except Exception:
                        name = ""
                if name in ("start_assist", "start"):
                    current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
                    manual_mode = current_mode in STAIRS_DOWN_MANUAL_MODES
                    left_ok = bool(motor_controller.enable_motor(LEFT_MOTOR_ID))
                    right_ok = bool(motor_controller.enable_motor(RIGHT_MOTOR_ID))
                    zero_ok = bool(left_ok and right_ok and motor_controller.execute_mechanical_zero())
                    reason = ""
                    enabled = bool(getattr(gait_analysis, "assist_enable", False))

                    if not left_ok or not right_ok:
                        reason = "motor_enable_failed"
                    elif not zero_ok:
                        reason = "mechanical_zero_failed"

                    if not reason and manual_mode:
                        # 启动系统时，手动模式默认保持“助力关闭”，
                        # 只有用户后续点击手动启停按钮才会真正开助力。
                        result = gait_analysis.set_stairs_down_manual_assist(enabled=False)
                        current_mode = str(result.get("mode", current_mode))
                        enabled = bool(result.get("enabled", False))
                        if not bool(result.get("ok", False)):
                            reason = str(result.get("reason", "")) or "manual_prepare_failed"
                    else:
                        enabled = bool(getattr(gait_analysis, "assist_enable", False))

                    ok = not reason
                    app_runtime_enabled = ok
                    current_mode_code = bt_state_mode_to_code.get(current_mode, -1)
                    motor_controller.get_logger().info(
                        "📱 APP操作: command start_assist "
                        f"mode={current_mode}, manual_mode={manual_mode}, "
                        f"enabled={enabled}, run_enabled={app_runtime_enabled}, "
                        f"ok={ok}, reason={reason or '-'}"
                    )
                    return _json_response(
                        {
                            "t": bt_msg_type_to_code["ack"],
                            "o": 1 if ok else 0,
                            "a": 1,
                            "m": current_mode_code,
                            "e": 1 if enabled else 0,
                            "r": _reason_to_code(reason),
                        }
                    )
                if name == "mechanical_zero":
                    ok = bool(motor_controller.execute_mechanical_zero())
                    motor_controller.get_logger().info(
                        f"📱 APP操作: command mechanical_zero ok={ok}"
                    )
                    return _json_response(
                        {
                            "t": bt_msg_type_to_code["ack"],
                            "o": 1 if ok else 0,
                            "a": 0,
                            "r": _reason_to_code("" if ok else "mechanical_zero_failed"),
                        }
                    )
                if name == "motor_enable":
                    left_ok = bool(motor_controller.enable_motor(LEFT_MOTOR_ID))
                    right_ok = bool(motor_controller.enable_motor(RIGHT_MOTOR_ID))
                    ok = bool(left_ok and right_ok)
                    motor_controller.get_logger().info(
                        f"📱 APP操作: command motor_enable ok={ok}, left_ok={left_ok}, right_ok={right_ok}"
                    )
                    return _json_response(
                        {
                            "t": bt_msg_type_to_code["ack"],
                            "o": 1 if ok else 0,
                            "a": 0,
                            "r": _reason_to_code("" if ok else "motor_enable_failed"),
                        }
                    )
                if name == "stairs_down_toggle":
                    value = payload.get("value", payload.get("v", None))
                    if value is None:
                        desired = None
                    elif isinstance(value, bool):
                        desired = value
                    else:
                        try:
                            desired = int(value) != 0
                        except Exception:
                            desired = bool(value)
                    result = gait_analysis.set_stairs_down_manual_assist(enabled=desired)
                    ok = bool(result.get("ok", False))
                    enabled = bool(result.get("enabled", False))
                    reason = str(result.get("reason", ""))
                    if ok and enabled:
                        app_runtime_enabled = True
                    resolved_mode = str(result.get("mode", gait_analysis.current_motion_mode))
                    resolved_mode_code = bt_state_mode_to_code.get(resolved_mode, -1)
                    motor_controller.get_logger().info(
                        f"📱 APP操作: command stairs_down_toggle enabled={enabled}, "
                        f"run_enabled={app_runtime_enabled}, ok={ok}, reason={reason or '-'}"
                    )
                    return _json_response(
                        {
                            "t": bt_msg_type_to_code["ack"],
                            "o": 1 if ok else 0,
                            "a": 2,
                            "e": 1 if enabled else 0,
                            "m": resolved_mode_code,
                            "r": _reason_to_code(reason),
                        }
                    )
                if name == "emergency_stop":
                    app_runtime_enabled = False
                    if str(getattr(gait_analysis, "current_motion_mode", "walking")) in STAIRS_DOWN_MANUAL_MODES:
                        gait_analysis.set_stairs_down_manual_assist(enabled=False)
                    motor_controller.emergency_stop()
                    motor_controller.get_logger().info("📱 APP操作: command emergency_stop")
                    return _json_response(
                        {
                            "t": bt_msg_type_to_code["ack"],
                            "o": 1,
                            "a": 0,
                            "c": 2,
                        }
                    )
                motor_controller.get_logger().warn(f"⚠️ APP未知命令: {name}")
                return _error_response("unknown_command")
            if msg_type == "publish_mode":
                mode = _decode_mode_from_payload(payload)
                msg = RosString()
                msg.data = mode
                motor_controller.motion_mode_callback(msg)
                motor_controller.get_logger().info(f"📱 APP操作: publish_mode mode={mode}")
                return _json_response(
                    {
                        "t": bt_msg_type_to_code["ack"],
                        "o": 1,
                        "a": 4,
                        "m": _mode_code(mode),
                    }
                )
            motor_controller.get_logger().warn(f"⚠️ APP未知消息类型: {msg_type}")
            return _error_response("unknown_type")

        bluetooth_server = build_default_server(
            log_info=motor_controller.get_logger().info,
            log_warn=motor_controller.get_logger().warn,
            log_error=motor_controller.get_logger().error,
            on_message=_handle_bt_message,
        )
        bluetooth_server.start()
        motor_controller.get_logger().info("✅ 蓝牙服务线程已启动")
    else:
        motor_controller.get_logger().info("🔕 蓝牙服务已禁用 (GAIT_BT_ENABLE=0)")

    try:
        motor_controller.get_logger().info("🔄 进入主控制循环...")
        
        while rclpy.ok():
            loop_start_time = time.time()
            current_time = time.time()
            state_motion_confirmed = None
            bt_client_connected = bool(
                bluetooth_server is not None and bluetooth_server.is_client_connected()
            )
            force_state_sync = bool(bt_client_connected and not last_bt_client_connected)
            last_bt_client_connected = bt_client_connected

            _sync_imu_measurement_for_mode()
            
            motor_controller.poll_feedback(timeout=0.001, max_messages=50)

            lhip_angle, lhip_velocity = None, None
            rhip_angle, rhip_velocity = None, None
            left_ts = getattr(motor_controller, "left_motor_timestamp", None)
            right_ts = getattr(motor_controller, "right_motor_timestamp", None)

            if motor_controller.has_fresh_feedback(LEFT_MOTOR_ID, now=current_time):
                lhip_angle = motor_controller.left_motor_angle
                lhip_velocity = motor_controller.left_motor_velocity

            if motor_controller.has_fresh_feedback(RIGHT_MOTOR_ID, now=current_time):
                rhip_angle = motor_controller.right_motor_angle
                rhip_velocity = motor_controller.right_motor_velocity

            if lhip_angle is None or rhip_angle is None:
                retry_count += 1
                if retry_count >= max_retries:
                    motor_controller.get_logger().warn(
                        f"⚠️ 连续{max_retries}次未获得完整 50Hz 周期反馈，检查 CAN 连接或电机上报配置"
                    )
                    retry_count = 0
                
                if debug_mode and motor_controller.log_counter % 10 == 0:
                    motor_controller.get_logger().debug("⚠️ 当前周期缺少新鲜状态帧，跳过本次计算")
                time.sleep(0.001)
                continue
            else:
                retry_count = 0

            if left_ts is None or right_ts is None:
                time.sleep(0.001)
                continue

            dual_sample_ready = (
                (last_processed_left_ts is None or left_ts > last_processed_left_ts)
                and (last_processed_right_ts is None or right_ts > last_processed_right_ts)
            )
            if not dual_sample_ready:
                rclpy.spin_once(motor_controller, timeout_sec=0.001)
                rclpy.spin_once(gait_analysis, timeout_sec=0.001)
                time.sleep(0.001)
                continue
            
            if (
                debug_mode
                and motor_controller.show_first_data_after_zero
                and lhip_angle is not None
                and rhip_angle is not None
            ):
                motor_controller.show_first_data_after_zero = False
                motor_controller.get_logger().info("📊 收到双电机首帧有效状态数据:")
                motor_controller.get_logger().info(
                    f"   左电机 - 角度: {lhip_angle:.4f} rad ({lhip_angle*180/3.14159:.2f}°), "
                    f"角速度: {lhip_velocity:.4f} rad/s"
                )
                motor_controller.get_logger().info(
                    f"   右电机 - 角度: {rhip_angle:.4f} rad ({rhip_angle*180/3.14159:.2f}°), "
                    f"角速度: {rhip_velocity:.4f} rad/s"
                )
            
            # 每收到一组新的双电机状态帧就更新一次控制输出；
            # 蓝牙 plot 遥测默认每5组50Hz状态帧打包成1个 plot_batch 发送，约10Hz。
            if lhip_angle is not None and rhip_angle is not None:
                mechanical_zero_ready = motor_controller.mechanical_zeroed

                if mechanical_zero_ready:
                    # 更新步态分析
                    sensor_timestamp = None
                    if left_ts is not None and right_ts is not None:
                        sensor_timestamp = max(left_ts, right_ts)
                    elif left_ts is not None:
                        sensor_timestamp = left_ts
                    elif right_ts is not None:
                        sensor_timestamp = right_ts
                    gait_analysis.process_input_data(
                        rhip_angle, lhip_angle, rhip_velocity, lhip_velocity, sensor_timestamp
                    )
                
                # 使用独立计算的左右电机助力力矩
                if mechanical_zero_ready and gait_analysis.left_hipAss and gait_analysis.right_hipAss:
                    # 获取左右电机各自的助力力矩
                    left_torque = gait_analysis.left_hipAss[-1]
                    right_torque = gait_analysis.right_hipAss[-1]
                    
                else:
                    # 如果没有助力数据，使用零助力（不再提供默认助力）
                    left_torque = 0.0
                    right_torque = 0.0
                
                # 保留兼容状态位；当前版本默认零点已就绪，仅在助力禁用或运动未确认时强制清零。
                motion_confirmed = motor_controller.motion_confirmed_after_zero if motor_controller.mechanical_zeroed else True
                state_motion_confirmed = motion_confirmed
                if (
                    not app_runtime_enabled
                    or (mechanical_zero_ready and not gait_analysis.assist_enable)
                    or not mechanical_zero_ready
                    or not motion_confirmed
                ):
                    left_torque = 0.0
                    right_torque = 0.0
                    if debug_mode and motor_controller.log_counter % 20 == 0:
                        if not app_runtime_enabled:
                            motor_controller.get_logger().info("🛑 尚未按下开始按钮，力矩保持为零")
                        elif not mechanical_zero_ready:
                            motor_controller.get_logger().info("🛑 系统未就绪，力矩保持为零")
                        elif not motion_confirmed:
                            motor_controller.get_logger().info("🛑 运动未确认，力矩保持为零")
                        elif not gait_analysis.assist_enable:
                            motor_controller.get_logger().info("🛑 助力已禁用，强制力矩为零")

                assist_armed = bool(
                    app_runtime_enabled
                    and mechanical_zero_ready
                    and motion_confirmed
                    and gait_analysis.assist_enable
                )
                assist_output_active = bool(
                    assist_armed and (abs(left_torque) > 1e-6 or abs(right_torque) > 1e-6)
                )
                gait_analysis.assist_output_active = assist_output_active
                gait_analysis.actual_left_torque = float(left_torque)
                gait_analysis.actual_right_torque = float(right_torque)
                
                # 直接发送 MIT 力矩环指令。
                # 左侧安装方向与右侧相反，因此左侧力矩命令取反，右侧保持正号。
                new_left_motor_torque = -left_torque
                new_right_motor_torque = right_torque
                
                # 记录数据到CSV文件
                if lhip_angle is not None and rhip_angle is not None:
                    # 获取当前相位值（test/walking_test使用独立右腿相位，其它模式右腿=左腿+π）
                    phase_left_raw = float(getattr(gait_analysis, "phi_L", 0.0))
                    try:
                        phase_left = gait_analysis._wrap_to_2pi(phase_left_raw)
                    except Exception:
                        phase_left = phase_left_raw
                    try:
                        if getattr(gait_analysis, "current_motion_mode", "") in ("test", "walking_test"):
                            phase_right_raw = float(
                                getattr(gait_analysis, "current_right_phase", phase_left + np.pi)
                            )
                            phase_right = gait_analysis._wrap_to_2pi(phase_right_raw)
                        else:
                            phase_right = gait_analysis._wrap_to_2pi(phase_left + np.pi)
                    except Exception:
                        phase_right = phase_left + np.pi
                    
                    # 记录数据到 RealTimeGaitAnalysis 的日志（gait_logs 目录）
                    gait_analysis.log_data(
                        left_angle=lhip_angle,
                        right_angle=rhip_angle,
                        left_velocity=lhip_velocity,
                        right_velocity=rhip_velocity,
                        phase_left=phase_left,
                        phase_right=phase_right,
                        assist_left=left_torque,
                        assist_right=right_torque,
                        motion_detected=gait_analysis.assist_enable,
                        assist_enabled=app_runtime_enabled and mechanical_zero_ready and motion_confirmed and gait_analysis.assist_enable
                    )
                    
                    # 同时记录到 MotorController 的校准日志（calibration_logs 目录）
                    motor_controller.log_complete_gait_data(
                        left_angle=lhip_angle,
                        right_angle=rhip_angle,
                        left_velocity=lhip_velocity,
                        right_velocity=rhip_velocity,
                        phase_left=phase_left,
                        phase_right=phase_right,
                        assist_left=left_torque,
                        assist_right=right_torque,
                        motion_detected=gait_analysis.assist_enable,
                        assist_enabled=app_runtime_enabled and mechanical_zero_ready and motion_confirmed and gait_analysis.assist_enable
                    )
                    
                    if bluetooth_server is not None and bt_plot_enabled and app_runtime_enabled:
                        payload = _build_gait_plot_payload(
                            ts=sensor_timestamp if sensor_timestamp is not None else current_time,
                            left_angle=lhip_angle,
                            right_angle=rhip_angle,
                            phase_left=phase_left,
                            phase_right=phase_right,
                            assist_left=left_torque,
                            assist_right=right_torque,
                        )
                        if bt_plot_mode == "batch":
                            bt_plot_batch_frames.append(payload)
                            if len(bt_plot_batch_frames) >= bt_plot_batch_size:
                                _flush_plot_batch()
                        else:
                            bt_plot_frame_counter += 1
                            if bt_plot_frame_counter >= bt_plot_every_n_frames:
                                _send_plot_payload(payload)
                                bt_plot_frame_counter = 0

                    if bluetooth_server is not None:
                        _send_state_payload_if_changed(
                            force=force_state_sync,
                            mechanical_zero_ready=mechanical_zero_ready,
                            motion_confirmed=motion_confirmed,
                        )
                
                # 添加平滑处理，避免 MIT 力矩突变。
                filtered_left_torque = (
                    torque_filter_alpha * new_left_motor_torque
                    + (1 - torque_filter_alpha) * last_left_motor_torque
                )
                filtered_right_torque = (
                    torque_filter_alpha * new_right_motor_torque
                    + (1 - torque_filter_alpha) * last_right_motor_torque
                )
                last_left_motor_torque = filtered_left_torque
                last_right_motor_torque = filtered_right_torque
                
                # 将 MIT 力矩指令发送给左右电机
                if mechanical_zero_ready:
                    left_success = motor_controller.send_mit_torque_command(
                        LEFT_MOTOR_ID,
                        filtered_left_torque,
                    )
                    right_success = motor_controller.send_mit_torque_command(
                        RIGHT_MOTOR_ID,
                        filtered_right_torque,
                    )
                else:
                    # 兼容分支：系统未就绪时不向电机发送指令
                    left_success = right_success = True
                
                if mechanical_zero_ready and left_success and right_success:
                    last_processed_left_ts = left_ts
                    last_processed_right_ts = right_ts
                elif mechanical_zero_ready and (not left_success or not right_success):
                    motor_controller.get_logger().warn("❌ 发送MIT力矩命令失败")
                else:
                    last_processed_left_ts = left_ts
                    last_processed_right_ts = right_ts
            
            # 处理ROS2回调 - 同时处理两个节点
            if bluetooth_server is not None:
                if force_state_sync:
                    bt_plot_batch_frames = []
                    bt_plot_frame_counter = 0
                _send_state_payload_if_changed(
                    force=force_state_sync,
                    mechanical_zero_ready=bool(motor_controller.mechanical_zeroed),
                    motion_confirmed=state_motion_confirmed,
                )
            rclpy.spin_once(motor_controller, timeout_sec=0.001)
            rclpy.spin_once(gait_analysis, timeout_sec=0.001)
            
            # 短暂延时以控制循环频率
            time.sleep(0.001)
            
            # 性能监控：记录循环时间
            loop_end_time = time.time()
            loop_duration = loop_end_time - loop_start_time
            loop_times.append(loop_duration)
            
            # 每5秒输出一次性能统计
            if debug_mode and (loop_end_time - last_perf_report_time) >= 5.0:
                avg_loop_time = np.mean(list(loop_times)) * 1000  # 转换为ms
                max_loop_time = np.max(list(loop_times)) * 1000
                actual_freq = 1.0 / np.mean(list(loop_times)) if np.mean(list(loop_times)) > 0 else 0
                motor_controller.get_logger().info(f"📊 性能统计 - 平均循环: {avg_loop_time:.2f}ms, 最大: {max_loop_time:.2f}ms, 实际频率: {actual_freq:.1f}Hz")
                last_perf_report_time = loop_end_time
    
    finally:
        if bluetooth_server is not None:
            motor_controller.get_logger().info("🛑 停止蓝牙服务")
            bluetooth_server.stop()
        # 关闭数据记录
        try:
            gait_analysis.close_data_logging()
        except NameError:
            pass  # gait_analysis可能未初始化

        try:
            gait_analysis.close_imu_model_detector()
        except NameError:
            pass  # gait_analysis可能未初始化
        
        try:
            motor_controller.destroy_node()
        except NameError:
            pass  # motor_controller可能未初始化
            
        try:
            gait_analysis.destroy_node()
        except NameError:
            pass  # gait_analysis可能未初始化
            
        try:
            rclpy.shutdown()
        except:
            pass  # ROS可能已经关闭

if __name__ == "__main__":
    try:
        main()
    finally:
        cleanup_can_interface()
