#!/usr/bin/env python3

"""
优化的步态控制系统主程序
自动提供关节助力功能，配合移动端APP进行实时参数调节
移除了多余的ROS话题发布代码，专注于核心控制功能
"""

import json
import os
import signal
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
    """Wait for both motors to report periodic 0x29 feedback frames."""
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
                motor_controller.get_logger().info("✅ 双电机 0x29 周期状态帧已就绪，提前结束初始化等待")
                break
        
        motor_controller.get_logger().info("🤖 双电机初始化命令发送完毕")
        
        # 设置默认运动模式为步行
        gait_analysis.set_motion_mode('walking')
        motor_controller.get_logger().info("🚶 默认运动模式设置为: 步行模式")
        
        print("\n🦾 关节助力系统已启动")
        print("="*50)
        print("⚠️ 重要提示:")
        print("   1. 当前按 AK V3.2.0 伺服 CAN 协议通信")
        print("   2. 左电机 CAN ID=1，右电机 CAN ID=2")
        print("   3. 已跳过机械标零，直接使用现场已设零点")
        print("   4. 电机状态帧采用 50Hz 周期上报")
        print("")
        print("📱 APP蓝牙控制命令示例:")
        print('   - 切换模式: {"type":"set_mode","mode":"walking"}')
        print('   - 参数调整: {"type":"set_params","mode":"walking","params":{"ext_Tmax":3.2}}')
        print('   - 紧急停止: {"type":"command","name":"emergency_stop"}')
        print('   - 下楼梯/测试手动启停: {"type":"command","name":"stairs_down_toggle"}')
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
    torque_filter_alpha = 0.3  # MIT力矩平滑系数
    last_state_signature = None
    last_state_send_time = 0.0
    last_imu_status_signature = None
    last_bt_client_connected = False
    last_processed_left_ts = None
    last_processed_right_ts = None
    bt_plot_batch_frames = []
    bt_plot_batch_size = max(1, int(os.environ.get("GAIT_BT_PLOT_BATCH_SIZE", "5")))
    bt_plot_every_n_frames = max(1, int(os.environ.get("GAIT_BT_PLOT_EVERY_N_FRAMES", "25")))
    bt_plot_mode = str(os.environ.get("GAIT_BT_PLOT_MODE", "latest")).strip().lower()
    bt_plot_enabled = os.environ.get("GAIT_BT_PLOT_ENABLE", "1") != "0"
    bt_plot_frame_counter = 0
    app_runtime_enabled = False
    try:
        bt_state_min_interval = max(
            0.0, float(os.environ.get("GAIT_BT_STATE_MIN_INTERVAL_SEC", "1.0"))
        )
    except Exception:
        bt_state_min_interval = 1.0
    
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
        motor_controller.get_logger().info("🔧 初始化蓝牙RFCOMM服务...")

        def _bt_round(value: float, decimals: int = 4) -> float:
            return round(float(value), decimals)

        def _send_bt_payload(payload: dict) -> bool:
            if bluetooth_server is None:
                return False
            return bluetooth_server.send_line(
                json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
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
            payload = {
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
            payload["name"] = str(mode_params.get("name", mode_key))
            payload["name_en"] = str(mode_params.get("name_en", mode_key))
            payload["description"] = str(mode_params.get("description", ""))
            return payload

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

        def _build_state_signature(payload: dict) -> str:
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
                "type": "plot",
                "ts": _bt_round(ts, 3),
                "left_angle": _bt_round(left_angle),
                "right_angle": _bt_round(right_angle),
                # Keep telemetry consistent with RAO input definition: q = theta_l - theta_r.
                "angle_diff": _bt_round(float(left_angle) - float(right_angle)),
                "phase": _bt_round(phase),
                "assist": _bt_round(assist),
            }

        def _send_state_payload_if_changed(
            *,
            force: bool = False,
            mechanical_zero_ready: bool | None = None,
            motion_confirmed: bool | None = None,
        ) -> bool:
            nonlocal last_state_signature, last_state_send_time
            if bluetooth_server is None:
                return False
            payload = _build_gait_state_payload(
                mechanical_zero_ready=mechanical_zero_ready,
                motion_confirmed=motion_confirmed,
            )
            signature = _build_state_signature(payload)
            if not force and signature == last_state_signature:
                return False
            now = time.time()
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
            payload = {
                "type": "plot_batch",
                "frames": bt_plot_batch_frames,
            }
            sent = _send_bt_payload(payload)
            if sent:
                bt_plot_batch_frames = []
            return sent

        def _send_plot_payload(payload: dict) -> bool:
            return _send_bt_payload(payload)

        def _handle_bt_message(line: str):
            nonlocal app_runtime_enabled
            payload = None
            text = line.strip()
            if not text:
                return None
            motor_controller.get_logger().info(f"📱 APP命令 <- {text}")
            try:
                payload = json.loads(text)
            except Exception:
                if text.lower() == "ping":
                    motor_controller.get_logger().info("📱 APP命令解析: ping")
                    return json.dumps({"type": "pong"})
                motor_controller.get_logger().warn("⚠️ APP命令解析失败: invalid_json")
                return json.dumps({"type": "error", "message": "invalid_json"})

            msg_type = str(payload.get("type", "")).lower()
            if msg_type == "ping":
                motor_controller.get_logger().info("📱 APP命令解析: ping")
                return json.dumps({"type": "pong"})
            if msg_type == "get_state":
                motor_controller.get_logger().info("📱 APP操作: get_state")
                return json.dumps(_build_gait_state_payload(), ensure_ascii=False)
            if msg_type == "imu_manage":
                slot = str(payload.get("slot", "")).strip().lower()
                connect = bool(payload.get("connect", True))
                mac = str(payload.get("mac", "")).strip().upper()
                result = gait_analysis.control_imu_connection(slot=slot, connect=connect, mac=mac)
                motor_controller.get_logger().info(
                    "📱 APP操作: imu_manage "
                    f"slot={slot}, connect={connect}, "
                    f"mac={str(result.get('mac', mac or '-')) or '-'}, "
                    f"ok={result.get('ok')}, reason={str(result.get('reason', '')) or '-'}"
                )
                return json.dumps(
                    {
                        "type": "imu_manage_ack",
                        "ok": bool(result.get("ok", False)),
                        "slot": str(result.get("slot", slot)),
                        "connected": bool(result.get("connected", False)),
                        "measuring": bool(result.get("measuring", False)),
                        "ready": bool(result.get("ready", False)),
                        "stale": bool(result.get("stale", True)),
                        "mac": str(result.get("mac", mac)),
                        "label": str(result.get("label", "")),
                        "reason": str(result.get("reason", "")),
                    },
                    ensure_ascii=False,
                )
            if msg_type == "set_mode":
                mode = str(payload.get("mode", "")).strip()
                if not mode:
                    motor_controller.get_logger().warn("⚠️ APP命令缺少字段: mode")
                    return json.dumps({"type": "error", "message": "mode_required"})
                ok = gait_analysis.set_motion_mode(mode)
                motor_controller.get_logger().info(
                    f"📱 APP操作: set_mode mode={mode}, ok={ok}"
                )
                return json.dumps({"type": "ack", "ok": ok, "message": f"mode={mode}"})
            if msg_type == "set_params":
                mode = str(payload.get("mode", gait_analysis.current_motion_mode)).strip()
                params = payload.get("params", {}) or {}
                updates = []
                for key, value in params.items():
                    name = f"{mode}.{key}"
                    try:
                        updates.append(Parameter(name, value=float(value)))
                    except Exception:
                        motor_controller.get_logger().warn(
                            f"⚠️ APP参数非法: {key}={value}"
                        )
                        return json.dumps({"type": "error", "message": f"invalid_value:{key}"})
                if not updates:
                    motor_controller.get_logger().warn("⚠️ APP命令缺少字段: params")
                    return json.dumps({"type": "error", "message": "no_params"})
                results = gait_analysis.set_parameters(updates)
                ok = all(getattr(r, "successful", False) for r in results)
                motor_controller.get_logger().info(
                    f"📱 APP操作: set_params mode={mode}, count={len(updates)}, ok={ok}"
                )
                return json.dumps({"type": "ack", "ok": ok, "message": f"params={len(updates)}"})
            if msg_type == "command":
                name = str(payload.get("name", "")).strip().lower()
                if name in ("start_assist", "start"):
                    current_mode = str(getattr(gait_analysis, "current_motion_mode", "walking"))
                    manual_mode = current_mode in STAIRS_DOWN_MANUAL_MODES
                    zero_ok = bool(motor_controller.execute_mechanical_zero())
                    left_ok = bool(motor_controller.enable_motor(LEFT_MOTOR_ID))
                    right_ok = bool(motor_controller.enable_motor(RIGHT_MOTOR_ID))
                    reason = ""
                    enabled = bool(getattr(gait_analysis, "assist_enable", False))

                    if not zero_ok:
                        reason = "mechanical_zero_failed"
                    elif not left_ok or not right_ok:
                        reason = "motor_enable_failed"

                    if not reason and manual_mode:
                        result = gait_analysis.set_stairs_down_manual_assist(enabled=True)
                        current_mode = str(result.get("mode", current_mode))
                        enabled = bool(result.get("enabled", False))
                        if not bool(result.get("ok", False)):
                            reason = str(result.get("reason", "")) or "manual_start_failed"
                    else:
                        enabled = bool(getattr(gait_analysis, "assist_enable", False))

                    ok = not reason
                    app_runtime_enabled = ok
                    motor_controller.get_logger().info(
                        "📱 APP操作: command start_assist "
                        f"mode={current_mode}, manual_mode={manual_mode}, "
                        f"enabled={enabled}, run_enabled={app_runtime_enabled}, "
                        f"ok={ok}, reason={reason or '-'}"
                    )
                    return json.dumps(
                        {
                            "type": "ack",
                            "ok": ok,
                            "message": "start_assist",
                            "mode": current_mode,
                            "enabled": enabled,
                            "reason": reason,
                        },
                        ensure_ascii=False,
                    )
                if name == "mechanical_zero":
                    motor_controller.get_logger().info("📱 APP操作: command mechanical_zero ignored (系统默认已就绪)")
                    return json.dumps(
                        {"type": "ack", "ok": True, "message": "system_already_ready"}
                    )
                if name == "motor_enable":
                    motor_controller.get_logger().info("📱 APP操作: command motor_enable ignored (系统默认已就绪)")
                    return json.dumps({"type": "ack", "ok": True, "message": "system_already_ready"})
                if name == "stairs_down_toggle":
                    value = payload.get("value", None)
                    desired = None if value is None else bool(value)
                    result = gait_analysis.set_stairs_down_manual_assist(enabled=desired)
                    ok = bool(result.get("ok", False))
                    enabled = bool(result.get("enabled", False))
                    reason = str(result.get("reason", ""))
                    if ok and enabled:
                        app_runtime_enabled = True
                    motor_controller.get_logger().info(
                        f"📱 APP操作: command stairs_down_toggle enabled={enabled}, "
                        f"run_enabled={app_runtime_enabled}, ok={ok}, reason={reason or '-'}"
                    )
                    return json.dumps(
                        {
                            "type": "ack",
                            "ok": ok,
                            "message": "stairs_down_assist",
                            "enabled": enabled,
                            "mode": str(result.get("mode", gait_analysis.current_motion_mode)),
                            "reason": reason,
                        },
                        ensure_ascii=False,
                    )
                if name == "emergency_stop":
                    app_runtime_enabled = False
                    if str(getattr(gait_analysis, "current_motion_mode", "walking")) in STAIRS_DOWN_MANUAL_MODES:
                        gait_analysis.set_stairs_down_manual_assist(enabled=False)
                    motor_controller.emergency_stop()
                    motor_controller.get_logger().info("📱 APP操作: command emergency_stop")
                    return json.dumps({"type": "ack", "ok": True, "message": name})
                motor_controller.get_logger().warn(f"⚠️ APP未知命令: {name}")
                return json.dumps({"type": "error", "message": f"unknown_command:{name}"})
            if msg_type == "publish_mode":
                mode = str(payload.get("mode", "")).strip()
                msg = RosString()
                msg.data = mode
                motor_controller.motion_mode_callback(msg)
                motor_controller.get_logger().info(f"📱 APP操作: publish_mode mode={mode}")
                return json.dumps({"type": "ack", "ok": True, "message": f"publish_mode={mode}"})
            motor_controller.get_logger().warn(f"⚠️ APP未知消息类型: {msg_type}")
            return json.dumps({"type": "error", "message": f"unknown_type:{msg_type}"})

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
