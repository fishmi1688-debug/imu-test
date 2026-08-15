import os
import struct
import time

import can
from rclpy.node import Node
from std_msgs.msg import String

from .can_utils import setup_can_interface
from .gait_constants import (
    CAN_INTERFACE,
    LEFT_MOTOR_ID,
    LEFT_MOTOR_SIGN,
    MIT_KD_MAX,
    MIT_KD_MIN,
    MIT_KP_MAX,
    MIT_KP_MIN,
    MIT_P_MAX,
    MIT_P_MIN,
    MIT_T_MAX,
    MIT_T_MIN,
    MIT_V_MAX,
    MIT_V_MIN,
    MOTOR_FEEDBACK_TIMEOUT_SEC,
    RIGHT_MOTOR_ID,
    RIGHT_MOTOR_SIGN,
    RS01_ACTIVE_REPORT_FUNCTION_ID,
    RS01_ACTIVE_REPORT_HZ,
    RS01_CONTROL_FUNCTION_ID,
    RS01_ENABLE_FUNCTION_ID,
    RS01_EP_SCAN_TIME_INDEX,
    RS01_FEEDBACK_FUNCTION_IDS,
    RS01_MASTER_ID,
    RS01_MECHANICAL_ZERO_FUNCTION_ID,
    RS01_PARAM_WRITE_FUNCTION_ID,
    RS01_STOP_FUNCTION_ID,
    WORKSPACE_ROOT,
)


class MotorController(Node):
    """RS01 双电机 CAN 控制器。"""

    def __init__(self, gait_analysis_ref=None, debug_mode=False):
        super().__init__("motor_controller")
        self.gait_analysis = gait_analysis_ref
        self.debug_mode = debug_mode
        self.log_counter = 0
        self.feedback_timeout_sec = MOTOR_FEEDBACK_TIMEOUT_SEC
        self.master_can_id = RS01_MASTER_ID
        self.feedback_function_ids = set(RS01_FEEDBACK_FUNCTION_IDS)
        self.active_report_target_hz = max(1, int(RS01_ACTIVE_REPORT_HZ))

        # 维持现有业务层字段，但启动后需由 APP “开始”命令触发真实标零。
        self.mechanical_zeroed = False
        self.left_motor_zeroed = False
        self.right_motor_zeroed = False
        self.show_first_data_after_zero = False
        self.motion_confirmed_after_zero = True
        self.zero_completion_time = time.time()

        # 电机反馈缓存
        self.left_motor_angle = 0.0
        self.left_motor_velocity = 0.0
        self.left_motor_current = 0.0
        self.left_motor_temperature = 0.0
        self.left_motor_error = 0
        self.left_motor_timestamp = None

        self.right_motor_angle = 0.0
        self.right_motor_velocity = 0.0
        self.right_motor_current = 0.0
        self.right_motor_temperature = 0.0
        self.right_motor_error = 0
        self.right_motor_timestamp = None

        # 创建校准日志文件
        self.calibration_log_enabled = True
        self.calibration_log_handle = None
        self.calibration_log_pending_rows = 0
        self.calibration_log_flush_interval_rows = max(
            1, int(os.environ.get("GAIT_CALIBRATION_CSV_FLUSH_ROWS", "50"))
        )
        self.calibration_log_flush_interval_sec = max(
            0.1, float(os.environ.get("GAIT_CALIBRATION_CSV_FLUSH_SEC", "1.0"))
        )
        self.calibration_log_last_flush_time = time.monotonic()
        if self.calibration_log_enabled:
            log_dir = os.path.join(WORKSPACE_ROOT, "calibration_logs")
            os.makedirs(log_dir, exist_ok=True)

            import datetime

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.calibration_log_file = os.path.join(log_dir, f"motor_angles_{timestamp}.csv")
            self.calibration_log_handle = open(self.calibration_log_file, "w", encoding="utf-8")
            self.calibration_log_handle.write(
                "时间戳,左电机角度,右电机角度,左腿相位,右腿相位,左侧助力,右侧助力,运动检测,助力使能,备注\n"
            )
            self.calibration_log_handle.flush()
            self.calibration_log_last_flush_time = time.monotonic()

            self.get_logger().info("📊 完整步态数据记录已启用（包含角度、相位、助力等）")
            self.get_logger().info(f"📁 数据日志: {self.calibration_log_file}")
            self.get_logger().info("💡 数据包含: 电机角度、速度、相位、助力力矩和状态信息")

        self.motion_mode_pub = self.create_publisher(String, "/motion_mode/current", 10)
        self.motion_mode_sub = self.create_subscription(
            String,
            "/motion_mode/set",
            self.motion_mode_callback,
            10,
        )
        self.mechanical_zero_sub = self.create_subscription(
            String,
            "/mechanical_zero/command",
            self.mechanical_zero_callback,
            10,
        )
        self.motor_enable_sub = self.create_subscription(
            String,
            "/motor_enable/command",
            self.motor_enable_callback,
            10,
        )

        setup_can_interface()
        self.bus = can.interface.Bus(
            channel=CAN_INTERFACE,
            bustype="socketcan",
            rx_queue_size=1000,
            fd=False,
        )
        self.get_logger().info("🔗 CAN总线连接成功")
        self.get_logger().info("🤖 电机控制器初始化完成")
        self.get_logger().info("📨 订阅话题: /motion_mode/set (设置运动模式)")
        self.get_logger().info("🔧 订阅话题: /mechanical_zero/command (收到后实际发送 RS01 标零命令)")
        self.get_logger().info("🔌 订阅话题: /motor_enable/command (收到后实际发送 RS01 使能命令)")
        self.get_logger().info(
            "📡 当前按 RS01 私有运控协议工作，主动上报帧按说明书类型 2/24 解析"
        )
        self.get_logger().info(
            f"⏱️ 主动上报目标频率: {self.active_report_target_hz} Hz"
        )

    def _log_calibration_data(self, motor_side, raw_value, calculated_angle, note=""):
        """兼容保留：完整记录由 log_complete_gait_data 处理。"""
        return None

    def log_complete_gait_data(
        self,
        left_angle,
        right_angle,
        left_velocity,
        right_velocity,
        phase_left,
        phase_right,
        assist_left,
        assist_right,
        motion_detected,
        assist_enabled,
    ):
        if not self.calibration_log_enabled:
            return

        try:
            import datetime

            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            line = (
                f"{timestamp},{left_angle:.6f},{right_angle:.6f},"
                f"{phase_left:.6f},{phase_right:.6f},"
                f"{assist_left:.6f},{assist_right:.6f},"
                f"{motion_detected},{assist_enabled},数据记录\n"
            )
            if self.calibration_log_handle is None:
                self.calibration_log_handle = open(
                    self.calibration_log_file, "a", encoding="utf-8"
                )
            self.calibration_log_handle.write(line)
            self.calibration_log_pending_rows += 1
            flush_due_by_rows = (
                self.calibration_log_pending_rows >= self.calibration_log_flush_interval_rows
            )
            flush_due_by_time = (
                time.monotonic() - self.calibration_log_last_flush_time
                >= self.calibration_log_flush_interval_sec
            )
            if flush_due_by_rows or flush_due_by_time:
                self.flush_calibration_log()
        except Exception as exc:
            if self.debug_mode:
                self.get_logger().warn(f"完整数据记录失败: {exc}")

    def flush_calibration_log(self):
        if self.calibration_log_handle is None:
            return
        try:
            self.calibration_log_handle.flush()
            self.calibration_log_pending_rows = 0
            self.calibration_log_last_flush_time = time.monotonic()
        except Exception as exc:
            if self.debug_mode:
                self.get_logger().warn(f"完整数据刷新失败: {exc}")

    def close_calibration_log(self):
        if self.calibration_log_handle is None:
            return
        try:
            self.calibration_log_handle.flush()
            self.calibration_log_handle.close()
        except Exception as exc:
            if self.debug_mode:
                self.get_logger().warn(f"完整数据记录关闭失败: {exc}")
        finally:
            self.calibration_log_handle = None
            self.calibration_log_pending_rows = 0

    def log_motor_calibration_point(self, position_description):
        import datetime

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        left_angle = self.left_motor_angle
        right_angle = self.right_motor_angle

        try:
            with open(self.calibration_log_file, "a", encoding="utf-8") as f:
                f.write(
                    f"{timestamp},,{left_angle},,{right_angle},{position_description} (手动记录)\n"
                )
            self.get_logger().info(f"📝 手动校准点已记录: {position_description}")
            self.get_logger().info(f"📐 左电机: {left_angle:.3f} rad, 右电机: {right_angle:.3f} rad")
        except Exception as exc:
            self.get_logger().warn(f"手动校准记录失败: {exc}")

    def _build_arbitration_id(self, function_id, motor_id, value_field=0):
        return (
            ((int(function_id) & 0x1F) << 24)
            | ((int(value_field) & 0xFFFF) << 8)
            | (int(motor_id) & 0xFF)
        )

    def _make_can_frame(self, function_id, motor_id, data=b"", value_field=0):
        payload = bytes(data or b"\x00" * 8)
        return can.Message(
            arbitration_id=self._build_arbitration_id(function_id, motor_id, value_field),
            is_extended_id=True,
            data=payload,
        )

    def _motor_label(self, motor_id):
        return "左电机" if motor_id == LEFT_MOTOR_ID else "右电机"

    def _get_sign(self, motor_id):
        return LEFT_MOTOR_SIGN if motor_id == LEFT_MOTOR_ID else RIGHT_MOTOR_SIGN

    def _timestamp_attr(self, motor_id):
        return "left_motor_timestamp" if motor_id == LEFT_MOTOR_ID else "right_motor_timestamp"

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, float(value)))

    def _float_to_uint(self, value, lower, upper, bits):
        span = upper - lower
        value = self._clamp(value, lower, upper)
        max_int = (1 << bits) - 1
        return int((value - lower) * (max_int / span))

    def _uint_to_float(self, raw_value, lower, upper, bits):
        span = upper - lower
        max_int = (1 << bits) - 1
        raw_value = max(0, min(int(raw_value), max_int))
        return lower + (span * raw_value / max_int)

    def _decode_temperature(self, raw_high, raw_low):
        return struct.unpack(">H", bytes([raw_high & 0xFF, raw_low & 0xFF]))[0] / 10.0

    def _build_active_report_payload(self, enabled):
        return bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01 if enabled else 0x00, 0x00])

    def _build_param_write_payload(self, index, raw_bytes):
        payload = bytearray(8)
        payload[0] = int(index) & 0xFF
        payload[1] = (int(index) >> 8) & 0xFF
        raw_bytes = bytes(raw_bytes)
        payload[4:4 + min(len(raw_bytes), 4)] = raw_bytes[:4]
        return bytes(payload)

    def _report_interval_code_from_hz(self, report_hz):
        report_hz = max(1.0, float(report_hz))
        interval_ms = 1000.0 / report_hz
        raw_value = int(round((interval_ms - 10.0) / 5.0)) + 1
        return max(1, raw_value)

    def _update_motor_feedback(
        self,
        motor_id,
        angle_rad,
        velocity_rad_s,
        current_amp,
        temperature_c,
        error_code,
        timestamp,
    ):
        sign = self._get_sign(motor_id)
        angle_rad *= sign
        velocity_rad_s *= sign
        current_amp *= sign

        if motor_id == LEFT_MOTOR_ID:
            self.left_motor_angle = angle_rad
            self.left_motor_velocity = velocity_rad_s
            self.left_motor_current = current_amp
            self.left_motor_temperature = temperature_c
            self.left_motor_error = error_code
            self.left_motor_timestamp = timestamp
        elif motor_id == RIGHT_MOTOR_ID:
            self.right_motor_angle = angle_rad
            self.right_motor_velocity = velocity_rad_s
            self.right_motor_current = current_amp
            self.right_motor_temperature = temperature_c
            self.right_motor_error = error_code
            self.right_motor_timestamp = timestamp

        if self.debug_mode and self.log_counter % 50 == 0:
            self.get_logger().info(
                f"📡 {self._motor_label(motor_id)}状态帧 "
                f"角度={angle_rad:.4f} rad, 角速度={velocity_rad_s:.4f} rad/s, "
                f"反馈力矩={current_amp:.2f} Nm, 温度={temperature_c:.1f}°C, 故障={error_code}"
            )
        self.log_counter += 1

    def motion_mode_callback(self, msg):
        new_mode = msg.data.strip().lower()
        self.get_logger().info(f"📨 接收到运动模式切换请求: {new_mode}")

        if new_mode == "emergency_stop":
            self.emergency_stop()
            return

        if self.gait_analysis:
            if self.gait_analysis.set_motion_mode(new_mode):
                self.get_logger().info(
                    f"✅ 运动模式已切换: {self.gait_analysis.motion_modes[new_mode]['name']}"
                )
            else:
                self.get_logger().warn(f"❌ 运动模式切换失败: {new_mode}")
        else:
            self.get_logger().error("步态分析对象未初始化，无法切换运动模式")

    def mechanical_zero_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"📨 接收到兼容机械标零命令: {command}")
        if command == "mechanical_zero":
            self.execute_mechanical_zero()
        else:
            self.get_logger().warn(f"❌ 未知的兼容机械标零命令: {command}")

    def motor_enable_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"📨 接收到兼容电机使能命令: {command}")

        if command in ("enable", "enable_motors", "on", "true", "1", ""):
            self.enable_motor(LEFT_MOTOR_ID)
            self.enable_motor(RIGHT_MOTOR_ID)
            self.get_logger().info("✅ 双电机已发送 RS01 使能命令")
        else:
            self.get_logger().warn(f"❌ 未知的兼容电机使能命令: {command}")

    def execute_mechanical_zero(self):
        """按说明书发送 RS01 机械标零命令。"""
        self.mechanical_zeroed = False
        self.left_motor_zeroed = False
        self.right_motor_zeroed = False

        left_ok = self.send_mechanical_zero_command(LEFT_MOTOR_ID)
        time.sleep(0.02)
        right_ok = self.send_mechanical_zero_command(RIGHT_MOTOR_ID)
        ok = bool(left_ok and right_ok)
        if not ok:
            self.get_logger().warn("❌ 机械标零命令发送失败")
            return False

        self.mechanical_zeroed = True
        self.left_motor_zeroed = True
        self.right_motor_zeroed = True
        self.motion_confirmed_after_zero = True
        self.show_first_data_after_zero = True
        self.zero_completion_time = time.time()
        self.get_logger().info("✅ 已发送双电机机械标零命令")
        return True

    def send_mechanical_zero_command(self, motor_id):
        frame = self._make_can_frame(
            RS01_MECHANICAL_ZERO_FUNCTION_ID,
            motor_id,
            data=bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            value_field=self.master_can_id,
        )
        ok = self.send_frame(frame)
        if ok:
            self.get_logger().info(
                f"🎯 已发送 {self._motor_label(motor_id)} 机械标零命令 (CAN ID: 0x{frame.arbitration_id:08X})"
            )
        return ok

    def read_zero_response(self, motor_id, timeout):
        self.read_feedback_loop(motor_id, timeout=timeout)
        if motor_id == LEFT_MOTOR_ID:
            return self.left_motor_angle
        if motor_id == RIGHT_MOTOR_ID:
            return self.right_motor_angle
        return None

    def send_frame(self, frame):
        try:
            self.bus.send(frame, timeout=0.01)
            return True
        except can.CanError as exc:
            self.get_logger().warn(f"发送失败: {exc}")
            return False

    def write_uint16_parameter(self, motor_id, index, value):
        raw_value = max(0, min(int(value), 0xFFFF))
        payload = self._build_param_write_payload(index, struct.pack("<H", raw_value))
        frame = self._make_can_frame(
            RS01_PARAM_WRITE_FUNCTION_ID,
            motor_id,
            data=payload,
            value_field=self.master_can_id,
        )
        if self.debug_mode:
            self.get_logger().debug(
                f"📝 参数写入: ID=0x{frame.arbitration_id:08X}, index=0x{index:04X}, value={raw_value}, data={payload.hex()}"
            )
        return self.send_frame(frame)

    def configure_active_report_rate(self, motor_id, report_hz=None):
        if report_hz is None:
            report_hz = self.active_report_target_hz
        interval_code = self._report_interval_code_from_hz(report_hz)
        ok = self.write_uint16_parameter(motor_id, RS01_EP_SCAN_TIME_INDEX, interval_code)
        if ok:
            interval_ms = 10.0 + (interval_code - 1) * 5.0
            actual_hz = 1000.0 / interval_ms
            self.get_logger().info(
                f"⏱️ 已设置 {self._motor_label(motor_id)} 主动上报周期: {interval_ms:.1f} ms ({actual_hz:.1f} Hz)"
            )
        return ok

    def set_active_report(self, motor_id, enabled=True, report_hz=None):
        rate_ok = True
        if enabled:
            rate_ok = self.configure_active_report_rate(motor_id, report_hz=report_hz)
        frame = self._make_can_frame(
            RS01_ACTIVE_REPORT_FUNCTION_ID,
            motor_id,
            data=self._build_active_report_payload(enabled),
            value_field=self.master_can_id,
        )
        if self.debug_mode:
            self.get_logger().debug(
                f"📶 主动上报配置: ID=0x{frame.arbitration_id:08X}, enable={enabled}, data={bytes(frame.data).hex()}"
            )
        enable_ok = self.send_frame(frame)
        return bool(rate_ok and enable_ok)

    def send_mit_command(self, motor_id, position, velocity, kp, kd, torque):
        """兼容旧命名：按 RS01 私有运控协议发送控制帧。"""
        position = self._clamp(position, MIT_P_MIN, MIT_P_MAX)
        velocity = self._clamp(velocity, MIT_V_MIN, MIT_V_MAX)
        kp = self._clamp(kp, MIT_KP_MIN, MIT_KP_MAX)
        kd = self._clamp(kd, MIT_KD_MIN, MIT_KD_MAX)
        torque = self._clamp(torque, MIT_T_MIN, MIT_T_MAX)

        p_int = self._float_to_uint(position, MIT_P_MIN, MIT_P_MAX, 16)
        v_int = self._float_to_uint(velocity, MIT_V_MIN, MIT_V_MAX, 16)
        kp_int = self._float_to_uint(kp, MIT_KP_MIN, MIT_KP_MAX, 16)
        kd_int = self._float_to_uint(kd, MIT_KD_MIN, MIT_KD_MAX, 16)
        t_int = self._float_to_uint(torque, MIT_T_MIN, MIT_T_MAX, 16)

        data = bytes(
            [
                (p_int >> 8) & 0xFF,
                p_int & 0xFF,
                (v_int >> 8) & 0xFF,
                v_int & 0xFF,
                (kp_int >> 8) & 0xFF,
                kp_int & 0xFF,
                (kd_int >> 8) & 0xFF,
                kd_int & 0xFF,
            ]
        )
        frame = self._make_can_frame(
            RS01_CONTROL_FUNCTION_ID,
            motor_id,
            data=data,
            value_field=t_int,
        )
        if self.debug_mode:
            self.get_logger().debug(
                f"🧠 RS01运控指令: ID=0x{frame.arbitration_id:08X}, p={position:.3f}, v={velocity:.3f}, "
                f"kp={kp:.3f}, kd={kd:.3f}, t={torque:.3f}, data={data.hex()}"
            )
        return self.send_frame(frame)

    def send_mit_torque_command(self, motor_id, torque):
        """RS01 运控模式的纯扭矩指令：位置/速度/Kp/Kd 全置零。"""
        return self.send_mit_command(
            motor_id,
            position=0.0,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=torque,
        )

    def enable_motor(self, motor_id, active_report=True):
        report_ok = True
        if active_report:
            report_ok = self.set_active_report(
                motor_id,
                enabled=True,
                report_hz=self.active_report_target_hz,
            )
        frame = self._make_can_frame(
            RS01_ENABLE_FUNCTION_ID,
            motor_id,
            data=b"\x00" * 8,
            value_field=self.master_can_id,
        )
        enable_ok = self.send_frame(frame)
        ok = bool(report_ok and enable_ok)
        if ok:
            report_text = "并打开主动上报" if active_report else "但不打开主动上报"
            self.get_logger().info(
                f"🔌 电机已使能{report_text}: {self._motor_label(motor_id)} "
                f"(enable=0x{frame.arbitration_id:08X})"
            )
        else:
            self.get_logger().warn(
                f"❌ 电机使能失败: {self._motor_label(motor_id)} "
                f"(report_ok={report_ok}, enable_ok={enable_ok})"
            )
        return ok

    def disable_motor(self, motor_id):
        frame = self._make_can_frame(
            RS01_STOP_FUNCTION_ID,
            motor_id,
            data=b"\x00" * 8,
            value_field=self.master_can_id,
        )
        ok = self.send_frame(frame)
        if ok:
            self.get_logger().warn(
                f"🔒 电机失能: {self._motor_label(motor_id)} (CAN ID: 0x{frame.arbitration_id:08X})"
            )
        return ok

    def set_run_mode(self, motor_id, mode):
        """兼容保留：RS01 上电默认即为运控模式。"""
        if self.debug_mode:
            self.get_logger().info(
                f"⏭️ 忽略 set_run_mode({self._motor_label(motor_id)}, mode={mode})，默认使用 RS01 运控模式"
            )
        return True

    def _process_feedback_message(self, msg, motor_id=None):
        feedback_motor_id = (msg.arbitration_id >> 8) & 0xFF
        function_id = (msg.arbitration_id >> 24) & 0x1F
        error_code = (msg.arbitration_id >> 16) & 0x3F
        if motor_id is None:
            motor_id = feedback_motor_id

        if len(msg.data) < 8:
            if self.debug_mode:
                self.get_logger().warn(
                    f"⚠️ {self._motor_label(motor_id)} 状态帧长度不足: len={len(msg.data)}"
                )
            return False

        position_counts = struct.unpack(">H", bytes(msg.data[0:2]))[0]
        velocity_counts = struct.unpack(">H", bytes(msg.data[2:4]))[0]
        current_counts = struct.unpack(">H", bytes(msg.data[4:6]))[0]
        temperature_c = self._decode_temperature(msg.data[6], msg.data[7])

        angle_rad = self._uint_to_float(position_counts, MIT_P_MIN, MIT_P_MAX, 16)
        velocity_rad_s = self._uint_to_float(velocity_counts, MIT_V_MIN, MIT_V_MAX, 16)
        current_amp = self._uint_to_float(current_counts, MIT_T_MIN, MIT_T_MAX, 16)
        timestamp = getattr(msg, "timestamp", None) or time.time()

        self._update_motor_feedback(
            motor_id,
            angle_rad=angle_rad,
            velocity_rad_s=velocity_rad_s,
            current_amp=current_amp,
            temperature_c=temperature_c,
            error_code=error_code,
            timestamp=timestamp,
        )
        if self.debug_mode and self.log_counter % 200 == 0:
            self.get_logger().debug(
                f"📥 反馈帧解析: function=0x{function_id:02X}, motor_id={feedback_motor_id}, master_id=0x{msg.arbitration_id & 0xFF:02X}"
            )
        return True

    def poll_feedback(self, timeout=0.0, max_messages=50):
        """被动读取 RS01 周期反馈帧（说明书类型 2/24）。"""
        processed = 0
        first_read = True

        while processed < max_messages:
            read_timeout = timeout if first_read else 0.0
            first_read = False
            try:
                msg = self.bus.recv(timeout=read_timeout)
            except can.CanError as exc:
                if self.debug_mode:
                    self.get_logger().warn(f"读取CAN反馈失败: {exc}")
                break

            if msg is None:
                break
            if not msg.is_extended_id:
                continue

            function_id = (msg.arbitration_id >> 24) & 0x1F
            motor_id = (msg.arbitration_id >> 8) & 0xFF

            if function_id not in self.feedback_function_ids:
                if self.debug_mode and self.log_counter % 200 == 0:
                    self.get_logger().info(
                        f"⏭️ 跳过非状态帧: ID=0x{msg.arbitration_id:08X}, function=0x{function_id:02X}"
                    )
                continue

            if motor_id not in (LEFT_MOTOR_ID, RIGHT_MOTOR_ID):
                if self.debug_mode:
                    self.get_logger().warn(
                        f"⚠️ 收到未知驱动器状态帧: ID=0x{msg.arbitration_id:08X}"
                    )
                continue

            if self._process_feedback_message(msg, motor_id=motor_id):
                processed += 1

        return processed

    def read_single_motor_feedback(self, motor_name):
        try:
            return self.poll_feedback(timeout=0.01, max_messages=20) > 0
        except Exception as exc:
            self.get_logger().warn(f"读取{motor_name}电机反馈失败: {exc}")
            return False

    def has_fresh_feedback(self, motor_id, now=None, timeout=None):
        if timeout is None:
            timeout = self.feedback_timeout_sec
        if now is None:
            now = time.time()

        timestamp = getattr(self, self._timestamp_attr(motor_id))
        if timestamp is None:
            return False
        return (now - timestamp) <= timeout

    def request_feedback(self, motor_id):
        """兼容保留：通过类型 24 打开主动上报。"""
        return self.set_active_report(
            motor_id,
            enabled=True,
            report_hz=self.active_report_target_hz,
        )

    def emergency_stop(self):
        self.get_logger().warn("🚨 执行紧急停止！")
        self.send_mit_torque_command(LEFT_MOTOR_ID, 0.0)
        self.send_mit_torque_command(RIGHT_MOTOR_ID, 0.0)
        self.disable_motor(LEFT_MOTOR_ID)
        self.disable_motor(RIGHT_MOTOR_ID)

        emergency_msg = String()
        emergency_msg.data = "emergency_stop:紧急停止"
        self.motion_mode_pub.publish(emergency_msg)
        self.get_logger().warn("🛑 紧急停止完成 - 所有电机已停止")

    def read_feedback_loop(self, target_id, timeout=0.01):
        """等待目标电机出现有效周期反馈帧。"""
        deadline = time.time() + max(timeout, 0.0)
        while time.time() < deadline:
            remaining = max(deadline - time.time(), 0.0)
            self.poll_feedback(timeout=min(remaining, 0.02), max_messages=50)
            if self.has_fresh_feedback(target_id, timeout=max(timeout, self.feedback_timeout_sec)):
                return True
        return self.has_fresh_feedback(target_id, timeout=max(timeout, self.feedback_timeout_sec))

    def destroy_node(self):
        self.close_calibration_log()
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()
