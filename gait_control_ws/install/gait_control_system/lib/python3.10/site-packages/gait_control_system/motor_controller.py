import math
import os
import struct
import time

import can
from rclpy.node import Node
from std_msgs.msg import String

from .can_utils import setup_can_interface
from .gait_constants import (
    AK10_9_GEAR_RATIO,
    AK10_9_POLE_PAIRS,
    CAN_INTERFACE,
    CURRENT_AMP_PER_COUNT,
    ERPM_PER_COUNT,
    LEFT_MOTOR_ID,
    LEFT_MOTOR_SIGN,
    MIT_CONTROL_FUNCTION_ID,
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
    POSITION_DEG_PER_COUNT,
    RIGHT_MOTOR_ID,
    RIGHT_MOTOR_SIGN,
    SERVO_DISABLE_FUNCTION_ID,
    SERVO_STATUS_FUNCTION_ID,
    WORKSPACE_ROOT,
)


class MotorController(Node):
    """AK V3.2.0 双电机 CAN 控制器。"""

    def __init__(self, gait_analysis_ref=None, debug_mode=False):
        super().__init__("motor_controller")
        self.gait_analysis = gait_analysis_ref
        self.debug_mode = debug_mode
        self.log_counter = 0
        self.feedback_timeout_sec = MOTOR_FEEDBACK_TIMEOUT_SEC

        # 用户已提前设好零点，保留旧字段仅用于兼容现有业务逻辑。
        self.mechanical_zeroed = True
        self.left_motor_zeroed = True
        self.right_motor_zeroed = True
        self.show_first_data_after_zero = True
        self.motion_confirmed_after_zero = True
        self.zero_completion_time = time.time()

        # 电机反馈缓存
        self.left_motor_angle = 0.0
        self.left_motor_velocity = 0.0
        self.left_motor_current = 0.0
        self.left_motor_temperature = 0
        self.left_motor_error = 0
        self.left_motor_timestamp = None

        self.right_motor_angle = 0.0
        self.right_motor_velocity = 0.0
        self.right_motor_current = 0.0
        self.right_motor_temperature = 0
        self.right_motor_error = 0
        self.right_motor_timestamp = None

        # 创建校准日志文件
        self.calibration_log_enabled = True
        if self.calibration_log_enabled:
            log_dir = os.path.join(WORKSPACE_ROOT, "calibration_logs")
            os.makedirs(log_dir, exist_ok=True)

            import datetime

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.calibration_log_file = os.path.join(log_dir, f"motor_angles_{timestamp}.csv")
            with open(self.calibration_log_file, "w", encoding="utf-8") as f:
                f.write("时间戳,左电机角度,右电机角度,左腿相位,右腿相位,左侧助力,右侧助力,运动检测,助力使能,备注\n")

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
        self.get_logger().info("🔧 订阅话题: /mechanical_zero/command (兼容保留，当前默认跳过)")
        self.get_logger().info("🔌 订阅话题: /motor_enable/command (兼容保留，收到后仅确认MIT已就绪)")
        self.get_logger().info(
            "📡 当前按 AK V3.2.0 伺服 CAN 协议工作，等待电机以 50Hz 周期上报 0x29 状态帧"
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
            with open(self.calibration_log_file, "a", encoding="utf-8") as f:
                f.write(
                    f"{timestamp},{left_angle:.6f},{right_angle:.6f},"
                    f"{phase_left:.6f},{phase_right:.6f},"
                    f"{assist_left:.6f},{assist_right:.6f},"
                    f"{motion_detected},{assist_enabled},数据记录\n"
                )
        except Exception as exc:
            if self.debug_mode:
                self.get_logger().warn(f"完整数据记录失败: {exc}")

    def log_motor_calibration_point(self, position_description):
        import datetime

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        left_angle = self.left_motor_angle
        right_angle = self.right_motor_angle

        try:
            with open(self.calibration_log_file, "a", encoding="utf-8") as f:
                f.write(f"{timestamp},,{left_angle},,{right_angle},{position_description} (手动记录)\n")
            self.get_logger().info(f"📝 手动校准点已记录: {position_description}")
            self.get_logger().info(f"📐 左电机: {left_angle:.3f} rad, 右电机: {right_angle:.3f} rad")
        except Exception as exc:
            self.get_logger().warn(f"手动校准记录失败: {exc}")

    def _build_arbitration_id(self, function_id, motor_id):
        return (int(function_id) << 8) | int(motor_id)

    def _make_can_frame(self, function_id, motor_id, data=b""):
        payload = list(data or [])
        return can.Message(
            arbitration_id=self._build_arbitration_id(function_id, motor_id),
            is_extended_id=True,
            data=payload,
        )

    def _motor_label(self, motor_id):
        return "左电机" if motor_id == LEFT_MOTOR_ID else "右电机"

    def _get_sign(self, motor_id):
        return LEFT_MOTOR_SIGN if motor_id == LEFT_MOTOR_ID else RIGHT_MOTOR_SIGN

    def _timestamp_attr(self, motor_id):
        return "left_motor_timestamp" if motor_id == LEFT_MOTOR_ID else "right_motor_timestamp"

    def _decode_temperature(self, raw_value):
        return struct.unpack("b", bytes([raw_value & 0xFF]))[0]

    def _clamp(self, value, lower, upper):
        return max(lower, min(upper, float(value)))

    def _float_to_uint(self, value, lower, upper, bits):
        span = upper - lower
        value = self._clamp(value, lower, upper)
        max_int = (1 << bits) - 1
        return int((value - lower) * (max_int / span))

    def _position_counts_to_radians(self, position_counts):
        return math.radians(position_counts * POSITION_DEG_PER_COUNT)

    def _erpm_to_output_rads(self, erpm):
        mechanical_rpm = erpm / AK10_9_POLE_PAIRS / AK10_9_GEAR_RATIO
        return mechanical_rpm * 2.0 * math.pi / 60.0

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
                f"📡 {self._motor_label(motor_id)}状态帧 0x{self._build_arbitration_id(SERVO_STATUS_FUNCTION_ID, motor_id):08X} "
                f"角度={angle_rad:.4f} rad, 角速度={velocity_rad_s:.4f} rad/s, "
                f"Iq={current_amp:.2f} A, 温度={temperature_c}°C, 故障={error_code}"
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
        self.get_logger().info(f"📨 接收到兼容电机就绪命令: {command}")

        if command in ("enable", "enable_motors", "on", "true", "1", ""):
            self.enable_motor(LEFT_MOTOR_ID)
            self.enable_motor(RIGHT_MOTOR_ID)
            self.get_logger().info("✅ 双电机已保持MIT就绪状态")
        else:
            self.get_logger().warn(f"❌ 未知的兼容电机就绪命令: {command}")

    def execute_mechanical_zero(self):
        """兼容保留：现场已设好零点，不再向电机发送标零命令。"""
        self.mechanical_zeroed = True
        self.left_motor_zeroed = True
        self.right_motor_zeroed = True
        self.motion_confirmed_after_zero = True
        self.show_first_data_after_zero = True
        self.zero_completion_time = time.time()
        self.get_logger().info("✅ 已跳过机械标零：当前版本默认使用现场已设好的零点")
        return True

    def send_mechanical_zero_command(self, motor_id):
        self.get_logger().info(f"⏭️ 忽略 {self._motor_label(motor_id)} 标零发送：当前版本不执行标零")
        return True

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

    def send_mit_command(self, motor_id, position, velocity, kp, kd, torque):
        """按 AK V3.2.0 MIT 协议打包并发送 0x08 力控帧。"""
        position = self._clamp(position, MIT_P_MIN, MIT_P_MAX)
        velocity = self._clamp(velocity, MIT_V_MIN, MIT_V_MAX)
        kp = self._clamp(kp, MIT_KP_MIN, MIT_KP_MAX)
        kd = self._clamp(kd, MIT_KD_MIN, MIT_KD_MAX)
        torque = self._clamp(torque, MIT_T_MIN, MIT_T_MAX)

        p_int = self._float_to_uint(position, MIT_P_MIN, MIT_P_MAX, 16)
        v_int = self._float_to_uint(velocity, MIT_V_MIN, MIT_V_MAX, 12)
        kp_int = self._float_to_uint(kp, MIT_KP_MIN, MIT_KP_MAX, 12)
        kd_int = self._float_to_uint(kd, MIT_KD_MIN, MIT_KD_MAX, 12)
        t_int = self._float_to_uint(torque, MIT_T_MIN, MIT_T_MAX, 12)

        data = bytes([
            (kp_int >> 4) & 0xFF,
            ((kp_int & 0x0F) << 4) | ((kd_int >> 8) & 0x0F),
            kd_int & 0xFF,
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
            t_int & 0xFF,
        ])
        frame = self._make_can_frame(MIT_CONTROL_FUNCTION_ID, motor_id, data)
        if self.debug_mode:
            self.get_logger().debug(
                f"🧠 MIT指令: ID=0x{frame.arbitration_id:08X}, p={position:.3f}, v={velocity:.3f}, "
                f"kp={kp:.3f}, kd={kd:.3f}, t={torque:.3f}, data={data.hex()}"
            )
        return self.send_frame(frame)

    def send_mit_torque_command(self, motor_id, torque):
        """MIT 力矩环：Kp/Kd/位置/速度全置零，仅输出目标扭矩。"""
        return self.send_mit_command(
            motor_id,
            position=0.0,
            velocity=0.0,
            kp=0.0,
            kd=0.0,
            torque=torque,
        )

    def enable_motor(self, motor_id):
        """MIT 模式无需预切换，兼容接口改为发送零力矩帧。"""
        ok = self.send_mit_torque_command(motor_id, 0.0)
        if ok:
            self.get_logger().info(
                f"🔌 电机MIT准备完成: {self._motor_label(motor_id)} "
                f"(CAN ID: 0x{self._build_arbitration_id(MIT_CONTROL_FUNCTION_ID, motor_id):08X})"
            )
        return ok

    def disable_motor(self, motor_id):
        frame = self._make_can_frame(SERVO_DISABLE_FUNCTION_ID, motor_id, b"")
        ok = self.send_frame(frame)
        if ok:
            self.get_logger().warn(
                f"🔒 电机失能: {self._motor_label(motor_id)} (CAN ID: 0x{frame.arbitration_id:08X})"
            )
        return ok

    def set_run_mode(self, motor_id, mode):
        """兼容保留：AK V3.2.0 伺服 CAN 协议无需额外设置旧版 run mode。"""
        if self.debug_mode:
            self.get_logger().info(
                f"⏭️ 忽略 set_run_mode({self._motor_label(motor_id)}, mode={mode})，当前协议直接使用功能 ID 控制"
            )
        return True

    def _process_feedback_message(self, msg, motor_id=None):
        if motor_id is None:
            motor_id = msg.arbitration_id & 0xFF

        if len(msg.data) < 8:
            if self.debug_mode:
                self.get_logger().warn(
                    f"⚠️ {self._motor_label(motor_id)} 状态帧长度不足: len={len(msg.data)}"
                )
            return False

        position_counts = struct.unpack(">h", bytes(msg.data[0:2]))[0]
        velocity_counts = struct.unpack(">h", bytes(msg.data[2:4]))[0]
        current_counts = struct.unpack(">h", bytes(msg.data[4:6]))[0]
        temperature_c = self._decode_temperature(msg.data[6])
        error_code = int(msg.data[7])

        angle_rad = self._position_counts_to_radians(position_counts)
        erpm = velocity_counts * ERPM_PER_COUNT
        velocity_rad_s = self._erpm_to_output_rads(erpm)
        current_amp = current_counts * CURRENT_AMP_PER_COUNT
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
        return True

    def poll_feedback(self, timeout=0.0, max_messages=50):
        """被动读取电机周期上报的 0x29 状态帧。"""
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

            function_id = msg.arbitration_id >> 8
            motor_id = msg.arbitration_id & 0xFF

            if function_id != SERVO_STATUS_FUNCTION_ID:
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
        """兼容保留：当前电机使用 50Hz 固定周期回报，不再主动请求反馈。"""
        return True

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
        """等待目标电机出现有效周期状态帧。"""
        deadline = time.time() + max(timeout, 0.0)
        while time.time() < deadline:
            remaining = max(deadline - time.time(), 0.0)
            self.poll_feedback(timeout=min(remaining, 0.02), max_messages=50)
            if self.has_fresh_feedback(target_id, timeout=max(timeout, self.feedback_timeout_sec)):
                return True
        return self.has_fresh_feedback(target_id, timeout=max(timeout, self.feedback_timeout_sec))

    def destroy_node(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()
