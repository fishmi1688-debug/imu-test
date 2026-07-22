
"""Shared constants for gait control modules."""
import math
import os

# AK V3.2.0 伺服模式下，Drive ID 直接使用低 8 位。
LEFT_MOTOR_ID = 0x01
RIGHT_MOTOR_ID = 0x02
CAN_INTERFACE = os.environ.get("GAIT_CAN_INTERFACE", "can0")
WORKSPACE_ROOT = os.environ.get('GAIT_WORKSPACE', '/home/zhang/gait_control_ws')

# AK V3.2.0 CAN 功能 ID（手册 4.1 / 4.3.1）
SERVO_STATUS_FUNCTION_ID = 0x29
SERVO_START_FUNCTION_ID = 0x2C
SERVO_DISABLE_FUNCTION_ID = 0x0F
SERVO_FEEDBACK_CONFIG_FUNCTION_ID = 0x10
MIT_CONTROL_FUNCTION_ID = 0x08

# AK10-9 输出端状态量换算（手册 4.3.1 与 3.1.5.3）
AK10_9_POLE_PAIRS = 21.0
AK10_9_GEAR_RATIO = 9.0
POSITION_DEG_PER_COUNT = 0.1
ERPM_PER_COUNT = 10.0
CURRENT_AMP_PER_COUNT = 0.01
LEFT_MOTOR_SIGN = -1.0
RIGHT_MOTOR_SIGN = 1.0
MOTOR_FEEDBACK_TIMEOUT_SEC = 0.15

# AK10-9 MIT 力控模式参数范围（手册 4.2）
MIT_P_MIN = -12.56
MIT_P_MAX = 12.56
MIT_V_MIN = -28.0
MIT_V_MAX = 28.0
MIT_T_MIN = -54.0
MIT_T_MAX = 54.0
MIT_KP_MIN = 0.0
MIT_KP_MAX = 500.0
MIT_KD_MIN = 0.0
MIT_KD_MAX = 5.0

AO_CONFIG = {
    # RAO in Li et al. uses raw q = theta_r - theta_l. Keep filter optional.
    "FILTER_ENABLED": False,
    "FILTER_CUTOFF": 0.3,
    "OSC_HARMONICS": 1,
    "OSC_ALPHA0_INIT": 0.0,
    "OSC_ALPHA_INIT": 0.4,
    "OSC_DEN_EPS": 1e-6,
    "OSC_OMEGA_INIT": 2 * math.pi * 1.0,
    "OSC_OMEGA_MIN": 2.5,
    "OSC_OMEGA_MAX": 25.0,
    # RAO dynamic Hebbian gains (Eq.2, Eq.4).
    "RAO_V_PHI": 8.0,
    "RAO_V_W": 3.0,
    "RAO_ETA": 1.5,
    # Zero-crossing frequency correction kw (Eq.5).
    "RAO_XI_W": 12.0,
    "RAO_SIGMA_W": 0.12,
    "RAO_DW": 0.35,
    # Auxiliary phase correction kphi (Eq.8).
    "RAO_XI_PHI": 1.0,
    "RAO_SIGMA_PHI": 0.12,
    "RAO_DPHI": 0.35,
}
