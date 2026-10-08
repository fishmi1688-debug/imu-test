"""
集中存放各运动模式的助力曲线初始参数。
在这里修改后，控制节点会同步使用最新默认值。
"""

from copy import deepcopy
from typing import Dict, Any

DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG = 25.0

# 基础默认参数（不要在其他模块直接修改此字典）
_BASE_MOTION_MODES: Dict[str, Dict[str, Any]] = {
    "imu_phase": {
        "name": "平地行走",
        "name_en": "walking",
        # 助力曲线沿用保留的下坡/IMU默认参数，相位来源为左右 MI1 CAN 大腿IMU。
        "ext_t0": 0.00,
        "ext_tf": 0.30,
        "ext_p": 0.70,
        "ext_Tmax": 5.0,
        "flex_t0": 0.50,
        "flex_tf": 0.80,
        "flex_p": 0.80,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "swing_threshold": DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG,
        "description": "使用左右MI1有线CAN大腿IMU生成相位，助力曲线沿用平地行走默认参数，手动启停",
    },
    "downhill": {
        "name": "下坡行走",
        "name_en": "Downhill",
        "ext_t0": 0.00,
        "ext_tf": 0.30,
        "ext_p": 0.70,
        "ext_Tmax": 5.0,
        "flex_t0": 0.50,
        "flex_tf": 0.80,
        "flex_p": 0.80,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "description": "适用于下坡行走",
    },
    "uphill": {
        "name": "上坡行走",
        "name_en": "Uphill",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 5.0,
        "flex_t0": 0.4,
        "flex_tf": 0.9,
        "flex_p": 0.2,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "description": "适用于上坡行走",
    },
    "cycling": {
        "name": "骑自行车",
        "name_en": "Cycling",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 5.0,
        "flex_t0": 0.40,
        "flex_tf": 0.90,
        "flex_p": 0.20,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "description": "适用于骑自行车运动",
    },
    "stairs_up": {
        "name": "上楼梯",
        "name_en": "Stairs Up",
        "ext_t0": 0.0,
        "ext_tf": 0.50,
        "ext_p": 0.40,
        "ext_Tmax": 5.0,
        "flex_t0": 0.55,
        "flex_tf": 0.85,
        "flex_p": 0.35,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "description": "适用于上楼梯运动",
    },
    "stairs_down": {
        "name": "下楼梯",
        "name_en": "Stairs Down",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 5.0,
        "flex_t0": 0.4,
        "flex_tf": 0.9,
        "flex_p": 0.2,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "description": "适用于下楼梯运动",
    },
    "imu_left_phase": {
        "name": "test",
        "name_en": "test",
        # 助力曲线沿用保留的下坡/IMU默认参数，相位来源为左 MI1 CAN 大腿IMU。
        "ext_t0": 0.00,
        "ext_tf": 0.30,
        "ext_p": 0.70,
        "ext_Tmax": 5.0,
        "flex_t0": 0.50,
        "flex_tf": 0.80,
        "flex_p": 0.80,
        "flex_Tmax": 5.0,
        "phase_bias": 0.0,
        "swing_threshold": DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG,
        "description": "使用左MI1有线CAN大腿IMU生成左腿相位，右腿相位=左腿相位+pi，手动启停",
    },
}


def get_default_motion_modes() -> Dict[str, Dict[str, Any]]:
    """返回一份全新的默认参数拷贝，避免跨模块共享同一实例。"""
    modes = deepcopy(_BASE_MOTION_MODES)
    for params in modes.values():
        params.setdefault("swing_threshold", DEFAULT_IMU_PHASE_SWING_THRESHOLD_DEG)
    return modes
