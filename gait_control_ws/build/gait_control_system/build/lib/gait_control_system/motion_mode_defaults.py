"""
集中存放各运动模式的助力曲线初始参数。
在这里修改后，控制节点会同步使用最新默认值。
"""

from copy import deepcopy
from typing import Dict, Any

from .phase_bias_fitting import (
    CYCLING_LINEAR_BIAS_AT_0P6,
    CYCLING_LINEAR_SLOPE,
    WALKING_LINEAR_BIAS_AT_0P6,
    WALKING_LINEAR_SLOPE,
)

# 基础默认参数（不要在其他模块直接修改此字典）
_BASE_MOTION_MODES: Dict[str, Dict[str, Any]] = {
    "walking": {
        "name": "平地行走",
        "name_en": "Walking",
        "ext_t0": 0.0,
        "ext_tf": 0.5,
        "ext_p": 0.70,
        "ext_Tmax": 10.0,
        "flex_t0": 0.55,
        "flex_tf": 0.95,
        "flex_p": 0.80,
        "flex_Tmax": 10.0,
        "phase_bias": -0.20,  # -1 到 1，正值提前，负值迟后
        "phase_bias_at_0p6": WALKING_LINEAR_BIAS_AT_0P6,  # 0.6Hz对应的相位偏置
        "phase_bias_slope": WALKING_LINEAR_SLOPE,  # 线性插值斜率 (偏置/Hz)
        "description": "适用于平地正常行走",
    },
    "stairs_up": {
        "name": "上楼梯",
        "name_en": "Stairs Up",
        "ext_t0": 0.0,
        "ext_tf": 0.50,
        "ext_p": 0.40,
        "ext_Tmax": 10.0,
        "flex_t0": 0.55,
        "flex_tf": 0.85,
        "flex_p": 0.35,
        "flex_Tmax": 10.0,
        "phase_bias": 0.0,
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "description": "适用于上楼梯运动",
    },
    "stairs_down": {
        "name": "下楼梯",
        "name_en": "Stairs Down",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 10.0,
        "flex_t0": 0.4,
        "flex_tf": 0.9,
        "flex_p": 0.2,
        "flex_Tmax": 10.0,
        "phase_bias": 0.0,
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "event_prob_threshold": 0.8,  # stairs_down复用cycling启停模型阈值（0~1）
        "description": "适用于下楼梯运动",
    },
    "test": {
        "name": "骑车测试模式",
        "name_en": "Cycling Test",
        # test模式使用独立峰值定时；默认助力曲线保持原有骑车测试参数
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 10.0,
        "flex_t0": 0.0,
        "flex_tf": 0.5,
        "flex_p": 0.2,
        "flex_Tmax": 10.0,
        "phase_bias": -0.4,
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "event_prob_threshold": 0.8,
        "description": "骑车测试模式（髋角峰值定时，手动启停）",
    },
    "walking_test": {
        "name": "步行测试模式",
        "name_en": "Walking Test",
        # walking_test 的策略与 test 一致；默认助力曲线参数与downhill一致
        "ext_t0": 0.00,
        "ext_tf": 0.30,
        "ext_p": 0.70,
        "ext_Tmax": 10.0,
        "flex_t0": 0.50,
        "flex_tf": 0.80,
        "flex_p": 0.80,
        "flex_Tmax": 10.0,
        "phase_bias": 0.12,
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "event_prob_threshold": 0.8,
        "description": "步行测试模式（髋角峰值定时，手动启停）",
    },
    "cycling": {
        "name": "骑自行车",
        "name_en": "Cycling",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 10.0,
        "flex_t0": 0.40,
        "flex_tf": 0.90,
        "flex_p": 0.20,
        "flex_Tmax": 10.0,
        "phase_bias": 0.06,  # 骑行时可能需要正的相位偏置以适应不同的动力学特性
        "phase_bias_at_0p6": CYCLING_LINEAR_BIAS_AT_0P6,
        "phase_bias_slope": CYCLING_LINEAR_SLOPE,
        "event_prob_threshold": 0.8,  # cycling启停事件概率阈值（0~1）
        "description": "适用于骑自行车运动",
    },
    "uphill": {
        "name": "上坡行走",
        "name_en": "Uphill",
        "ext_t0": 0.0,
        "ext_tf": 0.0,
        "ext_p": 0.0,
        "ext_Tmax": 10.0,
        "flex_t0": 0.4,
        "flex_tf": 0.9,
        "flex_p": 0.2,
        "flex_Tmax": 10.0,
        "phase_bias": 0.0,
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "event_prob_threshold": 0.8,  # uphill复用cycling启停模型阈值（0~1）
        "description": "适用于上坡行走",
    },
    "downhill": {
        "name": "下坡行走",
        "name_en": "Downhill",
        "ext_t0": 0.00,
        "ext_tf": 0.30,
        "ext_p": 0.70,
        "ext_Tmax": 10.0,
        "flex_t0": 0.50,
        "flex_tf": 0.80,
        "flex_p": 0.80,
        "flex_Tmax": 10.0,
        "phase_bias": -0.20, 
        "phase_bias_at_0p6": 0.0,
        "phase_bias_slope": 0.0,
        "description": "适用于下坡行走",
    },
}


def get_default_motion_modes() -> Dict[str, Dict[str, Any]]:
    """返回一份全新的默认参数拷贝，避免跨模块共享同一实例。"""
    return deepcopy(_BASE_MOTION_MODES)
