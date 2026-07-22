#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Frequency-to-phase-bias fitting for motion modes.

Each mode uses its own parameter set to avoid cross-mode coupling.
Phase bias is computed using a linear mapping defined by a 0.6Hz anchor and
a slope, then clamped to a safe range to avoid over-aggressive shifts.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

# ---------------- 可调参数（便于快速调节） ----------------
# 每个模式采用独立参数，避免交叉影响
LINEAR_BIAS_BASE_FREQ = 0.6

# 平地行走模式 (walking) 线性映射参数

WALKING_LINEAR_BIAS_AT_0P6 = -0.20# 0.6Hz 对应的相位偏置基准值；更改后会整体平移线性曲线
WALKING_LINEAR_SLOPE = 1.0        # 线性斜率(偏置/Hz)；正值使步频升高时偏置增加，负值相反
WALKING_SMOOTH_TAU = 0.0          # 平滑时间常数 (秒)，越大越平滑/响应慢
WALKING_FIXED_ALPHA = 0.5        # tau<=0 时使用的固定更新比例
WALKING_SMOOTH_MAX_DELTA = 1.0    # 每次更新的最大变化量，防止突跳
WALKING_SMALL_FREQ_DELTA = 0.10   # 步频变化阈值 (Hz)，小于此值认为变化轻微
WALKING_BIAS_HOLD_EPS = 0.05      # 偏执接近目标时的保持阈值，避免小抖动
WALKING_FREQ_BIN_BASE = 0.8       # 频率基准 (Hz)，作为第一个区间的中心
WALKING_FREQ_BIN_SIZE = 0.0       # 频率区间宽度 (Hz)
WALKING_FREQ_BIN_MODE = "center"  # 取区间代表频率：center / lower / upper
WALKING_BIAS_LIMITS = (-1.0, 1.0) # 相位偏置限幅，防止外推过大

# 骑行模式 (cycling) 线性映射参数
CYCLING_LINEAR_BIAS_AT_0P6 = 0.06
CYCLING_LINEAR_SLOPE = -1.0
CYCLING_SMOOTH_TAU = 0.0
CYCLING_FIXED_ALPHA = 0.5
CYCLING_SMOOTH_MAX_DELTA = 1.0
CYCLING_SMALL_FREQ_DELTA = 0.10
CYCLING_BIAS_HOLD_EPS = 0.05
CYCLING_FREQ_BIN_BASE = 0.8
CYCLING_FREQ_BIN_SIZE = 0.1
CYCLING_FREQ_BIN_MODE = "center"
CYCLING_BIAS_LIMITS = (-1.0, 1.0)

# ----------------------------------------------------------

MODE_PARAMS: Dict[str, Dict[str, object]] = {
    "walking": {
        "linear_bias_at_0p6": WALKING_LINEAR_BIAS_AT_0P6,
        "linear_slope": WALKING_LINEAR_SLOPE,
        "smooth_tau": WALKING_SMOOTH_TAU,
        "fixed_alpha": WALKING_FIXED_ALPHA,
        "smooth_max_delta": WALKING_SMOOTH_MAX_DELTA,
        "small_freq_delta": WALKING_SMALL_FREQ_DELTA,
        "bias_hold_eps": WALKING_BIAS_HOLD_EPS,
        "freq_bin_base": WALKING_FREQ_BIN_BASE,
        "freq_bin_size": WALKING_FREQ_BIN_SIZE,
        "freq_bin_mode": WALKING_FREQ_BIN_MODE,
        "bias_limits": WALKING_BIAS_LIMITS,
    },
    "cycling": {
        "linear_bias_at_0p6": CYCLING_LINEAR_BIAS_AT_0P6,
        "linear_slope": CYCLING_LINEAR_SLOPE,
        "smooth_tau": CYCLING_SMOOTH_TAU,
        "fixed_alpha": CYCLING_FIXED_ALPHA,
        "smooth_max_delta": CYCLING_SMOOTH_MAX_DELTA,
        "small_freq_delta": CYCLING_SMALL_FREQ_DELTA,
        "bias_hold_eps": CYCLING_BIAS_HOLD_EPS,
        "freq_bin_base": CYCLING_FREQ_BIN_BASE,
        "freq_bin_size": CYCLING_FREQ_BIN_SIZE,
        "freq_bin_mode": CYCLING_FREQ_BIN_MODE,
        "bias_limits": CYCLING_BIAS_LIMITS,
    },
}


def _clamp(value: float, limits: Tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))

def _linear_bias(freq_hz: float, bias_at_0p6: float, slope: float) -> float:
    return bias_at_0p6 + slope * (freq_hz - LINEAR_BIAS_BASE_FREQ)


def _quantize_frequency(
    freq_hz: float,
    bin_base: float,
    bin_size: float,
    bin_mode: str,
) -> float:
    """将频率量化到固定区间，用于每 0.1Hz 对应一个偏置值。"""
    if freq_hz is None or freq_hz <= 0:
        return freq_hz
    if bin_size is None or bin_size <= 0:
        return freq_hz
    if bin_base is None:
        bin_base = 0.0
    bin_start_ref = bin_base - 0.5 * bin_size
    bin_index = math.floor((freq_hz - bin_start_ref) / bin_size)
    bin_start = bin_start_ref + bin_index * bin_size
    mode = (bin_mode or "center").lower()
    if mode == "lower":
        return bin_start
    if mode == "upper":
        return bin_start + bin_size
    return bin_start + 0.5 * bin_size


def phase_bias_from_frequency(
    freq_hz: float,
    mode_key: str = "walking",
    linear_bias_at_0p6: Optional[float] = None,
    linear_slope: Optional[float] = None,
) -> float:
    """
    Get the recommended phase_bias for a given human step frequency.

    Args:
        freq_hz: Step frequency in Hz.
        mode_key: Motion mode key; uses mode-specific linear parameters.
        linear_bias_at_0p6: Optional bias value at 0.6Hz for linear mapping.
        linear_slope: Optional slope for linear mapping (bias per Hz).

    Returns:
        Phase bias value within mode-specific limits. For invalid inputs,
        returns 0.0 (no change).
    """
    params = MODE_PARAMS.get(mode_key)
    if params is None or freq_hz is None or freq_hz <= 0:
        return 0.0

    freq_q = _quantize_frequency(
        freq_hz,
        params["freq_bin_base"],
        params["freq_bin_size"],
        params["freq_bin_mode"],
    )
    bias_at_0p6 = (
        params["linear_bias_at_0p6"]
        if linear_bias_at_0p6 is None
        else linear_bias_at_0p6
    )
    slope = params["linear_slope"] if linear_slope is None else linear_slope
    bias = _linear_bias(freq_q, bias_at_0p6, slope)
    bias = _clamp(bias, params["bias_limits"])
    return float(bias)

__all__ = [
    "phase_bias_from_frequency",
    "phase_bias_with_smoothing",
    "LINEAR_BIAS_BASE_FREQ",
    "WALKING_LINEAR_BIAS_AT_0P6",
    "WALKING_LINEAR_SLOPE",
    "WALKING_SMOOTH_TAU",
    "WALKING_FIXED_ALPHA",
    "WALKING_SMOOTH_MAX_DELTA",
    "WALKING_SMALL_FREQ_DELTA",
    "WALKING_BIAS_HOLD_EPS",
    "WALKING_FREQ_BIN_BASE",
    "WALKING_FREQ_BIN_SIZE",
    "WALKING_FREQ_BIN_MODE",
    "WALKING_BIAS_LIMITS",
    "CYCLING_LINEAR_BIAS_AT_0P6",
    "CYCLING_LINEAR_SLOPE",
    "CYCLING_SMOOTH_TAU",
    "CYCLING_FIXED_ALPHA",
    "CYCLING_SMOOTH_MAX_DELTA",
    "CYCLING_SMALL_FREQ_DELTA",
    "CYCLING_BIAS_HOLD_EPS",
    "CYCLING_FREQ_BIN_BASE",
    "CYCLING_FREQ_BIN_SIZE",
    "CYCLING_FREQ_BIN_MODE",
    "CYCLING_BIAS_LIMITS",
    "MODE_PARAMS",
]


def phase_bias_with_smoothing(
    freq_hz: float,
    prev_bias: float,
    dt: float,
    mode_key: str = "walking",
    linear_bias_at_0p6: Optional[float] = None,
    linear_slope: Optional[float] = None,
    tau: float = None,
    max_delta_per_step: float = None,
    last_freq_hz: float = None,
    freq_delta_thresh: float = None,
    bias_hold_eps: float = None,
) -> float:
    """
    Smoothly transition phase_bias when frequency changes.

    Args:
        freq_hz: Current step frequency (Hz).
        prev_bias: Previous applied phase_bias.
        dt: Time step (s).
        mode_key: Motion mode key; uses mode-specific parameters.
        linear_bias_at_0p6: Optional bias value at 0.6Hz for linear mapping.
        linear_slope: Optional slope for linear mapping (bias per Hz).
        tau: Time constant for exponential smoothing (s).
        max_delta_per_step: Hard limit per update to avoid abrupt jumps.

    Returns:
        Smoothed phase_bias within mode-specific limits.
    """
    params = MODE_PARAMS.get(mode_key)
    if params is None:
        return prev_bias if prev_bias is not None else 0.0

    if prev_bias is None:
        prev_bias = 0.0

    smooth_tau = params["smooth_tau"] if tau is None else tau
    smooth_max_delta = (
        params["smooth_max_delta"] if max_delta_per_step is None else max_delta_per_step
    )
    small_freq_delta = (
        params["small_freq_delta"] if freq_delta_thresh is None else freq_delta_thresh
    )
    hold_eps = params["bias_hold_eps"] if bias_hold_eps is None else bias_hold_eps
    bias_limits = params["bias_limits"]
    fixed_alpha = params.get("fixed_alpha", 0.2)

    freq_q = _quantize_frequency(
        freq_hz,
        params["freq_bin_base"],
        params["freq_bin_size"],
        params["freq_bin_mode"],
    )
    if freq_q is None or freq_q <= 0:
        return _clamp(prev_bias, bias_limits)

    target = phase_bias_from_frequency(
        freq_q,
        mode_key,
        linear_bias_at_0p6=linear_bias_at_0p6,
        linear_slope=linear_slope,
    )

    # 频率变化很小且偏执已接近目标时保持不变，避免抖动
    if small_freq_delta is not None and last_freq_hz is not None:
        last_freq_q = _quantize_frequency(
            last_freq_hz,
            params["freq_bin_base"],
            params["freq_bin_size"],
            params["freq_bin_mode"],
        )
        if (
            last_freq_q is not None
            and abs(freq_q - last_freq_q) < small_freq_delta
            and abs(target - prev_bias) < hold_eps
        ):
            return _clamp(prev_bias, bias_limits)
    if dt is None or dt <= 0 or smooth_tau <= 0:
        alpha = max(0.0, min(1.0, float(fixed_alpha)))
    else:
        alpha = 1.0 - math.exp(-dt / smooth_tau)

    blended = prev_bias + alpha * (target - prev_bias)
    delta = blended - prev_bias
    if smooth_max_delta is not None:
        delta = max(-smooth_max_delta, min(smooth_max_delta, delta))
    result = prev_bias + delta
    return _clamp(result, bias_limits)
