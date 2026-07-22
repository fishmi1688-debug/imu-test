import math
from typing import Dict, Tuple

TEST_MODE_PHASE_MIN_CYCLE_SEC = 0.3  # test模式单腿峰值周期下限（秒）
TEST_MODE_PHASE_MAX_CYCLE_SEC = 3.0  # test模式单腿峰值周期上限（秒）
TEST_MODE_PEAK_SLOPE_EPS = 0.02      # test模式峰值判定的最小斜率变化阈值（rad/采样）


class TestModePhaseTracker:
    """test模式峰值相位跟踪器：左右腿独立峰值触发，峰值对应相位0点。"""

    def __init__(
        self,
        min_cycle_sec: float = TEST_MODE_PHASE_MIN_CYCLE_SEC,
        max_cycle_sec: float = TEST_MODE_PHASE_MAX_CYCLE_SEC,
        peak_slope_eps: float = TEST_MODE_PEAK_SLOPE_EPS,
        peak_threshold: float = 0.5,
        estimated_human_frequency: float = 1.0,
    ):
        self.min_cycle_sec = float(min_cycle_sec)
        self.max_cycle_sec = float(max_cycle_sec)
        self.peak_slope_eps = float(peak_slope_eps)
        self.peak_threshold = float(peak_threshold)

        self.phase_state: Dict[str, Dict[str, float | bool | None]] = {}
        self.left_phase_valid = False
        self.right_phase_valid = False
        self.left_assist_ready = False
        self.right_assist_ready = False
        self.reset(estimated_human_frequency=estimated_human_frequency, keep_period=False)

    def _clamp_cycle_period(self, period_sec: float) -> float:
        return max(self.min_cycle_sec, min(self.max_cycle_sec, float(period_sec)))

    def _default_cycle_period(self, estimated_human_frequency: float) -> float:
        freq = max(float(estimated_human_frequency), 1e-3)
        return self._clamp_cycle_period(1.0 / freq)

    def reset(self, estimated_human_frequency: float = 1.0, keep_period: bool = False) -> None:
        """重置左右腿峰值相位跟踪状态。"""
        default_cycle_period = self._default_cycle_period(estimated_human_frequency)
        new_state: Dict[str, Dict[str, float | bool | None]] = {}
        for leg_key in ("left", "right"):
            prev_state = self.phase_state.get(leg_key, {})
            cycle_period = default_cycle_period
            if keep_period:
                try:
                    cycle_period = self._clamp_cycle_period(
                        float(prev_state.get("cycle_period", default_cycle_period))
                    )
                except Exception:
                    cycle_period = default_cycle_period
            new_state[leg_key] = {
                "last_angle": None,
                "last_slope": 0.0,
                "last_peak_time": None,
                "cycle_start_time": None,
                "cycle_period": cycle_period,
                "assist_ready": False,
                "trend": "unknown",
            }
        self.phase_state = new_state
        self.left_phase_valid = False
        self.right_phase_valid = False
        self.left_assist_ready = False
        self.right_assist_ready = False

    def _update_leg_phase(
        self, leg_key: str, angle: float, current_time: float
    ) -> Tuple[float, bool, bool, bool]:
        state = self.phase_state[leg_key]
        angle_now = float(angle)
        last_angle = state.get("last_angle")

        if last_angle is None:
            state["last_angle"] = angle_now
            state["last_slope"] = 0.0
            return 0.0, False, False, False

        slope = angle_now - float(last_angle)
        trend = str(state.get("trend", "unknown"))
        peak_detected = False

        # 使用带滞回的rising/falling状态机，而不是要求两帧直接跨过正负阈值。
        # 这样对峰顶较平、采样较稀或实机信号有轻微平台段时更稳健。
        if slope >= self.peak_slope_eps:
            trend = "rising"
        elif slope <= -self.peak_slope_eps:
            if trend == "rising" and float(last_angle) >= self.peak_threshold:
                peak_detected = True
            trend = "falling"

        if peak_detected:
            peak_time = float(current_time)
            last_peak_time = state.get("last_peak_time")
            if last_peak_time is not None:
                cycle_period = peak_time - float(last_peak_time)
                if self.min_cycle_sec <= cycle_period <= self.max_cycle_sec:
                    state["cycle_period"] = float(cycle_period)
                    state["assist_ready"] = True
            state["last_peak_time"] = peak_time
            state["cycle_start_time"] = peak_time

        state["last_angle"] = angle_now
        state["last_slope"] = slope
        state["trend"] = trend

        cycle_start_time = state.get("cycle_start_time")
        if cycle_start_time is None:
            return 0.0, False, peak_detected, bool(state.get("assist_ready", False))

        cycle_period = self._clamp_cycle_period(float(state.get("cycle_period", 1.0)))
        elapsed = float(current_time) - float(cycle_start_time)
        if elapsed < 0.0:
            elapsed = 0.0
        phase_normalized = (elapsed / cycle_period) % 1.0
        return (
            float(phase_normalized),
            True,
            peak_detected,
            bool(state.get("assist_ready", False)),
        )

    def update(self, left_angle: float, right_angle: float, current_time: float, gait_state: int) -> dict:
        """更新test模式左右相位，返回相位/有效位/峰值检测信息。"""
        left_phase_norm, left_valid, left_peak, left_assist_ready = self._update_leg_phase(
            "left", left_angle, current_time
        )
        right_phase_norm, right_valid, right_peak, right_assist_ready = self._update_leg_phase(
            "right", right_angle, current_time
        )
        self.left_phase_valid = bool(left_valid)
        self.right_phase_valid = bool(right_valid)
        self.left_assist_ready = bool(left_assist_ready)
        self.right_assist_ready = bool(right_assist_ready)
        phase_active = bool(gait_state == 1 and (left_valid or right_valid))
        return {
            "left_phase_norm": float(left_phase_norm),
            "right_phase_norm": float(right_phase_norm),
            "left_phase_rad": float(left_phase_norm) * (2.0 * math.pi),
            "right_phase_rad": float(right_phase_norm) * (2.0 * math.pi),
            "left_valid": self.left_phase_valid,
            "right_valid": self.right_phase_valid,
            "left_assist_ready": self.left_assist_ready,
            "right_assist_ready": self.right_assist_ready,
            "phase_active": phase_active,
            "left_peak": bool(left_peak),
            "right_peak": bool(right_peak),
            "left_period_sec": float(self.phase_state["left"].get("cycle_period", 0.0) or 0.0),
            "right_period_sec": float(self.phase_state["right"].get("cycle_period", 0.0) or 0.0),
        }
