#!/usr/bin/env python3

"""Cycling 模式专用：基于脚部 IMU 的“启停事件翻转助力”检测器。"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from .imu_model_start_stop_detector import IMUModelStartStopDetector

DEFAULT_CYCLING_IMU_MAC = "D4:22:CD:00:8A:5B"
DEFAULT_CYCLING_MODEL_PATH = Path(__file__).resolve().parent / "model" / "model_cycling.joblib"
# 事件概率阈值（模型输出“启停”概率 >= 该值时视为一次启停事件）
# 调大: 更保守，误触发更少，但可能漏检真实启停
# 调小: 更灵敏，更容易触发翻转，也更容易误触发
DEFAULT_EVENT_PROB_THRESHOLD = 0.8
# 触发翻转所需连续“启停”事件窗口数
# 调大: 抗噪更强，但翻转响应更慢
# 调小: 翻转更快，但抖动风险更高
DEFAULT_EVENT_CONSECUTIVE = 2
# 事件锁存释放所需连续“非启停”窗口数（避免一次事件被重复翻转）
# 调大: 更不容易重复触发同一事件，但下一次新事件响应会稍慢
# 调小: 释放更快，可能在边界噪声下重复翻转
DEFAULT_RELEASE_CONSECUTIVE = 2
# 释放滞回：解锁阈值 = event_prob_threshold - release_prob_hysteresis
# 调大: 更抗抖动，阈值附近不易反复翻转；但“下一次新事件”响应略慢
# 调小: 更灵敏，但更容易在边界概率附近抖动
DEFAULT_RELEASE_PROB_HYSTERESIS = 0.10


class CyclingToggleIMUDetector(IMUModelStartStopDetector):
    """检测到一次“启停”事件就翻转一次助力开关（仅用于 cycling）。"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        mac_address: str = DEFAULT_CYCLING_IMU_MAC,
        sample_rate_hz: float = 30.0,
        event_prob_threshold: float = DEFAULT_EVENT_PROB_THRESHOLD,
        event_consecutive: int = DEFAULT_EVENT_CONSECUTIVE,
        release_consecutive: int = DEFAULT_RELEASE_CONSECUTIVE,
        release_prob_hysteresis: float = DEFAULT_RELEASE_PROB_HYSTERESIS,
        data_timeout_sec: float = 1.0,
        connect_timeout_sec: float = 10.0,
        reconnect_interval_sec: float = 2.0,
    ):
        super().__init__(
            model_path=model_path or str(DEFAULT_CYCLING_MODEL_PATH),
            mac_address=mac_address,
            sample_rate_hz=sample_rate_hz,
            start_votes_required=1,
            stop_votes_required=1,
            data_timeout_sec=data_timeout_sec,
            connect_timeout_sec=connect_timeout_sec,
            reconnect_interval_sec=reconnect_interval_sec,
            walk_prob_threshold=event_prob_threshold,
        )
        self.event_prob_threshold = float(np.clip(event_prob_threshold, 0.0, 1.0))
        self.event_consecutive = max(1, int(event_consecutive))
        self.release_consecutive = max(1, int(release_consecutive))
        self.release_prob_hysteresis = float(np.clip(release_prob_hysteresis, 0.0, 1.0))

        self._assist_toggled_state = False
        self._event_streak = 0
        self._normal_streak = 0
        self._event_latched = False
        self._toggle_count = 0
        self._last_prediction.update(
            {
                "event_probability": 0.0,
                "toggle_triggered": False,
                "assist_enabled": False,
            }
        )

    def reset_toggle_state(self, assist_enabled: bool = False) -> None:
        """复位开关状态；默认静默（False）。"""
        state = bool(assist_enabled)
        with self._lock:
            self._assist_toggled_state = state
            self._event_streak = 0
            self._normal_streak = 0
            self._event_latched = False
            self._last_output_state = 1 if state else 0
            self._last_prediction.update(
                {
                    "toggle_triggered": False,
                    "assist_enabled": state,
                }
            )

    def detect(self) -> tuple[int, bool, dict[str, Any]]:
        now = time.time()
        with self._lock:
            stale = (now - self._last_sample_time) > self.data_timeout_sec
            if stale:
                # 失联/超时时故障安全：关闭助力
                self._assist_toggled_state = False
                self._event_streak = 0
                self._normal_streak = 0
                self._event_latched = False

            gait_state = 1 if self._assist_toggled_state else 0
            state_changed = gait_state != self._last_output_state
            self._last_output_state = gait_state

            info = dict(self._last_prediction)
            info["connected"] = bool(self._connected)
            info["mac_address"] = self.mac_address
            info["last_error"] = self._last_error
            info["stale"] = stale
            info["sample_rate_hz"] = self.sample_rate_hz
            info["window_size"] = self._window_size
            info["step"] = self._step
            info["event_prob_threshold"] = self.event_prob_threshold
            info["release_prob_hysteresis"] = self.release_prob_hysteresis
            info["release_threshold"] = max(
                0.0, float(self.event_prob_threshold - self.release_prob_hysteresis)
            )
            info["event_consecutive"] = self.event_consecutive
            info["release_consecutive"] = self.release_consecutive
            info["event_latched"] = self._event_latched
            info["toggle_count"] = self._toggle_count
            info["assist_enabled"] = bool(self._assist_toggled_state)
        return gait_state, state_changed, info

    def _handle_sample(self, sample_6d: np.ndarray) -> None:
        with self._lock:
            self._sample_buffer.append(sample_6d)
            self._sample_seq += 1
            self._last_sample_time = time.time()

            if len(self._sample_buffer) < self._window_size:
                self._last_prediction["ready"] = False
                return
            if (self._sample_seq - self._last_infer_seq) < self._step:
                return

            window = np.asarray(self._sample_buffer, dtype=float)
            self._last_infer_seq = self._sample_seq

        raw_pred_id, event_prob = self._infer_window(window)
        release_threshold = max(
            0.0, float(self.event_prob_threshold - self.release_prob_hysteresis)
        )
        is_event = event_prob >= self.event_prob_threshold
        is_release_candidate = event_prob <= release_threshold
        toggle_triggered = False

        with self._lock:
            if is_event:
                self._event_streak += 1
                self._normal_streak = 0
                if not self._event_latched and self._event_streak >= self.event_consecutive:
                    self._assist_toggled_state = not self._assist_toggled_state
                    self._toggle_count += 1
                    self._event_latched = True
                    toggle_triggered = True
            else:
                self._event_streak = 0
                if self._event_latched:
                    if is_release_candidate:
                        self._normal_streak += 1
                        if self._normal_streak >= self.release_consecutive:
                            self._event_latched = False
                            self._normal_streak = 0
                    else:
                        # 阈值附近滞回区间：保持锁存，避免边界抖动导致重复翻转。
                        self._normal_streak = 0
                else:
                    self._normal_streak = 0

            self._last_prediction = {
                "label_id": int(raw_pred_id),
                "predicted_label": self._id_to_label.get(int(raw_pred_id), str(int(raw_pred_id))),
                "event_probability": float(event_prob),
                "release_threshold": float(release_threshold),
                "motion_probability": float(event_prob),  # 兼容现有日志字段
                "toggle_triggered": bool(toggle_triggered),
                "assist_enabled": bool(self._assist_toggled_state),
                "ready": True,
                "timestamp": time.time(),
            }
