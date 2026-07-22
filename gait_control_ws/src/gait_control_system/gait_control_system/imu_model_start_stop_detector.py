#!/usr/bin/env python3

"""IMU + 逻辑回归模型的实时步态启停检测器。"""

from __future__ import annotations

import asyncio
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

_JOBLIB_IMPORT_ERROR: Optional[Exception] = None
try:
    import joblib
except Exception as exc:  # pragma: no cover - 环境依赖保护
    joblib = None  # type: ignore[assignment]
    _JOBLIB_IMPORT_ERROR = exc

_MOVELLA_IMPORT_ERROR: Optional[Exception] = None
try:
    from bleak import BleakClient, BleakScanner
    from .Xsens_DOT_PC_Reader.movella_dot_py.core.parser import PayloadParser
    from .Xsens_DOT_PC_Reader.movella_dot_py.core.sensor import MovellaDOTSensor
    from .Xsens_DOT_PC_Reader.movella_dot_py.models.data_structures import SensorConfiguration
    from .Xsens_DOT_PC_Reader.movella_dot_py.models.enums import (
        FilterProfile,
        OutputRate,
        PayloadMode,
    )
except Exception as exc:  # pragma: no cover - 环境依赖保护
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]
    PayloadParser = None  # type: ignore[assignment]
    MovellaDOTSensor = None  # type: ignore[assignment]
    SensorConfiguration = None  # type: ignore[assignment]
    FilterProfile = None  # type: ignore[assignment]
    OutputRate = None  # type: ignore[assignment]
    PayloadMode = None  # type: ignore[assignment]
    _MOVELLA_IMPORT_ERROR = exc

DEFAULT_IMU_MAC = "D4:22:CD:00:8A:5A"
DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "model" / "model_walk.joblib"
_BLE_CONNECT_LOCK = threading.Lock()
# 步态识别平滑与阈值（可按需要调整）
# DEFAULT_SMOOTHING_CONSECUTIVE:
# - 调大: 启停判定更平滑、更抗抖动，但响应更慢（开始/停止都会延后）
# - 调小: 响应更快，但更容易受瞬时噪声影响而抖动
# - 建议范围: 1~5（30Hz下，2表示约2个判定窗口连续一致才切状态）
DEFAULT_SMOOTHING_CONSECUTIVE = 2
# DEFAULT_WALK_PROB_THRESHOLD:
# - 调大: 只有更高“行走概率”才判为行走，误触发更少，但可能漏检慢速/轻微动作
# - 调小: 更容易判为行走，启动更灵敏，但误触发概率会上升
# - 建议范围: 0.40~0.70（常用起点 0.50）
DEFAULT_WALK_PROB_THRESHOLD = 0.5


def _read_env_float(name: str, default: float, minimum: float) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return default
    return max(minimum, value)


def _env_enabled(name: str, default: bool = True) -> bool:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    return raw_value.strip().lower() not in ("0", "false", "no", "off")


def _normalize_mac(mac_address: str) -> str:
    return str(mac_address or "").strip().upper()


def _iter_scanner_devices(scan_result: Any) -> list[Any]:
    if scan_result is None:
        return []
    if isinstance(scan_result, dict):
        values = scan_result.values()
    else:
        values = scan_result
    devices: list[Any] = []
    for item in values:
        if isinstance(item, tuple) and item:
            item = item[0]
        devices.append(item)
    return devices


def _zero_crossing_rate(x: np.ndarray) -> float:
    if len(x) < 2:
        return 0.0
    signs = np.sign(x)
    signs[signs == 0] = 1
    return float(np.mean(signs[:-1] * signs[1:] < 0))


def _compute_features(window_8d: np.ndarray) -> list[float]:
    feats: list[float] = []
    for i in range(window_8d.shape[1]):
        x = window_8d[:, i]
        mean = float(np.mean(x))
        std = float(np.std(x))
        min_v = float(np.min(x))
        max_v = float(np.max(x))
        ptp = max_v - min_v
        rms = float(np.sqrt(np.mean(x**2)))
        energy = float(np.mean(x**2))
        zcr = _zero_crossing_rate(x)
        feats.extend([mean, std, min_v, max_v, ptp, rms, energy, zcr])
    return feats


def _add_extra_channels(data_6d: np.ndarray) -> np.ndarray:
    accel_mag = np.linalg.norm(data_6d[:, 0:3], axis=1)
    gyro_mag = np.linalg.norm(data_6d[:, 3:6], axis=1)
    return np.column_stack([data_6d, accel_mag, gyro_mag])


def _pick_motion_label_id(id_to_label: dict[int, str], classes: list[int]) -> int:
    motion_words = ("运动", "行走", "walking", "walk", "gait", "move")
    for label_id, label in id_to_label.items():
        text = str(label).lower()
        if any(word in text for word in motion_words):
            return int(label_id)
    if 1 in classes:
        return 1
    return int(max(classes)) if classes else 1


class IMUModelStartStopDetector:
    """通过左大腿 IMU 实时读取 6 轴数据并执行模型判停。"""

    def __init__(
        self,
        model_path: Optional[str] = None,
        mac_address: str = DEFAULT_IMU_MAC,
        sample_rate_hz: float = 30.0,
        start_votes_required: int = DEFAULT_SMOOTHING_CONSECUTIVE,
        stop_votes_required: int = DEFAULT_SMOOTHING_CONSECUTIVE,
        data_timeout_sec: float = 1.0,
        connect_timeout_sec: float = 10.0,
        reconnect_interval_sec: float = 2.0,
        walk_prob_threshold: float = DEFAULT_WALK_PROB_THRESHOLD,
    ):
        self.model_path = Path(model_path) if model_path else DEFAULT_MODEL_PATH
        self.mac_address = mac_address.strip() or DEFAULT_IMU_MAC
        self.sample_rate_hz = float(sample_rate_hz)
        self.start_votes_required = max(1, int(start_votes_required))
        self.stop_votes_required = max(1, int(stop_votes_required))
        self.walk_prob_threshold = float(np.clip(walk_prob_threshold, 0.0, 1.0))
        self.data_timeout_sec = float(data_timeout_sec)
        self.connect_timeout_sec = float(connect_timeout_sec)
        self.reconnect_interval_sec = float(reconnect_interval_sec)
        self.scan_before_connect = _env_enabled("GAIT_IMU_SCAN_BEFORE_CONNECT", True)
        self.scan_timeout_sec = _read_env_float(
            "GAIT_IMU_SCAN_TIMEOUT_SEC",
            max(12.0, self.connect_timeout_sec),
            1.0,
        )
        self.connect_lock_timeout_sec = _read_env_float(
            "GAIT_IMU_CONNECT_LOCK_TIMEOUT_SEC",
            self.scan_timeout_sec + self.connect_timeout_sec + 5.0,
            1.0,
        )

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._running = False
        self._connected = False
        self._measuring = False
        self._measurement_enabled = False
        self._last_error = ""

        self._model = None
        self._scaler = None
        self._window_size = 21
        self._step = 3
        self._id_to_label: dict[int, str] = {}
        self._motion_label_id = 1
        self._stop_label_id = 0
        self._motion_class_index: Optional[int] = None
        self._model_ready = self._load_model()

        self._sample_buffer: deque[np.ndarray] = deque(maxlen=self._window_size)
        self._sample_seq = 0
        self._last_infer_seq = 0
        self._last_sample_time = 0.0

        self._stable_state = 0
        self._motion_votes = 0
        self._stop_votes = 0
        self._last_output_state = 0
        self._last_prediction = {
            "label_id": self._stop_label_id,
            "predicted_label": self._id_to_label.get(self._stop_label_id, "停止"),
            "motion_probability": 0.0,
            "ready": False,
            "timestamp": 0.0,
        }

        self._parser = None
        if PayloadParser is not None and PayloadMode is not None:
            self._parser = PayloadParser(PayloadMode.RATE_QUANTITIES)

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def _load_model(self) -> bool:
        if _JOBLIB_IMPORT_ERROR is not None or joblib is None:
            self._set_error(f"joblib 不可用: {_JOBLIB_IMPORT_ERROR}")
            return False
        if not self.model_path.exists():
            self._set_error(f"模型文件不存在: {self.model_path}")
            return False
        try:
            artifact = joblib.load(self.model_path)
            self._model = artifact["model"]
            self._scaler = artifact["scaler"]
            self._window_size = int(artifact.get("window_size", self._window_size))
            self._step = int(artifact.get("step", self._step))
            self.sample_rate_hz = float(artifact.get("sample_rate_hz", self.sample_rate_hz))
            self._id_to_label = {
                int(k): str(v) for k, v in (artifact.get("id_to_label", {}) or {}).items()
            }
            classes = [int(c) for c in getattr(self._model, "classes_", [])]
            self._motion_label_id = _pick_motion_label_id(self._id_to_label, classes)
            self._stop_label_id = 0 if 0 in classes else (min(classes) if classes else 0)
            self._motion_class_index = None
            if classes and self._motion_label_id in classes:
                self._motion_class_index = classes.index(self._motion_label_id)
            return True
        except Exception as exc:
            self._set_error(f"模型加载失败: {exc}")
            return False

    def start(self) -> bool:
        if _MOVELLA_IMPORT_ERROR is not None:
            self._set_error(f"蓝牙依赖不可用: {_MOVELLA_IMPORT_ERROR}")
            return False
        if not self._model_ready:
            return False
        if self._running:
            return True

        self._stop_event.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._thread_main,
            daemon=True,
            name="imu-model-start-stop",
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=5.0)
        self._thread = None
        with self._lock:
            self._connected = False
            self._measuring = False
            self._measurement_enabled = False
            self._running = False

    def set_measurement_enabled(self, enabled: bool) -> None:
        """控制是否进入测量流；False 时保持已连接待机。"""
        with self._lock:
            self._measurement_enabled = bool(enabled)
            if not enabled:
                self._measuring = False
                self._stable_state = 0
                self._motion_votes = 0
                self._stop_votes = 0

    def detect(self) -> tuple[int, bool, dict[str, Any]]:
        now = time.time()
        with self._lock:
            stale = (now - self._last_sample_time) > self.data_timeout_sec
            if stale:
                self._stable_state = 0
                self._motion_votes = 0
                self._stop_votes = 0
            gait_state = int(self._stable_state)
            state_changed = gait_state != self._last_output_state
            self._last_output_state = gait_state
            info = dict(self._last_prediction)
            info["connected"] = bool(self._connected)
            info["measuring"] = bool(self._measuring)
            info["mac_address"] = self.mac_address
            info["last_error"] = self._last_error
            info["stale"] = stale
            info["sample_rate_hz"] = self.sample_rate_hz
            info["window_size"] = self._window_size
            info["step"] = self._step
            info["motion_votes"] = self._motion_votes
            info["stop_votes"] = self._stop_votes
            info["walk_prob_threshold"] = self.walk_prob_threshold
        return gait_state, state_changed, info

    def _thread_main(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._run_ble_worker())
        finally:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            if pending:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()
            self._loop = None
            with self._lock:
                self._connected = False
                self._running = False

    async def _run_ble_worker(self) -> None:
        while not self._stop_event.is_set():
            sensor = None
            measuring_now = False
            try:
                self._set_error(f"等待IMU蓝牙连接通道: {self.mac_address}")
                lock_acquired = await asyncio.to_thread(
                    _BLE_CONNECT_LOCK.acquire, True, self.connect_lock_timeout_sec
                )
                if not lock_acquired:
                    raise RuntimeError("等待IMU蓝牙连接通道超时")
                try:
                    connect_target = await self._resolve_connect_target()
                    sensor = self._build_sensor(connect_target)
                    if sensor is None:
                        return
                    self._set_error(f"正在连接IMU: {self.mac_address}")
                    await sensor.client.connect(timeout=self.connect_timeout_sec)  # type: ignore[arg-type]
                    sensor.is_connected = True
                    await sensor.configure_sensor()
                    sensor.notification_handler = self._notification_handler
                finally:
                    _BLE_CONNECT_LOCK.release()
                with self._lock:
                    self._connected = True
                    self._measuring = False
                    self._last_error = ""
                disconnect_checks = 0
                while not self._stop_event.is_set():
                    client = getattr(sensor, "client", None)
                    connected_now = bool(client is not None and getattr(client, "is_connected", False))
                    if connected_now:
                        disconnect_checks = 0
                    else:
                        disconnect_checks += 1
                        # 部分BlueZ场景会出现短暂 is_connected 抖动，连续确认后再判定断线。
                        if disconnect_checks >= 20:  # about 2s at 0.1s interval
                            raise RuntimeError("IMU蓝牙连接断开(确认)")

                    with self._lock:
                        should_measure = bool(self._measurement_enabled)
                    if should_measure and not measuring_now:
                        await sensor.start_measurement()
                        measuring_now = True
                        with self._lock:
                            self._measuring = True
                    elif not should_measure and measuring_now:
                        await sensor.stop_measurement()
                        measuring_now = False
                        with self._lock:
                            self._measuring = False
                    await asyncio.sleep(0.1)
            except Exception as exc:
                self._set_error(f"IMU连接失败: {exc}")
            finally:
                with self._lock:
                    self._connected = False
                    self._measuring = False
                if sensor is not None:
                    await self._safe_disconnect(sensor, measuring_now)

            if not self._stop_event.is_set():
                await asyncio.sleep(self.reconnect_interval_sec)

    async def _resolve_connect_target(self):
        target_mac = _normalize_mac(self.mac_address)
        if not self.scan_before_connect or BleakScanner is None:
            return target_mac

        self._set_error(f"正在扫描IMU: {target_mac}")
        find_by_address = getattr(BleakScanner, "find_device_by_address", None)
        if callable(find_by_address):
            device = await find_by_address(target_mac, timeout=self.scan_timeout_sec)
            if device is not None:
                return device

        devices = _iter_scanner_devices(await BleakScanner.discover(timeout=self.scan_timeout_sec))
        for device in devices:
            if _normalize_mac(getattr(device, "address", "")) == target_mac:
                return device

        nearby = []
        for device in devices:
            address = str(getattr(device, "address", "") or "")
            name = str(getattr(device, "name", "") or "")
            if name or address.startswith("D4:22:CD"):
                nearby.append(f"{address}/{name or '-'}")
        suffix = f"，附近={nearby[:5]}" if nearby else ""
        raise RuntimeError(f"扫描超时，未发现目标IMU {target_mac}{suffix}")

    def _build_sensor(self, connect_target: Any = None):
        if (
            MovellaDOTSensor is None
            or SensorConfiguration is None
            or OutputRate is None
            or FilterProfile is None
            or PayloadMode is None
            or BleakClient is None
        ):
            self._set_error("Movella DOT 依赖不可用")
            return None

        output_rate = OutputRate.RATE_30
        if int(round(self.sample_rate_hz)) == 60:
            output_rate = OutputRate.RATE_60

        sensor = MovellaDOTSensor(
            SensorConfiguration(
                output_rate=output_rate,
                filter_profile=FilterProfile.DYNAMIC,
                payload_mode=PayloadMode.RATE_QUANTITIES,
            )
        )
        target = connect_target if connect_target is not None else self.mac_address
        target_address = str(getattr(target, "address", self.mac_address) or self.mac_address)
        target_name = str(getattr(target, "name", "") or target_address)
        sensor.client = BleakClient(target, timeout=self.connect_timeout_sec)
        sensor._device_address = target_address
        sensor._device_name = target_name
        return sensor

    async def _safe_disconnect(self, sensor, was_measuring: bool = False) -> None:
        client = getattr(sensor, "client", None)
        connected = bool(client is not None and getattr(client, "is_connected", False))
        if was_measuring and connected:
            try:
                await sensor.stop_measurement()
            except Exception:
                pass
        try:
            await sensor.disconnect()
        except Exception:
            pass

    def _notification_handler(self, sender: int, data: bytearray) -> None:
        _ = sender
        if self._parser is None:
            return
        try:
            parsed = self._parser.parse(data)
        except Exception:
            return

        if parsed.acceleration is None or parsed.angular_velocity is None:
            return

        sample = np.array(
            [
                float(parsed.acceleration.x),
                float(parsed.acceleration.y),
                float(parsed.acceleration.z),
                float(parsed.angular_velocity.x),
                float(parsed.angular_velocity.y),
                float(parsed.angular_velocity.z),
            ],
            dtype=float,
        )
        self._handle_sample(sample)

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

        raw_pred_id, motion_prob = self._infer_window(window)
        is_walk = motion_prob >= self.walk_prob_threshold
        pred_id = self._motion_label_id if is_walk else self._stop_label_id
        now = time.time()
        with self._lock:
            if is_walk:
                self._motion_votes += 1
                # 【Bug修复】原来硬清零 _stop_votes=0，导致一旦出现单帧"行走"预测，
                # 整个停止票数被清空，需要重新积累 stop_votes_required 张票。
                # 在模型噪声较高时（平均30%帧偶发"行走"）永远无法积累足够停止票，
                # 表现为"一直显示运动"。
                # 改为软衰减：停止票减1（最低0），保留已积累的历史投票信息。
                self._stop_votes = max(0, self._stop_votes - 1)
                if self._stable_state == 0 and self._motion_votes >= self.start_votes_required:
                    self._stable_state = 1
            else:
                self._stop_votes += 1
                # 同理：行走票也改为软衰减，避免单帧停止预测使行走计数硬清零
                self._motion_votes = max(0, self._motion_votes - 1)
                if self._stable_state == 1 and self._stop_votes >= self.stop_votes_required:
                    self._stable_state = 0

            self._last_prediction = {
                "label_id": int(pred_id),
                "predicted_label": self._id_to_label.get(int(pred_id), str(int(pred_id))),
                "raw_label_id": int(raw_pred_id),
                "raw_predicted_label": self._id_to_label.get(int(raw_pred_id), str(int(raw_pred_id))),
                "motion_probability": float(motion_prob),
                "ready": True,
                "timestamp": now,
            }

    def _infer_window(self, window_6d: np.ndarray) -> tuple[int, float]:
        if self._model is None or self._scaler is None:
            return self._stop_label_id, 0.0

        try:
            window_8d = _add_extra_channels(window_6d)
            features = np.asarray(_compute_features(window_8d), dtype=float).reshape(1, -1)
            x_scaled = self._scaler.transform(features)
            pred_id = int(self._model.predict(x_scaled)[0])

            motion_prob = 0.0
            if hasattr(self._model, "predict_proba"):
                proba = self._model.predict_proba(x_scaled)[0]
                if self._motion_class_index is not None and self._motion_class_index < len(proba):
                    motion_prob = float(proba[self._motion_class_index])
                elif pred_id == self._motion_label_id:
                    motion_prob = float(np.max(proba))
            else:
                motion_prob = 1.0 if pred_id == self._motion_label_id else 0.0
            return pred_id, motion_prob
        except Exception as exc:
            self._set_error(f"模型推理失败: {exc}")
            return self._stop_label_id, 0.0
