#!/usr/bin/env python3
"""双 IMU 实时步态相位推理脚本。

特性:
- 同时连接左脚、右脚 2 个 Xsens DOT IMU。
- 可选同时连接左、右 2 个足底压力鞋垫。
- 输入通道顺序严格按 `IMU_CHANNELS` 配置组帧。
- 默认使用 `gait_phase_model.keras` 与 `feature_scaler.pkl` 做实时推理。
- 不依赖 TensorFlow / onnxruntime，直接解析 Keras/ONNX 里的全连接权重并用 numpy 做前向推理。
- 实时保存 IMU、鞋垫和相位 CSV 到 `步态模型/data/<时间>/`。
- 提供 CSV 回放模式，便于在没有蓝牙时验证数据流。
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from typing import Optional

import numpy as np
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from model_io import load_feature_scaler, load_float_dense_weights


BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
MOVELLA_READER_ROOT = PROJECT_ROOT / "Xsens_DOT_PC_Reader"

DEFAULT_MODEL_PATH = BASE_DIR / "gait_phase_model.keras"
DEFAULT_SCALER_PATH = BASE_DIR / "feature_scaler.pkl"
DEFAULT_DATA_ROOT = BASE_DIR / "data"

ROLE_DISPLAY_NAMES = {
    "left_foot": "左脚",
    "right_foot": "右脚",
}
ROLE_ORDER = ("left_foot", "right_foot")
TRIGGER_ROLE = "left_foot"
DEFAULT_ROLE_TO_MAC = {
    "left_foot": "D4:22:CD:00:84:61",
    "right_foot": "D4:22:CD:00:85:22",
}
IMU_MAC_PREFIX = "D4:22:CD:00"
MOVELLA_COMPANY_ID = 0x0886
MOVELLA_CONFIG_SERVICE_UUID = "15171000-4947-11e9-8646-d663bd873d93"
MOVELLA_MEASUREMENT_SERVICE_UUID = "15172000-4947-11e9-8646-d663bd873d93"
INSOLE_V2_LEFT_MAC_MARKER = "FF2502051A4B"
INSOLE_V2_RIGHT_MAC_MARKER = "FF25020518F3"
INSOLE_ROLE_DISPLAY_NAMES = {
    "left_insole": "左鞋垫",
    "right_insole": "右鞋垫",
}
INSOLE_ROLE_ORDER = ("left_insole", "right_insole")
DEFAULT_INSOLE_ROLE_TO_MAC = {
    "left_insole": "FF:25:02:05:1A:4B",
    "right_insole": "FF:25:02:05:18:F3",
}

IMU_CHANNELS = [
    "left_imu_Euler_Y",
    "right_imu_Euler_Y",
    "left_imu_Euler_X",
    "right_imu_Euler_X",
    "left_imu_Gyr_X",
    "left_imu_Gyr_Y",
    "left_imu_Gyr_Z",
    "left_imu_Acc_X",
    "left_imu_Acc_Y",
    "left_imu_Acc_Z",
]
N_CHANNELS = len(IMU_CHANNELS)
N_FEATS_PER_CH = 7
INPUT_DIM = N_CHANNELS * N_FEATS_PER_CH

WINDOW_N = 27
DEFAULT_SAMPLE_RATE_HZ = 30.0
DEFAULT_FILTER_PROFILE = "GENERAL"
DEFAULT_PAYLOAD_MODE = "CUSTOM_MODE_5"
DEFAULT_STALE_TIMEOUT_SEC = 0.7
DEFAULT_STATUS_INTERVAL_SEC = 1.0
DEFAULT_POLL_INTERVAL_MS = 100
DEFAULT_SESSION_TIME_FORMAT = "%Y%m%d_%H%M%S"
DEFAULT_ROW_TIME_FORMAT = "%Y-%m-%d %H:%M:%S.%f"
DEFAULT_TK_DPI = 96.0
DEFAULT_BLE_CONNECT_TIMEOUT_SEC = 15.0
DEFAULT_BLE_SERVICE_TIMEOUT_SEC = 8.0
DEFAULT_BLE_DISCONNECT_TIMEOUT_SEC = 5.0
DEFAULT_BLE_CONNECT_RETRIES = 3
DEFAULT_BLE_RETRY_DELAY_SEC = 1.0
DEFAULT_BLE_SETTLE_SEC = 0.35
DEFAULT_UI_FONT_SIZE = 13
DEFAULT_SMALL_FONT_SIZE = 11
DEFAULT_MONO_FONT_SIZE = 12
COMBINED_INPUT_SOURCE = "左脚+右脚"
PLOT_HISTORY_LENGTH = 240
PLOT_COLORS = {
    "phase": "#0f4c81",
    "raw_phase": "#8a8f98",
    "grid": "#d9dde3",
    "axis": "#7b8794",
}
PREFERRED_UI_FONTS = [
    "Noto Sans CJK SC",
    "Source Han Sans CN",
    "Microsoft YaHei UI",
    "Microsoft YaHei",
    "WenQuanYi Micro Hei",
    "PingFang SC",
    "Heiti SC",
    "SimHei",
    "DejaVu Sans",
]
PREFERRED_MONO_FONTS = [
    "JetBrains Mono",
    "Cascadia Mono",
    "DejaVu Sans Mono",
    "Consolas",
]
SENSOR_VALUE_FIELDS = (
    ("欧拉 X (deg)", "euler_x"),
    ("欧拉 Y (deg)", "euler_y"),
    ("欧拉 Z (deg)", "euler_z"),
    ("加速度 X", "acc_x"),
    ("加速度 Y", "acc_y"),
    ("加速度 Z", "acc_z"),
    ("陀螺仪 X", "gyro_x"),
    ("陀螺仪 Y", "gyro_y"),
    ("陀螺仪 Z", "gyro_z"),
)
INSOLE_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
INSOLE_READ_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
INSOLE_FRAME_LENGTH = 39
INSOLE_VALUE_COUNT = 18


@dataclass
class TkRenderingInfo:
    applied_scaling: float
    system_dpi: float
    source: str


def choose_font_family(root: tk.Tk, candidates: list[str], fallback_font_name: str) -> str:
    available = set(tkfont.families(root))
    for family in candidates:
        if family in available:
            return family
    return str(tkfont.nametofont(fallback_font_name, root=root).actual("family"))


def detect_system_dpi(root: tk.Tk) -> float:
    try:
        dpi = float(root.winfo_fpixels("1i"))
    except (tk.TclError, ValueError):
        dpi = DEFAULT_TK_DPI
    if not np.isfinite(dpi) or dpi <= 0.0:
        return DEFAULT_TK_DPI
    return dpi


def resolve_tk_scaling(root: tk.Tk, ui_scale: Optional[float]) -> TkRenderingInfo:
    system_dpi = detect_system_dpi(root)
    if ui_scale is not None:
        return TkRenderingInfo(
            applied_scaling=float(ui_scale),
            system_dpi=system_dpi,
            source="命令行参数",
        )

    scaling_override = os.environ.get("TK_UI_SCALING", "").strip()
    if scaling_override:
        try:
            scaling = float(scaling_override)
        except ValueError:
            scaling = system_dpi / 72.0
            source = "系统 DPI"
        else:
            if np.isfinite(scaling) and scaling > 0.0:
                source = "环境变量 TK_UI_SCALING"
            else:
                scaling = system_dpi / 72.0
                source = "系统 DPI"
        return TkRenderingInfo(
            applied_scaling=float(scaling),
            system_dpi=system_dpi,
            source=source,
        )

    return TkRenderingInfo(
        applied_scaling=system_dpi / 72.0,
        system_dpi=system_dpi,
        source="系统 DPI",
    )


def configure_tk_rendering(root: tk.Tk, ui_scale: Optional[float] = None) -> TkRenderingInfo:
    rendering_info = resolve_tk_scaling(root, ui_scale=ui_scale)
    scaling = rendering_info.applied_scaling

    root.tk.call("tk", "scaling", scaling)

    ui_family = choose_font_family(root, PREFERRED_UI_FONTS, "TkDefaultFont")
    mono_family = choose_font_family(root, PREFERRED_MONO_FONTS, "TkFixedFont")
    font_config = {
        "TkDefaultFont": (ui_family, DEFAULT_UI_FONT_SIZE),
        "TkTextFont": (ui_family, DEFAULT_UI_FONT_SIZE),
        "TkMenuFont": (ui_family, DEFAULT_UI_FONT_SIZE),
        "TkHeadingFont": (ui_family, DEFAULT_UI_FONT_SIZE),
        "TkCaptionFont": (ui_family, DEFAULT_UI_FONT_SIZE),
        "TkSmallCaptionFont": (ui_family, DEFAULT_SMALL_FONT_SIZE),
        "TkTooltipFont": (ui_family, DEFAULT_SMALL_FONT_SIZE),
        "TkIconFont": (ui_family, DEFAULT_SMALL_FONT_SIZE),
        "TkFixedFont": (mono_family, DEFAULT_MONO_FONT_SIZE),
    }
    for font_name, (family, size) in font_config.items():
        try:
            tkfont.nametofont(font_name, root=root).configure(family=family, size=size)
        except tk.TclError:
            pass

    root.option_add("*Font", f"{{{ui_family}}} {DEFAULT_UI_FONT_SIZE}")
    root.option_add("*Text.Font", f"{{{mono_family}}} {DEFAULT_MONO_FONT_SIZE}")

    style = ttk.Style(root)
    try:
        if "clam" in style.theme_names():
            style.theme_use("clam")
    except tk.TclError:
        pass
    style.configure(".", font=(ui_family, DEFAULT_UI_FONT_SIZE))
    style.configure("TLabel", font=(ui_family, DEFAULT_UI_FONT_SIZE))
    style.configure("TButton", font=(ui_family, DEFAULT_UI_FONT_SIZE))
    style.configure("TEntry", font=(ui_family, DEFAULT_UI_FONT_SIZE))
    style.configure("TSpinbox", font=(ui_family, DEFAULT_UI_FONT_SIZE))
    style.configure("TLabelframe.Label", font=(ui_family, DEFAULT_UI_FONT_SIZE, "bold"))
    return rendering_info


@dataclass
class SensorSample:
    role: str
    mac_address: str
    timestamp_us: Optional[int]
    arrival_time: float
    quat_w: Optional[float]
    quat_x: Optional[float]
    quat_y: Optional[float]
    quat_z: Optional[float]
    euler_x: float
    euler_y: float
    euler_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass
class ReadinessSnapshot:
    all_ready: bool
    fresh_roles: dict[str, bool]
    age_by_role: dict[str, Optional[float]]
    missing_roles: tuple[str, ...]
    stale_roles: tuple[str, ...]

    def summary_text(self) -> str:
        parts: list[str] = []
        for role in ROLE_ORDER:
            is_fresh = self.fresh_roles.get(role, False)
            age = self.age_by_role.get(role)
            label = ROLE_DISPLAY_NAMES[role]
            if is_fresh:
                parts.append(f"{label}=OK")
            elif age is None:
                parts.append(f"{label}=未收到")
            else:
                parts.append(f"{label}=超时({age:.2f}s)")
        return " | ".join(parts)


@dataclass
class PhasePrediction:
    input_source: str
    timestamp_us: Optional[int]
    raw_phase: float
    phase: float
    stride_rate_hz: float
    cos_value: float
    sin_value: float
    window_size: int
    total_rows: int
    total_predictions: int


@dataclass
class InsoleSample:
    role: str
    mac_address: str
    timestamp_ms: int
    arrival_time: float
    foot_id: int
    values: list[int]


@dataclass
class InsoleScanResult:
    address: str
    display_name: str
    rssi: int
    inferred_role: Optional[str]
    ble_device: object


@dataclass
class IMUScanResult:
    address: str
    display_name: str
    rssi: int
    inferred_role: Optional[str]
    ble_device: object


class SessionCsvRecorder:
    def __init__(self, data_root: Path, event_emitter=None):
        self.data_root = data_root.expanduser().resolve()
        self.event_emitter = event_emitter
        self.session_dir: Optional[Path] = None
        self._lock = threading.Lock()
        self._files: dict[str, object] = {}
        self._writers: dict[str, object] = {}

    def start_session(self) -> Path:
        with self._lock:
            self._close_locked()
            self.data_root.mkdir(parents=True, exist_ok=True)
            base_name = datetime.now().strftime(DEFAULT_SESSION_TIME_FORMAT)
            session_dir = self.data_root / base_name
            suffix = 1
            while session_dir.exists():
                session_dir = self.data_root / f"{base_name}_{suffix}"
                suffix += 1
            session_dir.mkdir(parents=True, exist_ok=False)
            self.session_dir = session_dir

        self._emit("session", path=str(session_dir))
        self._emit("log", message=f"数据保存目录: {session_dir}")
        return session_dir

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def record_imu_sample(self, sample: SensorSample) -> None:
        row = [
            self._local_time_text(),
            sample.arrival_time,
            self._nullable(sample.timestamp_us),
            sample.role,
            ROLE_DISPLAY_NAMES.get(sample.role, sample.role),
            sample.mac_address,
            self._nullable(sample.quat_w),
            self._nullable(sample.quat_x),
            self._nullable(sample.quat_y),
            self._nullable(sample.quat_z),
            sample.euler_x,
            sample.euler_y,
            sample.euler_z,
            sample.acc_x,
            sample.acc_y,
            sample.acc_z,
            sample.gyro_x,
            sample.gyro_y,
            sample.gyro_z,
        ]
        header = [
            "local_time",
            "arrival_time_monotonic",
            "timestamp_us",
            "role",
            "role_label",
            "mac_address",
            "quat_w",
            "quat_x",
            "quat_y",
            "quat_z",
            "euler_x",
            "euler_y",
            "euler_z",
            "acc_x",
            "acc_y",
            "acc_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
        ]
        self._write_row(f"imu:{sample.role}", f"{sample.role}_imu.csv", header, row)

    def record_insole_sample(self, sample: InsoleSample) -> None:
        header = [
            "local_time",
            "arrival_time_monotonic",
            "timestamp_ms",
            "role",
            "role_label",
            "mac_address",
            "foot_id",
        ] + [f"p{index}" for index in range(1, INSOLE_VALUE_COUNT + 1)]
        row = [
            self._local_time_text(),
            sample.arrival_time,
            sample.timestamp_ms,
            sample.role,
            INSOLE_ROLE_DISPLAY_NAMES.get(sample.role, sample.role),
            sample.mac_address,
            sample.foot_id,
            *sample.values,
        ]
        self._write_row(
            f"insole:{sample.role}",
            f"{sample.role}_pressure.csv",
            header,
            row,
        )

    def record_phase_prediction(self, prediction: PhasePrediction) -> None:
        header = [
            "local_time",
            "timestamp_us",
            "input_source",
            "phase",
            "raw_phase",
            "stride_rate_hz",
            "cos_value",
            "sin_value",
            "window_size",
            "total_rows",
            "total_predictions",
        ]
        row = [
            self._local_time_text(),
            self._nullable(prediction.timestamp_us),
            prediction.input_source,
            prediction.phase,
            prediction.raw_phase,
            prediction.stride_rate_hz,
            prediction.cos_value,
            prediction.sin_value,
            prediction.window_size,
            prediction.total_rows,
            prediction.total_predictions,
        ]
        self._write_row("phase", "phase_predictions.csv", header, row)

    def _write_row(self, key: str, filename: str, header: list[str], row: list[object]) -> None:
        with self._lock:
            sink = self._ensure_writer_locked(key, filename, header)
            if sink is None:
                return
            writer = self._writers[key]
            handle = self._files[key]
            writer.writerow(row)
            handle.flush()

    def _ensure_writer_locked(self, key: str, filename: str, header: list[str]):
        if self.session_dir is None:
            return None
        if key in self._writers:
            return self._writers[key]
        path = self.session_dir / filename
        handle = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(handle)
        writer.writerow(header)
        handle.flush()
        self._files[key] = handle
        self._writers[key] = writer
        return writer

    def _close_locked(self) -> None:
        for handle in self._files.values():
            try:
                handle.close()
            except Exception:
                pass
        self._files.clear()
        self._writers.clear()
        self.session_dir = None

    def _emit(self, event_type: str, **payload) -> None:
        if self.event_emitter is None:
            return
        self.event_emitter(event_type, **payload)

    @staticmethod
    def _local_time_text() -> str:
        return datetime.now().strftime(DEFAULT_ROW_TIME_FORMAT)[:-3]

    @staticmethod
    def _nullable(value):
        return "" if value is None else value


class InsolePacketParser:
    def __init__(self):
        self._cache: list[int] = []

    def clear(self) -> None:
        self._cache.clear()

    def parse(self, packet: bytes | bytearray) -> Optional[tuple[int, list[int]]]:
        if not packet:
            return None

        values = [int(byte) & 0xFF for byte in packet]
        try:
            head_index = values.index(0xAA)
        except ValueError:
            head_index = -1

        if values[0] == 0xAA:
            self._cache = list(values)
        elif head_index >= 0:
            self._cache = list(values[head_index:])
        elif self._cache:
            self._cache.extend(values)
        else:
            return None

        if not self._cache or self._cache[0] != 0xAA:
            self._cache.clear()
            return None

        if len(self._cache) < INSOLE_FRAME_LENGTH:
            return None

        frame = self._cache[:INSOLE_FRAME_LENGTH]
        remains = self._cache[INSOLE_FRAME_LENGTH:]
        self._cache = remains
        if self._cache and self._cache[0] != 0xAA:
            try:
                next_head_index = self._cache.index(0xAA)
            except ValueError:
                self._cache = []
            else:
                self._cache = self._cache[next_head_index:]

        return self._parse_frame(frame)

    def _parse_frame(self, frame: list[int]) -> Optional[tuple[int, list[int]]]:
        if not frame or frame[0] != 0xAA:
            return None

        raw_sensor_count = max(0, (len(frame) - 3) // 2)
        if raw_sensor_count < INSOLE_VALUE_COUNT:
            return None

        foot_id = frame[1]
        raw_values: list[int] = []
        for index in range(raw_sensor_count):
            high = frame[2 + index * 2]
            low = frame[3 + index * 2]
            raw_values.append(high * 256 + low)

        values = raw_values[:INSOLE_VALUE_COUNT]
        if len(values) < INSOLE_VALUE_COUNT:
            return None
        return foot_id, values


def canonical_mac(mac: str) -> str:
    return mac.strip().replace("-", ":").upper()


def infer_insole_role(mac_address: str, name: str = "") -> Optional[str]:
    normalized_mac = mac_address.replace(":", "").upper()
    normalized_name = name.strip().upper()
    if INSOLE_V2_LEFT_MAC_MARKER in normalized_mac or "左" in name or "LEFT" in normalized_name:
        return "left_insole"
    if INSOLE_V2_RIGHT_MAC_MARKER in normalized_mac or "右" in name or "RIGHT" in normalized_name:
        return "right_insole"
    return None


def infer_imu_role(
    mac_address: str,
    configured_role_to_mac: Optional[dict[str, str]] = None,
    name: str = "",
) -> Optional[str]:
    normalized_mac = canonical_mac(mac_address)
    if configured_role_to_mac is not None:
        for role in ROLE_ORDER:
            configured_mac = configured_role_to_mac.get(role)
            if configured_mac and canonical_mac(configured_mac) == normalized_mac:
                return role

    normalized_name = name.strip().upper()
    if "左" in name or "LEFT" in normalized_name:
        return "left_foot"
    if "右" in name or "RIGHT" in normalized_name:
        return "right_foot"
    return None


def normalize_uuid(raw: str) -> str:
    return raw.lower().replace("-", "")


def has_insole_service_uuid(service_uuids: Optional[list[str]] = None) -> bool:
    if not service_uuids:
        return False
    short_uuid = "fff0"
    full_prefix = normalize_uuid(INSOLE_SERVICE_UUID)[:8]
    for uuid_text in service_uuids:
        normalized = normalize_uuid(str(uuid_text))
        if short_uuid in normalized or normalized.startswith(full_prefix):
            return True
    return False


def is_probable_insole_name(name: str) -> bool:
    normalized_name = name.strip().upper()
    if not normalized_name:
        return False
    return (
        normalized_name.startswith("NB-")
        or "INSOLE" in normalized_name
        or "鞋垫" in name
    )


def is_insole_scan_candidate(
    name: str,
    address: str,
    service_uuids: Optional[list[str]] = None,
    configured_role_to_mac: Optional[dict[str, str]] = None,
) -> bool:
    normalized_address = canonical_mac(address).replace(":", "")
    allowed_addresses = {
        INSOLE_V2_LEFT_MAC_MARKER,
        INSOLE_V2_RIGHT_MAC_MARKER,
    }
    insole_vendor_prefix = "FF250205"
    if configured_role_to_mac is not None:
        allowed_addresses.update(
            canonical_mac(mac).replace(":", "")
            for mac in configured_role_to_mac.values()
            if str(mac).strip()
        )
    return (
        normalized_address in allowed_addresses
        or normalized_address.startswith(insole_vendor_prefix)
        or has_insole_service_uuid(service_uuids)
        or is_probable_insole_name(name)
    )


def is_imu_scan_candidate(
    name: str,
    address: str,
    service_uuids: Optional[list[str]] = None,
    manufacturer_ids: Optional[set[int]] = None,
    configured_role_to_mac: Optional[dict[str, str]] = None,
) -> bool:
    normalized_address = canonical_mac(address)

    configured_addresses = {
        canonical_mac(mac)
        for mac in (configured_role_to_mac or DEFAULT_ROLE_TO_MAC).values()
        if str(mac).strip()
    }
    if normalized_address in configured_addresses:
        return True

    if normalized_address.startswith(IMU_MAC_PREFIX):
        return True

    normalized_name = name.strip().upper()
    if (
        "MOVELLA DOT" in normalized_name
        or "XSENS DOT" in normalized_name
        or ("MOVELLA" in normalized_name and "DOT" in normalized_name)
    ):
        return True

    target_services = {
        normalize_uuid(MOVELLA_CONFIG_SERVICE_UUID),
        normalize_uuid(MOVELLA_MEASUREMENT_SERVICE_UUID),
    }
    for uuid_text in service_uuids or []:
        if normalize_uuid(str(uuid_text)) in target_services:
            return True

    if manufacturer_ids and MOVELLA_COMPANY_ID in manufacturer_ids:
        return True

    return False


def iter_service_collection_items(services) -> list[object]:
    raw = getattr(services, "services", None)
    if isinstance(raw, dict):
        return list(raw.values())
    try:
        return list(services)
    except TypeError:
        return []


def has_gatt_service_uuid(services, uuid_text: str) -> bool:
    target = normalize_uuid(uuid_text)
    get_service = getattr(services, "get_service", None)
    if callable(get_service):
        try:
            return get_service(uuid_text) is not None
        except Exception:
            pass
    for service in iter_service_collection_items(services):
        if normalize_uuid(str(getattr(service, "uuid", ""))) == target:
            return True
    return False


def has_gatt_characteristic_uuid(services, uuid_text: str) -> bool:
    target = normalize_uuid(uuid_text)
    for service in iter_service_collection_items(services):
        characteristics = getattr(service, "characteristics", None)
        if isinstance(characteristics, dict):
            iterator = characteristics.values()
        else:
            iterator = characteristics or []
        for characteristic in iterator:
            if normalize_uuid(str(getattr(characteristic, "uuid", ""))) == target:
                return True
    return False


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def circular_delta(current: float, previous: float) -> float:
    return (current - previous + 0.5) % 1.0 - 0.5


def recover_phase(cos_value: float, sin_value: float) -> float:
    return (math.atan2(sin_value, cos_value) / (2.0 * math.pi)) % 1.0


def quaternion_to_euler_xyz_degrees(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return 0.0, 0.0, 0.0
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def extract_window_features(window: np.ndarray) -> np.ndarray:
    mid = len(window) // 2
    return np.array(
        [
            window.max(),
            window.min(),
            window.mean(),
            window.std(),
            window[0],
            window[mid],
            window[-1],
        ],
        dtype=np.float32,
    )


def combine_samples_to_model_channels(
    left_sample: SensorSample,
    right_sample: SensorSample,
) -> np.ndarray:
    return np.array(
        [
            left_sample.euler_y,
            right_sample.euler_y,
            left_sample.euler_x,
            right_sample.euler_x,
            left_sample.gyro_x,
            left_sample.gyro_y,
            left_sample.gyro_z,
            left_sample.acc_x,
            left_sample.acc_y,
            left_sample.acc_z,
        ],
        dtype=np.float32,
    )


def extract_model_feature_vector(window_rows: np.ndarray) -> np.ndarray:
    feats = [extract_window_features(window_rows[:, idx]) for idx in range(window_rows.shape[1])]
    return np.concatenate(feats).astype(np.float32)


def detect_csv_header_row(path: Path) -> int:
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for idx, line in enumerate(f):
            stripped = line.lstrip()
            if stripped.startswith("Label,"):
                return idx
            if stripped.startswith("label,"):
                return idx
            if stripped.startswith("seq,"):
                return idx
    return 0


def _pick_column(columns: list[str], candidates: tuple[str, ...]) -> Optional[str]:
    lowered = {col.lower(): col for col in columns}
    for name in candidates:
        found = lowered.get(name.lower())
        if found is not None:
            return found
    return None


def load_replay_samples(path: Path, role: str, fallback_mac: str) -> list[SensorSample]:
    import pandas as pd

    header_row = detect_csv_header_row(path)
    df = pd.read_csv(path, skiprows=header_row)
    df.columns = [str(col).strip() for col in df.columns]
    columns = list(df.columns)

    quat_w_col = _pick_column(columns, ("Quat_W", "quat_w"))
    quat_x_col = _pick_column(columns, ("Quat_X", "quat_x"))
    quat_y_col = _pick_column(columns, ("Quat_Y", "quat_y"))
    quat_z_col = _pick_column(columns, ("Quat_Z", "quat_z"))
    euler_x_col = _pick_column(columns, ("Euler_X", "euler_roll"))
    euler_y_col = _pick_column(columns, ("Euler_Y", "euler_pitch"))
    euler_z_col = _pick_column(columns, ("Euler_Z", "euler_yaw"))
    acc_x_col = _pick_column(columns, ("Acc_X", "accel_x"))
    acc_y_col = _pick_column(columns, ("Acc_Y", "accel_y"))
    acc_z_col = _pick_column(columns, ("Acc_Z", "accel_z"))
    gyr_x_col = _pick_column(columns, ("Gyr_X", "ang_vel_x", "gyro_x"))
    gyr_y_col = _pick_column(columns, ("Gyr_Y", "ang_vel_y", "gyro_y"))
    gyr_z_col = _pick_column(columns, ("Gyr_Z", "ang_vel_z", "gyro_z"))
    timestamp_col = _pick_column(columns, ("SampleTimeFine", "timestamp_us"))
    mac_col = _pick_column(columns, ("mac_address",))

    required = [acc_x_col, acc_y_col, acc_z_col, gyr_x_col, gyr_y_col, gyr_z_col]
    if any(col is None for col in required):
        raise ValueError(f"CSV 缺少加速度/角速度列，无法回放: {path}")

    samples: list[SensorSample] = []
    base_arrival = time.monotonic()
    for idx, row in df.iterrows():
        try:
            acc_x = float(row[acc_x_col])  # type: ignore[index]
            acc_y = float(row[acc_y_col])  # type: ignore[index]
            acc_z = float(row[acc_z_col])  # type: ignore[index]
            gyro_x = float(row[gyr_x_col])  # type: ignore[index]
            gyro_y = float(row[gyr_y_col])  # type: ignore[index]
            gyro_z = float(row[gyr_z_col])  # type: ignore[index]
        except Exception:
            continue

        quat_w = quat_x = quat_y = quat_z = None
        if quat_w_col and quat_x_col and quat_y_col and quat_z_col:
            try:
                quat_w = float(row[quat_w_col])
                quat_x = float(row[quat_x_col])
                quat_y = float(row[quat_y_col])
                quat_z = float(row[quat_z_col])
            except Exception:
                quat_w = quat_x = quat_y = quat_z = None

        has_euler = False
        if euler_x_col and euler_y_col and euler_z_col:
            try:
                euler_x = float(row[euler_x_col])  # type: ignore[index]
                euler_y = float(row[euler_y_col])  # type: ignore[index]
                euler_z = float(row[euler_z_col])  # type: ignore[index]
                has_euler = bool(np.isfinite([euler_x, euler_y, euler_z]).all())
            except Exception:
                has_euler = False

        if not has_euler:
            if None in (quat_w, quat_x, quat_y, quat_z):
                continue
            euler_x, euler_y, euler_z = quaternion_to_euler_xyz_degrees(
                float(quat_w),
                float(quat_x),
                float(quat_y),
                float(quat_z),
            )

        timestamp_us: Optional[int] = None
        if timestamp_col is not None:
            try:
                timestamp_us = int(float(row[timestamp_col]))
            except Exception:
                timestamp_us = None

        mac_address = fallback_mac
        if mac_col is not None:
            try:
                mac_address = canonical_mac(str(row[mac_col]))
            except Exception:
                mac_address = fallback_mac

        sample = SensorSample(
            role=role,
            mac_address=canonical_mac(mac_address),
            timestamp_us=timestamp_us,
            arrival_time=base_arrival + idx * 1e-3,
            quat_w=quat_w,
            quat_x=quat_x,
            quat_y=quat_y,
            quat_z=quat_z,
            euler_x=float(euler_x),
            euler_y=float(euler_y),
            euler_z=float(euler_z),
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
        )
        left = sample if role == "left_foot" else sample
        right = sample if role == "right_foot" else sample
        probe = combine_samples_to_model_channels(left, right)
        if np.isfinite(probe).all():
            samples.append(sample)

    if not samples:
        raise ValueError(f"CSV 中没有可用于回放的有效样本: {path}")
    return samples


class DenseGaitPhaseNet:
    def __init__(self, model_path: Path):
        params = load_float_dense_weights(model_path)
        self.w1 = params["w1"]
        self.b1 = params["b1"]
        self.w2 = params["w2"]
        self.b2 = params["b2"]
        self.w3 = params["w3"]
        self.b3 = params["b3"]

        if self.w1.ndim != 2 or self.w2.ndim != 2 or self.w3.ndim != 2:
            raise ValueError("模型权重维度异常，当前只支持全连接网络。")
        if self.w3.shape[1] != 3:
            raise ValueError(f"输出层维度异常，预期 3，实际 {self.w3.shape}")

        self.input_dim = int(self.w1.shape[0])
        self.model_path = model_path
        self.model_format = model_path.suffix.lower().lstrip(".")

    def predict(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.input_dim:
            raise ValueError(f"模型输入应为 (N, {self.input_dim})，收到 {x.shape}")
        h1 = relu(x @ self.w1 + self.b1)
        h2 = relu(h1 @ self.w2 + self.b2)
        return h2 @ self.w3 + self.b3


class SensorFreshnessTracker:
    def __init__(self, roles: tuple[str, ...], stale_timeout_sec: float):
        self.roles = roles
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.latest_by_role: dict[str, SensorSample] = {}
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self.latest_by_role.clear()

    def update(self, sample: SensorSample) -> ReadinessSnapshot:
        with self._lock:
            self.latest_by_role[sample.role] = sample
            return self._snapshot_locked(now=time.monotonic())

    def snapshot(self) -> ReadinessSnapshot:
        with self._lock:
            return self._snapshot_locked(now=time.monotonic())

    def latest_samples(self) -> dict[str, SensorSample]:
        with self._lock:
            return dict(self.latest_by_role)

    def _snapshot_locked(self, now: float) -> ReadinessSnapshot:
        fresh_roles: dict[str, bool] = {}
        age_by_role: dict[str, Optional[float]] = {}
        missing_roles: list[str] = []
        stale_roles: list[str] = []

        for role in self.roles:
            sample = self.latest_by_role.get(role)
            if sample is None:
                fresh_roles[role] = False
                age_by_role[role] = None
                missing_roles.append(role)
                continue

            age = max(0.0, now - sample.arrival_time)
            age_by_role[role] = age
            is_fresh = age <= self.stale_timeout_sec
            fresh_roles[role] = is_fresh
            if not is_fresh:
                stale_roles.append(role)

        return ReadinessSnapshot(
            all_ready=not missing_roles and not stale_roles,
            fresh_roles=fresh_roles,
            age_by_role=age_by_role,
            missing_roles=tuple(missing_roles),
            stale_roles=tuple(stale_roles),
        )


class RealtimeGaitPhasePredictor:
    def __init__(
        self,
        model_path: Path,
        scaler_path: Path,
        sample_rate_hz: float,
    ):
        self.model = DenseGaitPhaseNet(model_path)
        self.scaler = load_feature_scaler(scaler_path)
        self.sample_rate_hz = float(sample_rate_hz)

        if self.model.input_dim != INPUT_DIM:
            raise ValueError(
                f"当前双 IMU 通道配置需要模型输入维度 {INPUT_DIM}，"
                f"但 {self.model.model_format.upper()} 模型实际输入是 {self.model.input_dim}。"
                "需要使用与 IMU_CHANNELS 配套训练/导出的模型。"
            )

        scaler_dim = int(getattr(self.scaler, "n_features_in_", INPUT_DIM))
        if scaler_dim != INPUT_DIM:
            raise ValueError(
                f"当前双 IMU 通道配置需要 scaler 输入维度 {INPUT_DIM}，"
                f"但 feature_scaler.pkl 实际是 {scaler_dim}。"
                "需要使用与新通道配置配套重新训练得到的 scaler。"
            )

        self.window_size = WINDOW_N
        self.buffer: deque[np.ndarray] = deque(maxlen=self.window_size)
        self.total_rows = 0
        self.total_predictions = 0
        self.prev_phase: Optional[float] = None

    def reset(self) -> None:
        self.buffer.clear()
        self.total_rows = 0
        self.total_predictions = 0
        self.prev_phase = None

    def push_combined_row(
        self,
        row: np.ndarray,
        timestamp_us: Optional[int] = None,
    ) -> Optional[PhasePrediction]:
        self.buffer.append(np.asarray(row, dtype=np.float32))
        self.total_rows += 1

        if len(self.buffer) < self.window_size:
            return None

        window = np.array(self.buffer, dtype=np.float32)
        feature_vector = extract_model_feature_vector(window).reshape(1, -1)
        scaled = self.scaler.transform(feature_vector).astype(np.float32)
        pred = self.model.predict(scaled)[0]

        cos_value = float(pred[0])
        sin_value = float(pred[1])
        stride_rate_hz = float(pred[2])
        raw_phase = recover_phase(cos_value, sin_value)
        phase = self._apply_phase_rate_limit_step(raw_phase, stride_rate_hz)

        self.total_predictions += 1
        return PhasePrediction(
            input_source=COMBINED_INPUT_SOURCE,
            timestamp_us=timestamp_us,
            raw_phase=raw_phase,
            phase=phase,
            stride_rate_hz=stride_rate_hz,
            cos_value=cos_value,
            sin_value=sin_value,
            window_size=self.window_size,
            total_rows=self.total_rows,
            total_predictions=self.total_predictions,
        )

    def _apply_phase_rate_limit_step(self, raw_phase: float, stride_rate_hz: float) -> float:
        if not np.isfinite(stride_rate_hz):
            stride_rate_hz = 1.0
        r_clamped = float(np.clip(stride_rate_hz, 0.3, 2.5))
        if self.prev_phase is None:
            self.prev_phase = raw_phase
            return raw_phase

        expected_delta = r_clamped / self.sample_rate_hz
        delta = circular_delta(raw_phase, self.prev_phase)
        min_delta = expected_delta * 0.2
        max_delta = expected_delta * 1.5
        if delta < min_delta:
            delta = expected_delta
        elif delta > max_delta:
            delta = max_delta

        phase = (self.prev_phase + delta) % 1.0
        self.prev_phase = phase
        return phase


class MultiSensorRuntime:
    def __init__(
        self,
        predictor: RealtimeGaitPhasePredictor,
        stale_timeout_sec: float,
        event_queue: Queue,
        recorder: Optional[SessionCsvRecorder] = None,
    ):
        self.predictor = predictor
        self.tracker = SensorFreshnessTracker(ROLE_ORDER, stale_timeout_sec)
        self.event_queue = event_queue
        self.recorder = recorder
        self._last_wait_message_at = 0.0

    def emit(self, event_type: str, **payload) -> None:
        self.event_queue.put({"type": event_type, **payload})

    def reset(self) -> None:
        self.predictor.reset()
        self.tracker.reset()
        self._last_wait_message_at = 0.0

    def process_sample(self, sample: SensorSample) -> Optional[PhasePrediction]:
        readiness = self.tracker.update(sample)
        if self.recorder is not None:
            self.recorder.record_imu_sample(sample)
        self.emit("sample", sample=sample, readiness=readiness)

        if sample.role != TRIGGER_ROLE:
            return None

        if not readiness.all_ready:
            self._maybe_emit_wait_status(readiness)
            return None

        latest = self.tracker.latest_samples()
        left_sample = latest.get("left_foot")
        right_sample = latest.get("right_foot")
        if left_sample is None or right_sample is None:
            self._maybe_emit_wait_status(readiness)
            return None

        row = combine_samples_to_model_channels(left_sample, right_sample)
        prediction = self.predictor.push_combined_row(row=row, timestamp_us=left_sample.timestamp_us)
        if prediction is not None:
            if self.recorder is not None:
                self.recorder.record_phase_prediction(prediction)
            self.emit("prediction", prediction=prediction, readiness=readiness)
            return prediction

        self._maybe_emit_wait_status(readiness)
        return None

    def _maybe_emit_wait_status(self, readiness: ReadinessSnapshot) -> None:
        now = time.monotonic()
        if now - self._last_wait_message_at < DEFAULT_STATUS_INTERVAL_SEC:
            return
        if len(self.predictor.buffer) < self.predictor.window_size and readiness.all_ready:
            self.emit(
                "status",
                status=(
                    f"等待联合窗口填满: {len(self.predictor.buffer)}/{self.predictor.window_size} "
                    f"| 输入通道数={N_CHANNELS} | 特征维度={INPUT_DIM}"
                ),
            )
        else:
            self.emit("status", status=f"等待左右脚 IMU 都在线: {readiness.summary_text()}")
        self._last_wait_message_at = now


def import_live_dependencies():
    if str(MOVELLA_READER_ROOT) not in sys.path:
        sys.path.append(str(MOVELLA_READER_ROOT))
    try:
        from bleak import BleakClient
        from bleak import BleakScanner
        from movella_dot_py.core.sensor import MovellaDOTSensor
        from movella_dot_py.models.data_structures import SensorConfiguration
        from movella_dot_py.models.enums import FilterProfile, OutputRate, PayloadMode
    except ImportError as exc:
        raise RuntimeError(
            "实时蓝牙模式缺少依赖。请安装 `bleak`，并确保 `Xsens_DOT_PC_Reader` 可导入。"
        ) from exc
    return (
        BleakClient,
        BleakScanner,
        MovellaDOTSensor,
        SensorConfiguration,
        FilterProfile,
        OutputRate,
        PayloadMode,
    )


def resolve_output_rate_enum(output_rate_enum_cls, sample_rate_hz: float):
    desired_hz = int(round(sample_rate_hz))
    valid = {int(item): item for item in output_rate_enum_cls}
    if desired_hz in valid:
        return valid[desired_hz]
    if 30 in valid:
        return valid[30]
    return next(iter(valid.values()))


def build_sample_from_live_packet(role: str, mac_address: str, parsed) -> Optional[SensorSample]:
    if parsed.acceleration is None or parsed.angular_velocity is None:
        return None

    quat_w = quat_x = quat_y = quat_z = None
    if parsed.quaternion is not None:
        quat_w = float(parsed.quaternion.w)
        quat_x = float(parsed.quaternion.x)
        quat_y = float(parsed.quaternion.y)
        quat_z = float(parsed.quaternion.z)

    if parsed.euler_angles is not None:
        euler_x = float(parsed.euler_angles.roll)
        euler_y = float(parsed.euler_angles.pitch)
        euler_z = float(parsed.euler_angles.yaw)
    elif None not in (quat_w, quat_x, quat_y, quat_z):
        euler_x, euler_y, euler_z = quaternion_to_euler_xyz_degrees(
            float(quat_w),
            float(quat_x),
            float(quat_y),
            float(quat_z),
        )
    else:
        return None

    sample = SensorSample(
        role=role,
        mac_address=canonical_mac(mac_address),
        timestamp_us=int(parsed.timestamp.microseconds) if parsed.timestamp is not None else None,
        arrival_time=time.monotonic(),
        quat_w=quat_w,
        quat_x=quat_x,
        quat_y=quat_y,
        quat_z=quat_z,
        euler_x=euler_x,
        euler_y=euler_y,
        euler_z=euler_z,
        acc_x=float(parsed.acceleration.x),
        acc_y=float(parsed.acceleration.y),
        acc_z=float(parsed.acceleration.z),
        gyro_x=float(parsed.angular_velocity.x),
        gyro_y=float(parsed.angular_velocity.y),
        gyro_z=float(parsed.angular_velocity.z),
    )
    return sample if np.isfinite(combine_samples_to_model_channels(sample, sample)).all() else None


def build_insole_sample(role: str, mac_address: str, parsed: tuple[int, list[int]]) -> InsoleSample:
    foot_id, values = parsed
    return InsoleSample(
        role=role,
        mac_address=canonical_mac(mac_address),
        timestamp_ms=int(time.time() * 1000),
        arrival_time=time.monotonic(),
        foot_id=int(foot_id),
        values=list(values),
    )


class MultiDotLiveController:
    def __init__(
        self,
        role_to_mac: dict[str, str],
        insole_role_to_mac: dict[str, str],
        runtime: MultiSensorRuntime,
        sample_rate_hz: float,
        filter_profile_name: str,
        payload_mode_name: str,
        event_queue: Queue,
        recorder: SessionCsvRecorder,
        ble_connect_timeout_sec: float = DEFAULT_BLE_CONNECT_TIMEOUT_SEC,
        ble_service_timeout_sec: float = DEFAULT_BLE_SERVICE_TIMEOUT_SEC,
        ble_disconnect_timeout_sec: float = DEFAULT_BLE_DISCONNECT_TIMEOUT_SEC,
        ble_connect_retries: int = DEFAULT_BLE_CONNECT_RETRIES,
        ble_retry_delay_sec: float = DEFAULT_BLE_RETRY_DELAY_SEC,
        ble_settle_sec: float = DEFAULT_BLE_SETTLE_SEC,
    ):
        self.role_to_mac = {role: canonical_mac(mac) for role, mac in role_to_mac.items()}
        self.insole_role_to_mac = {
            role: canonical_mac(mac) for role, mac in insole_role_to_mac.items()
        }
        self.runtime = runtime
        self.sample_rate_hz = float(sample_rate_hz)
        self.filter_profile_name = filter_profile_name
        self.payload_mode_name = payload_mode_name
        self.event_queue = event_queue
        self.recorder = recorder
        self.sensors: dict[str, object] = {}
        self.insole_clients: dict[str, object] = {}
        self.insole_parsers: dict[str, InsolePacketParser] = {}
        self.insole_streaming_roles: set[str] = set()
        self.is_streaming = False
        self.ble_connect_timeout_sec = float(ble_connect_timeout_sec)
        self.ble_service_timeout_sec = float(ble_service_timeout_sec)
        self.ble_disconnect_timeout_sec = float(ble_disconnect_timeout_sec)
        self.ble_connect_retries = max(1, int(ble_connect_retries))
        self.ble_retry_delay_sec = max(0.0, float(ble_retry_delay_sec))
        self.ble_settle_sec = max(0.0, float(ble_settle_sec))

        self._loop = asyncio.new_event_loop()
        self._loop_ready = threading.Event()
        self._ble_op_lock: Optional[asyncio.Lock] = None
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        if not self._loop_ready.wait(timeout=3.0):
            raise RuntimeError("BLE 控制线程启动失败")

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._ble_op_lock = asyncio.Lock()
        self._loop_ready.set()
        self._loop.run_forever()

    def emit(self, event_type: str, **payload) -> None:
        self.event_queue.put({"type": event_type, **payload})

    def emit_insole_state(self, role: str, status: str, address: Optional[str] = None) -> None:
        self.emit("insole_state", role=role, status=status, address=address)

    def emit_imu_state(self, role: str, status: str, address: Optional[str] = None) -> None:
        self.emit("imu_state", role=role, status=status, address=address)

    def run_coro(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def _require_ble_lock(self) -> asyncio.Lock:
        if self._ble_op_lock is None:
            raise RuntimeError("BLE 控制线程尚未就绪")
        return self._ble_op_lock

    async def _await_with_timeout(self, awaitable, timeout_sec: float):
        if timeout_sec > 0.0:
            return await asyncio.wait_for(awaitable, timeout=timeout_sec)
        return await awaitable

    async def _settle_delay(self) -> None:
        if self.ble_settle_sec > 0.0:
            await asyncio.sleep(self.ble_settle_sec)

    async def _retry_delay(self) -> None:
        if self.ble_retry_delay_sec > 0.0:
            await asyncio.sleep(self.ble_retry_delay_sec)

    def _is_client_connected(self, client: object) -> bool:
        if client is None:
            return False
        connected = getattr(client, "is_connected", None)
        if callable(connected):
            try:
                connected = connected()
            except Exception:
                return False
        if connected is None:
            return True
        return bool(connected)

    def _is_sensor_connected(self, sensor: object) -> bool:
        if sensor is None:
            return False
        return bool(getattr(sensor, "is_connected", False)) and self._is_client_connected(
            getattr(sensor, "client", None)
        )

    def _emit_retry_log(self, label: str, attempt: int, exc: Exception) -> None:
        message = f"{label} 失败，第 {attempt}/{self.ble_connect_retries} 次: {exc}"
        if attempt < self.ble_connect_retries:
            message += f"，{self.ble_retry_delay_sec:.1f}s 后重试"
        self.emit("log", message=message)

    async def _safe_disconnect_client(self, client: object) -> None:
        if client is None:
            return
        disconnect = getattr(client, "disconnect", None)
        if not callable(disconnect):
            return
        try:
            await self._await_with_timeout(disconnect(), self.ble_disconnect_timeout_sec)
        except Exception:
            pass

    async def _safe_disconnect_sensor(self, sensor: object) -> None:
        if sensor is None:
            return
        disconnect = getattr(sensor, "disconnect", None)
        try:
            if callable(disconnect):
                await self._await_with_timeout(disconnect(), self.ble_disconnect_timeout_sec)
            else:
                await self._safe_disconnect_client(getattr(sensor, "client", None))
        except Exception:
            pass
        finally:
            try:
                sensor.is_connected = False
            except Exception:
                pass

    async def _safe_stop_insole_notify(self, role: str, client: object) -> None:
        if client is None:
            return
        try:
            await self._await_with_timeout(
                client.stop_notify(INSOLE_READ_UUID),
                self.ble_service_timeout_sec,
            )
        except Exception as exc:
            self.emit("log", message=f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 停止通知失败: {exc}")

    async def _safe_stop_measurement(self, role: str, sensor: object) -> None:
        if sensor is None:
            return
        try:
            await self._await_with_timeout(sensor.stop_measurement(), self.ble_service_timeout_sec)
        except Exception as exc:
            self.emit("log", message=f"{ROLE_DISPLAY_NAMES[role]} 停止测量失败: {exc}")

    async def _get_client_services(self, client: object):
        services = None
        get_services = getattr(client, "get_services", None)
        if callable(get_services):
            services = await self._await_with_timeout(
                get_services(),
                self.ble_service_timeout_sec,
            )
        else:
            services = getattr(client, "services", None)
        if services is None:
            raise RuntimeError("服务发现结果为空")
        return services

    def _make_notification_handler(self, role: str, sensor):
        parser = sensor.data_collector.parser

        def handler(sender, data: bytearray):
            del sender
            try:
                parsed = parser.parse(data)
                sample = build_sample_from_live_packet(role, self.role_to_mac[role], parsed)
                if sample is None:
                    return
                self.runtime.process_sample(sample)
            except Exception as exc:
                self.emit("log", message=f"{ROLE_DISPLAY_NAMES[role]} 通知处理失败: {exc}")

        return handler

    def _make_insole_notification_handler(self, role: str):
        parser = self.insole_parsers[role]

        def handler(sender, data: bytearray):
            del sender
            try:
                parsed = parser.parse(data)
                if parsed is None:
                    return
                sample = build_insole_sample(role, self.insole_role_to_mac[role], parsed)
                self.recorder.record_insole_sample(sample)
            except Exception as exc:
                self.emit("log", message=f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 通知处理失败: {exc}")

        return handler

    def _normalize_scan_items(self, discovered) -> list[tuple[object, object]]:
        if discovered is None:
            return []
        if isinstance(discovered, dict):
            items: list[tuple[object, object]] = []
            for value in discovered.values():
                if isinstance(value, tuple):
                    if len(value) >= 2:
                        items.append((value[0], value[1]))
                    elif len(value) == 1:
                        items.append((value[0], None))
                else:
                    items.append((value, None))
            return items
        try:
            return [(device, None) for device in discovered]
        except TypeError:
            return []

    async def _discover_with_adv_fallback(
        self,
        BleakScanner,
        timeout_sec: float,
    ) -> list[tuple[object, object]]:
        # Preferred path on newer bleak versions.
        try:
            discovered = await self._await_with_timeout(
                BleakScanner.discover(timeout=timeout_sec, return_adv=True),
                timeout_sec + 2.0,
            )
            items = self._normalize_scan_items(discovered)
            if items:
                return items
        except TypeError:
            pass
        except Exception as exc:
            self.emit("log", message=f"return_adv 扫描失败，将回退: {exc}")

        # Fallback path for older bleak APIs: plain discover.
        try:
            discovered = await self._await_with_timeout(
                BleakScanner.discover(timeout=timeout_sec),
                timeout_sec + 2.0,
            )
            items = self._normalize_scan_items(discovered)
            if items:
                return items
        except Exception as exc:
            self.emit("log", message=f"discover 扫描失败，将回退: {exc}")

        # Last fallback: callback-based scan to capture advertisement fields.
        captured: dict[str, tuple[object, object]] = {}

        def _on_detect(*args):
            if not args:
                return
            device = args[0]
            adv = args[1] if len(args) > 1 else None
            address = canonical_mac(getattr(device, "address", "") or "")
            if not address:
                return
            captured[address] = (device, adv)

        scanner = None
        scanner_constructors = [
            {"detection_callback": _on_detect, "scanning_mode": "active"},
            {"detection_callback": _on_detect},
            {"scanning_mode": "active"},
            {},
        ]
        for kwargs in scanner_constructors:
            try:
                scanner = BleakScanner(**kwargs)
                break
            except TypeError:
                continue
            except Exception:
                continue
        if scanner is None:
            try:
                scanner = BleakScanner(_on_detect)
            except Exception as exc:
                self.emit("log", message=f"创建扫描器失败: {exc}")
                return []

        register_callback = getattr(scanner, "register_detection_callback", None)
        if callable(register_callback):
            try:
                register_callback(_on_detect)
            except Exception:
                pass

        try:
            await self._await_with_timeout(scanner.start(), self.ble_service_timeout_sec)
            await asyncio.sleep(max(1.0, timeout_sec))
        finally:
            try:
                await self._await_with_timeout(scanner.stop(), self.ble_service_timeout_sec)
            except Exception:
                pass

        if captured:
            return list(captured.values())

        discover_adv = getattr(scanner, "discovered_devices_and_advertisement_data", None)
        if discover_adv is not None:
            items = self._normalize_scan_items(discover_adv)
            if items:
                return items

        discover_devices = getattr(scanner, "discovered_devices", None)
        if discover_devices is not None:
            items = self._normalize_scan_items(discover_devices)
            if items:
                return items

        return []

    async def _scan_all_devices_locked(
        self,
        timeout_sec: float = 5.0,
    ) -> tuple[list[IMUScanResult], list[InsoleScanResult]]:
        _, BleakScanner, _, _, _, _, _ = import_live_dependencies()
        self.emit("status", status=f"设备搜索中 ({timeout_sec:.0f}s)...")
        items = await self._discover_with_adv_fallback(BleakScanner, timeout_sec=timeout_sec)

        imu_results: list[IMUScanResult] = []
        insole_results: list[InsoleScanResult] = []

        raw_seen = 0
        for item in items:
            if isinstance(item, tuple):
                device, adv = item
            else:
                device, adv = item, None

            address = canonical_mac(getattr(device, "address", "") or "")
            if not address:
                continue
            raw_seen += 1
            adv_local_name = ""
            if adv is not None:
                adv_local_name = str(getattr(adv, "local_name", "") or "").strip()
            device_name = str(getattr(device, "name", "") or "").strip()
            candidate_name = adv_local_name or device_name
            imu_display_name = candidate_name or "Movella DOT"
            insole_display_name = candidate_name or "Insole_V2"
            rssi = int(getattr(device, "rssi", -127) or -127)
            service_uuids = []
            manufacturer_ids: set[int] = set()
            if adv is not None:
                service_uuids = list(getattr(adv, "service_uuids", []) or [])
                service_data = getattr(adv, "service_data", {}) or {}
                service_uuids.extend(str(uuid_text) for uuid_text in service_data.keys())
                manufacturer_data = getattr(adv, "manufacturer_data", {}) or {}
                for key in manufacturer_data.keys():
                    try:
                        manufacturer_ids.add(int(key))
                    except (TypeError, ValueError):
                        continue
            metadata = getattr(device, "metadata", {}) or {}
            if not service_uuids:
                service_uuids.extend(str(uuid_text) for uuid_text in (metadata.get("uuids", []) or []))
            metadata_service_data = metadata.get("service_data", {}) or {}
            service_uuids.extend(str(uuid_text) for uuid_text in metadata_service_data.keys())
            metadata_manufacturer_data = metadata.get("manufacturer_data", {}) or {}
            for key in metadata_manufacturer_data.keys():
                try:
                    manufacturer_ids.add(int(key))
                except (TypeError, ValueError):
                    continue
            if is_imu_scan_candidate(
                candidate_name,
                address,
                service_uuids,
                manufacturer_ids,
                configured_role_to_mac=self.role_to_mac,
            ):
                imu_results.append(
                    IMUScanResult(
                        address=address,
                        display_name=imu_display_name,
                        rssi=rssi,
                        inferred_role=infer_imu_role(
                            address,
                            configured_role_to_mac=self.role_to_mac,
                            name=imu_display_name,
                        ),
                        ble_device=device,
                    )
                )
            if is_insole_scan_candidate(
                candidate_name,
                address,
                service_uuids,
                configured_role_to_mac=self.insole_role_to_mac,
            ):
                insole_results.append(
                    InsoleScanResult(
                        address=address,
                        display_name=insole_display_name,
                        rssi=rssi,
                        inferred_role=infer_insole_role(address, insole_display_name),
                        ble_device=device,
                    )
                )

        imu_deduped = {item.address: item for item in imu_results}
        imu_ordered = sorted(
            imu_deduped.values(),
            key=lambda item: (
                item.inferred_role or "zzzz",
                -item.rssi,
                item.display_name,
                item.address,
            ),
        )
        insole_deduped = {item.address: item for item in insole_results}
        insole_ordered = sorted(
            insole_deduped.values(),
            key=lambda item: (
                item.inferred_role or "zzzz",
                -item.rssi,
                item.display_name,
                item.address,
            ),
        )
        self.emit(
            "log",
            message=(
                f"设备扫描统计: 原始设备 {raw_seen}，"
                f"IMU 候选 {len(imu_ordered)}，鞋垫候选 {len(insole_ordered)}"
            ),
        )
        return imu_ordered, insole_ordered

    async def _scan_imus_locked(self, timeout_sec: float = 5.0) -> list[IMUScanResult]:
        imu_results, _ = await self._scan_all_devices_locked(timeout_sec=timeout_sec)
        self.emit("status", status=f"IMU 搜索完成，共发现 {len(imu_results)} 个设备")
        return imu_results

    async def scan_imus(self, timeout_sec: float = 5.0) -> list[IMUScanResult]:
        async with self._require_ble_lock():
            return await self._scan_imus_locked(timeout_sec=timeout_sec)

    async def _scan_insoles_locked(self, timeout_sec: float = 5.0) -> list[InsoleScanResult]:
        _, insole_results = await self._scan_all_devices_locked(timeout_sec=timeout_sec)
        self.emit("status", status=f"鞋垫搜索完成，共发现 {len(insole_results)} 个设备")
        return insole_results

    async def scan_insoles(self, timeout_sec: float = 5.0) -> list[InsoleScanResult]:
        async with self._require_ble_lock():
            return await self._scan_insoles_locked(timeout_sec=timeout_sec)

    async def scan_all_devices(
        self,
        timeout_sec: float = 5.0,
    ) -> tuple[list[IMUScanResult], list[InsoleScanResult]]:
        async with self._require_ble_lock():
            return await self._scan_all_devices_locked(timeout_sec=timeout_sec)

    async def _connect_single_imu_locked(
        self,
        role: str,
        config,
        BleakClient,
        MovellaDOTSensor,
        address: Optional[str] = None,
        display_name: Optional[str] = None,
        ble_device: object = None,
    ):
        mac = canonical_mac(address or self.role_to_mac[role])
        display = display_name or ROLE_DISPLAY_NAMES[role]
        last_exc = None

        for attempt in range(1, self.ble_connect_retries + 1):
            sensor = MovellaDOTSensor(config)
            client_target = ble_device if attempt == 1 and ble_device is not None else mac
            sensor.client = BleakClient(client_target)
            sensor._device_address = mac
            sensor._device_name = display
            sensor._device_tag = ROLE_DISPLAY_NAMES[role]

            self.emit_imu_state(role, "状态: 连接中", address=mac)
            self.emit(
                "log",
                message=(
                    f"开始连接 {ROLE_DISPLAY_NAMES[role]} IMU: {display} ({mac}) "
                    f"(尝试 {attempt}/{self.ble_connect_retries})"
                ),
            )
            try:
                await self._await_with_timeout(
                    sensor.client.connect(),
                    self.ble_connect_timeout_sec,
                )
                sensor.is_connected = True
                await self._settle_delay()

                services = await self._get_client_services(sensor.client)
                if not has_gatt_service_uuid(services, MOVELLA_CONFIG_SERVICE_UUID):
                    raise RuntimeError(f"未发现 IMU 配置服务 {MOVELLA_CONFIG_SERVICE_UUID}")
                if not has_gatt_service_uuid(services, MOVELLA_MEASUREMENT_SERVICE_UUID):
                    raise RuntimeError(f"未发现 IMU 测量服务 {MOVELLA_MEASUREMENT_SERVICE_UUID}")

                info = None
                try:
                    info = await self._await_with_timeout(
                        sensor.get_device_info(),
                        self.ble_service_timeout_sec,
                    )
                except Exception as exc:
                    self.emit(
                        "log",
                        message=(
                            f"{display} 读取设备信息失败，将继续使用目标 MAC。原因: {exc}"
                        ),
                    )

                if info is not None:
                    actual_mac = canonical_mac(info.mac_address)
                    if actual_mac != mac:
                        raise RuntimeError(
                            f"{ROLE_DISPLAY_NAMES[role]} MAC 校验失败，期望 {mac}，实际读到 {actual_mac}"
                        )
                    tag = info.device_tag or display or ROLE_DISPLAY_NAMES[role]
                    sensor._device_tag = tag
                    self.emit(
                        "log",
                        message=(
                            f"{ROLE_DISPLAY_NAMES[role]} 已连接: mac={actual_mac}, tag={tag}, "
                            f"firmware={info.firmware_version}, output_rate={info.output_rate}Hz"
                        ),
                    )

                await self._await_with_timeout(
                    sensor.configure_sensor(),
                    self.ble_service_timeout_sec,
                )
                sensor.notification_handler = self._make_notification_handler(role, sensor)
                self.role_to_mac[role] = mac
                self.sensors[role] = sensor
                self.emit_imu_state(role, "状态: 已连接", address=mac)
                await self._settle_delay()
                return sensor
            except Exception as exc:
                last_exc = exc
                self._emit_retry_log(f"{ROLE_DISPLAY_NAMES[role]} IMU 连接", attempt, exc)
                await self._safe_disconnect_sensor(sensor)
                if attempt < self.ble_connect_retries:
                    self.emit_imu_state(
                        role,
                        f"状态: 重试中 ({attempt + 1}/{self.ble_connect_retries})",
                        address=mac,
                    )
                    await self._retry_delay()

        self.emit_imu_state(role, "状态: 未连接")
        raise RuntimeError(
            f"{ROLE_DISPLAY_NAMES[role]} IMU 连续 {self.ble_connect_retries} 次连接失败: {last_exc}"
        ) from last_exc

    async def _disconnect_imu_locked(self, role: str, silent: bool = False) -> bool:
        sensor = self.sensors.pop(role, None)
        if sensor is None:
            if not silent:
                self.emit_imu_state(role, "状态: 未连接")
            return True

        if self.is_streaming:
            await self._safe_stop_measurement(role, sensor)

        await self._safe_disconnect_sensor(sensor)
        if not silent:
            self.emit_imu_state(role, "状态: 已断开")
            self.emit("log", message=f"{ROLE_DISPLAY_NAMES[role]} IMU 已断开")
        return True

    async def connect_imu(
        self,
        role: str,
        address: str,
        display_name: Optional[str] = None,
        ble_device: object = None,
    ) -> bool:
        if role not in ROLE_ORDER:
            raise ValueError(f"未知 IMU 角色: {role}")
        async with self._require_ble_lock():
            (
                BleakClient,
                _BleakScanner,
                MovellaDOTSensor,
                SensorConfiguration,
                FilterProfile,
                OutputRate,
                PayloadMode,
            ) = import_live_dependencies()

            output_rate = resolve_output_rate_enum(OutputRate, self.sample_rate_hz)
            config = SensorConfiguration(
                output_rate=output_rate,
                filter_profile=getattr(FilterProfile, self.filter_profile_name),
                payload_mode=getattr(PayloadMode, self.payload_mode_name),
            )

            existing_sensor = self.sensors.get(role)
            mac = canonical_mac(address)
            if existing_sensor is not None and self._is_sensor_connected(existing_sensor):
                if self.role_to_mac.get(role) == mac:
                    return True
                await self._disconnect_imu_locked(role, silent=True)

            await self._connect_single_imu_locked(
                role=role,
                config=config,
                BleakClient=BleakClient,
                MovellaDOTSensor=MovellaDOTSensor,
                address=mac,
                display_name=display_name,
                ble_device=ble_device,
            )
            if all(self._is_sensor_connected(self.sensors.get(item)) for item in ROLE_ORDER):
                self.emit("status", status="左右脚 IMU 已全部连接")
            return True

    async def disconnect_imu(self, role: str, silent: bool = False) -> bool:
        if role not in ROLE_ORDER:
            raise ValueError(f"未知 IMU 角色: {role}")
        async with self._require_ble_lock():
            return await self._disconnect_imu_locked(role=role, silent=silent)

    async def _connect_imus_locked(self) -> bool:
        (
            BleakClient,
            _BleakScanner,
            MovellaDOTSensor,
            SensorConfiguration,
            FilterProfile,
            OutputRate,
            PayloadMode,
        ) = import_live_dependencies()

        output_rate = resolve_output_rate_enum(OutputRate, self.sample_rate_hz)
        config = SensorConfiguration(
            output_rate=output_rate,
            filter_profile=getattr(FilterProfile, self.filter_profile_name),
            payload_mode=getattr(PayloadMode, self.payload_mode_name),
        )

        connected_imus: dict[str, object] = {}
        try:
            for role in ROLE_ORDER:
                existing_sensor = self.sensors.get(role)
                if self._is_sensor_connected(existing_sensor):
                    continue
                if existing_sensor is not None:
                    await self._disconnect_imu_locked(role, silent=True)
                sensor = await self._connect_single_imu_locked(
                    role=role,
                    config=config,
                    BleakClient=BleakClient,
                    MovellaDOTSensor=MovellaDOTSensor,
                )
                connected_imus[role] = sensor
            self.emit("status", status="左右脚 IMU 已全部连接")
            return True
        except Exception:
            for role in reversed(tuple(connected_imus.keys())):
                await self._disconnect_imu_locked(role, silent=True)
            raise

    async def connect_imus(self) -> bool:
        async with self._require_ble_lock():
            return await self._connect_imus_locked()

    async def _connect_insole_locked(
        self,
        role: str,
        address: str,
        display_name: Optional[str] = None,
        ble_device: object = None,
    ) -> bool:
        BleakClient, _, _, _, _, _, _ = import_live_dependencies()
        if role not in INSOLE_ROLE_ORDER:
            raise ValueError(f"未知鞋垫角色: {role}")

        mac = canonical_mac(address)
        label = display_name or INSOLE_ROLE_DISPLAY_NAMES[role]
        current_client = self.insole_clients.get(role)
        current_mac = self.insole_role_to_mac.get(role)
        if current_client is not None and current_mac == mac and self._is_client_connected(current_client):
            return True

        if current_client is not None:
            await self._disconnect_insole_locked(role, silent=True)

        last_exc = None
        for attempt in range(1, self.ble_connect_retries + 1):
            client_target = ble_device if attempt == 1 and ble_device is not None else mac
            client = BleakClient(client_target)
            self.emit_insole_state(role, "状态: 连接中", address=mac)
            self.emit(
                "log",
                message=(
                    f"开始连接 {INSOLE_ROLE_DISPLAY_NAMES[role]}: {label} ({mac}) "
                    f"(尝试 {attempt}/{self.ble_connect_retries})"
                ),
            )
            try:
                await self._await_with_timeout(
                    client.connect(),
                    self.ble_connect_timeout_sec,
                )
                await self._settle_delay()

                services = None
                get_services = getattr(client, "get_services", None)
                if callable(get_services):
                    services = await self._await_with_timeout(
                        get_services(),
                        self.ble_service_timeout_sec,
                    )
                else:
                    services = getattr(client, "services", None)
                if services is None:
                    raise RuntimeError("服务发现结果为空")
                if not has_gatt_service_uuid(services, INSOLE_SERVICE_UUID):
                    raise RuntimeError(f"未发现鞋垫服务 {INSOLE_SERVICE_UUID}")
                if not has_gatt_characteristic_uuid(services, INSOLE_READ_UUID):
                    raise RuntimeError(f"未发现鞋垫通知特征 {INSOLE_READ_UUID}")

                self.insole_clients[role] = client
                self.insole_parsers[role] = InsolePacketParser()
                self.insole_role_to_mac[role] = mac
                self.emit_insole_state(role, "状态: 已连接", address=mac)
                self.emit(
                    "log",
                    message=f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 已连接: {label} ({mac})",
                )
                if self.is_streaming:
                    await self._await_with_timeout(
                        client.start_notify(
                            INSOLE_READ_UUID,
                            self._make_insole_notification_handler(role),
                        ),
                        self.ble_service_timeout_sec,
                    )
                    self.insole_streaming_roles.add(role)
                    self.emit_insole_state(role, "状态: 已开始接收", address=mac)
                await self._settle_delay()
                return True
            except Exception as exc:
                last_exc = exc
                self.insole_clients.pop(role, None)
                self.insole_parsers.pop(role, None)
                self._emit_retry_log(f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 连接", attempt, exc)
                await self._safe_disconnect_client(client)
                if attempt < self.ble_connect_retries:
                    self.emit_insole_state(
                        role,
                        f"状态: 重试中 ({attempt + 1}/{self.ble_connect_retries})",
                        address=mac,
                    )
                    await self._retry_delay()

        self.emit_insole_state(role, "状态: 未连接")
        raise RuntimeError(
            f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 连续 {self.ble_connect_retries} 次连接失败: {last_exc}"
        ) from last_exc

    async def connect_insole(
        self,
        role: str,
        address: str,
        display_name: Optional[str] = None,
        ble_device: object = None,
    ) -> bool:
        async with self._require_ble_lock():
            return await self._connect_insole_locked(
                role=role,
                address=address,
                display_name=display_name,
                ble_device=ble_device,
            )

    async def _disconnect_insole_locked(self, role: str, silent: bool = False) -> bool:
        client = self.insole_clients.pop(role, None)
        self.insole_parsers.pop(role, None)
        if client is None:
            if not silent:
                self.emit_insole_state(role, initial_insole_status_text(role, self.insole_role_to_mac))
            return True

        if role in self.insole_streaming_roles:
            await self._safe_stop_insole_notify(role, client)
            self.insole_streaming_roles.discard(role)

        await self._safe_disconnect_client(client)
        if not silent:
            self.emit_insole_state(role, "状态: 已断开")
            self.emit("log", message=f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 已断开")
        return True

    async def disconnect_insole(self, role: str, silent: bool = False) -> bool:
        async with self._require_ble_lock():
            return await self._disconnect_insole_locked(role=role, silent=silent)

    async def _connect_configured_insoles_locked(self) -> bool:
        for role in INSOLE_ROLE_ORDER:
            mac = self.insole_role_to_mac.get(role)
            if not mac:
                self.emit_insole_state(role, "状态: 未配置")
                continue
            await self._connect_insole_locked(
                role,
                mac,
                display_name=INSOLE_ROLE_DISPLAY_NAMES[role],
            )
        return True

    async def connect_configured_insoles(self) -> bool:
        async with self._require_ble_lock():
            return await self._connect_configured_insoles_locked()

    async def connect_all(self) -> bool:
        async with self._require_ble_lock():
            await self._connect_imus_locked()
            await self._connect_configured_insoles_locked()
            return True

    async def _start_all_locked(self) -> bool:
        if not self.sensors and not self.insole_clients:
            raise RuntimeError("尚未连接任何传感器。")
        if self.is_streaming:
            return True
        missing_imu_roles = [
            role for role in ROLE_ORDER if not self._is_sensor_connected(self.sensors.get(role))
        ]
        if missing_imu_roles:
            labels = "、".join(ROLE_DISPLAY_NAMES[role] for role in missing_imu_roles)
            raise RuntimeError(f"开始预测前请先连接 {labels} IMU。")

        self.recorder.start_session()
        self.runtime.reset()
        started_imu_roles: list[str] = []
        try:
            for role in ROLE_ORDER:
                sensor = self.sensors[role]
                if sensor.data_collector:
                    sensor.data_collector.clear()
                await self._await_with_timeout(
                    sensor.start_measurement(),
                    self.ble_service_timeout_sec,
                )
                started_imu_roles.append(role)
                self.emit_imu_state(role, "状态: 已开始接收", address=self.role_to_mac.get(role))
                await self._settle_delay()

            self.insole_streaming_roles.clear()
            for role in INSOLE_ROLE_ORDER:
                client = self.insole_clients.get(role)
                if client is None:
                    continue
                parser = self.insole_parsers.get(role)
                if parser is not None:
                    parser.clear()
                await self._await_with_timeout(
                    client.start_notify(
                        INSOLE_READ_UUID,
                        self._make_insole_notification_handler(role),
                    ),
                    self.ble_service_timeout_sec,
                )
                self.insole_streaming_roles.add(role)
                self.emit_insole_state(
                    role,
                    "状态: 已开始接收",
                    address=self.insole_role_to_mac.get(role),
                )
                await self._settle_delay()

            self.is_streaming = True
            self.emit(
                "status",
                status=(
                    f"已开始实时推理与保存 | 输入={COMBINED_INPUT_SOURCE} | 通道数={N_CHANNELS} "
                    f"| 特征维度={INPUT_DIM} | 窗口={WINDOW_N}"
                ),
            )
            return True
        except Exception:
            for role in list(self.insole_streaming_roles):
                client = self.insole_clients.get(role)
                if client is None:
                    continue
                await self._safe_stop_insole_notify(role, client)
                self.emit_insole_state(role, "状态: 已连接", address=self.insole_role_to_mac.get(role))
            self.insole_streaming_roles.clear()
            for role in reversed(started_imu_roles):
                await self._safe_stop_measurement(role, self.sensors[role])
                self.emit_imu_state(role, "状态: 已连接", address=self.role_to_mac.get(role))
            self.recorder.close()
            self.is_streaming = False
            raise

    async def start_all(self) -> bool:
        async with self._require_ble_lock():
            return await self._start_all_locked()

    async def _stop_all_locked(self) -> bool:
        if not self.is_streaming:
            return True

        for role in list(self.insole_streaming_roles):
            client = self.insole_clients.get(role)
            if client is None:
                continue
            await self._safe_stop_insole_notify(role, client)
            self.emit_insole_state(role, "状态: 已连接", address=self.insole_role_to_mac.get(role))
        self.insole_streaming_roles.clear()

        for role in ROLE_ORDER:
            sensor = self.sensors.get(role)
            if sensor is None:
                continue
            await self._safe_stop_measurement(role, sensor)
            self.emit_imu_state(role, "状态: 已连接", address=self.role_to_mac.get(role))
        self.is_streaming = False
        self.recorder.close()
        self.emit("status", status="已停止实时接收并关闭当前保存文件")
        return True

    async def stop_all(self) -> bool:
        async with self._require_ble_lock():
            return await self._stop_all_locked()

    async def _disconnect_all_locked(self) -> bool:
        if self.is_streaming:
            try:
                await self._stop_all_locked()
            except Exception:
                pass

        sensor_roles = list(reversed(tuple(self.sensors.keys())))
        for role in sensor_roles:
            await self._disconnect_imu_locked(role, silent=False)
        for role in INSOLE_ROLE_ORDER:
            await self._disconnect_insole_locked(role, silent=False)
        self.is_streaming = False
        self.recorder.close()
        self.emit("status", status="蓝牙连接已全部断开")
        return True

    async def disconnect_all(self) -> bool:
        async with self._require_ble_lock():
            return await self._disconnect_all_locked()

    def shutdown(self) -> None:
        self.recorder.close()
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)


def validate_role_to_mac(role_to_mac: dict[str, str]) -> dict[str, str]:
    normalized = {role: canonical_mac(mac) for role, mac in role_to_mac.items()}
    unique = set(normalized.values())
    if len(unique) != len(normalized):
        raise ValueError(f"两个 IMU 的 MAC 不能重复: {normalized}")
    return normalized


def validate_optional_role_to_mac(role_to_mac: dict[str, str]) -> dict[str, str]:
    normalized = {
        role: canonical_mac(mac)
        for role, mac in role_to_mac.items()
        if str(mac).strip()
    }
    unique = set(normalized.values())
    if len(unique) != len(normalized):
        raise ValueError(f"两个鞋垫的 MAC 不能重复: {normalized}")
    return normalized


def validate_all_devices_distinct(*role_maps: dict[str, str]) -> None:
    merged: dict[str, str] = {}
    for role_map in role_maps:
        for role, mac in role_map.items():
            if mac in merged.values():
                raise ValueError(f"MAC 地址重复，无法区分设备: {role} -> {mac}")
            merged[role] = mac


def format_optional_mac(mac: Optional[str]) -> str:
    return mac if mac else "未配置"


def initial_imu_status_text() -> str:
    return "状态: 未连接"


def initial_insole_status_text(role: str, insole_role_to_mac: dict[str, str]) -> str:
    return "状态: 未连接" if insole_role_to_mac.get(role) else "状态: 未配置"


def print_event(event: dict) -> None:
    event_type = event["type"]
    if event_type == "session":
        print(f"[保存] {event['path']}", flush=True)
        return
    if event_type == "imu_state":
        print(
            f"[IMU] {ROLE_DISPLAY_NAMES[event['role']]} {event['status']}",
            flush=True,
        )
        return
    if event_type == "insole_state":
        print(
            f"[鞋垫] {INSOLE_ROLE_DISPLAY_NAMES[event['role']]} {event['status']}",
            flush=True,
        )
        return
    if event_type == "log":
        print(f"[LOG] {event['message']}", flush=True)
        return
    if event_type == "status":
        print(f"[状态] {event['status']}", flush=True)
        return
    if event_type == "prediction":
        pred: PhasePrediction = event["prediction"]
        readiness: ReadinessSnapshot = event["readiness"]
        print(
            (
                f"[相位] 输入={pred.input_source} "
                f"phase={pred.phase * 100:6.2f}% "
                f"raw={pred.raw_phase * 100:6.2f}% "
                f"r={pred.stride_rate_hz:5.2f}Hz "
                f"row={pred.total_rows:5d} "
                f"pred={pred.total_predictions:5d} "
                f"| {readiness.summary_text()}"
            ),
            flush=True,
        )


def drain_event_queue(event_queue: Queue, timeout: float = 0.2) -> bool:
    try:
        event = event_queue.get(timeout=timeout)
    except Empty:
        return False
    print_event(event)
    while True:
        try:
            event = event_queue.get_nowait()
        except Empty:
            break
        print_event(event)
    return True


def build_live_runtime_components(args, predictor: RealtimeGaitPhasePredictor, event_queue: Queue):
    def emit_event(event_type: str, **payload) -> None:
        event_queue.put({"type": event_type, **payload})

    recorder = SessionCsvRecorder(data_root=args.data_root, event_emitter=emit_event)
    runtime = MultiSensorRuntime(
        predictor=predictor,
        stale_timeout_sec=args.stale_timeout_sec,
        event_queue=event_queue,
        recorder=recorder,
    )
    controller = MultiDotLiveController(
        role_to_mac=args.role_to_mac,
        insole_role_to_mac=args.insole_role_to_mac,
        runtime=runtime,
        sample_rate_hz=args.sample_rate_hz,
        filter_profile_name=args.filter_profile,
        payload_mode_name=args.payload_mode,
        event_queue=event_queue,
        recorder=recorder,
        ble_connect_timeout_sec=args.ble_connect_timeout_sec,
        ble_service_timeout_sec=args.ble_service_timeout_sec,
        ble_disconnect_timeout_sec=args.ble_disconnect_timeout_sec,
        ble_connect_retries=args.ble_connect_retries,
        ble_retry_delay_sec=args.ble_retry_delay_sec,
        ble_settle_sec=args.ble_settle_sec,
    )
    return runtime, recorder, controller


def run_replay_headless(args, predictor: RealtimeGaitPhasePredictor, event_queue: Queue) -> int:
    runtime = MultiSensorRuntime(
        predictor=predictor,
        stale_timeout_sec=args.stale_timeout_sec,
        event_queue=event_queue,
    )
    runtime.reset()

    replay_paths = {
        "left_foot": Path(args.replay_left_foot),
        "right_foot": Path(args.replay_right_foot),
    }
    all_samples = {
        role: load_replay_samples(path, role=role, fallback_mac=args.role_to_mac[role])
        for role, path in replay_paths.items()
    }
    min_len = min(len(samples) for samples in all_samples.values())
    if min_len <= 0:
        raise RuntimeError("回放数据为空。")

    print(
        f"开始 CSV 回放，总步数 {min_len} | 输入={COMBINED_INPUT_SOURCE} | 特征维度={INPUT_DIM}",
        flush=True,
    )

    replay_order = ("right_foot", "left_foot")
    last_sleep_time = None
    for idx in range(min_len):
        for role in replay_order:
            sample = all_samples[role][idx]
            if args.replay_speed > 0 and sample.timestamp_us is not None and last_sleep_time is not None:
                dt_sec = max(0.0, (sample.timestamp_us - last_sleep_time) / 1_000_000.0)
                time.sleep(dt_sec / args.replay_speed)
            if sample.timestamp_us is not None:
                last_sleep_time = sample.timestamp_us
            runtime.process_sample(sample)
            drain_event_queue(event_queue, timeout=0.0)

    print("CSV 回放结束。", flush=True)
    return 0


def run_live_headless(args, predictor: RealtimeGaitPhasePredictor, event_queue: Queue) -> int:
    _, _, controller = build_live_runtime_components(args, predictor=predictor, event_queue=event_queue)

    try:
        controller.run_coro(controller.connect_all()).result()
        while drain_event_queue(event_queue, timeout=0.0):
            pass

        controller.run_coro(controller.start_all()).result()
        while drain_event_queue(event_queue, timeout=0.0):
            pass

        print("开始实时输出步态相位，按 Ctrl+C 结束。", flush=True)
        while True:
            drain_event_queue(event_queue, timeout=0.5)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，准备断开 IMU 与鞋垫。", flush=True)
        return 0
    finally:
        try:
            disconnect_timeout = max(
                8.0,
                (
                    len(ROLE_ORDER) + len(INSOLE_ROLE_ORDER)
                ) * args.ble_disconnect_timeout_sec + 4.0,
            )
            controller.run_coro(controller.disconnect_all()).result(timeout=disconnect_timeout)
        except Exception:
            pass
        while drain_event_queue(event_queue, timeout=0.0):
            pass
        controller.shutdown()


class LivePhaseUI:
    def __init__(self, args, predictor: RealtimeGaitPhasePredictor):
        self.args = args
        self.predictor = predictor
        self.event_queue: Queue = Queue()
        _, self.recorder, self.controller = build_live_runtime_components(
            self.args,
            predictor=self.predictor,
            event_queue=self.event_queue,
        )

        self.root = tk.Tk()
        self.tk_rendering_info = configure_tk_rendering(self.root, ui_scale=self.args.ui_scale)
        self.root.title("双 IMU + 双鞋垫 实时步态相位")
        self.root.geometry("1320x860")
        self.root.minsize(1120, 760)
        self.plot_canvas: Optional[tk.Canvas] = None
        self.phase_window: Optional[tk.Toplevel] = None

        self.latest_samples: dict[str, SensorSample] = {}
        self.phase_history: deque[tuple[float, float]] = deque(maxlen=PLOT_HISTORY_LENGTH)
        self.imu_scan_results: list[IMUScanResult] = []
        self.imu_controls_enabled = True
        self.insole_scan_results: list[InsoleScanResult] = []
        self.insole_controls_enabled = True
        self.connected_imu_address_by_role = {
            role: None for role in ROLE_ORDER
        }
        self.connected_insole_address_by_role = {
            role: None for role in INSOLE_ROLE_ORDER
        }
        self.streaming_active = False

        self.status_var = tk.StringVar(value="未连接")
        self.readiness_var = tk.StringVar(value="设备同步: 左脚=未收到 | 右脚=未收到")
        self.imu_scan_status_var = tk.StringVar(value="尚未搜索 IMU")
        self.insole_scan_status_var = tk.StringVar(value="尚未搜索鞋垫")
        self.ui_scale_var = tk.StringVar(
            value=(
                f"界面缩放: {self.tk_rendering_info.applied_scaling:.3f} "
                f"| 系统 DPI: {self.tk_rendering_info.system_dpi:.1f} "
                f"| 来源: {self.tk_rendering_info.source}"
            )
        )
        self.window_var = tk.StringVar(
            value=(
                f"窗口 {WINDOW_N} 点 | 采样率 {self.args.sample_rate_hz:.0f}Hz | "
                f"通道 {N_CHANNELS} | 输入维度 {INPUT_DIM}"
            )
        )
        self.model_var = tk.StringVar(
            value=f"模型: {self.args.model.name} | scaler: {self.args.scaler.name}"
        )
        self.channel_order_var = tk.StringVar(value=" -> ".join(IMU_CHANNELS))
        self.phase_var = tk.StringVar(value="等待数据")
        self.raw_phase_var = tk.StringVar(value="原始相位: -")
        self.stride_rate_var = tk.StringVar(value="步频 r: -")
        self.cos_sin_var = tk.StringVar(value="cos/sin: -")
        self.counter_var = tk.StringVar(value="窗口状态: -")
        self.timestamp_var = tk.StringVar(value="时间戳: -")

        self.role_state_vars = {
            role: tk.StringVar(value=initial_imu_status_text()) for role in ROLE_ORDER
        }
        self.role_address_vars = {
            role: tk.StringVar(value="设备: 未连接") for role in ROLE_ORDER
        }
        self.role_timestamp_vars = {
            role: tk.StringVar(value="时间戳: -") for role in ROLE_ORDER
        }
        self.role_value_vars = {
            role: {field: tk.StringVar(value="-") for _, field in SENSOR_VALUE_FIELDS}
            for role in ROLE_ORDER
        }
        self.insole_state_vars = {
            role: tk.StringVar(value=initial_insole_status_text(role, self.args.insole_role_to_mac))
            for role in INSOLE_ROLE_ORDER
        }

        self._build_ui()
        self._build_phase_window()
        self._refresh_action_buttons()
        self._append_log(
            "界面缩放: "
            f"{self.tk_rendering_info.applied_scaling:.3f} | "
            f"系统 DPI={self.tk_rendering_info.system_dpi:.1f} | "
            f"来源={self.tk_rendering_info.source}"
        )
        self._append_log(f"输入通道顺序: {', '.join(IMU_CHANNELS)}")
        self._append_log(f"模型: {self.args.model} | scaler: {self.args.scaler}")
        self._poll_events()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.pack(fill=tk.BOTH, expand=True)

        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill=tk.X, pady=(0, 12))

        info_frame = ttk.LabelFrame(top_frame, text="连接信息", padding=10)
        info_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        ttk.Label(info_frame, textvariable=self.window_var, wraplength=700).pack(anchor=tk.W)
        ttk.Label(info_frame, textvariable=self.model_var, wraplength=700).pack(
            anchor=tk.W,
            pady=(6, 0),
        )
        ttk.Label(info_frame, textvariable=self.readiness_var, wraplength=700).pack(
            anchor=tk.W,
            pady=(6, 0),
        )
        ttk.Label(info_frame, text=f"{ROLE_DISPLAY_NAMES['left_foot']} IMU").pack(
            anchor=tk.W,
            pady=(6, 0),
        )
        ttk.Label(info_frame, textvariable=self.role_address_vars["left_foot"]).pack(
            anchor=tk.W,
            pady=(2, 0),
        )
        ttk.Label(info_frame, textvariable=self.role_state_vars["left_foot"], wraplength=700).pack(
            anchor=tk.W,
            pady=(2, 0),
        )
        ttk.Label(info_frame, text=f"{ROLE_DISPLAY_NAMES['right_foot']} IMU").pack(
            anchor=tk.W,
            pady=(6, 0),
        )
        ttk.Label(info_frame, textvariable=self.role_address_vars["right_foot"]).pack(
            anchor=tk.W,
            pady=(2, 0),
        )
        ttk.Label(info_frame, textvariable=self.role_state_vars["right_foot"], wraplength=700).pack(
            anchor=tk.W,
            pady=(2, 0),
        )
        ttk.Label(info_frame, textvariable=self.insole_state_vars["left_insole"]).pack(
            anchor=tk.W,
            pady=(6, 0),
        )
        ttk.Label(info_frame, textvariable=self.insole_state_vars["right_insole"]).pack(
            anchor=tk.W,
            pady=(4, 0),
        )
        ttk.Label(info_frame, textvariable=self.ui_scale_var, wraplength=700).pack(
            anchor=tk.W,
            pady=(6, 0),
        )

        control_frame = ttk.LabelFrame(top_frame, text="控制", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(control_frame, text="状态:").pack(anchor=tk.W)
        ttk.Label(control_frame, textvariable=self.status_var, wraplength=240).pack(
            anchor=tk.W,
            pady=(0, 8),
        )

        self.start_btn = ttk.Button(
            control_frame,
            text="开始预测",
            command=self._on_start,
            state=tk.DISABLED,
        )
        self.stop_btn = ttk.Button(
            control_frame,
            text="停止",
            command=self._on_stop,
            state=tk.DISABLED,
        )
        self.disconnect_btn = ttk.Button(
            control_frame,
            text="断开",
            command=self._on_disconnect,
            state=tk.DISABLED,
        )
        for button in (
            self.start_btn,
            self.stop_btn,
            self.disconnect_btn,
        ):
            button.pack(fill=tk.X, pady=4)
        self.show_phase_window_btn = ttk.Button(
            control_frame,
            text="显示相位窗口",
            command=self._show_phase_window,
        )
        self.show_phase_window_btn.pack(fill=tk.X, pady=(8, 4))

        self._build_device_search_panel(main_frame)

        middle_frame = ttk.Frame(main_frame)
        middle_frame.pack(fill=tk.X, pady=(0, 12))

        prediction_frame = ttk.LabelFrame(middle_frame, text="模型输出", padding=12)
        prediction_frame.pack(fill=tk.BOTH, expand=True)
        tk.Label(
            prediction_frame,
            textvariable=self.phase_var,
            font=("TkDefaultFont", 30, "bold"),
            fg=PLOT_COLORS["phase"],
        ).pack(anchor=tk.CENTER, pady=(8, 12))
        ttk.Label(prediction_frame, textvariable=self.raw_phase_var).pack(anchor=tk.W)
        ttk.Label(prediction_frame, textvariable=self.stride_rate_var).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(prediction_frame, textvariable=self.cos_sin_var).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(
            prediction_frame,
            textvariable=self.counter_var,
            wraplength=340,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(
            prediction_frame,
            textvariable=self.timestamp_var,
            wraplength=340,
            justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(6, 0))

        log_frame = ttk.LabelFrame(main_frame, text="运行日志", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = ScrolledText(log_frame, height=12, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _build_phase_window(self) -> None:
        self.phase_window = tk.Toplevel(self.root)
        self.phase_window.title("实时相位曲线")
        self.phase_window.geometry("980x420")
        self.phase_window.minsize(760, 320)

        frame = ttk.Frame(self.phase_window, padding=8)
        frame.pack(fill=tk.BOTH, expand=True)
        self.plot_canvas = tk.Canvas(
            frame,
            height=320,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#d0d7de",
        )
        self.plot_canvas.pack(fill=tk.BOTH, expand=True)
        self.plot_canvas.bind("<Configure>", lambda _event: self._redraw_phase_plot())
        self.phase_window.protocol("WM_DELETE_WINDOW", self._on_phase_window_close)
        self._redraw_phase_plot()

    def _show_phase_window(self) -> None:
        if self.phase_window is None or not self.phase_window.winfo_exists():
            self._build_phase_window()
            return
        self.phase_window.deiconify()
        self.phase_window.lift()

    def _on_phase_window_close(self) -> None:
        if self.phase_window is None or not self.phase_window.winfo_exists():
            return
        self.phase_window.withdraw()

    def _build_device_search_panel(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="设备搜索", padding=10)
        frame.pack(fill=tk.X, pady=(0, 12))

        header = ttk.Frame(frame)
        header.pack(fill=tk.X)
        status_col = ttk.Frame(header)
        status_col.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Label(status_col, textvariable=self.imu_scan_status_var).pack(anchor=tk.W)
        ttk.Label(status_col, textvariable=self.insole_scan_status_var).pack(anchor=tk.W, pady=(2, 0))
        self.device_scan_btn = ttk.Button(
            header,
            text="搜索 IMU + 鞋垫",
            command=self._on_scan_all_devices,
        )
        self.device_scan_btn.pack(side=tk.RIGHT, padx=(8, 0))

        imu_section = ttk.LabelFrame(frame, text="IMU 结果", padding=8)
        imu_section.pack(fill=tk.X, pady=(8, 6))
        self.imu_scan_results_frame = ttk.Frame(imu_section)
        self.imu_scan_results_frame.pack(fill=tk.X)
        self._render_imu_scan_results()

        insole_section = ttk.LabelFrame(frame, text="鞋垫结果", padding=8)
        insole_section.pack(fill=tk.X, pady=(0, 0))
        self.insole_scan_results_frame = ttk.Frame(insole_section)
        self.insole_scan_results_frame.pack(fill=tk.X)
        self._render_insole_scan_results()

    def _build_sensor_panel(self, parent, role: str):
        frame = ttk.LabelFrame(parent, text=f"{ROLE_DISPLAY_NAMES[role]} IMU", padding=10)
        ttk.Label(frame, textvariable=self.role_address_vars[role]).grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="w",
        )
        ttk.Label(frame, textvariable=self.role_state_vars[role], wraplength=280).grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(4, 0),
        )
        ttk.Label(frame, textvariable=self.role_timestamp_vars[role], wraplength=280).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(4, 8),
        )
        for idx, (label, field) in enumerate(SENSOR_VALUE_FIELDS, start=3):
            ttk.Label(frame, text=f"{label}:").grid(
                row=idx,
                column=0,
                sticky="w",
                padx=(0, 8),
                pady=2,
            )
            ttk.Label(frame, textvariable=self.role_value_vars[role][field]).grid(
                row=idx,
                column=1,
                sticky="w",
                pady=2,
            )
        return frame

    def _append_log(self, message: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_buttons(self, connect: bool, start: bool, stop: bool, disconnect: bool) -> None:
        del connect
        self.start_btn.configure(state=tk.NORMAL if start else tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL if stop else tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL if disconnect else tk.DISABLED)

    def _update_device_scan_button_state(self) -> None:
        if hasattr(self, "device_scan_btn"):
            enabled = self.imu_controls_enabled and self.insole_controls_enabled
            self.device_scan_btn.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def _set_imu_controls_enabled(self, enabled: bool) -> None:
        self.imu_controls_enabled = enabled
        self._update_device_scan_button_state()
        if hasattr(self, "imu_scan_results_frame"):
            self._render_imu_scan_results()

    def _set_insole_controls_enabled(self, enabled: bool) -> None:
        self.insole_controls_enabled = enabled
        self._update_device_scan_button_state()
        if hasattr(self, "insole_scan_results_frame"):
            self._render_insole_scan_results()

    def _has_all_imus_connected(self) -> bool:
        return all(self.connected_imu_address_by_role.get(role) for role in ROLE_ORDER)

    def _has_any_connected_device(self) -> bool:
        return bool(
            any(self.connected_imu_address_by_role.values())
            or any(self.connected_insole_address_by_role.values())
        )

    def _refresh_action_buttons(self) -> None:
        if self.streaming_active:
            self._set_buttons(False, False, True, True)
            return
        self._set_buttons(
            False,
            self._has_all_imus_connected(),
            False,
            self._has_any_connected_device(),
        )

    def _run_async(self, coro, done_callback) -> None:
        future = self.controller.run_coro(coro)

        def _handle_done(fut):
            error = None
            result = None
            try:
                result = fut.result()
            except Exception as exc:
                error = exc
            self.root.after(0, done_callback, result, error)

        future.add_done_callback(_handle_done)

    def _resolve_imu_scan_target_role(self, item: IMUScanResult) -> Optional[str]:
        if item.inferred_role is not None:
            return item.inferred_role
        free_roles = [
            role
            for role in ROLE_ORDER
            if self.connected_imu_address_by_role.get(role) in (None, item.address)
        ]
        if free_roles:
            return free_roles[0]
        return None

    def _render_imu_scan_results(self) -> None:
        for child in self.imu_scan_results_frame.winfo_children():
            child.destroy()

        if not self.imu_scan_results:
            ttk.Label(
                self.imu_scan_results_frame,
                text="暂无搜索结果",
                foreground="#666666",
            ).pack(anchor=tk.W)
            return

        for item in self.imu_scan_results:
            target_role = self._resolve_imu_scan_target_role(item)
            role_label = ROLE_DISPLAY_NAMES[target_role] if target_role is not None else "侧别未知"
            connected = (
                target_role is not None
                and self.connected_imu_address_by_role.get(target_role) == item.address
            )
            row = ttk.Frame(self.imu_scan_results_frame)
            row.pack(fill=tk.X, pady=4)
            ttk.Label(
                row,
                text=(
                    f"{item.display_name} | {item.address} | RSSI={item.rssi} | 目标={role_label}"
                ),
                wraplength=880,
                justify=tk.LEFT,
            ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            button = ttk.Button(
                row,
                text="断开" if connected else "连接",
                state=(
                    tk.NORMAL
                    if target_role is not None and self.imu_controls_enabled
                    else tk.DISABLED
                ),
                command=lambda entry=item, role=target_role, is_connected=connected: (
                    self._on_toggle_imu_connection(role, entry, disconnect=is_connected)
                    if role is not None
                    else None
                ),
            )
            button.pack(side=tk.RIGHT, padx=(8, 0))

    def _on_scan_all_devices(self) -> None:
        self.imu_scan_status_var.set("IMU 搜索中...")
        self.insole_scan_status_var.set("鞋垫搜索中...")
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            self._set_imu_controls_enabled(True)
            self._set_insole_controls_enabled(True)
            if error is not None:
                self.imu_scan_status_var.set(f"IMU 搜索失败: {error}")
                self.insole_scan_status_var.set(f"鞋垫搜索失败: {error}")
                self._append_log(f"设备搜索失败: {error}")
                self._refresh_action_buttons()
                return

            imu_results: list[IMUScanResult] = []
            insole_results: list[InsoleScanResult] = []
            if isinstance(result, tuple) and len(result) >= 2:
                imu_results = list(result[0] or [])
                insole_results = list(result[1] or [])

            self.imu_scan_results = imu_results
            self.insole_scan_results = insole_results

            if self.imu_scan_results:
                self.imu_scan_status_var.set(f"发现 {len(self.imu_scan_results)} 个 IMU 设备")
            else:
                self.imu_scan_status_var.set("未发现 IMU 设备")

            if self.insole_scan_results:
                self.insole_scan_status_var.set(f"发现 {len(self.insole_scan_results)} 个鞋垫设备")
            else:
                self.insole_scan_status_var.set("未发现鞋垫设备")

            self._render_imu_scan_results()
            self._render_insole_scan_results()
            self._refresh_action_buttons()

        self._run_async(self.controller.scan_all_devices(), _done)

    def _on_scan_imus(self) -> None:
        self.imu_scan_status_var.set("IMU 搜索中...")
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)

        def _done(result, error):
            self._set_imu_controls_enabled(True)
            if error is not None:
                self.imu_scan_status_var.set(f"IMU 搜索失败: {error}")
                self._append_log(f"IMU 搜索失败: {error}")
                self._refresh_action_buttons()
                return
            self.imu_scan_results = list(result or [])
            if self.imu_scan_results:
                self.imu_scan_status_var.set(f"发现 {len(self.imu_scan_results)} 个 IMU 设备")
            else:
                self.imu_scan_status_var.set("未发现 IMU 设备")
            self._render_imu_scan_results()
            self._refresh_action_buttons()

        self._run_async(self.controller.scan_imus(), _done)

    def _on_toggle_imu_connection(
        self,
        role: str,
        item: IMUScanResult,
        disconnect: bool,
    ) -> None:
        action_text = "断开" if disconnect else "连接"
        self.imu_scan_status_var.set(
            f"{action_text}{ROLE_DISPLAY_NAMES[role]}: {item.display_name}"
        )
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)

        if disconnect:
            coro = self.controller.disconnect_imu(role)
        else:
            coro = self.controller.connect_imu(
                role=role,
                address=item.address,
                display_name=item.display_name,
                ble_device=item.ble_device,
            )

        def _done(result, error):
            self._set_imu_controls_enabled(True)
            if error is not None or not result:
                self.imu_scan_status_var.set(f"{action_text}{ROLE_DISPLAY_NAMES[role]}失败")
                self._append_log(f"{action_text}{ROLE_DISPLAY_NAMES[role]}失败: {error or '未知错误'}")
            else:
                if disconnect:
                    self.imu_scan_status_var.set(f"{ROLE_DISPLAY_NAMES[role]} 已断开")
                else:
                    self.imu_scan_status_var.set(f"{ROLE_DISPLAY_NAMES[role]} 已连接: {item.display_name}")
            self._refresh_action_buttons()

        self._run_async(coro, _done)

    def _resolve_insole_scan_target_role(self, item: InsoleScanResult) -> Optional[str]:
        if item.inferred_role is not None:
            return item.inferred_role
        free_roles = [
            role
            for role in INSOLE_ROLE_ORDER
            if self.connected_insole_address_by_role.get(role) in (None, item.address)
        ]
        if free_roles:
            return free_roles[0]
        return None

    def _render_insole_scan_results(self) -> None:
        for child in self.insole_scan_results_frame.winfo_children():
            child.destroy()

        if not self.insole_scan_results:
            ttk.Label(
                self.insole_scan_results_frame,
                text="暂无搜索结果",
                foreground="#666666",
            ).pack(anchor=tk.W)
            return

        for item in self.insole_scan_results:
            target_role = self._resolve_insole_scan_target_role(item)
            role_label = (
                INSOLE_ROLE_DISPLAY_NAMES[target_role]
                if target_role is not None
                else "侧别未知"
            )
            connected = (
                target_role is not None
                and self.connected_insole_address_by_role.get(target_role) == item.address
            )
            row = ttk.Frame(self.insole_scan_results_frame)
            row.pack(fill=tk.X, pady=4)
            ttk.Label(
                row,
                text=(
                    f"{item.display_name} | {item.address} | RSSI={item.rssi} | 目标={role_label}"
                ),
                wraplength=880,
                justify=tk.LEFT,
            ).pack(side=tk.LEFT, fill=tk.X, expand=True)
            button = ttk.Button(
                row,
                text="断开" if connected else "连接",
                state=(
                    tk.NORMAL
                    if target_role is not None and self.insole_controls_enabled
                    else tk.DISABLED
                ),
                command=lambda entry=item, role=target_role, is_connected=connected: (
                    self._on_toggle_insole_connection(role, entry, disconnect=is_connected)
                    if role is not None
                    else None
                ),
            )
            button.pack(side=tk.RIGHT, padx=(8, 0))

    def _on_scan_insoles(self) -> None:
        self.insole_scan_status_var.set("鞋垫搜索中...")
        self._set_buttons(False, False, False, False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            self._set_insole_controls_enabled(True)
            if error is not None:
                self.insole_scan_status_var.set(f"鞋垫搜索失败: {error}")
                self._append_log(f"鞋垫搜索失败: {error}")
                self._refresh_action_buttons()
                return
            self.insole_scan_results = list(result or [])
            if self.insole_scan_results:
                self.insole_scan_status_var.set(f"发现 {len(self.insole_scan_results)} 个鞋垫设备")
            else:
                self.insole_scan_status_var.set("未发现鞋垫设备")
            self._render_insole_scan_results()
            self._refresh_action_buttons()

        self._run_async(self.controller.scan_insoles(), _done)

    def _on_toggle_insole_connection(
        self,
        role: str,
        item: InsoleScanResult,
        disconnect: bool,
    ) -> None:
        action_text = "断开" if disconnect else "连接"
        self.insole_scan_status_var.set(
            f"{action_text}{INSOLE_ROLE_DISPLAY_NAMES[role]}: {item.display_name}"
        )
        self._set_buttons(False, False, False, False)
        self._set_insole_controls_enabled(False)

        if disconnect:
            coro = self.controller.disconnect_insole(role)
        else:
            coro = self.controller.connect_insole(
                role=role,
                address=item.address,
                display_name=item.display_name,
                ble_device=item.ble_device,
            )

        def _done(result, error):
            self._set_insole_controls_enabled(True)
            if error is not None or not result:
                self.insole_scan_status_var.set(
                    f"{action_text}{INSOLE_ROLE_DISPLAY_NAMES[role]}失败"
                )
                self._append_log(
                    f"{action_text}{INSOLE_ROLE_DISPLAY_NAMES[role]}失败: {error or '未知错误'}"
                )
            else:
                if disconnect:
                    self.insole_scan_status_var.set(
                        f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 已断开"
                    )
                else:
                    self.insole_scan_status_var.set(
                        f"{INSOLE_ROLE_DISPLAY_NAMES[role]} 已连接: {item.display_name}"
                    )
            self._refresh_action_buttons()

        self._run_async(coro, _done)

    def _reset_sensor_values(self) -> None:
        for role in ROLE_ORDER:
            self.role_timestamp_vars[role].set("时间戳: -")
            self.role_state_vars[role].set(initial_imu_status_text())
            self.role_address_vars[role].set("设备: 未连接")
            self.connected_imu_address_by_role[role] = None
            for _, field in SENSOR_VALUE_FIELDS:
                self.role_value_vars[role][field].set("-")
        for role in INSOLE_ROLE_ORDER:
            self.insole_state_vars[role].set(initial_insole_status_text(role, self.args.insole_role_to_mac))
            self.connected_insole_address_by_role[role] = None
        self.latest_samples.clear()
        self._render_imu_scan_results()
        self._render_insole_scan_results()
        self._refresh_action_buttons()

    def _reset_prediction_view(self) -> None:
        self.phase_var.set("等待数据")
        self.raw_phase_var.set("原始相位: -")
        self.stride_rate_var.set("步频 r: -")
        self.cos_sin_var.set("cos/sin: -")
        self.counter_var.set("窗口状态: -")
        self.timestamp_var.set("时间戳: -")
        self.phase_history.clear()
        self._redraw_phase_plot()

    def _on_connect(self) -> None:
        self.status_var.set("连接中...")
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            if error or not result:
                self.status_var.set("连接失败")
                self._append_log(f"连接失败: {error or '未知错误'}")
                self._refresh_action_buttons()
                self._set_imu_controls_enabled(True)
                self._set_insole_controls_enabled(True)
                return
            self._refresh_action_buttons()
            self._set_imu_controls_enabled(True)
            self._set_insole_controls_enabled(True)

        self._run_async(self.controller.connect_imus(), _done)

    def _on_start(self) -> None:
        self.status_var.set("启动中...")
        self._reset_prediction_view()
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            if error or not result:
                self.status_var.set("启动失败")
                self._append_log(f"启动失败: {error or '未知错误'}")
                self.streaming_active = False
                self._refresh_action_buttons()
                self._set_imu_controls_enabled(True)
                self._set_insole_controls_enabled(True)
                return
            self.streaming_active = True
            self._refresh_action_buttons()

        self._run_async(self.controller.start_all(), _done)

    def _on_stop(self) -> None:
        self.status_var.set("停止中...")
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            if error or not result:
                self.status_var.set("停止失败")
                self._append_log(f"停止失败: {error or '未知错误'}")
                self._refresh_action_buttons()
                return
            self.streaming_active = False
            self._refresh_action_buttons()
            self._set_imu_controls_enabled(True)
            self._set_insole_controls_enabled(True)

        self._run_async(self.controller.stop_all(), _done)

    def _on_disconnect(self) -> None:
        self.status_var.set("断开中...")
        self._set_buttons(False, False, False, False)
        self._set_imu_controls_enabled(False)
        self._set_insole_controls_enabled(False)

        def _done(result, error):
            if error or not result:
                self.status_var.set("断开失败")
                self._append_log(f"断开失败: {error or '未知错误'}")
                self._refresh_action_buttons()
                if not self.streaming_active:
                    self._set_imu_controls_enabled(True)
                    self._set_insole_controls_enabled(True)
                return
            self.readiness_var.set("设备同步: 左脚=未收到 | 右脚=未收到")
            self.streaming_active = False
            self._reset_sensor_values()
            self._reset_prediction_view()
            self._set_imu_controls_enabled(True)
            self._set_insole_controls_enabled(True)

        self._run_async(self.controller.disconnect_all(), _done)

    def _update_sensor_view(self, role: str, sample: SensorSample) -> None:
        self.latest_samples[role] = sample
        self.role_timestamp_vars[role].set(
            "时间戳: -"
            if sample.timestamp_us is None
            else f"时间戳: {sample.timestamp_us} us"
        )
        for _, field in SENSOR_VALUE_FIELDS:
            self.role_value_vars[role][field].set(f"{getattr(sample, field):.4f}")

    def _update_readiness(self, readiness: ReadinessSnapshot) -> None:
        self.readiness_var.set(f"设备同步: {readiness.summary_text()}")
        for role in ROLE_ORDER:
            if readiness.fresh_roles.get(role, False):
                text = "状态: 在线"
            elif role in readiness.missing_roles:
                text = "状态: 未收到数据"
            else:
                age = readiness.age_by_role.get(role)
                text = f"状态: 超时 ({age:.2f}s)" if age is not None else "状态: 未收到数据"
            self.role_state_vars[role].set(text)

    def _update_prediction_view(self, prediction: PhasePrediction) -> None:
        self.phase_var.set(f"{prediction.phase * 100:.2f}%")
        self.raw_phase_var.set(f"原始相位: {prediction.raw_phase * 100:.2f}%")
        self.stride_rate_var.set(f"步频 r: {prediction.stride_rate_hz:.3f} Hz")
        self.cos_sin_var.set(
            f"cos={prediction.cos_value:.4f} | sin={prediction.sin_value:.4f}"
        )
        self.counter_var.set(
            f"窗口={prediction.window_size} | 已收联合行={prediction.total_rows} "
            f"| 已输出={prediction.total_predictions}"
        )
        self.timestamp_var.set(
            "时间戳: -"
            if prediction.timestamp_us is None
            else f"时间戳: {prediction.timestamp_us} us"
        )
        self.phase_history.append((prediction.phase, prediction.raw_phase))
        self._redraw_phase_plot()

    def _phase_segments(
        self,
        values: list[float],
        plot_left: float,
        plot_top: float,
        plot_width: float,
        plot_height: float,
    ) -> list[list[tuple[float, float]]]:
        if not values:
            return []

        segments: list[list[tuple[float, float]]] = []
        current: list[tuple[float, float]] = []
        n_values = len(values)

        for idx, value in enumerate(values):
            if not np.isfinite(value):
                if current:
                    segments.append(current)
                    current = []
                continue

            clipped = float(np.clip(value, 0.0, 1.0))
            x = plot_left + (idx / max(1, n_values - 1)) * plot_width
            y = plot_top + (1.0 - clipped) * plot_height
            current.append((x, y))

        if current:
            segments.append(current)
        return segments

    def _draw_phase_series(
        self,
        values: list[float],
        plot_left: float,
        plot_top: float,
        plot_width: float,
        plot_height: float,
        color: str,
        width: int,
        dash: Optional[tuple[int, int]] = None,
    ) -> None:
        canvas = self.plot_canvas
        if canvas is None or not canvas.winfo_exists():
            return
        segments = self._phase_segments(values, plot_left, plot_top, plot_width, plot_height)
        for segment in segments:
            if len(segment) >= 2:
                coords = [coord for point in segment for coord in point]
                canvas.create_line(
                    *coords,
                    fill=color,
                    width=width,
                    dash=dash,
                )
            elif len(segment) == 1:
                x, y = segment[0]
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)

        if values:
            last_x = plot_left + plot_width
            last_y = plot_top + (1.0 - float(np.clip(values[-1], 0.0, 1.0))) * plot_height
            canvas.create_oval(
                last_x - 3,
                last_y - 3,
                last_x + 3,
                last_y + 3,
                fill=color,
                outline=color,
            )

    def _redraw_phase_plot(self) -> None:
        canvas = self.plot_canvas
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), int(canvas.cget("width")))
        height = max(canvas.winfo_height(), int(canvas.cget("height")))
        if width < 100 or height < 100:
            return

        plot_left = 52
        plot_top = 18
        plot_right = width - 16
        plot_bottom = height - 28
        plot_width = max(10, plot_right - plot_left)
        plot_height = max(10, plot_bottom - plot_top)

        canvas.create_rectangle(
            plot_left,
            plot_top,
            plot_right,
            plot_bottom,
            outline=PLOT_COLORS["grid"],
        )
        for ratio in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = plot_top + (1.0 - ratio) * plot_height
            canvas.create_line(
                plot_left,
                y,
                plot_right,
                y,
                fill=PLOT_COLORS["grid"],
                dash=(2, 4),
            )
            canvas.create_text(
                plot_left - 8,
                y,
                text=f"{ratio * 100:.0f}",
                fill=PLOT_COLORS["axis"],
                anchor="e",
            )
        canvas.create_text(
            plot_left,
            plot_top - 8,
            text="相位 (%)",
            fill=PLOT_COLORS["axis"],
            anchor="w",
        )
        canvas.create_text(
            plot_right,
            plot_bottom + 16,
            text=f"最近 {PLOT_HISTORY_LENGTH} 个预测窗口",
            fill=PLOT_COLORS["axis"],
            anchor="e",
        )

        legend_x = plot_right - 150
        legend_y = plot_top + 12
        canvas.create_line(
            legend_x,
            legend_y,
            legend_x + 28,
            legend_y,
            fill=PLOT_COLORS["phase"],
            width=3,
        )
        canvas.create_text(legend_x + 36, legend_y, text="后处理相位", anchor="w")
        canvas.create_line(
            legend_x,
            legend_y + 18,
            legend_x + 28,
            legend_y + 18,
            fill=PLOT_COLORS["raw_phase"],
            width=2,
            dash=(4, 3),
        )
        canvas.create_text(legend_x + 36, legend_y + 18, text="模型原始相位", anchor="w")

        if not self.phase_history:
            canvas.create_text(
                width / 2,
                height / 2,
                text="等待模型输出相位",
                fill=PLOT_COLORS["axis"],
            )
            return

        phase_values = [item[0] for item in self.phase_history]
        raw_values = [item[1] for item in self.phase_history]
        self._draw_phase_series(
            raw_values,
            plot_left,
            plot_top,
            plot_width,
            plot_height,
            color=PLOT_COLORS["raw_phase"],
            width=2,
            dash=(4, 3),
        )
        self._draw_phase_series(
            phase_values,
            plot_left,
            plot_top,
            plot_width,
            plot_height,
            color=PLOT_COLORS["phase"],
            width=3,
        )

    def _poll_events(self) -> None:
        latest_samples: dict[str, SensorSample] = {}
        latest_prediction: Optional[PhasePrediction] = None
        latest_readiness: Optional[ReadinessSnapshot] = None
        try:
            while True:
                event = self.event_queue.get_nowait()
                event_type = event["type"]
                if event_type == "session":
                    pass
                elif event_type == "imu_state":
                    role = event["role"]
                    status = event["status"]
                    address = event.get("address")
                    self.role_state_vars[role].set(status)
                    if address:
                        self.role_address_vars[role].set(f"设备: {address}")
                    if status in ("状态: 已连接", "状态: 已开始接收"):
                        if address:
                            self.connected_imu_address_by_role[role] = address
                    else:
                        self.connected_imu_address_by_role[role] = None
                        if status != "状态: 连接中":
                            self.role_address_vars[role].set("设备: 未连接")
                    self._render_imu_scan_results()
                    if not self.streaming_active:
                        self._refresh_action_buttons()
                elif event_type == "insole_state":
                    role = event["role"]
                    status = event["status"]
                    address = event.get("address")
                    self.insole_state_vars[role].set(status)
                    if status in ("状态: 已连接", "状态: 已开始接收"):
                        if address:
                            self.connected_insole_address_by_role[role] = address
                    else:
                        self.connected_insole_address_by_role[role] = None
                    self._render_insole_scan_results()
                    if not self.streaming_active:
                        self._refresh_action_buttons()
                elif event_type == "log":
                    self._append_log(event["message"])
                elif event_type == "status":
                    self.status_var.set(event["status"])
                elif event_type == "sample":
                    sample: SensorSample = event["sample"]
                    latest_samples[sample.role] = sample
                    latest_readiness = event.get("readiness")
                elif event_type == "prediction":
                    latest_prediction = event["prediction"]
                    latest_readiness = event.get("readiness")
        except Empty:
            pass

        for role, sample in latest_samples.items():
            self._update_sensor_view(role, sample)
        if latest_readiness is not None:
            self._update_readiness(latest_readiness)
        if latest_prediction is not None:
            self._update_prediction_view(latest_prediction)

        self.root.after(DEFAULT_POLL_INTERVAL_MS, self._poll_events)

    def _destroy_phase_window(self) -> None:
        if self.phase_window is not None and self.phase_window.winfo_exists():
            self.phase_window.destroy()
        self.phase_window = None
        self.plot_canvas = None

    def _on_close(self) -> None:
        def _done(result, error):
            if error:
                self._append_log(f"关闭前断开失败: {error}")
            self._destroy_phase_window()
            self.controller.shutdown()
            self.root.destroy()

        if self.controller.sensors or self.controller.insole_clients:
            self._run_async(self.controller.disconnect_all(), _done)
        else:
            self._destroy_phase_window()
            self.controller.shutdown()
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="连接双 IMU / 双鞋垫并实时输出步态相位。")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help=f"Keras/ONNX 模型路径，默认: {DEFAULT_MODEL_PATH}",
    )
    parser.add_argument(
        "--scaler",
        type=Path,
        default=DEFAULT_SCALER_PATH,
        help=f"特征标准化器路径，默认: {DEFAULT_SCALER_PATH}",
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=float,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help="实时推理采样率，默认 30Hz。",
    )
    parser.add_argument(
        "--filter-profile",
        default=DEFAULT_FILTER_PROFILE,
        help="Movella 滤波配置，默认 GENERAL。",
    )
    parser.add_argument(
        "--payload-mode",
        default=DEFAULT_PAYLOAD_MODE,
        help="Movella payload 模式，默认 CUSTOM_MODE_5。",
    )
    parser.add_argument(
        "--stale-timeout-sec",
        type=float,
        default=DEFAULT_STALE_TIMEOUT_SEC,
        help="任一 IMU 超过该时间未刷新就暂停输出，默认 0.7 秒。",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=DEFAULT_DATA_ROOT,
        help=f"实时数据保存根目录，默认: {DEFAULT_DATA_ROOT}",
    )
    parser.add_argument(
        "--ui-scale",
        type=float,
        default=None,
        help="界面缩放倍数。默认自动跟随系统 DPI；设置后会覆盖自动缩放。",
    )
    parser.add_argument(
        "--ble-connect-timeout-sec",
        type=float,
        default=DEFAULT_BLE_CONNECT_TIMEOUT_SEC,
        help=f"BLE 连接超时秒数，默认 {DEFAULT_BLE_CONNECT_TIMEOUT_SEC:.1f}。",
    )
    parser.add_argument(
        "--ble-service-timeout-sec",
        type=float,
        default=DEFAULT_BLE_SERVICE_TIMEOUT_SEC,
        help=f"BLE 服务发现/启动通知超时秒数，默认 {DEFAULT_BLE_SERVICE_TIMEOUT_SEC:.1f}。",
    )
    parser.add_argument(
        "--ble-disconnect-timeout-sec",
        type=float,
        default=DEFAULT_BLE_DISCONNECT_TIMEOUT_SEC,
        help=f"BLE 断开超时秒数，默认 {DEFAULT_BLE_DISCONNECT_TIMEOUT_SEC:.1f}。",
    )
    parser.add_argument(
        "--ble-connect-retries",
        type=int,
        default=DEFAULT_BLE_CONNECT_RETRIES,
        help=f"BLE 连接失败时的最大重试次数，默认 {DEFAULT_BLE_CONNECT_RETRIES}。",
    )
    parser.add_argument(
        "--ble-retry-delay-sec",
        type=float,
        default=DEFAULT_BLE_RETRY_DELAY_SEC,
        help=f"BLE 重试前等待秒数，默认 {DEFAULT_BLE_RETRY_DELAY_SEC:.1f}。",
    )
    parser.add_argument(
        "--ble-settle-sec",
        type=float,
        default=DEFAULT_BLE_SETTLE_SEC,
        help=f"每次 BLE 操作后的稳定等待秒数，默认 {DEFAULT_BLE_SETTLE_SEC:.2f}。",
    )

    parser.add_argument(
        "--left-foot-mac",
        default=DEFAULT_ROLE_TO_MAC["left_foot"],
        help="左脚 IMU 的 MAC 地址。",
    )
    parser.add_argument(
        "--right-foot-mac",
        default=DEFAULT_ROLE_TO_MAC["right_foot"],
        help="右脚 IMU 的 MAC 地址。",
    )
    parser.add_argument(
        "--left-insole-mac",
        default=DEFAULT_INSOLE_ROLE_TO_MAC["left_insole"],
        help="左鞋垫 MAC，可留空表示不连接该鞋垫。",
    )
    parser.add_argument(
        "--right-insole-mac",
        default=DEFAULT_INSOLE_ROLE_TO_MAC["right_insole"],
        help="右鞋垫 MAC，可留空表示不连接该鞋垫。",
    )

    parser.add_argument("--replay-left-foot", help="左脚 CSV 回放文件。")
    parser.add_argument("--replay-right-foot", help="右脚 CSV 回放文件。")
    parser.add_argument(
        "--replay-speed",
        type=float,
        default=0.0,
        help="回放倍速，0 表示不 sleep 直接跑完。",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无窗口模式，直接在终端打印状态和相位。",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    args.model = args.model.expanduser().resolve()
    args.scaler = args.scaler.expanduser().resolve()
    args.data_root = args.data_root.expanduser().resolve()
    if args.ui_scale is not None and args.ui_scale <= 0.0:
        raise SystemExit(f"--ui-scale 必须大于 0，当前收到: {args.ui_scale}")
    if args.ble_connect_timeout_sec <= 0.0:
        raise SystemExit(
            f"--ble-connect-timeout-sec 必须大于 0，当前收到: {args.ble_connect_timeout_sec}"
        )
    if args.ble_service_timeout_sec <= 0.0:
        raise SystemExit(
            f"--ble-service-timeout-sec 必须大于 0，当前收到: {args.ble_service_timeout_sec}"
        )
    if args.ble_disconnect_timeout_sec <= 0.0:
        raise SystemExit(
            f"--ble-disconnect-timeout-sec 必须大于 0，当前收到: {args.ble_disconnect_timeout_sec}"
        )
    if args.ble_connect_retries < 1:
        raise SystemExit(
            f"--ble-connect-retries 必须大于等于 1，当前收到: {args.ble_connect_retries}"
        )
    if args.ble_retry_delay_sec < 0.0:
        raise SystemExit(
            f"--ble-retry-delay-sec 不能小于 0，当前收到: {args.ble_retry_delay_sec}"
        )
    if args.ble_settle_sec < 0.0:
        raise SystemExit(f"--ble-settle-sec 不能小于 0，当前收到: {args.ble_settle_sec}")

    args.role_to_mac = validate_role_to_mac(
        {
            "left_foot": args.left_foot_mac,
            "right_foot": args.right_foot_mac,
        }
    )
    args.insole_role_to_mac = validate_optional_role_to_mac(
        {
            "left_insole": args.left_insole_mac,
            "right_insole": args.right_insole_mac,
        }
    )
    validate_all_devices_distinct(args.role_to_mac, args.insole_role_to_mac)

    if not args.model.exists():
        raise SystemExit(f"模型文件不存在: {args.model}")
    if args.model.suffix.lower() not in (".keras", ".onnx"):
        raise SystemExit(
            f"multi_imu_gait_phase_live.py 只接受 .keras/.onnx 模型，当前收到: {args.model}"
        )
    if not args.scaler.exists():
        raise SystemExit(f"scaler 文件不存在: {args.scaler}")

    predictor = RealtimeGaitPhasePredictor(
        model_path=args.model,
        scaler_path=args.scaler,
        sample_rate_hz=args.sample_rate_hz,
    )
    event_queue: Queue = Queue()

    replay_mode = bool(args.replay_left_foot or args.replay_right_foot)
    if replay_mode:
        if not (args.replay_left_foot and args.replay_right_foot):
            raise SystemExit("回放模式需要同时提供 --replay-left-foot 和 --replay-right-foot")
        if not args.headless:
            raise SystemExit("回放模式当前请配合 --headless 使用。")
        return run_replay_headless(args, predictor=predictor, event_queue=event_queue)

    if args.headless:
        return run_live_headless(args, predictor=predictor, event_queue=event_queue)

    try:
        app = LivePhaseUI(args=args, predictor=predictor)
    except tk.TclError as exc:
        raise SystemExit(f"无法启动图形界面: {exc}。如需终端模式请加 --headless。") from exc
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
