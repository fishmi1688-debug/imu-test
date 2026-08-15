#!/usr/bin/env python3

"""SocketCAN source for wired thigh IMUs used by IMU phase modes."""

from __future__ import annotations

import math
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

QUAT_SCALE = 1.0 / 32767.0
GYRO_SCALE_RAD_S = 2.0 ** -9
ACC_SCALE_M_S2 = 2.0 ** -8

DEFAULT_CAN_INTERFACE = "can0"
DEFAULT_DATA_TIMEOUT_SEC = 0.5
DEFAULT_EXPECTED_RATE_HZ = 50.0

DEFAULT_FRAME_IDS = {
    "left": {
        "timestamp": 0x004,
        "quaternion": 0x021,
        "gyro": 0x031,
        "accel": 0x033,
    },
    "right": {
        "timestamp": 0x005,
        "quaternion": 0x022,
        "gyro": 0x032,
        "accel": 0x034,
    },
}


def _env_enabled(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return raw.strip().lower() not in ("0", "false", "no", "off")


def _read_env_float(name: str, default: float, minimum: float) -> float:
    try:
        return max(minimum, float(os.environ.get(name, str(default))))
    except Exception:
        return max(minimum, float(default))


def parse_can_id(text: str) -> int:
    cleaned = str(text or "").strip().lower()
    if not cleaned:
        raise ValueError("CAN ID is empty")
    value = int(cleaned, 16)
    if value < 0 or value > CAN_EFF_MASK:
        raise ValueError(f"CAN ID out of range: {text}")
    return value


def format_can_id(can_id: int) -> str:
    width = 3 if int(can_id) <= CAN_SFF_MASK else 8
    return f"{int(can_id):0{width}X}"


def _read_can_id_from_env(side: str, signal_name: str, default: int) -> int:
    side_key = side.upper()
    signal_key = signal_name.upper()
    names = (
        f"GAIT_IMU_PHASE_CAN_{side_key}_{signal_key}_ID",
        f"GAIT_IMU_PHASE_{side_key}_{signal_key}_ID",
    )
    for name in names:
        raw = os.environ.get(name)
        if raw:
            return parse_can_id(raw)
    return int(default)


def build_frame_ids_from_env(side: str) -> dict[str, int]:
    defaults = DEFAULT_FRAME_IDS.get(side)
    if defaults is None:
        raise ValueError(f"invalid IMU side: {side}")
    return {
        signal_name: _read_can_id_from_env(side, signal_name, can_id)
        for signal_name, can_id in defaults.items()
    }


def decode_sample_time_us(payload: bytes) -> int:
    if len(payload) < 4:
        raise ValueError(f"timestamp frame needs >= 4 bytes, got {len(payload)}")
    return struct.unpack(">I", payload[:4])[0]


def decode_quaternion(payload: bytes) -> tuple[float, float, float, float]:
    if len(payload) < 8:
        raise ValueError(f"quaternion frame needs >= 8 bytes, got {len(payload)}")
    raw = struct.unpack(">hhhh", payload[:8])
    return tuple(value * QUAT_SCALE for value in raw)  # type: ignore[return-value]


def decode_gyro_deg_s(payload: bytes) -> tuple[float, float, float]:
    if len(payload) < 6:
        raise ValueError(f"gyro frame needs >= 6 bytes, got {len(payload)}")
    raw = struct.unpack(">hhh", payload[:6])
    return tuple(math.degrees(value * GYRO_SCALE_RAD_S) for value in raw)  # type: ignore[return-value]


def decode_acc_m_s2(payload: bytes) -> tuple[float, float, float]:
    if len(payload) < 6:
        raise ValueError(f"accel frame needs >= 6 bytes, got {len(payload)}")
    raw = struct.unpack(">hhh", payload[:6])
    return tuple(value * ACC_SCALE_M_S2 for value in raw)  # type: ignore[return-value]


@dataclass
class PartialCanImuSample:
    timestamp_us: Optional[int] = None
    quat: Optional[tuple[float, float, float, float]] = None
    gyro_deg_s: Optional[tuple[float, float, float]] = None
    acc_m_s2: Optional[tuple[float, float, float]] = None
    emitted: bool = False

    def start_new_timestamp(self, timestamp_us: Optional[int]) -> None:
        self.timestamp_us = timestamp_us
        self.quat = None
        self.gyro_deg_s = None
        self.acc_m_s2 = None
        self.emitted = False

    def complete(self, require_quaternion: bool) -> bool:
        quat_ready = self.quat is not None or not require_quaternion
        return quat_ready and self.gyro_deg_s is not None and self.acc_m_s2 is not None


class WiredCanImuPhaseSource:
    """Detector-compatible source that exposes latest wired IMU 6D samples."""

    def __init__(
        self,
        side: str,
        interface: Optional[str] = None,
        frame_ids: Optional[dict[str, int]] = None,
        data_timeout_sec: float = DEFAULT_DATA_TIMEOUT_SEC,
        require_quaternion: Optional[bool] = None,
    ):
        side_key = str(side or "").strip().lower()
        if side_key not in ("left", "right"):
            raise ValueError(f"invalid IMU side: {side}")
        self.side = side_key
        self.interface = str(
            interface
            if interface is not None
            else os.environ.get("GAIT_IMU_PHASE_CAN_INTERFACE", DEFAULT_CAN_INTERFACE)
        ).strip() or DEFAULT_CAN_INTERFACE
        self.frame_ids = dict(frame_ids or build_frame_ids_from_env(self.side))
        self.id_to_signal = {int(can_id): signal for signal, can_id in self.frame_ids.items()}
        self.data_timeout_sec = float(data_timeout_sec)
        self.sample_rate_hz = _read_env_float(
            "GAIT_IMU_PHASE_CAN_RATE_HZ",
            DEFAULT_EXPECTED_RATE_HZ,
            1.0,
        )
        self.require_quaternion = (
            _env_enabled("GAIT_IMU_PHASE_CAN_REQUIRE_QUATERNION", True)
            if require_quaternion is None
            else bool(require_quaternion)
        )
        self.mac_address = f"wired:{self.interface}:{self.side}"

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._partial = PartialCanImuSample()
        self._latest_sample: Optional[tuple[tuple[float, float, float, float, float, float], float, int]] = None
        self._seq = 0
        self._frame_count = 0
        self._connected = False
        self._measuring = False
        self._measurement_enabled = False
        self._last_error = ""

    @property
    def last_error(self) -> str:
        return self._last_error

    def set_measurement_enabled(self, enabled: bool) -> bool:
        target = bool(enabled)
        with self._lock:
            self._measurement_enabled = target
            self._measuring = bool(self._connected and target)
        return True

    def start(self) -> bool:
        if not hasattr(socket, "PF_CAN") or not hasattr(socket, "CAN_RAW"):
            self._last_error = "SocketCAN is not available on this system"
            return False
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._connected = True
                self._measuring = bool(self._measurement_enabled)
                return True

        try:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.settimeout(0.2)
            self._install_filters(sock)
            sock.bind((self.interface,))
        except OSError as exc:
            self._last_error = f"open SocketCAN {self.interface} failed: {exc}"
            try:
                sock.close()  # type: ignore[possibly-undefined]
            except Exception:
                pass
            with self._lock:
                self._connected = False
                self._measuring = False
            return False
        except Exception as exc:
            self._last_error = str(exc)
            with self._lock:
                self._connected = False
                self._measuring = False
            return False

        with self._lock:
            self._sock = sock
            self._partial = PartialCanImuSample()
            self._latest_sample = None
            self._frame_count = 0
            self._connected = True
            self._measuring = bool(self._measurement_enabled)
            self._last_error = ""
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._read_loop,
                name=f"wired-imu-{self.side}-{self.interface}",
                daemon=True,
            )
            self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        with self._lock:
            sock = self._sock
            self._sock = None
            self._thread = None
            self._connected = False
            self._measuring = False
            self._measurement_enabled = False
            self._latest_sample = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def request_measurement_restart(self) -> bool:
        return False

    def get_latest_sample_6d(self):
        with self._lock:
            return self._latest_sample

    def _install_filters(self, sock: socket.socket) -> None:
        sol_can_raw = getattr(socket, "SOL_CAN_RAW", None)
        can_raw_filter = getattr(socket, "CAN_RAW_FILTER", None)
        if sol_can_raw is None or can_raw_filter is None:
            return
        filter_blob = b"".join(
            struct.pack(
                "=II",
                int(can_id),
                CAN_EFF_MASK if int(can_id) > CAN_SFF_MASK else CAN_SFF_MASK,
            )
            for can_id in self.id_to_signal
        )
        if filter_blob:
            sock.setsockopt(sol_can_raw, can_raw_filter, filter_blob)

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                sock = self._sock
                measuring = self._measuring
            if sock is None:
                return
            if not measuring:
                time.sleep(0.02)
                continue
            try:
                raw_frame = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as exc:
                self._last_error = f"SocketCAN read failed: {exc}"
                break
            if len(raw_frame) < CAN_FRAME_SIZE:
                continue
            try:
                self._handle_raw_frame(raw_frame, time.time())
            except Exception as exc:
                self._last_error = f"CAN frame parse failed: {exc}"

        with self._lock:
            self._connected = False
            self._measuring = False

    def _handle_raw_frame(self, raw_frame: bytes, arrival_time: float) -> None:
        can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FORMAT, raw_frame)
        if can_id_raw & (CAN_RTR_FLAG | CAN_ERR_FLAG):
            return
        frame_id = can_id_raw & (CAN_EFF_MASK if can_id_raw & CAN_EFF_FLAG else CAN_SFF_MASK)
        signal_name = self.id_to_signal.get(frame_id)
        if signal_name is None:
            return

        payload = data[: min(int(dlc), len(data))]
        partial = self._partial
        if signal_name == "timestamp":
            partial.start_new_timestamp(decode_sample_time_us(payload))
        elif signal_name == "quaternion":
            if partial.timestamp_us is None and partial.gyro_deg_s is None and partial.acc_m_s2 is None:
                partial.start_new_timestamp(None)
            partial.quat = decode_quaternion(payload)
        elif signal_name == "gyro":
            if partial.timestamp_us is None and partial.quat is None and partial.acc_m_s2 is None:
                partial.start_new_timestamp(None)
            partial.gyro_deg_s = decode_gyro_deg_s(payload)
        elif signal_name == "accel":
            if partial.timestamp_us is None and partial.quat is None and partial.gyro_deg_s is None:
                partial.start_new_timestamp(None)
            partial.acc_m_s2 = decode_acc_m_s2(payload)
        else:
            return

        self._frame_count += 1
        if partial.emitted or not partial.complete(self.require_quaternion):
            return

        acc = partial.acc_m_s2
        gyro = partial.gyro_deg_s
        if acc is None or gyro is None:
            return
        sample_6d = (
            float(acc[0]),
            float(acc[1]),
            float(acc[2]),
            float(gyro[0]),
            float(gyro[1]),
            float(gyro[2]),
        )
        partial.emitted = True
        with self._lock:
            self._seq += 1
            self._latest_sample = (sample_6d, float(arrival_time), int(self._seq))
            self._last_error = ""
