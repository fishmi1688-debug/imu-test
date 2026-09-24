#!/usr/bin/env python3

"""SocketCAN source for Molex MI1 thigh IMUs used by gait phase modes."""

from __future__ import annotations

import errno
import math
import os
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from .gait_constants import CAN_INTERFACE


DEFAULT_CAN_INTERFACE = CAN_INTERFACE
DEFAULT_LEFT_NODE_ID = 0x01
DEFAULT_RIGHT_NODE_ID = 0x02
DEFAULT_PROTOCOL = "auto"
DEFAULT_DATA_TIMEOUT_SEC = 0.5
DEFAULT_MAX_FIELD_AGE_SEC = 0.15
DEFAULT_MAX_FIELD_SPAN_SEC = 0.10
DEFAULT_EXPECTED_RATE_HZ = 50.0
DEFAULT_MIN_ACCEL_NORM_G = 0.20
DEFAULT_MAX_ACCEL_NORM_G = 6.00
DEFAULT_MIN_QUAT_NORM = 0.50
DEFAULT_MAX_QUAT_NORM = 1.50
DEFAULT_MAX_GYRO_DPS = 800.0

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

J1939_PGN_ACCEL = 0xFF34
J1939_PGN_GYRO = 0xFF37
J1939_PGN_QUAT = 0xFF46

CANOPEN_TPDO_ACCEL_BASE = 0x180
CANOPEN_TPDO_GYRO_BASE = 0x280
CANOPEN_TPDO_QUAT_BASE = 0x480

J1939_ACCEL_SCALE_G = 0.00048828
J1939_GYRO_SCALE_DPS = 0.061035
CANOPEN_ACCEL_SCALE_G = 0.001
CANOPEN_GYRO_SCALE_DPS = 0.1
QUAT_SCALE = 0.0001


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quat:
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class DecodedFrame:
    protocol: str
    node_id: int
    signal_name: str
    value: Vec3 | Quat


@dataclass
class ImuState:
    node_id: int
    protocol: Optional[str] = None
    accel_g: Optional[Vec3] = None
    gyro_dps: Optional[Vec3] = None
    quat: Optional[Quat] = None
    updated_at: dict[str, float] = field(default_factory=dict)
    frame_counts: dict[str, int] = field(
        default_factory=lambda: {"accel": 0, "gyro": 0, "quat": 0}
    )

    def update(self, decoded: DecodedFrame, now: float) -> None:
        if self.protocol is None:
            self.protocol = decoded.protocol
        elif self.protocol != decoded.protocol:
            self.protocol = "mixed"

        if decoded.signal_name == "accel":
            self.accel_g = decoded.value  # type: ignore[assignment]
        elif decoded.signal_name == "gyro":
            self.gyro_dps = decoded.value  # type: ignore[assignment]
        elif decoded.signal_name == "quat":
            self.quat = decoded.value  # type: ignore[assignment]
        else:
            raise ValueError(f"unknown MI1 signal: {decoded.signal_name}")

        self.updated_at[decoded.signal_name] = float(now)
        self.frame_counts[decoded.signal_name] = (
            self.frame_counts.get(decoded.signal_name, 0) + 1
        )

    def is_complete(self) -> bool:
        return self.accel_g is not None and self.gyro_dps is not None and self.quat is not None

    def is_fresh(self, now: float, max_age_sec: float, max_span_sec: float = 0.0) -> bool:
        if not self.is_complete():
            return False
        field_times = [self.updated_at.get(name, -1e30) for name in ("accel", "gyro", "quat")]
        if max_age_sec <= 0.0:
            age_ok = True
        else:
            age_ok = all(now - field_time <= max_age_sec for field_time in field_times)
        if not age_ok:
            return False
        if max_span_sec > 0.0 and max(field_times) - min(field_times) > max_span_sec:
            return False
        return True

    def has_new_complete_sample(
        self,
        now: float,
        max_age_sec: float,
        max_span_sec: float,
        last_counts: dict[str, int],
    ) -> bool:
        if not self.is_fresh(now, max_age_sec, max_span_sec):
            return False
        return all(
            self.frame_counts.get(name, 0) > last_counts.get(name, 0)
            for name in ("accel", "gyro", "quat")
        )

    def mark_sample_emitted(self, last_counts: dict[str, int]) -> None:
        for name in ("accel", "gyro", "quat"):
            last_counts[name] = self.frame_counts.get(name, 0)

    def to_phase_sample(self) -> tuple[float, float, float, float, float, float, float, float, float, float]:
        if self.accel_g is None or self.gyro_dps is None or self.quat is None:
            raise ValueError("cannot build MI1 sample from incomplete state")
        return (
            float(self.accel_g.x),
            float(self.accel_g.y),
            float(self.accel_g.z),
            float(self.gyro_dps.x),
            float(self.gyro_dps.y),
            float(self.gyro_dps.z),
            float(self.quat.w),
            float(self.quat.x),
            float(self.quat.y),
            float(self.quat.z),
        )

    def available_fields_text(self) -> str:
        fields = [
            name
            for name in ("accel", "gyro", "quat")
            if self.frame_counts.get(name, 0) > 0
        ]
        return "+".join(fields) if fields else "none"


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


def parse_int_auto(text: str) -> int:
    return int(str(text).strip(), 0)


def validate_node_id(value: int) -> int:
    node_id = int(value)
    if not 1 <= node_id <= 126:
        raise ValueError("MI1 CAN node ID must be in range 1..126")
    return node_id


def format_node_id(node_id: int) -> str:
    return f"0x{int(node_id) & 0xFF:02X}"


def _default_node_id_for_side(side: str) -> int:
    return DEFAULT_LEFT_NODE_ID if side == "left" else DEFAULT_RIGHT_NODE_ID


def _read_node_id_from_env(side: str, default: int) -> int:
    side_key = side.upper()
    names = (
        f"GAIT_MI1_{side_key}_NODE_ID",
        f"GAIT_IMU_PHASE_MI1_{side_key}_NODE_ID",
        f"GAIT_IMU_PHASE_{side_key}_NODE_ID",
    )
    for name in names:
        raw = os.environ.get(name)
        if raw:
            return validate_node_id(parse_int_auto(raw))
    return validate_node_id(default)


def _read_protocol_from_env(default: str = DEFAULT_PROTOCOL) -> str:
    raw = (
        os.environ.get("GAIT_MI1_CAN_PROTOCOL")
        or os.environ.get("GAIT_IMU_PHASE_MI1_CAN_PROTOCOL")
        or os.environ.get("GAIT_IMU_PHASE_CAN_PROTOCOL")
        or default
    )
    protocol = str(raw).strip().lower()
    if protocol not in ("auto", "j1939", "canopen"):
        raise ValueError(f"invalid MI1 CAN protocol: {raw}")
    return protocol


def decode_socketcan_frame(frame: bytes) -> Optional[tuple[int, bool, bytes]]:
    if len(frame) < CAN_FRAME_SIZE:
        return None

    can_id_flags, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, frame[:CAN_FRAME_SIZE])
    if can_id_flags & (CAN_ERR_FLAG | CAN_RTR_FLAG):
        return None

    is_extended = bool(can_id_flags & CAN_EFF_FLAG)
    can_id = can_id_flags & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    return can_id, is_extended, payload[: min(int(dlc), 8)]


def decode_vec3_i16(payload: bytes, scale: float) -> Optional[Vec3]:
    if len(payload) < 6:
        return None
    x_raw, y_raw, z_raw = struct.unpack_from("<hhh", payload)
    return Vec3(x_raw * scale, y_raw * scale, z_raw * scale)


def decode_quat_i16(payload: bytes) -> Optional[Quat]:
    if len(payload) < 8:
        return None
    w_raw, x_raw, y_raw, z_raw = struct.unpack_from("<hhhh", payload)
    return Quat(
        w_raw * QUAT_SCALE,
        x_raw * QUAT_SCALE,
        y_raw * QUAT_SCALE,
        z_raw * QUAT_SCALE,
    )


def decode_j1939(can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if not is_extended:
        return None

    data_page = (can_id >> 24) & 0x01
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    source_address = can_id & 0xFF
    if pf != 0xFF:
        return None

    pgn = (data_page << 16) | (pf << 8) | ps
    if pgn == J1939_PGN_ACCEL:
        value = decode_vec3_i16(payload, J1939_ACCEL_SCALE_G)
        return DecodedFrame("j1939", source_address, "accel", value) if value is not None else None
    if pgn == J1939_PGN_GYRO:
        value = decode_vec3_i16(payload, J1939_GYRO_SCALE_DPS)
        return DecodedFrame("j1939", source_address, "gyro", value) if value is not None else None
    if pgn == J1939_PGN_QUAT:
        value = decode_quat_i16(payload)
        return DecodedFrame("j1939", source_address, "quat", value) if value is not None else None
    return None


def decode_canopen(can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if is_extended:
        return None

    node_id = can_id - CANOPEN_TPDO_ACCEL_BASE
    if 1 <= node_id <= 127:
        value = decode_vec3_i16(payload, CANOPEN_ACCEL_SCALE_G)
        return DecodedFrame("canopen", node_id, "accel", value) if value is not None else None

    node_id = can_id - CANOPEN_TPDO_GYRO_BASE
    if 1 <= node_id <= 127:
        value = decode_vec3_i16(payload, CANOPEN_GYRO_SCALE_DPS)
        return DecodedFrame("canopen", node_id, "gyro", value) if value is not None else None

    node_id = can_id - CANOPEN_TPDO_QUAT_BASE
    if 1 <= node_id <= 127:
        value = decode_quat_i16(payload)
        return DecodedFrame("canopen", node_id, "quat", value) if value is not None else None

    return None


def decode_imu_frame(protocol: str, can_id: int, is_extended: bool, payload: bytes) -> Optional[DecodedFrame]:
    if protocol in ("auto", "j1939"):
        decoded = decode_j1939(can_id, is_extended, payload)
        if decoded is not None:
            return decoded
    if protocol in ("auto", "canopen"):
        return decode_canopen(can_id, is_extended, payload)
    return None


class WiredMi1CanImuPhaseSource:
    """Detector-compatible source exposing latest MI1 acc/gyro/quaternion samples."""

    def __init__(
        self,
        side: str,
        interface: Optional[str] = None,
        protocol: Optional[str] = None,
        node_id: Optional[int] = None,
        data_timeout_sec: float = DEFAULT_DATA_TIMEOUT_SEC,
        max_field_age_sec: Optional[float] = None,
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
        self.protocol = str(protocol or _read_protocol_from_env()).strip().lower()
        if self.protocol not in ("auto", "j1939", "canopen"):
            raise ValueError(f"invalid MI1 CAN protocol: {self.protocol}")
        self.node_id = int(
            validate_node_id(
                node_id
                if node_id is not None
                else _read_node_id_from_env(self.side, _default_node_id_for_side(self.side))
            )
        )
        self.data_timeout_sec = float(data_timeout_sec)
        self.max_field_age_sec = (
            max(0.0, float(max_field_age_sec))
            if max_field_age_sec is not None
            else _read_env_float(
                "GAIT_MI1_MAX_FIELD_AGE_SEC",
                DEFAULT_MAX_FIELD_AGE_SEC,
                0.0,
            )
        )
        self.max_field_span_sec = _read_env_float(
            "GAIT_MI1_MAX_FIELD_SPAN_SEC",
            DEFAULT_MAX_FIELD_SPAN_SEC,
            0.0,
        )
        self.sample_rate_hz = _read_env_float(
            "GAIT_IMU_PHASE_MI1_RATE_HZ",
            DEFAULT_EXPECTED_RATE_HZ,
            1.0,
        )
        self.reject_invalid_samples = _env_enabled("GAIT_MI1_REJECT_INVALID_SAMPLES", True)
        self.min_accel_norm_g = _read_env_float(
            "GAIT_MI1_MIN_ACCEL_NORM_G",
            DEFAULT_MIN_ACCEL_NORM_G,
            0.0,
        )
        self.max_accel_norm_g = _read_env_float(
            "GAIT_MI1_MAX_ACCEL_NORM_G",
            DEFAULT_MAX_ACCEL_NORM_G,
            0.0,
        )
        self.min_quat_norm = _read_env_float(
            "GAIT_MI1_MIN_QUAT_NORM",
            DEFAULT_MIN_QUAT_NORM,
            0.0,
        )
        self.max_quat_norm = _read_env_float(
            "GAIT_MI1_MAX_QUAT_NORM",
            DEFAULT_MAX_QUAT_NORM,
            0.0,
        )
        self.max_gyro_dps = _read_env_float(
            "GAIT_MI1_MAX_GYRO_DPS",
            DEFAULT_MAX_GYRO_DPS,
            0.0,
        )
        self.mac_address = self._make_source_id()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._state = ImuState(self.node_id)
        self._last_emitted_counts = {"accel": 0, "gyro": 0, "quat": 0}
        self._latest_sample: Optional[
            tuple[tuple[float, float, float, float, float, float, float, float, float, float], float, int]
        ] = None
        self._seq = 0
        self._frame_count = 0
        self._connected = False
        self._measuring = False
        self._measurement_enabled = False
        self._last_error = ""
        self._use_filters = _env_enabled("GAIT_MI1_CAN_USE_FILTERS", False)

    @property
    def last_error(self) -> str:
        return self._last_error

    def _make_source_id(self) -> str:
        return (
            f"wired:mi1:{self.interface}:{self.side}:"
            f"{format_node_id(self.node_id)}:{self.protocol}"
        )

    def set_measurement_enabled(self, enabled: bool) -> bool:
        target = bool(enabled)
        with self._lock:
            self._measurement_enabled = target
            self._measuring = bool(self._connected and target)
        return True

    def start(self) -> bool:
        if not hasattr(socket, "AF_CAN") or not hasattr(socket, "CAN_RAW"):
            self._last_error = "SocketCAN is not available on this system"
            return False
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._connected = True
                self._measuring = bool(self._measurement_enabled)
                return True

        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.settimeout(0.2)
            if self._use_filters:
                self._install_filters(sock)
            sock.bind((self.interface,))
        except OSError as exc:
            self._last_error = self._socket_error_text(exc)
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
            self._state = ImuState(self.node_id)
            self._last_emitted_counts = {"accel": 0, "gyro": 0, "quat": 0}
            self._latest_sample = None
            self._frame_count = 0
            self._seq = 0
            self._connected = True
            self._measuring = bool(self._measurement_enabled)
            self._last_error = ""
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._read_loop,
                name=f"mi1-{self.side}-{self.interface}-{format_node_id(self.node_id)}",
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
        with self._lock:
            should_measure = bool(self._measurement_enabled)
        self.stop()
        with self._lock:
            self._measurement_enabled = should_measure
        return self.start() if should_measure else False

    def get_latest_sample_6d(self):
        with self._lock:
            return self._latest_sample

    def get_latest_sample_10d(self):
        return self.get_latest_sample_6d()

    def available_fields_text(self) -> str:
        with self._lock:
            return self._state.available_fields_text()

    def _socket_error_text(self, exc: OSError) -> str:
        if exc.errno == errno.ENODEV:
            return f"CAN interface {self.interface!r} does not exist or is not up"
        if exc.errno in (errno.EPERM, errno.EACCES):
            return f"permission denied opening {self.interface!r}; run with CAN access"
        return f"open SocketCAN {self.interface} failed: {exc}"

    def _install_filters(self, sock: socket.socket) -> None:
        sol_can_raw = getattr(socket, "SOL_CAN_RAW", None)
        can_raw_filter = getattr(socket, "CAN_RAW_FILTER", None)
        if sol_can_raw is None or can_raw_filter is None:
            return

        filter_pairs: list[tuple[int, int]] = []
        if self.protocol in ("auto", "canopen"):
            for can_id in (
                CANOPEN_TPDO_ACCEL_BASE + self.node_id,
                CANOPEN_TPDO_GYRO_BASE + self.node_id,
                CANOPEN_TPDO_QUAT_BASE + self.node_id,
            ):
                filter_pairs.append((can_id, CAN_SFF_MASK))
        if self.protocol in ("auto", "j1939"):
            for pgn in (J1939_PGN_ACCEL, J1939_PGN_GYRO, J1939_PGN_QUAT):
                # Match extended flag plus PGN/source, ignoring priority/reserved bits.
                can_id = CAN_EFF_FLAG | ((int(pgn) << 8) | self.node_id)
                mask = CAN_EFF_FLAG | 0x01FFFFFF
                filter_pairs.append((can_id, mask))

        filter_blob = b"".join(struct.pack("=II", can_id, can_mask) for can_id, can_mask in filter_pairs)
        if filter_blob:
            sock.setsockopt(sol_can_raw, can_raw_filter, filter_blob)

    def _invalid_sample_reason(
        self,
        sample: tuple[float, float, float, float, float, float, float, float, float, float],
    ) -> str:
        if not self.reject_invalid_samples:
            return ""
        if not all(math.isfinite(value) for value in sample):
            return "non_finite"

        acc_norm = math.sqrt(sample[0] * sample[0] + sample[1] * sample[1] + sample[2] * sample[2])
        if self.min_accel_norm_g > 0.0 and acc_norm < self.min_accel_norm_g:
            return f"acc_norm<{self.min_accel_norm_g:.2f}g"
        if self.max_accel_norm_g > 0.0 and acc_norm > self.max_accel_norm_g:
            return f"acc_norm>{self.max_accel_norm_g:.2f}g"

        max_abs_gyro = max(abs(sample[3]), abs(sample[4]), abs(sample[5]))
        if self.max_gyro_dps > 0.0 and max_abs_gyro > self.max_gyro_dps:
            return f"gyro_abs>{self.max_gyro_dps:.1f}dps"

        quat_norm = math.sqrt(
            sample[6] * sample[6]
            + sample[7] * sample[7]
            + sample[8] * sample[8]
            + sample[9] * sample[9]
        )
        if self.min_quat_norm > 0.0 and quat_norm < self.min_quat_norm:
            return f"quat_norm<{self.min_quat_norm:.2f}"
        if self.max_quat_norm > 0.0 and quat_norm > self.max_quat_norm:
            return f"quat_norm>{self.max_quat_norm:.2f}"
        return ""

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
                self._last_error = f"MI1 CAN frame parse failed: {exc}"

        with self._lock:
            self._connected = False
            self._measuring = False

    def _handle_raw_frame(self, raw_frame: bytes, arrival_time: float) -> None:
        decoded_frame = decode_socketcan_frame(raw_frame)
        if decoded_frame is None:
            return

        can_id, is_extended, payload = decoded_frame
        decoded = decode_imu_frame(self.protocol, can_id, is_extended, payload)
        if decoded is None or int(decoded.node_id) != self.node_id:
            return

        self._state.update(decoded, float(arrival_time))
        self._frame_count += 1
        if not self._state.has_new_complete_sample(
            float(arrival_time),
            self.max_field_age_sec,
            self.max_field_span_sec,
            self._last_emitted_counts,
        ):
            return

        sample = self._state.to_phase_sample()
        invalid_reason = self._invalid_sample_reason(sample)
        if invalid_reason:
            self._state.mark_sample_emitted(self._last_emitted_counts)
            self._last_error = f"MI1 invalid sample: {invalid_reason}"
            return

        self._state.mark_sample_emitted(self._last_emitted_counts)
        with self._lock:
            self._seq += 1
            self._latest_sample = (sample, float(arrival_time), int(self._seq))
            self._last_error = ""
