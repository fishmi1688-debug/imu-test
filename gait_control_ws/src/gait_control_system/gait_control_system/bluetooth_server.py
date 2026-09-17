#!/usr/bin/env python3
"""BLE Notify transport for gait telemetry and control."""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import struct
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, Iterable, List, Optional, Tuple

from bluezero import peripheral as bluezero_peripheral
from gi.repository import GLib

DEFAULT_GAIT_BT_UUID = "f9c2d0b4-9c48-4d4a-925b-0c42f3a9b002"
DEFAULT_GAIT_BT_NAME = "GaitControl"
DEFAULT_GAIT_BT_BIND_ADDR = ""
GAIT_NOTIFY_CHAR_UUID = "f9c2d0b4-9c48-4d4a-925b-0c42f3a9b003"
GAIT_CONTROL_CHAR_UUID = "f9c2d0b4-9c48-4d4a-925b-0c42f3a9b004"
CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"
HEX_TEXT_PREFIX = "HX:"
HEX_BINARY_PREFIX = "HB:"
MAX_NOTIFY_CHUNK_BYTES = 20
DEFAULT_SEND_QUEUE_MAX_ITEMS = 64
FRAME_MAGIC = b"GBF1"
FRAME_HEADER_SIZE = 8
FRAME_HEADER_STRUCT = struct.Struct("<4sBBH")
FRAME_MAX_PAYLOAD_BYTES = 8192
CONTROL_KIND_REQUEST = 0x10
CONTROL_KIND_RESPONSE = 0x11
CONTROL_VERSION = 1
CONTROL_TYPE_INT32 = 1
CONTROL_TYPE_FLOAT32 = 2
CONTROL_TYPE_BOOL = 3
CONTROL_TYPE_BYTES = 4
TYPE_PING = 1
TYPE_GET_STATE = 2
TYPE_SET_MODE = 3
TYPE_SET_PARAMS = 4
TYPE_COMMAND = 5
TYPE_SET_STREAM = 6
TYPE_IMU_MANAGE = 7
TYPE_STATE = 8
TYPE_PLOT = 9
TYPE_PLOT_BATCH = 10
TYPE_ACK = 11
TYPE_ERROR = 12
TYPE_PONG = 13
TYPE_IMU_MANAGE_ACK = 15
ACTION_START_ASSIST = 1
ACTION_STAIRS_DOWN_ASSIST = 2
ACTION_SET_STREAM = 3
ACTION_SET_MODE = 4
ACTION_SET_PARAMS = 5
CONTROL_REQ_MSG_MIN = 1
CONTROL_REQ_MSG_MAX = 7
CONTROL_FIELD_MODE = 1
CONTROL_FIELD_COMMAND = 2
CONTROL_FIELD_VALUE = 3
CONTROL_FIELD_PLOT_FORMAT = 4
CONTROL_FIELD_PLOT_MODE = 5
CONTROL_FIELD_PLOT_BATCH_SIZE = 6
CONTROL_FIELD_PLOT_EVERY_N = 7
CONTROL_FIELD_IMU_SLOT = 8
CONTROL_FIELD_IMU_CONNECT = 9
CONTROL_FIELD_IMU_MAC_BYTES = 10
CONTROL_FIELD_PARAM_UPDATES = 11
CONTROL_FIELD_ALLOW_OFFMODE = 12
CONTROL_FIELD_RESULT = 20
CONTROL_FIELD_ACTION = 21
CONTROL_FIELD_MODE_APPLIED = 22
CONTROL_FIELD_ENABLED = 23
CONTROL_FIELD_REASON = 24
CONTROL_FIELD_PLOT_FORMAT_APPLIED = 25
CONTROL_FIELD_PLOT_MODE_APPLIED = 26
CONTROL_FIELD_PLOT_BATCH_APPLIED = 27
CONTROL_FIELD_PLOT_EVERY_N_APPLIED = 28
CONTROL_FIELD_PARAM_COUNT = 29
CONTROL_FIELD_MODE_OVERRIDDEN = 30
CONTROL_FIELD_ERROR_CODE = 31
CONTROL_FIELD_IMU_CONNECTED = 32
CONTROL_FIELD_IMU_MEASURING = 33
CONTROL_FIELD_IMU_READY = 34
CONTROL_FIELD_IMU_STALE = 35
CONTROL_FIELD_STATE_VERSION = 40
CONTROL_FIELD_STATE_MODE = 41
CONTROL_FIELD_STATE_GAIT = 42
CONTROL_FIELD_STATE_FLAGS = 43
CONTROL_FIELD_STATE_SCORE = 44
CONTROL_FIELD_STATE_PARAMS = 45
CONTROL_FIELD_IMU_PHASE_LEFT_CONNECTED = 46
CONTROL_FIELD_IMU_PHASE_LEFT_MEASURING = 47
CONTROL_FIELD_IMU_PHASE_LEFT_READY = 48
CONTROL_FIELD_IMU_PHASE_LEFT_STALE = 49
CONTROL_FIELD_IMU_PHASE_RIGHT_CONNECTED = 50
CONTROL_FIELD_IMU_PHASE_RIGHT_MEASURING = 51
CONTROL_FIELD_IMU_PHASE_RIGHT_READY = 52
CONTROL_FIELD_IMU_PHASE_RIGHT_STALE = 53
FLAG_PHASE_ACTIVE = 0
FLAG_ASSIST_ENABLED = 3
FLAG_ASSIST_ARMED = 4
FLAG_ASSIST_OUTPUT_ACTIVE = 5
FLAG_MECHANICAL_ZERO_READY = 6
FLAG_MOTION_CONFIRMED = 7
FLAG_TEST_LEFT_PHASE_VALID = 9
FLAG_TEST_RIGHT_PHASE_VALID = 10
FLAG_TEST_LEFT_ASSIST_READY = 11
FLAG_TEST_RIGHT_ASSIST_READY = 12
FLAG_IMU_CONNECTED = 13
FLAG_IMU_READY = 14
MODE_KEYS = [
    "walking",
    "stairs_up",
    "stairs_down",
    "test",
    "walking_test",
    "cycling",
    "uphill",
    "downhill",
    "imu_phase",
    "imu_left_phase",
    "model_phase",
    "imu_left_ao_phase",
    "imu_ao_phase",
    "walking_diff_test",
]


@dataclass
class StreamSettings:
    plot_format: int = 2
    plot_mode: int = 0
    batch_size: int = 10
    every_n: int = 25
    mode_code: int = 0


@dataclass
class _QueuedNotify:
    payload: bytes
    soft_realtime: bool
    payload_desc: str
    session_id: int


def _default_log(message: str) -> None:
    print(message, flush=True)


def _env_int(name: str, default: int, minimum: int = 1) -> int:
    try:
        value = int(os.environ.get(name, str(default)))
    except Exception:
        return max(minimum, int(default))
    return max(minimum, value)


_MAC_ADDR_RE = re.compile(r"^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


def _describe_socket_error(exc: OSError) -> str:
    errno_text = f"errno={exc.errno}" if getattr(exc, "errno", None) is not None else "errno=?"
    text = str(exc).strip() or exc.__class__.__name__
    return f"{exc.__class__.__name__}({errno_text}, {text})"


def _pack_i32(value: int) -> bytes:
    return struct.pack("<i", int(value))


def _pack_f32(value: float) -> bytes:
    return struct.pack("<f", float(value))


def _parse_i32(raw: bytes) -> Optional[int]:
    if len(raw) != 4:
        return None
    try:
        return int(struct.unpack("<i", raw)[0])
    except Exception:
        return None


def _parse_f32(raw: bytes) -> Optional[float]:
    if len(raw) != 4:
        return None
    try:
        return float(struct.unpack("<f", raw)[0])
    except Exception:
        return None


def _parse_bool(raw: bytes) -> Optional[int]:
    if len(raw) < 1:
        return None
    return 1 if (raw[0] & 0xFF) != 0 else 0


def _decode_param_updates_bytes(raw: bytes) -> list[list[float]]:
    updates: list[list[float]] = []
    if not raw or (len(raw) % 5) != 0:
        return updates
    offset = 0
    while offset + 5 <= len(raw):
        code = int(raw[offset] & 0xFF)
        value = _parse_f32(raw[offset + 1: offset + 5])
        if value is not None and code > 0:
            updates.append([code, value])
        offset += 5
    return updates


def _encode_param_updates_bytes(items: object) -> bytes:
    if not isinstance(items, list):
        return b""
    out = bytearray()
    for row in items:
        if not isinstance(row, (list, tuple)) or len(row) < 2:
            continue
        try:
            code = int(row[0])
            value = float(row[1])
        except Exception:
            continue
        if code <= 0 or code > 255:
            continue
        out.append(code & 0xFF)
        out.extend(_pack_f32(value))
    return bytes(out)


def _parse_binary_frame_header(buffer: bytes) -> Optional[tuple[int, int, int, int]]:
    if len(buffer) < FRAME_HEADER_SIZE:
        return None
    if buffer[:4] != FRAME_MAGIC:
        return None
    try:
        _magic, kind, count, payload_len = FRAME_HEADER_STRUCT.unpack_from(buffer, 0)
    except Exception:
        return None
    if payload_len < 0 or payload_len > FRAME_MAX_PAYLOAD_BYTES:
        return None
    total_len = FRAME_HEADER_SIZE + payload_len
    return int(kind), int(count), int(payload_len), int(total_len)


def _looks_like_binary_prefix(buffer: bytes) -> bool:
    if not buffer:
        return False
    prefix_len = min(len(buffer), len(FRAME_MAGIC))
    return buffer[:prefix_len] == FRAME_MAGIC[:prefix_len]


def _build_control_response_frame(payload_obj: dict) -> Optional[bytes]:
    try:
        msg_type = int(payload_obj.get("t", -1))
    except Exception:
        return None
    if msg_type <= 0:
        return None
    fields: list[tuple[int, int, bytes]] = []

    def add_i32(field_id: int, key: str) -> None:
        if key not in payload_obj:
            return
        try:
            value = int(payload_obj.get(key))
        except Exception:
            return
        fields.append((field_id, CONTROL_TYPE_INT32, _pack_i32(value)))

    def add_f32(field_id: int, key: str) -> None:
        if key not in payload_obj:
            return
        try:
            value = float(payload_obj.get(key))
        except Exception:
            return
        fields.append((field_id, CONTROL_TYPE_FLOAT32, _pack_f32(value)))

    def add_bool(field_id: int, key: str) -> None:
        add_bool_any(field_id, key)

    def add_bool_any(field_id: int, *keys: str) -> None:
        value = None
        for key in keys:
            if key in payload_obj:
                value = payload_obj.get(key)
                break
        else:
            return
        if isinstance(value, bool):
            out = 1 if value else 0
        else:
            try:
                out = 1 if int(value) != 0 else 0
            except Exception:
                return
        fields.append((field_id, CONTROL_TYPE_BOOL, bytes([out])))

    add_bool(CONTROL_FIELD_RESULT, "o")
    add_i32(CONTROL_FIELD_ACTION, "a")
    add_i32(CONTROL_FIELD_MODE_APPLIED, "m")
    add_bool(CONTROL_FIELD_ENABLED, "e")
    add_i32(CONTROL_FIELD_REASON, "r")
    add_i32(CONTROL_FIELD_PLOT_FORMAT_APPLIED, "pf")
    add_i32(CONTROL_FIELD_PLOT_MODE_APPLIED, "pm")
    add_i32(CONTROL_FIELD_PLOT_BATCH_APPLIED, "pn")
    add_i32(CONTROL_FIELD_PLOT_EVERY_N_APPLIED, "pe")
    add_i32(CONTROL_FIELD_PARAM_COUNT, "n")
    add_bool(CONTROL_FIELD_MODE_OVERRIDDEN, "ov")
    add_i32(CONTROL_FIELD_ERROR_CODE, "ec")
    add_i32(CONTROL_FIELD_IMU_SLOT, "s")
    add_bool(CONTROL_FIELD_IMU_CONNECTED, "k")
    add_bool(CONTROL_FIELD_IMU_MEASURING, "q")
    add_bool(CONTROL_FIELD_IMU_READY, "d")
    add_bool(CONTROL_FIELD_IMU_STALE, "z")
    add_i32(CONTROL_FIELD_STATE_VERSION, "v")
    add_i32(CONTROL_FIELD_STATE_MODE, "mi")
    add_i32(CONTROL_FIELD_STATE_GAIT, "gs")
    add_i32(CONTROL_FIELD_STATE_FLAGS, "f")
    add_f32(CONTROL_FIELD_STATE_SCORE, "ds")
    add_i32(CONTROL_FIELD_COMMAND, "c")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_LEFT_CONNECTED, "iplc", "imu_phase_left_connected")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_LEFT_MEASURING, "iplq", "imu_phase_left_measuring")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_LEFT_READY, "ipld", "imu_phase_left_ready")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_LEFT_STALE, "iplz", "imu_phase_left_stale")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_RIGHT_CONNECTED, "iprc", "imu_phase_right_connected")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_RIGHT_MEASURING, "iprq", "imu_phase_right_measuring")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_RIGHT_READY, "iprd", "imu_phase_right_ready")
    add_bool_any(CONTROL_FIELD_IMU_PHASE_RIGHT_STALE, "iprz", "imu_phase_right_stale")

    if "u" in payload_obj:
        raw_updates = _encode_param_updates_bytes(payload_obj.get("u"))
        if raw_updates:
            fields.append((CONTROL_FIELD_STATE_PARAMS, CONTROL_TYPE_BYTES, raw_updates))

    payload = bytearray()
    payload.append(CONTROL_VERSION & 0xFF)
    payload.append(msg_type & 0xFF)
    payload.append(len(fields) & 0xFF)
    for field_id, value_type, raw in fields:
        raw_len = min(len(raw), 0xFFFF)
        payload.append(field_id & 0xFF)
        payload.append(value_type & 0xFF)
        payload.append(raw_len & 0xFF)
        payload.append((raw_len >> 8) & 0xFF)
        payload.extend(raw[:raw_len])
    payload_len = len(payload)
    if payload_len > FRAME_MAX_PAYLOAD_BYTES:
        return None
    header = FRAME_HEADER_STRUCT.pack(
        FRAME_MAGIC,
        CONTROL_KIND_RESPONSE & 0xFF,
        1,
        payload_len,
    )
    return header + bytes(payload)


def _decode_control_request_frame(frame: bytes) -> Optional[dict]:
    header = _parse_binary_frame_header(frame)
    if header is None:
        return None
    kind, _count, payload_len, total_len = header
    if kind != CONTROL_KIND_REQUEST or len(frame) != total_len:
        return None
    payload = frame[FRAME_HEADER_SIZE: FRAME_HEADER_SIZE + payload_len]
    if len(payload) < 3:
        return None
    version = payload[0] & 0xFF
    if version != CONTROL_VERSION:
        return None
    msg_type = payload[1] & 0xFF
    if msg_type < CONTROL_REQ_MSG_MIN or msg_type > CONTROL_REQ_MSG_MAX:
        return None
    field_count = payload[2] & 0xFF
    offset = 3
    decoded: dict[str, object] = {"t": int(msg_type)}
    for _ in range(field_count):
        if offset + 4 > len(payload):
            return None
        field_id = payload[offset] & 0xFF
        value_type = payload[offset + 1] & 0xFF
        value_len = int(payload[offset + 2] | (payload[offset + 3] << 8))
        offset += 4
        if offset + value_len > len(payload):
            return None
        raw = payload[offset: offset + value_len]
        offset += value_len
        if field_id == CONTROL_FIELD_MODE:
            value = _parse_i32(raw)
            if value is not None:
                decoded["m"] = value
        elif field_id == CONTROL_FIELD_COMMAND:
            value = _parse_i32(raw)
            if value is not None:
                decoded["c"] = value
        elif field_id == CONTROL_FIELD_VALUE:
            value = _parse_bool(raw)
            if value is not None:
                decoded["v"] = value
        elif field_id == CONTROL_FIELD_PLOT_FORMAT:
            value = _parse_i32(raw)
            if value is not None:
                decoded["pf"] = value
        elif field_id == CONTROL_FIELD_PLOT_MODE:
            value = _parse_i32(raw)
            if value is not None:
                decoded["pm"] = value
        elif field_id == CONTROL_FIELD_PLOT_BATCH_SIZE:
            value = _parse_i32(raw)
            if value is not None:
                decoded["pn"] = value
        elif field_id == CONTROL_FIELD_PLOT_EVERY_N:
            value = _parse_i32(raw)
            if value is not None:
                decoded["pe"] = value
        elif field_id == CONTROL_FIELD_IMU_SLOT:
            value = _parse_i32(raw)
            if value is not None:
                decoded["s"] = value
        elif field_id == CONTROL_FIELD_IMU_CONNECT:
            value = _parse_bool(raw)
            if value is not None:
                decoded["x"] = value
        elif field_id == CONTROL_FIELD_IMU_MAC_BYTES and value_type == CONTROL_TYPE_BYTES:
            decoded["ma"] = [int(b & 0xFF) for b in raw]
        elif field_id == CONTROL_FIELD_PARAM_UPDATES and value_type == CONTROL_TYPE_BYTES:
            decoded["u"] = _decode_param_updates_bytes(raw)
        elif field_id == CONTROL_FIELD_ALLOW_OFFMODE:
            value = _parse_bool(raw)
            if value is not None:
                decoded["ao"] = value
    return decoded


def _is_soft_realtime_payload(payload_desc: str) -> bool:
    return (
        payload_desc == "plot"
        or payload_desc == "state"
        or payload_desc == "plot_batch"
        or payload_desc.startswith("plot_batch[")
        or payload_desc.startswith("plot_bin")
    )


def _describe_payload(message: str) -> str:
    text = message.strip()
    if not text:
        return "empty"
    if text.lower() == "ping":
        return "ping"
    try:
        payload = json.loads(text)
    except Exception:
        return "raw"
    payload_type = str(payload.get("type", "")).strip()
    if not payload_type:
        try:
            payload_type = {
                1: "ping",
                2: "get_state",
                3: "set_mode",
                4: "set_params",
                5: "command",
                6: "set_stream",
                7: "imu_manage",
                8: "state",
                9: "plot",
                10: "plot_batch",
                11: "ack",
                12: "error",
                13: "pong",
                14: "telemetry",
                15: "imu_manage_ack",
                16: "imu_manage_status",
            }.get(int(payload.get("t", -1)), "unknown")
        except Exception:
            payload_type = "unknown"
    if payload_type == "plot_batch":
        frames = payload.get("frames")
        if isinstance(frames, list):
            return f"plot_batch[{len(frames)}]"
        compact_frames = payload.get("f")
        if isinstance(compact_frames, list):
            return f"plot_batch[{len(compact_frames)}]"
    return payload_type


def _encode_text_transport(message: str, hex_protocol: bool) -> str:
    if not hex_protocol:
        return message
    payload = message.encode("utf-8")
    return HEX_TEXT_PREFIX + payload.hex().upper()


def _encode_binary_transport(payload: bytes, hex_protocol: bool) -> Optional[str]:
    if not hex_protocol:
        return None
    return HEX_BINARY_PREFIX + payload.hex().upper()


def _decode_transport_line(line: str, hex_protocol: bool) -> tuple[Optional[str], Optional[bytes]]:
    text = line.strip()
    if not text:
        return None, None
    if not hex_protocol:
        return "text", text.encode("utf-8")
    prefix_upper = text[:3].upper()
    if prefix_upper == HEX_TEXT_PREFIX:
        hex_text = text[3:].strip()
        try:
            return "text", bytes.fromhex(hex_text)
        except Exception:
            return None, None
    if prefix_upper == HEX_BINARY_PREFIX:
        hex_text = text[3:].strip()
        try:
            return "binary", bytes.fromhex(hex_text)
        except Exception:
            return None, None
    return "text", text.encode("utf-8")


def _detect_local_bt_addr() -> Optional[str]:
    commands = [
        ["bluetoothctl", "show"],
        ["hciconfig", "-a"],
    ]
    patterns = [
        re.compile(r"^Controller\s+([0-9A-Fa-f:]{17})\b"),
        re.compile(r"\bBD Address:\s*([0-9A-Fa-f:]{17})\b"),
    ]
    for cmd in commands:
        if shutil.which(cmd[0]) is None:
            continue
        try:
            result = subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except Exception:
            continue
        for line in result.stdout.splitlines():
            for pattern in patterns:
                match = pattern.search(line.strip())
                if match:
                    addr = match.group(1).upper()
                    if _MAC_ADDR_RE.match(addr):
                        return addr
    return None


def _normalize_bind_addr(
    raw_addr: Optional[str],
    log_warn: Callable[[str], None],
    any_addr: str = DEFAULT_GAIT_BT_BIND_ADDR,
) -> str:
    if raw_addr is None:
        detected = _detect_local_bt_addr()
        if detected:
            return detected
        return any_addr
    addr = raw_addr.strip()
    if not addr or addr.lower() in ("any", "bdaddr_any", "00:00:00:00:00:00"):
        detected = _detect_local_bt_addr()
        if detected:
            return detected
        return any_addr
    if not _MAC_ADDR_RE.match(addr):
        log_warn(f"⚠️ GAIT_BT_BIND_ADDR 无效: {raw_addr}，将使用自动检测结果")
        detected = _detect_local_bt_addr()
        if detected:
            return detected
        return any_addr
    return addr


class BluetoothBleNotifyServer:
    def __init__(
        self,
        service_uuid: str = DEFAULT_GAIT_BT_UUID,
        service_name: str = DEFAULT_GAIT_BT_NAME,
        bind_addr: Optional[str] = None,
        hex_protocol: Optional[bool] = None,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        log_error: Optional[Callable[[str], None]] = None,
        on_message: Optional[Callable[[str], Optional[str]]] = None,
    ) -> None:
        self._service_uuid = service_uuid
        self._service_name = service_name
        self._bind_addr_raw = bind_addr
        self._hex_protocol = os.environ.get("GAIT_BT_HEX_PROTOCOL", "1") != "0" if hex_protocol is None else bool(hex_protocol)
        self._log_info = log_info or _default_log
        self._log_warn = log_warn or self._log_info
        self._log_error = log_error or self._log_info
        self._on_message = on_message
        self._peripheral: Optional[bluezero_peripheral.Peripheral] = None
        self._data_characteristic = None
        self._control_characteristic = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._client_connected = threading.Event()
        self._notifications_enabled = threading.Event()
        self._send_queue: Deque[_QueuedNotify] = deque()
        self._send_lock = threading.Lock()
        self._send_scheduled = False
        self._notify_chunk_size = _env_int("GAIT_BT_NOTIFY_CHUNK", MAX_NOTIFY_CHUNK_BYTES)
        self._send_queue_max_items = _env_int("GAIT_BT_SEND_QUEUE_MAX", DEFAULT_SEND_QUEUE_MAX_ITEMS)
        self._dropped_soft_realtime_items = 0
        self._notify_session_id = 0
        self._handshake_logged = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._client_connected.clear()
        self._notifications_enabled.clear()
        self._handshake_logged = False
        with self._send_lock:
            self._notify_session_id += 1
            self._send_queue.clear()
            self._send_scheduled = False
        peripheral = self._peripheral
        if peripheral is not None:
            try:
                peripheral.mainloop.quit()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def is_client_connected(self) -> bool:
        return self._client_connected.is_set() and self._notifications_enabled.is_set()

    def send_line(self, line: str) -> bool:
        if not self.is_client_connected():
            return False
        payload_desc = _describe_payload(line)
        payload = self._try_encode_binary_status_payload(line, payload_desc)
        if payload is None:
            wire_line = _encode_text_transport(line, self._hex_protocol)
            payload = (wire_line + "\n").encode("utf-8")
        return self.send_bytes(payload, payload_desc=payload_desc, soft_realtime=_is_soft_realtime_payload(payload_desc))

    def _try_encode_binary_status_payload(self, line: str, payload_desc: str) -> Optional[bytes]:
        if payload_desc.startswith("plot"):
            return None
        try:
            payload_obj = json.loads(line)
        except Exception:
            return None
        if not isinstance(payload_obj, dict):
            return None
        return _build_control_response_frame(payload_obj)

    def send_bytes(
        self,
        payload: bytes,
        payload_desc: str = "binary",
        soft_realtime: bool = True,
    ) -> bool:
        if not self.is_client_connected():
            return False
        if not payload:
            return True
        chunks = self._split_notify_payload(bytes(payload))
        if not chunks:
            return True
        with self._send_lock:
            session_id = self._notify_session_id
            if soft_realtime:
                dropped = self._drop_queued_soft_realtime_items()
                if dropped:
                    self._note_dropped_soft_realtime(dropped)
            else:
                dropped = self._drop_queued_soft_realtime_items_if_needed(len(chunks))
                if dropped:
                    self._note_dropped_soft_realtime(dropped)

            for chunk in chunks:
                self._send_queue.append(
                    _QueuedNotify(
                        payload=chunk,
                        soft_realtime=soft_realtime,
                        payload_desc=payload_desc,
                        session_id=session_id,
                    )
                )
            if self._send_scheduled:
                return True
            self._send_scheduled = True
        GLib.idle_add(self._drain_send_queue)
        return True

    def _split_notify_payload(self, payload: bytes) -> List[bytes]:
        chunk_size = max(1, int(self._notify_chunk_size))
        if len(payload) <= chunk_size:
            return [payload]
        return [
            payload[offset: offset + chunk_size]
            for offset in range(0, len(payload), chunk_size)
        ]

    def _drop_queued_soft_realtime_items(self) -> int:
        if not self._send_queue:
            return 0
        original_len = len(self._send_queue)
        self._send_queue = deque(
            item for item in self._send_queue
            if not item.soft_realtime
        )
        return original_len - len(self._send_queue)

    def _drop_queued_soft_realtime_items_if_needed(self, incoming_count: int) -> int:
        if len(self._send_queue) + max(0, incoming_count) <= self._send_queue_max_items:
            return 0
        return self._drop_queued_soft_realtime_items()

    def _note_dropped_soft_realtime(self, count: int) -> None:
        if count <= 0:
            return
        self._dropped_soft_realtime_items += int(count)
        dropped = self._dropped_soft_realtime_items
        if dropped == count or dropped % 100 == 0:
            self._log_warn(f"⚠️ BLE实时数据队列拥塞，已丢弃旧实时分片: total={dropped}")

    def _drain_send_queue(self) -> bool:
        if self._stop_event.is_set():
            with self._send_lock:
                self._send_queue.clear()
                self._send_scheduled = False
            return False
        if not self.is_client_connected():
            with self._send_lock:
                self._send_queue.clear()
                self._send_scheduled = False
            return False
        item = None
        with self._send_lock:
            if self._send_queue:
                item = self._send_queue.popleft()
            else:
                self._send_scheduled = False
                return False
            if item.session_id != self._notify_session_id:
                return True
        try:
            if self._data_characteristic is not None:
                self._data_characteristic.set_value(list(item.payload))
        except Exception as exc:
            payload_desc = item.payload_desc if item is not None else "unknown"
            self._log_warn(f"⚠️ BLE Notify发送失败(type={payload_desc}): {_describe_socket_error(exc) if isinstance(exc, OSError) else exc}")
            with self._send_lock:
                self._send_queue.clear()
                self._send_scheduled = False
            return False
        with self._send_lock:
            if not self._send_queue:
                self._send_scheduled = False
                return False
        return True

    def _run(self) -> None:
        if os.environ.get("GAIT_BT_SETUP_IN_SERVER", "1") != "0":
            setup_bluetooth_adapter(self._service_name)

        adapter_addr = _normalize_bind_addr(self._bind_addr_raw, self._log_warn)
        if not adapter_addr:
            self._log_error("❌ 无法定位本机蓝牙适配器，BLE 服务无法启动")
            return

        try:
            peripheral = bluezero_peripheral.Peripheral(
                adapter_addr,
                local_name=self._service_name,
            )
        except Exception as exc:
            self._log_error(f"❌ 创建BLE Peripheral失败: {exc}")
            return

        self._peripheral = peripheral
        peripheral.add_service(1, self._service_uuid, True)
        peripheral.add_characteristic(
            1,
            1,
            GAIT_NOTIFY_CHAR_UUID,
            [],
            False,
            ["read", "notify"],
            read_callback=self._read_notify_value,
            notify_callback=self._on_notify_state_changed,
        )
        peripheral.add_descriptor(
            1,
            1,
            1,
            CCCD_UUID,
            [0x00, 0x00],
            ["read", "write"],
        )
        peripheral.add_characteristic(
            1,
            2,
            GAIT_CONTROL_CHAR_UUID,
            [],
            False,
            ["write", "write-without-response"],
            write_callback=self._on_control_write,
        )

        self._data_characteristic = peripheral.characteristics[0]
        self._control_characteristic = peripheral.characteristics[1]

        peripheral.on_connect = self._on_connect
        peripheral.on_disconnect = self._on_disconnect

        self._log_info(
            f"📡 蓝牙BLE Notify服务已启动: name={self._service_name}, uuid={self._service_uuid}, adapter={adapter_addr}"
        )
        try:
            peripheral.publish()
        except KeyboardInterrupt:
            pass
        except Exception as exc:
            self._log_error(f"❌ BLE服务运行失败: {exc}")
        finally:
            self._client_connected.clear()
            self._notifications_enabled.clear()
            with self._send_lock:
                self._notify_session_id += 1
                self._send_queue.clear()
                self._send_scheduled = False

    def _on_connect(self, device, *args) -> None:
        self._log_info(f"🔗 蓝牙设备已连接: {device}")

    def _on_disconnect(self, device, *args) -> None:
        self._client_connected.clear()
        self._notifications_enabled.clear()
        self._handshake_logged = False
        with self._send_lock:
            self._notify_session_id += 1
            self._send_queue.clear()
            self._send_scheduled = False
        self._log_info(f"🔌 蓝牙连接已断开: {device}")

    def _read_notify_value(self, *_args) -> list[int]:
        return []

    def _on_notify_state_changed(self, notifying, _characteristic) -> None:
        if notifying:
            with self._send_lock:
                self._notify_session_id += 1
                self._send_queue.clear()
                self._send_scheduled = False
            self._handshake_logged = False
            self._notifications_enabled.set()
            self._client_connected.set()
            self._log_info("✅ 已启用BLE Notify，准备发送数据")
            self.send_line(json.dumps({"t": 13, "o": 1}, ensure_ascii=False, separators=(",", ":")))
        else:
            self._notifications_enabled.clear()
            self._client_connected.clear()
            self._handshake_logged = False
            with self._send_lock:
                self._notify_session_id += 1
                self._send_queue.clear()
                self._send_scheduled = False
            self._log_info("ℹ️ BLE Notify已关闭")

    def _on_control_write(self, value, options) -> None:
        raw = bytes(value)
        if not raw:
            return
        decoded_payload = _decode_control_request_frame(raw)
        if decoded_payload is None:
            try:
                text = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                text = ""
            if not text:
                return
            response = self._on_message(text) if self._on_message else None
        else:
            decoded_line = json.dumps(decoded_payload, ensure_ascii=False, separators=(",", ":"))
            response = None
            if self._on_message:
                try:
                    response = self._on_message(decoded_line)
                except Exception as exc:
                    self._log_warn(f"⚠️ 处理BLE控制帧失败: {exc}")
        if response:
            self.send_line(response)
        if decoded_payload is not None and int(decoded_payload.get("t", 0)) == 1:
            if not self._handshake_logged:
                self._handshake_logged = True
                self._log_info("🤝 BLE握手完成，允许发送plot/state")


def build_default_server(
    log_info: Optional[Callable[[str], None]] = None,
    log_warn: Optional[Callable[[str], None]] = None,
    log_error: Optional[Callable[[str], None]] = None,
    on_message: Optional[Callable[[str], Optional[str]]] = None,
) -> BluetoothBleNotifyServer:
    service_uuid = os.environ.get("GAIT_BT_UUID", DEFAULT_GAIT_BT_UUID)
    service_name = os.environ.get("GAIT_BT_NAME", DEFAULT_GAIT_BT_NAME)
    bind_addr = os.environ.get("GAIT_BT_BIND_ADDR")
    return BluetoothBleNotifyServer(
        service_uuid=service_uuid,
        service_name=service_name,
        bind_addr=bind_addr,
        log_info=log_info,
        log_warn=log_warn,
        log_error=log_error,
        on_message=on_message or _default_message_handler,
    )


def _default_message_handler(message: str) -> Optional[str]:
    text = str(message or "").strip()
    if not text:
        return None
    if text.lower() == "ping":
        return json.dumps({"t": 13, "o": 1}, ensure_ascii=False, separators=(",", ":"))
    try:
        payload = json.loads(text)
    except Exception:
        return None
    payload_type = str(payload.get("type", "")).strip().lower()
    if payload_type == "ping":
        return json.dumps({"t": 13, "o": 1}, ensure_ascii=False, separators=(",", ":"))
    try:
        if int(payload.get("t", -1)) == 1:
            return json.dumps({"t": 13, "o": 1}, ensure_ascii=False, separators=(",", ":"))
    except Exception:
        pass
    return None


def build_plot_frame_batch(
    sample_index: int,
    count: int,
    sample_rate_hz: float,
    wave_hz: float,
    amplitude_deg: float,
) -> bytes:
    count = max(1, min(255, int(count)))
    sample_rate_hz = max(1.0, float(sample_rate_hz))
    now_ms = int(time.time() * 1000.0)
    period_ms = int(round(1000.0 / sample_rate_hz))
    start_ms = now_ms - (count - 1) * period_ms
    payload = bytearray()
    plot_record = struct.Struct("<Qhhhhh")
    for i in range(count):
        idx = sample_index + i
        t = idx / sample_rate_hz
        phase = 2.0 * math.pi * wave_hz * t
        left = amplitude_deg * math.sin(phase)
        right = amplitude_deg * math.sin(phase + math.pi)
        diff = left - right
        phase_value = phase % (2.0 * math.pi)
        assist = 8.0 * max(0.0, math.sin(phase))
        payload.extend(
            plot_record.pack(
                max(0, start_ms + i * period_ms),
                clip_i16(left, 100.0),
                clip_i16(right, 100.0),
                clip_i16(diff, 100.0),
                clip_i16(phase_value, 1000.0),
                clip_i16(assist, 100.0),
            )
        )
    return FRAME_HEADER_STRUCT.pack(FRAME_MAGIC, 2 if count > 1 else 1, count, len(payload)) + bytes(payload)


def build_state_frame(settings: StreamSettings, seq: int) -> bytes:
    flags = 0
    for bit in (
        FLAG_PHASE_ACTIVE,
        FLAG_ASSIST_ENABLED,
        FLAG_ASSIST_ARMED,
        FLAG_MECHANICAL_ZERO_READY,
        FLAG_MOTION_CONFIRMED,
        FLAG_IMU_CONNECTED,
        FLAG_IMU_READY,
    ):
        flags |= 1 << bit
    if (seq // max(1, settings.batch_size)) % 4 in (1, 2):
        flags |= 1 << FLAG_ASSIST_OUTPUT_ACTIVE
    if settings.mode_code in (3, 4, 13):
        flags |= 1 << FLAG_TEST_LEFT_PHASE_VALID
        flags |= 1 << FLAG_TEST_RIGHT_PHASE_VALID
        flags |= 1 << FLAG_TEST_LEFT_ASSIST_READY
        flags |= 1 << FLAG_TEST_RIGHT_ASSIST_READY

    fields = [
        field_i32(CONTROL_FIELD_STATE_VERSION, 1),
        field_i32(CONTROL_FIELD_STATE_MODE, settings.mode_code),
        field_i32(CONTROL_FIELD_STATE_GAIT, 2),
        field_i32(CONTROL_FIELD_STATE_FLAGS, flags),
        field_f32(CONTROL_FIELD_STATE_SCORE, 0.92),
    ]
    return build_control_response_frame(TYPE_STATE, fields)


def build_pong_frame() -> bytes:
    return build_control_response_frame(TYPE_PONG, [field_bool(CONTROL_FIELD_RESULT, True)])


def build_set_stream_ack_frame(settings: StreamSettings) -> bytes:
    fields = [
        field_bool(CONTROL_FIELD_RESULT, True),
        field_i32(CONTROL_FIELD_ACTION, ACTION_SET_STREAM),
        field_i32(CONTROL_FIELD_PLOT_FORMAT_APPLIED, settings.plot_format),
        field_i32(CONTROL_FIELD_PLOT_MODE_APPLIED, settings.plot_mode),
        field_i32(CONTROL_FIELD_PLOT_BATCH_APPLIED, settings.batch_size),
        field_i32(CONTROL_FIELD_PLOT_EVERY_N_APPLIED, settings.every_n),
    ]
    return build_control_response_frame(TYPE_ACK, fields)


def build_simple_ack_frame(action: int, param_count: int = 0) -> bytes:
    fields = [
        field_bool(CONTROL_FIELD_RESULT, True),
        field_i32(CONTROL_FIELD_ACTION, int(action)),
    ]
    if param_count > 0:
        fields.append(field_i32(CONTROL_FIELD_PARAM_COUNT, param_count))
    return build_control_response_frame(TYPE_ACK, fields)


def build_imu_manage_ack_frame(request: Dict[str, object]) -> bytes:
    fields = [
        field_bool(CONTROL_FIELD_RESULT, True),
        field_i32(CONTROL_FIELD_IMU_SLOT, int_or_default(request.get("s"), 0)),
        field_bool(CONTROL_FIELD_IMU_CONNECTED, True),
        field_bool(CONTROL_FIELD_IMU_MEASURING, bool(int_or_default(request.get("x"), 1))),
        field_bool(CONTROL_FIELD_IMU_READY, True),
        field_bool(CONTROL_FIELD_IMU_STALE, False),
    ]
    return build_control_response_frame(TYPE_IMU_MANAGE_ACK, fields)


def build_error_frame(error_code: int) -> bytes:
    return build_control_response_frame(TYPE_ERROR, [field_bool(CONTROL_FIELD_RESULT, False), field_i32(CONTROL_FIELD_ERROR_CODE, error_code)])


def field_i32(field_id: int, value: int) -> tuple[int, int, bytes]:
    return field_id, CONTROL_TYPE_INT32, _pack_i32(value)


def field_f32(field_id: int, value: float) -> tuple[int, int, bytes]:
    return field_id, CONTROL_TYPE_FLOAT32, _pack_f32(value)


def field_bool(field_id: int, value: bool) -> tuple[int, int, bytes]:
    return field_id, CONTROL_TYPE_BOOL, bytes([1 if value else 0])


def build_control_response_frame(msg_type: int, fields: Iterable[Tuple[int, int, bytes]]) -> bytes:
    payload = bytearray()
    payload.append(CONTROL_VERSION & 0xFF)
    payload.append(msg_type & 0xFF)
    fields = list(fields)
    payload.append(len(fields) & 0xFF)
    for field_id, value_type, raw in fields:
        raw = bytes(raw)
        raw_len = min(len(raw), 0xFFFF)
        payload.append(field_id & 0xFF)
        payload.append(value_type & 0xFF)
        payload.append(raw_len & 0xFF)
        payload.append((raw_len >> 8) & 0xFF)
        payload.extend(raw[:raw_len])
    header = FRAME_HEADER_STRUCT.pack(
        FRAME_MAGIC,
        CONTROL_KIND_RESPONSE & 0xFF,
        1,
        len(payload),
    )
    return header + bytes(payload)


def int_or_default(value: object, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def clip_i16(value: float, scale: float) -> int:
    scaled = int(round(float(value) * float(scale)))
    return max(-32768, min(32767, scaled))


def setup_bluetooth_adapter(alias: str = "") -> None:
    commands: List[List[str]] = []
    if shutil.which("rfkill"):
        commands.append(["rfkill", "unblock", "bluetooth"])
    if shutil.which("bluetoothctl"):
        commands.extend(
            [
                ["bluetoothctl", "power", "on"],
                ["bluetoothctl", "pairable", "on"],
                ["bluetoothctl", "discoverable", "on"],
            ]
        )
        if alias.strip():
            commands.append(["bluetoothctl", "system-alias", alias.strip()])
    else:
        print("[warn] bluetoothctl not found; cannot prepare adapter automatically", flush=True)
        return

    for cmd in commands:
        subprocess.run(cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

def main() -> int:
    server = build_default_server()
    _default_log("🔧 启动蓝牙BLE Notify服务（独立模式）...")
    server.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        _default_log("🛑 收到中断，停止蓝牙服务")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
