#!/usr/bin/env python3

import json
import os
import re
import select
import shutil
import socket
import struct
import subprocess
import threading
import time
from queue import Empty, Queue
from typing import Callable, Optional

DEFAULT_GAIT_BT_UUID = "f9c2d0b4-9c48-4d4a-925b-0c42f3a9b002"
DEFAULT_GAIT_BT_NAME = "GaitControl"
DEFAULT_GAIT_BT_CHANNEL = 1
DEFAULT_GAIT_BT_BIND_ADDR = getattr(socket, "BDADDR_ANY", "00:00:00:00:00:00")
SOL_BLUETOOTH = getattr(socket, "SOL_BLUETOOTH", 274)
BT_SECURITY = getattr(socket, "BT_SECURITY", 4)
BT_SECURITY_LOW = getattr(socket, "BT_SECURITY_LOW", 1)

try:
    from bluetooth import (  # type: ignore
        advertise_service,
        BluetoothSocket,
        RFCOMM,
        SERIAL_PORT_CLASS,
        SERIAL_PORT_PROFILE,
        BDADDR_ANY as _PYBLUEZ_ANY,
    )
    _PYBLUEZ_AVAILABLE = True
except Exception:
    advertise_service = None
    BluetoothSocket = None
    RFCOMM = None
    SERIAL_PORT_CLASS = None
    SERIAL_PORT_PROFILE = None
    _PYBLUEZ_ANY = None
    _PYBLUEZ_AVAILABLE = False


def _default_log(message: str) -> None:
    print(message, flush=True)

_MAC_ADDR_RE = re.compile(r"^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


def _is_ping_message(message: str) -> bool:
    text = message.strip()
    if not text:
        return False
    if text.lower() == "ping":
        return True
    try:
        payload = json.loads(text)
    except Exception:
        return False
    return str(payload.get("type", "")).strip().lower() == "ping"


def _describe_socket_error(exc: OSError) -> str:
    errno_text = f"errno={exc.errno}" if getattr(exc, "errno", None) is not None else "errno=?"
    text = str(exc).strip() or exc.__class__.__name__
    return f"{exc.__class__.__name__}({errno_text}, {text})"


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
    payload_type = str(payload.get("type", "unknown")).strip() or "unknown"
    if payload_type == "plot_batch":
        frames = payload.get("frames")
        if isinstance(frames, list):
            return f"plot_batch[{len(frames)}]"
    return payload_type


def _is_soft_realtime_payload(payload_desc: str) -> bool:
    return payload_desc == "plot" or payload_desc == "state" or payload_desc.startswith("plot_batch[")


def _is_drop_tolerant_response(payload_desc: str) -> bool:
    return payload_desc in ("ack", "imu_manage_ack", "state", "pong")


def _read_env_float(name: str, default: float, minimum: float) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return default
    if value < minimum:
        return minimum
    return value


def _normalize_bind_addr(
    raw_addr: Optional[str],
    log_warn: Callable[[str], None],
    any_addr: str = DEFAULT_GAIT_BT_BIND_ADDR,
) -> str:
    if raw_addr is None:
        return any_addr
    addr = raw_addr.strip()
    if not addr or addr.lower() in ("any", "bdaddr_any", "00:00:00:00:00:00"):
        return any_addr
    if not _MAC_ADDR_RE.match(addr):
        log_warn(
            f"⚠️ GAIT_BT_BIND_ADDR 无效: {raw_addr}，将使用 {any_addr}"
        )
        return any_addr
    return addr


def _detect_local_bt_addr() -> Optional[str]:
    """Best-effort lookup of the primary local Bluetooth adapter address."""
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


def _resolve_bind_addr(
    raw_addr: Optional[str],
    log_info: Callable[[str], None],
    log_warn: Callable[[str], None],
    any_addr: str = DEFAULT_GAIT_BT_BIND_ADDR,
) -> str:
    normalized = _normalize_bind_addr(raw_addr, log_warn, any_addr)
    if raw_addr is not None:
        return normalized
    detected = _detect_local_bt_addr()
    if detected:
        log_info(f"📍 蓝牙RFCOMM服务将绑定本机适配器地址: {detected}")
        return detected
    log_warn(f"⚠️ 未检测到本机蓝牙适配器地址，将回退绑定 {normalized}")
    return normalized


def _can_run_sudo_non_interactive() -> bool:
    if os.geteuid() == 0:
        return True
    if shutil.which("sudo") is None:
        return False
    try:
        result = subprocess.run(
            ["sudo", "-n", "true"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return result.returncode == 0
    except Exception:
        return False


def _register_spp_with_sdptool(
    channel: int,
    log_info: Callable[[str], None],
    log_warn: Callable[[str], None],
) -> bool:
    if shutil.which("sdptool") is None:
        log_warn("⚠️ 未找到 sdptool，无法注册 SDP 服务")
        return False
    cmd = ["sdptool", "add", "--channel", str(channel), "SP"]
    if os.geteuid() != 0:
        if _can_run_sudo_non_interactive():
            cmd = ["sudo", "-E"] + cmd
        else:
            log_warn("⚠️ sdptool 需要权限且 sudo -n 不可用，已跳过 SDP 注册")
            return False
    try:
        result = subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        log_info("✅ 已使用 sdptool 注册 SPP 服务 (SDP)")
        if result.stdout.strip():
            log_info(result.stdout.strip())
        return True
    except subprocess.CalledProcessError as exc:
        details = (exc.stderr or exc.stdout or "").strip()
        log_warn(f"⚠️ sdptool 注册失败: {details or exc}")
    except Exception as exc:
        log_warn(f"⚠️ sdptool 执行异常: {exc}")
    return False


def _set_bt_security_low(
    sock_obj: object,
    log_warn: Callable[[str], None],
) -> None:
    """Best-effort RFCOMM low-security mode to allow unpaired/insecure Android connects.

    内核 struct bt_security = { uint8 level; uint8 key_size; } 虽然只有2字节，
    但 setsockopt 对 SOL_BLUETOOTH/BT_SECURITY 的 optlen 要求按 int 对齐（4字节），
    否则 setsockopt 静默失败、安全级别维持默认，导致未配对设备无法传输数据。
    """
    raw_sock = getattr(sock_obj, "_sock", sock_obj)
    if raw_sock is None or not hasattr(raw_sock, "setsockopt"):
        return
    try:
        # 4字节：level(1B) + key_size(1B) + padding(2B) — 避免 setsockopt optlen 校验失败
        raw_sock.setsockopt(
            SOL_BLUETOOTH,
            BT_SECURITY,
            struct.pack("BBBB", BT_SECURITY_LOW, 0, 0, 0),
        )
    except Exception as exc:
        log_warn(f"⚠️ 设置蓝牙低安全级别失败: {exc}")


class BluetoothRfcommServer:
    def __init__(
        self,
        service_uuid: str = DEFAULT_GAIT_BT_UUID,
        service_name: str = DEFAULT_GAIT_BT_NAME,
        channel: int = DEFAULT_GAIT_BT_CHANNEL,
        bind_addr: Optional[str] = None,
        backend: Optional[str] = None,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        log_error: Optional[Callable[[str], None]] = None,
        on_message: Optional[Callable[[str], Optional[str]]] = None,
    ) -> None:
        self._service_uuid = service_uuid
        self._service_name = service_name
        self._channel = channel
        self._bind_addr_raw = bind_addr
        self._backend_raw = backend
        self._log_info = log_info or _default_log
        self._log_warn = log_warn or self._log_info
        self._log_error = log_error or self._log_info
        self._on_message = on_message
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._client_connected = threading.Event()
        self._handshake_completed = threading.Event()
        self._send_queue: "Queue[str]" = Queue(maxsize=16)
        self._last_drop_log = 0.0
        self._send_suspended_until = 0.0
        self._client_poll_timeout = _read_env_float(
            "GAIT_BT_RECV_POLL_TIMEOUT_SEC", 0.1, 0.01
        )
        self._client_io_timeout = _read_env_float(
            "GAIT_BT_IO_TIMEOUT_SEC", 5.0, 0.1
        )
        self._response_timeout = _read_env_float(
            "GAIT_BT_RESPONSE_TIMEOUT_SEC", 0.2, 0.01
        )

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._close_client()
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None

    def is_client_connected(self) -> bool:
        return self._client_connected.is_set()

    def send_line(self, line: str) -> bool:
        if not self._client_connected.is_set() or not self._handshake_completed.is_set():
            return False
        try:
            self._send_queue.put_nowait(line)
            return True
        except Exception:
            dropped = False
            try:
                self._send_queue.get_nowait()
                dropped = True
            except Empty:
                dropped = False
            if dropped:
                try:
                    self._send_queue.put_nowait(line)
                    return True
                except Exception:
                    pass
            now = time.time()
            if now - self._last_drop_log > 2.0:
                self._log_warn("⚠️ 蓝牙发送队列已满，已丢弃旧数据并优先保留最新数据")
                self._last_drop_log = now
            return False

    def _run(self) -> None:
        backend = (self._backend_raw or "auto").strip().lower()
        if backend in ("pybluez", "bluez"):
            if not _PYBLUEZ_AVAILABLE:
                self._log_warn("⚠️ GAIT_BT_BACKEND=pybluez 但未检测到 PyBluez，改用原生socket")
                backend = "native"
        elif backend in ("native", "socket", "stdlib"):
            backend = "native"
        else:
            backend = "pybluez" if _PYBLUEZ_AVAILABLE else "native"

        if backend == "native" and not hasattr(socket, "AF_BLUETOOTH"):
            self._log_error("❌ 当前Python/系统不支持蓝牙 socket(AF_BLUETOOTH)")
            return
        try:
            any_addr = _PYBLUEZ_ANY or DEFAULT_GAIT_BT_BIND_ADDR
            bind_addr = _resolve_bind_addr(
                self._bind_addr_raw,
                self._log_info,
                self._log_warn,
                any_addr,
            )
            if backend == "pybluez" and BluetoothSocket is not None and RFCOMM is not None:
                self._server_sock = BluetoothSocket(RFCOMM)
            else:
                backend = "native"
                self._server_sock = socket.socket(
                    socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
                )
            _set_bt_security_low(self._server_sock, self._log_warn)
            self._server_sock.bind((bind_addr, self._channel))
            self._server_sock.listen(1)
            self._server_sock.settimeout(1.0)
        except OSError as exc:
            self._log_error(f"❌ 蓝牙服务启动失败: {exc}")
            if exc.errno in (1, 13):
                self._log_warn("   提示: RFCOMM 需要特权权限。可使用 sudo 启动，或给 python3 设置 cap_net_admin/cap_net_raw。")
                self._log_warn("   示例: sudo setcap 'cap_net_admin,cap_net_raw+eip' $(readlink -f $(which python3))")
            self._cleanup_server_socket()
            return

        sdp_registered = False
        if _PYBLUEZ_AVAILABLE and advertise_service is not None:
            if not hasattr(self._server_sock, "_sock"):
                self._log_warn(
                    "⚠️ 蓝牙服务广播不可用: 当前socket不兼容PyBluez的SDP注册"
                )
            else:
                try:
                    advertise_service(
                        self._server_sock,
                        self._service_name,
                        service_id=self._service_uuid,
                        service_classes=[self._service_uuid, SERIAL_PORT_CLASS],
                        profiles=[SERIAL_PORT_PROFILE],
                    )
                    self._log_info(
                        f"✅ 已广播蓝牙服务: {self._service_name} ({self._service_uuid})"
                    )
                    sdp_registered = True
                except Exception as exc:
                    self._log_warn(f"⚠️ 蓝牙服务广播失败: {exc}")
        if not sdp_registered:
            if _PYBLUEZ_AVAILABLE and advertise_service is not None:
                self._log_warn("⚠️ 蓝牙服务未注册到 SDP，尝试使用 sdptool 注册 SPP 服务")
            else:
                self._log_warn("⚠️ 未检测到 PyBluez，蓝牙服务未注册到 SDP。")
                self._log_warn("   如需被Android自动发现，请安装 PyBluez 或手动注册 SDP。")
            sdp_registered = _register_spp_with_sdptool(
                self._channel, self._log_info, self._log_warn
            )
        self._log_info(
            "📡 蓝牙RFCOMM服务已启动: "
            f"name={self._service_name}, channel={self._channel}, bind={bind_addr}, backend={backend}"
        )

        while not self._stop_event.is_set():
            try:
                client_sock, client_info = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self._client_sock = client_sock
            self._handshake_completed.clear()
            _set_bt_security_low(client_sock, self._log_warn)
            self._log_info(f"🔗 蓝牙设备已连接: {client_info}")
            try:
                self._handle_client(client_sock)
            finally:
                self._close_client()
                self._log_info("🔌 蓝牙连接已断开")

        self._cleanup_server_socket()

    def _handle_client(self, client_sock: socket.socket) -> None:
        client_sock.settimeout(self._client_io_timeout)
        self._client_connected.set()
        buffer = ""
        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select(
                    [client_sock], [], [], self._client_poll_timeout
                )
            except (OSError, ValueError):
                break
            if not readable:
                if not self._drain_send_queue(client_sock):
                    return
                continue
            try:
                data = client_sock.recv(1024)
            except socket.timeout:
                data = None
            except OSError:
                break
            if data:
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    response = None
                    if self._on_message:
                        try:
                            response = self._on_message(line)
                        except Exception as exc:
                            self._log_warn(f"⚠️ 处理蓝牙消息失败: {exc}")
                    if response:
                        if not self._send_response(client_sock, response):
                            payload_desc = _describe_payload(response)
                            if not _is_drop_tolerant_response(payload_desc):
                                return
                        if _is_ping_message(line):
                            self._handshake_completed.set()
                            self._log_info("🤝 蓝牙握手完成，开始允许发送 plot/state")
            elif data == b"":
                break
            if not self._drain_send_queue(client_sock):
                return
        self._client_connected.clear()

    def _close_client(self) -> None:
        self._client_connected.clear()
        self._handshake_completed.clear()
        if self._client_sock:
            try:
                self._client_sock.close()
            except OSError:
                pass
            self._client_sock = None
        while True:
            try:
                self._send_queue.get_nowait()
            except Empty:
                break

    def _send_response(self, client_sock: socket.socket, response: str) -> bool:
        payload_desc = _describe_payload(response)
        payload = (response + "\n").encode("utf-8")
        try:
            _, writable, _ = select.select([], [client_sock], [], self._response_timeout)
        except (OSError, ValueError) as exc:
            self._log_warn(
                f"⚠️ 蓝牙响应socket状态检查失败(type={payload_desc}): {_describe_socket_error(exc)}"
            )
            return False
        if not writable:
            if _is_drop_tolerant_response(payload_desc):
                self._log_warn(f"⚠️ 蓝牙响应发送缓冲区繁忙，已丢弃响应(type={payload_desc})")
                return False
            self._log_warn(f"⚠️ 蓝牙响应发送缓冲区繁忙(type={payload_desc})")
            return False
        previous_timeout = client_sock.gettimeout()
        try:
            client_sock.settimeout(self._response_timeout)
            client_sock.sendall(payload)
            return True
        except OSError as exc:
            self._log_warn(
                f"⚠️ 蓝牙响应发送失败(type={payload_desc}): {_describe_socket_error(exc)}"
            )
            return False
        finally:
            try:
                client_sock.settimeout(previous_timeout)
            except OSError:
                pass

    def _drain_send_queue(self, client_sock: socket.socket, max_batch: int = 50) -> bool:
        if time.time() < self._send_suspended_until:
            return True
        for _ in range(max_batch):
            try:
                line = self._send_queue.get_nowait()
            except Empty:
                return True
            payload_desc = _describe_payload(line)
            payload_size = len((line + "\n").encode("utf-8"))
            try:
                _, writable, _ = select.select([], [client_sock], [], 0)
            except (OSError, ValueError) as exc:
                self._log_warn(
                    "⚠️ 蓝牙socket状态检查失败"
                    f"(type={payload_desc}, bytes={payload_size}): {_describe_socket_error(exc)}"
                )
                return False
            if not writable:
                if _is_soft_realtime_payload(payload_desc):
                    self._log_warn(
                        "⚠️ 蓝牙发送缓冲区繁忙，已丢弃本轮遥测"
                        f"(type={payload_desc}, bytes={payload_size})"
                    )
                    self._send_suspended_until = time.time() + 1.0
                    while True:
                        try:
                            self._send_queue.get_nowait()
                        except Empty:
                            break
                    return True
                return False
            try:
                client_sock.sendall((line + "\n").encode("utf-8"))
            except OSError as exc:
                if isinstance(exc, TimeoutError) and _is_soft_realtime_payload(payload_desc):
                    self._log_warn(
                        "⚠️ 蓝牙实时数据发送超时，已丢弃本轮遥测"
                        f"(type={payload_desc}, bytes={payload_size}): {_describe_socket_error(exc)}"
                    )
                    self._send_suspended_until = time.time() + 1.0
                    while True:
                        try:
                            self._send_queue.get_nowait()
                        except Empty:
                            break
                    return True
                self._log_warn(
                    "⚠️ 蓝牙数据发送失败"
                    f"(type={payload_desc}, bytes={payload_size}): {_describe_socket_error(exc)}"
                )
                return False
        return True

    def _cleanup_server_socket(self) -> None:
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None


def build_default_server(
    log_info: Optional[Callable[[str], None]] = None,
    log_warn: Optional[Callable[[str], None]] = None,
    log_error: Optional[Callable[[str], None]] = None,
    on_message: Optional[Callable[[str], Optional[str]]] = None,
) -> BluetoothRfcommServer:
    service_uuid = os.environ.get("GAIT_BT_UUID", DEFAULT_GAIT_BT_UUID)
    service_name = os.environ.get("GAIT_BT_NAME", DEFAULT_GAIT_BT_NAME)
    channel = int(os.environ.get("GAIT_BT_CHANNEL", str(DEFAULT_GAIT_BT_CHANNEL)))
    bind_addr = os.environ.get("GAIT_BT_BIND_ADDR")
    backend = os.environ.get("GAIT_BT_BACKEND")
    return BluetoothRfcommServer(
        service_uuid=service_uuid,
        service_name=service_name,
        channel=channel,
        bind_addr=bind_addr,
        backend=backend,
        log_info=log_info,
        log_warn=log_warn,
        log_error=log_error,
        on_message=on_message or _default_message_handler,
    )


def _default_message_handler(message: str) -> Optional[str]:
    if message.lower() == "ping":
        return "pong"
    return None


def main() -> None:
    server = build_default_server()
    _default_log("🔧 启动蓝牙RFCOMM服务（独立模式）...")
    server.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        _default_log("🛑 收到中断，停止蓝牙服务")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
