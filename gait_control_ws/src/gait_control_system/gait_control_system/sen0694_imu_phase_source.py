#!/usr/bin/env python3

"""DFRobot SEN0694 I2C source for wired IMU phase modes."""

from __future__ import annotations

import ctypes
import errno
import fcntl
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional


I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

SEN0694_LEFT_DEFAULT_ADDR = 0x4A
SEN0694_RIGHT_DEFAULT_ADDR = 0x4B
SEN0694_PID_9DOF = 0x0009
DFR_VID = 0x3343

REG_I2C_VID = 0x0000
REG_I2C_PID = 0x0001
REG_I2C_SENSORS_MODE = 0x0005
REG_I2C_ACC_RANGE_CONF = 0x0006
REG_I2C_GYR_RANGE_CONF = 0x0007
REG_I2C_ACC_DATA_X = 0x0010

SENSOR_MODE_NORMAL = 0x0002
DEFAULT_I2C_ADAPTER_NAME = "i2c-gpio-h2"
DEFAULT_FALLBACK_I2C_DEV = "/dev/i2c-9"
DEFAULT_SEN0694_RATE_HZ = 50.0
DEFAULT_DATA_TIMEOUT_SEC = 0.5


def _env_enabled(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return raw.strip().lower() not in ("0", "false", "no", "off")


def _read_env_float(name: str, default: float, minimum: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return max(minimum, float(default))
    try:
        return max(minimum, float(raw))
    except (TypeError, ValueError):
        return max(minimum, float(default))


def _read_env_int(
    name: str,
    default: int,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    raw = os.environ.get(name)
    try:
        value = int(str(raw), 0) if raw is not None else int(default)
    except (TypeError, ValueError):
        value = int(default)
    if minimum is not None:
        value = max(int(minimum), value)
    if maximum is not None:
        value = min(int(maximum), value)
    return value


def parse_i2c_addr(text: str) -> int:
    value = int(str(text).strip(), 0)
    if value < 0 or value > 0x7F:
        raise ValueError(f"I2C address out of range: {text}")
    return value


def format_i2c_addr(addr: int) -> str:
    return f"0x{int(addr) & 0x7F:02X}"


def _i2c_entry_key(entry: str) -> int:
    try:
        return int(entry.rsplit("-", 1)[1])
    except (IndexError, ValueError):
        return 9999


def list_i2c_adapters() -> list[tuple[str, str]]:
    adapters: list[tuple[str, str]] = []
    base = "/sys/class/i2c-dev"
    try:
        entries = sorted(os.listdir(base), key=_i2c_entry_key)
    except OSError:
        return adapters

    for entry in entries:
        if not entry.startswith("i2c-"):
            continue
        name = ""
        try:
            with open(
                os.path.join(base, entry, "name"),
                "r",
                encoding="ascii",
                errors="replace",
            ) as fp:
                name = fp.read().strip()
        except OSError:
            pass
        adapters.append((f"/dev/{entry}", name))
    return adapters


def detect_i2c_dev_by_name(adapter_name: str = DEFAULT_I2C_ADAPTER_NAME) -> Optional[str]:
    exact_matches: list[str] = []
    partial_matches: list[str] = []
    for dev_path, name in list_i2c_adapters():
        if name == adapter_name:
            exact_matches.append(dev_path)
        elif adapter_name and adapter_name in name:
            partial_matches.append(dev_path)

    if exact_matches:
        return exact_matches[0]
    if partial_matches:
        return partial_matches[0]
    return None


def describe_i2c_adapters() -> str:
    adapters = list_i2c_adapters()
    if not adapters:
        return "(no adapters found in /sys/class/i2c-dev)"
    return "; ".join(f"{dev_path}: {name or '(no name)'}" for dev_path, name in adapters)


def accel_range_to_reg(g_range: int) -> tuple[int, float]:
    table = {2: (0, 2.0), 4: (1, 4.0), 8: (2, 8.0), 16: (3, 16.0)}
    if g_range not in table:
        raise ValueError("accel range must be 2, 4, 8, or 16")
    return table[g_range]


def gyro_range_to_reg(dps_range: int) -> tuple[int, float]:
    table = {
        125: (0, 125.0),
        250: (1, 250.0),
        500: (2, 500.0),
        1000: (3, 1000.0),
        2000: (4, 2000.0),
    }
    if dps_range not in table:
        raise ValueError("gyro range must be 125, 250, 500, 1000, or 2000")
    return table[dps_range]


def sleep_us(usec: int) -> None:
    if usec > 0:
        time.sleep(float(usec) / 1_000_000.0)


def le_u16(data: bytes, offset: int = 0) -> int:
    return data[offset] | (data[offset + 1] << 8)


def le_i16(data: bytes, offset: int = 0) -> int:
    value = le_u16(data, offset)
    return value - 0x10000 if value & 0x8000 else value


class I2CMsg(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class I2CRdwrIoctlData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(I2CMsg)),
        ("nmsgs", ctypes.c_uint32),
    ]


@dataclass
class Vec3:
    x: float
    y: float
    z: float


@dataclass
class Sen0694Sample:
    accel_g: Vec3
    gyro_dps: Vec3


class LinuxI2C:
    def __init__(self, dev_path: str, combined: bool = False):
        self.dev_path = str(dev_path)
        self.combined = bool(combined)
        self.fd = os.open(self.dev_path, os.O_RDWR | getattr(os, "O_CLOEXEC", 0))
        self.current_addr: Optional[int] = None
        self.last_reg: dict[int, int] = {}

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def set_addr(self, addr: int) -> None:
        if self.current_addr == addr:
            return
        fcntl.ioctl(self.fd, I2C_SLAVE, int(addr))
        self.current_addr = int(addr)

    def write_reg16(self, addr: int, reg: int, data: bytes) -> None:
        self.set_addr(addr)
        payload = bytes((reg & 0xFF, (reg >> 8) & 0xFF)) + bytes(data)
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, f"short I2C write: {written}/{len(payload)}")

    def write_reg16_pointer(self, addr: int, reg: int) -> None:
        if self.combined:
            self.last_reg[int(addr)] = int(reg)
            return

        self.set_addr(addr)
        payload = bytes((reg & 0xFF, (reg >> 8) & 0xFF))
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, f"short I2C register write: {written}/{len(payload)}")

    def read_reg16_data(self, addr: int, length: int) -> bytes:
        if self.combined and int(addr) in self.last_reg:
            reg = self.last_reg.pop(int(addr))
            return self.read_reg16_combined(addr, reg, length)

        self.set_addr(addr)
        data = os.read(self.fd, length)
        if len(data) != length:
            raise OSError(errno.EIO, f"short I2C read: {len(data)}/{length}")
        return data

    def read_reg16_combined(self, addr: int, reg: int, length: int) -> bytes:
        write_buf = (ctypes.c_uint8 * 2)(reg & 0xFF, (reg >> 8) & 0xFF)
        read_buf = (ctypes.c_uint8 * length)()
        msgs = (I2CMsg * 2)()

        msgs[0].addr = int(addr)
        msgs[0].flags = 0
        msgs[0].len = 2
        msgs[0].buf = write_buf
        msgs[1].addr = int(addr)
        msgs[1].flags = I2C_M_RD
        msgs[1].len = int(length)
        msgs[1].buf = read_buf

        xfer = I2CRdwrIoctlData(msgs, 2)
        fcntl.ioctl(self.fd, I2C_RDWR, xfer, True)
        self.current_addr = None
        return bytes(read_buf)

    def read_reg16(self, addr: int, reg: int, length: int, read_delay_us: int = 0) -> bytes:
        if self.combined and read_delay_us == 0:
            return self.read_reg16_combined(addr, reg, length)

        self.write_reg16_pointer(addr, reg)
        sleep_us(read_delay_us)
        return self.read_reg16_data(addr, length)


class Sen0694:
    def __init__(
        self,
        i2c: LinuxI2C,
        addr: int,
        read_delay_us: int,
        accel_range_g: int,
        gyro_range_dps: int,
    ):
        self.i2c = i2c
        self.addr = int(addr)
        self.read_delay_us = int(read_delay_us)
        _, self.accel_range = accel_range_to_reg(int(accel_range_g))
        _, self.gyro_range = gyro_range_to_reg(int(gyro_range_dps))

    def read_u16(self, reg: int) -> int:
        return le_u16(self.i2c.read_reg16(self.addr, reg, 2, self.read_delay_us))

    def write_u16(self, reg: int, value: int) -> None:
        self.i2c.write_reg16(
            self.addr,
            reg,
            bytes((value & 0xFF, (value >> 8) & 0xFF)),
        )

    def begin(
        self,
        configure_sensor: bool = True,
        accel_range_g: int = 2,
        gyro_range_dps: int = 250,
    ) -> tuple[int, int]:
        vid = self.read_u16(REG_I2C_VID)
        pid = self.read_u16(REG_I2C_PID)
        if pid < SEN0694_PID_9DOF:
            raise RuntimeError(
                f"[{format_i2c_addr(self.addr)}] unexpected PID 0x{pid:04X}, "
                f"expected >= 0x{SEN0694_PID_9DOF:04X}"
            )

        if configure_sensor:
            accel_reg, self.accel_range = accel_range_to_reg(int(accel_range_g))
            gyro_reg, self.gyro_range = gyro_range_to_reg(int(gyro_range_dps))
            self.write_u16(REG_I2C_SENSORS_MODE, SENSOR_MODE_NORMAL)
            time.sleep(0.1)
            self.write_u16(REG_I2C_ACC_RANGE_CONF, accel_reg)
            time.sleep(0.05)
            self.write_u16(REG_I2C_GYR_RANGE_CONF, gyro_reg)
            time.sleep(0.1)

        return int(vid), int(pid)

    def prepare_sample(self) -> None:
        self.i2c.write_reg16_pointer(self.addr, REG_I2C_ACC_DATA_X)

    def read_prepared_sample(self) -> Sen0694Sample:
        return self.decode_sample(self.i2c.read_reg16_data(self.addr, 12))

    def read_sample(self) -> Sen0694Sample:
        self.prepare_sample()
        sleep_us(self.read_delay_us)
        return self.read_prepared_sample()

    def decode_sample(self, data: bytes) -> Sen0694Sample:
        if len(data) != 12:
            raise RuntimeError(
                f"[{format_i2c_addr(self.addr)}] short IMU read: "
                f"got {len(data)} bytes, expected 12"
            )

        ax = le_i16(data, 0)
        ay = le_i16(data, 2)
        az = le_i16(data, 4)
        gx = le_i16(data, 6)
        gy = le_i16(data, 8)
        gz = le_i16(data, 10)

        return Sen0694Sample(
            Vec3(
                ax * self.accel_range / 32768.0,
                ay * self.accel_range / 32768.0,
                az * self.accel_range / 32768.0,
            ),
            Vec3(
                gx * self.gyro_range / 32768.0,
                gy * self.gyro_range / 32768.0,
                gz * self.gyro_range / 32768.0,
            ),
        )


def _default_addr_for_side(side: str) -> int:
    return SEN0694_LEFT_DEFAULT_ADDR if side == "left" else SEN0694_RIGHT_DEFAULT_ADDR


def _read_side_addr_from_env(side: str, default: int) -> int:
    side_key = side.upper()
    names = (
        f"GAIT_IMU_PHASE_{side_key}_I2C_ADDR",
        f"GAIT_SEN0694_{side_key}_ADDR",
    )
    for name in names:
        raw = os.environ.get(name)
        if raw:
            return parse_i2c_addr(raw)
    return int(default)


def _read_i2c_dev_from_env() -> Optional[str]:
    dev_path = (
        os.environ.get("GAIT_IMU_PHASE_I2C_DEV")
        or os.environ.get("GAIT_SEN0694_I2C_DEV")
        or ""
    ).strip()
    if dev_path:
        return dev_path

    bus_text = (
        os.environ.get("GAIT_IMU_PHASE_I2C_BUS")
        or os.environ.get("GAIT_SEN0694_I2C_BUS")
        or ""
    ).strip()
    if bus_text:
        return f"/dev/i2c-{int(bus_text, 0)}"
    return None


class WiredSen0694ImuPhaseSource:
    """Detector-compatible source exposing latest SEN0694 6D samples."""

    def __init__(
        self,
        side: str,
        dev_path: Optional[str] = None,
        address: Optional[int] = None,
        data_timeout_sec: float = DEFAULT_DATA_TIMEOUT_SEC,
        sample_rate_hz: Optional[float] = None,
        configure_sensor: Optional[bool] = None,
        accel_range_g: Optional[int] = None,
        gyro_range_dps: Optional[int] = None,
        read_delay_us: Optional[int] = None,
        combined: Optional[bool] = None,
    ):
        side_key = str(side or "").strip().lower()
        if side_key not in ("left", "right"):
            raise ValueError(f"invalid IMU side: {side}")
        self.side = side_key
        self.dev_path = dev_path or _read_i2c_dev_from_env()
        self.adapter_name = os.environ.get(
            "GAIT_IMU_PHASE_I2C_ADAPTER_NAME",
            DEFAULT_I2C_ADAPTER_NAME,
        ).strip() or DEFAULT_I2C_ADAPTER_NAME
        self.address = int(
            address
            if address is not None
            else _read_side_addr_from_env(self.side, _default_addr_for_side(self.side))
        )
        self.sample_rate_hz = (
            max(1.0, float(sample_rate_hz))
            if sample_rate_hz is not None
            else _read_env_float(
                "GAIT_IMU_PHASE_I2C_RATE_HZ",
                DEFAULT_SEN0694_RATE_HZ,
                1.0,
            )
        )
        self.data_timeout_sec = float(data_timeout_sec)
        self.configure_sensor = (
            _env_enabled("GAIT_IMU_PHASE_I2C_CONFIGURE", True)
            if configure_sensor is None
            else bool(configure_sensor)
        )
        self.accel_range_g = int(
            accel_range_g
            if accel_range_g is not None
            else _read_env_int("GAIT_IMU_PHASE_I2C_ACCEL_RANGE_G", 2)
        )
        self.gyro_range_dps = int(
            gyro_range_dps
            if gyro_range_dps is not None
            else _read_env_int("GAIT_IMU_PHASE_I2C_GYRO_RANGE_DPS", 250)
        )
        self.read_delay_us = int(
            read_delay_us
            if read_delay_us is not None
            else _read_env_int("GAIT_IMU_PHASE_I2C_READ_DELAY_US", 0, 0)
        )
        self.combined = (
            _env_enabled("GAIT_IMU_PHASE_I2C_COMBINED", False)
            if combined is None
            else bool(combined)
        )
        self.mac_address = self._make_source_id()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._bus: Optional[LinuxI2C] = None
        self._sensor: Optional[Sen0694] = None
        self._latest_sample: Optional[tuple[tuple[float, float, float, float, float, float], float, int]] = None
        self._seq = 0
        self._connected = False
        self._measuring = False
        self._measurement_enabled = False
        self._last_error = ""

    @property
    def last_error(self) -> str:
        return self._last_error

    def _make_source_id(self) -> str:
        dev_text = self.dev_path or "auto"
        return f"wired:sen0694:{dev_text}:{self.side}:{format_i2c_addr(self.address)}"

    def _resolve_dev_path(self) -> str:
        if self.dev_path:
            return str(self.dev_path)
        return detect_i2c_dev_by_name(self.adapter_name) or DEFAULT_FALLBACK_I2C_DEV

    def set_measurement_enabled(self, enabled: bool) -> bool:
        target = bool(enabled)
        with self._lock:
            self._measurement_enabled = target
            self._measuring = bool(self._connected and target)
        return True

    def start(self) -> bool:
        stale_bus = None
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._connected = True
                self._measuring = bool(self._measurement_enabled)
                return True
            stale_bus = self._bus
            self._bus = None
            self._sensor = None

        if stale_bus is not None:
            try:
                stale_bus.close()
            except OSError:
                pass

        try:
            bus, sensor, dev_path = self._open_sensor()
        except Exception as exc:
            self._last_error = str(exc)
            with self._lock:
                self._connected = False
                self._measuring = False
            return False

        with self._lock:
            self.dev_path = dev_path
            self.mac_address = self._make_source_id()
            self._bus = bus
            self._sensor = sensor
            self._latest_sample = None
            self._seq = 0
            self._connected = True
            self._measuring = bool(self._measurement_enabled)
            self._last_error = ""
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._read_loop,
                name=f"sen0694-{self.side}-{format_i2c_addr(self.address)}",
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
            bus = self._bus
            self._bus = None
            self._sensor = None
            self._thread = None
            self._connected = False
            self._measuring = False
            self._measurement_enabled = False
            self._latest_sample = None

        if bus is not None:
            try:
                bus.close()
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

    def _open_sensor(self) -> tuple[LinuxI2C, Sen0694, str]:
        dev_path = self._resolve_dev_path()
        try:
            bus = LinuxI2C(dev_path, combined=self.combined)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"{dev_path} does not exist; available I2C adapters: "
                f"{describe_i2c_adapters()}"
            ) from exc
        except PermissionError as exc:
            raise RuntimeError(f"permission denied opening {dev_path}; run with I2C access") from exc

        try:
            sensor = Sen0694(
                bus,
                self.address,
                self.read_delay_us,
                self.accel_range_g,
                self.gyro_range_dps,
            )
            vid, pid = sensor.begin(
                configure_sensor=self.configure_sensor,
                accel_range_g=self.accel_range_g,
                gyro_range_dps=self.gyro_range_dps,
            )
            if vid != DFR_VID:
                self._last_error = (
                    f"[{format_i2c_addr(self.address)}] unexpected VID 0x{vid:04X}, "
                    f"expected 0x{DFR_VID:04X}; continuing"
                )
            _ = pid
        except Exception:
            bus.close()
            raise

        return bus, sensor, dev_path

    def _read_loop(self) -> None:
        period = 1.0 / max(float(self.sample_rate_hz), 1.0)
        next_t = time.monotonic()

        while not self._stop_event.is_set():
            with self._lock:
                sensor = self._sensor
                measuring = self._measuring
            if sensor is None:
                return
            if not measuring:
                next_t = time.monotonic()
                time.sleep(0.02)
                continue

            try:
                sample = sensor.read_sample()
                sample_time = time.time()
                self._handle_sample(sample, sample_time)
            except Exception as exc:
                self._last_error = f"SEN0694 I2C read failed: {exc}"
                break

            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

        with self._lock:
            self._connected = False
            self._measuring = False

    def _handle_sample(self, sample: Sen0694Sample, sample_time: float) -> None:
        sample_6d = (
            float(sample.accel_g.x),
            float(sample.accel_g.y),
            float(sample.accel_g.z),
            float(sample.gyro_dps.x),
            float(sample.gyro_dps.y),
            float(sample.gyro_dps.z),
        )
        with self._lock:
            self._seq += 1
            self._latest_sample = (sample_6d, float(sample_time), int(self._seq))
            if self._last_error.startswith("SEN0694 I2C read failed"):
                self._last_error = ""
