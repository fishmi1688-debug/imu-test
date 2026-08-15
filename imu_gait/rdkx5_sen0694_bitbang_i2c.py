#!/usr/bin/env python3

import argparse
import ctypes
import errno
import fcntl
import math
import os
import signal
import sys
import time
from dataclasses import dataclass


I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

SEN0694_DEFAULT_ADDR = 0x4A
SEN0694_SECOND_DEFAULT_ADDR = 0x4B
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


def _i2c_entry_key(entry):
    try:
        return int(entry.rsplit("-", 1)[1])
    except (IndexError, ValueError):
        return 9999


def list_i2c_adapters():
    adapters = []
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
            with open(os.path.join(base, entry, "name"), "r", encoding="ascii", errors="replace") as fp:
                name = fp.read().strip()
        except OSError:
            pass
        adapters.append((f"/dev/{entry}", name))
    return adapters


def detect_i2c_dev_by_name(adapter_name=DEFAULT_I2C_ADAPTER_NAME):
    exact_matches = []
    partial_matches = []
    for dev_path, name in list_i2c_adapters():
        if name == adapter_name:
            exact_matches.append(dev_path)
        elif adapter_name in name:
            partial_matches.append(dev_path)

    if exact_matches:
        return exact_matches[0]
    if partial_matches:
        return partial_matches[0]
    return None


def describe_i2c_adapters():
    adapters = list_i2c_adapters()
    if not adapters:
        return "  (no adapters found in /sys/class/i2c-dev)"
    return "\n".join(f"  {dev_path}: {name or '(no name)'}" for dev_path, name in adapters)


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
class ImuSample:
    accel_g: Vec3
    gyro_dps: Vec3


def sleep_us(usec):
    if usec > 0:
        time.sleep(usec / 1_000_000.0)


def le_u16(data, offset=0):
    return data[offset] | (data[offset + 1] << 8)


def le_i16(data, offset=0):
    value = le_u16(data, offset)
    return value - 0x10000 if value & 0x8000 else value


def accel_range_to_reg(g):
    table = {2: (0, 2.0), 4: (1, 4.0), 8: (2, 8.0), 16: (3, 16.0)}
    if g not in table:
        raise ValueError("accel range must be 2, 4, 8, or 16")
    return table[g]


def gyro_range_to_reg(dps):
    table = {
        125: (0, 125.0),
        250: (1, 250.0),
        500: (2, 500.0),
        1000: (3, 1000.0),
        2000: (4, 2000.0),
    }
    if dps not in table:
        raise ValueError("gyro range must be 125, 250, 500, 1000, or 2000")
    return table[dps]


class LinuxI2C:
    def __init__(self, dev_path, combined=False):
        self.dev_path = dev_path
        self.combined = combined
        self.fd = os.open(dev_path, os.O_RDWR | getattr(os, "O_CLOEXEC", 0))
        self.current_addr = None
        self.last_reg = {}

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def set_addr(self, addr):
        if self.current_addr == addr:
            return
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        self.current_addr = addr

    def write_reg16(self, addr, reg, data):
        self.set_addr(addr)
        payload = bytes((reg & 0xFF, (reg >> 8) & 0xFF)) + bytes(data)
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, f"short I2C write: {written}/{len(payload)}")

    def write_reg16_pointer(self, addr, reg):
        if self.combined:
            self.last_reg[addr] = reg
            return

        self.set_addr(addr)
        payload = bytes((reg & 0xFF, (reg >> 8) & 0xFF))
        written = os.write(self.fd, payload)
        if written != len(payload):
            raise OSError(errno.EIO, f"short I2C register write: {written}/{len(payload)}")

    def read_reg16_data(self, addr, length):
        if self.combined and addr in self.last_reg:
            reg = self.last_reg.pop(addr)
            return self.read_reg16_combined(addr, reg, length)

        self.set_addr(addr)
        data = os.read(self.fd, length)
        if len(data) != length:
            raise OSError(errno.EIO, f"short I2C read: {len(data)}/{length}")
        return data

    def read_reg16_combined(self, addr, reg, length):
        write_buf = (ctypes.c_uint8 * 2)(reg & 0xFF, (reg >> 8) & 0xFF)
        read_buf = (ctypes.c_uint8 * length)()
        msgs = (I2CMsg * 2)()

        msgs[0].addr = addr
        msgs[0].flags = 0
        msgs[0].len = 2
        msgs[0].buf = write_buf
        msgs[1].addr = addr
        msgs[1].flags = I2C_M_RD
        msgs[1].len = length
        msgs[1].buf = read_buf

        xfer = I2CRdwrIoctlData(msgs, 2)
        fcntl.ioctl(self.fd, I2C_RDWR, xfer, True)
        self.current_addr = None
        return bytes(read_buf)

    def read_reg16(self, addr, reg, length, read_delay_us=0):
        if self.combined and read_delay_us == 0:
            return self.read_reg16_combined(addr, reg, length)

        self.write_reg16_pointer(addr, reg)
        sleep_us(read_delay_us)
        return self.read_reg16_data(addr, length)


class Sen0694:
    def __init__(self, i2c, addr, read_delay_us, accel_range_g, gyro_range_dps):
        self.i2c = i2c
        self.addr = addr
        self.read_delay_us = read_delay_us
        _, self.accel_range = accel_range_to_reg(accel_range_g)
        _, self.gyro_range = gyro_range_to_reg(gyro_range_dps)

    def read_u16(self, reg):
        return le_u16(self.i2c.read_reg16(self.addr, reg, 2, self.read_delay_us))

    def write_u16(self, reg, value):
        self.i2c.write_reg16(self.addr, reg, bytes((value & 0xFF, (value >> 8) & 0xFF)))

    def begin(self, configure_sensor=True, accel_range_g=2, gyro_range_dps=250):
        vid = self.read_u16(REG_I2C_VID)
        pid = self.read_u16(REG_I2C_PID)
        if vid != DFR_VID:
            print(
                f"[0x{self.addr:02X}] Unexpected VID 0x{vid:04X}, expected 0x{DFR_VID:04X}; continue carefully.",
                file=sys.stderr,
            )
        if pid < SEN0694_PID_9DOF:
            raise RuntimeError(f"[0x{self.addr:02X}] Unexpected PID 0x{pid:04X}, expected >= 0x{SEN0694_PID_9DOF:04X}")

        print(f"[0x{self.addr:02X}] SEN0694 detected: VID=0x{vid:04X} PID=0x{pid:04X}", file=sys.stderr)

        if not configure_sensor:
            print(f"[0x{self.addr:02X}] Sensor configuration skipped.", file=sys.stderr)
            return

        accel_reg, self.accel_range = accel_range_to_reg(accel_range_g)
        gyro_reg, self.gyro_range = gyro_range_to_reg(gyro_range_dps)

        self.write_u16(REG_I2C_SENSORS_MODE, SENSOR_MODE_NORMAL)
        time.sleep(0.1)
        self.write_u16(REG_I2C_ACC_RANGE_CONF, accel_reg)
        time.sleep(0.05)
        self.write_u16(REG_I2C_GYR_RANGE_CONF, gyro_reg)
        time.sleep(0.1)
        print(
            f"[0x{self.addr:02X}] Configured: normal mode, accel=+/- {accel_range_g}g, gyro=+/- {gyro_range_dps}dps",
            file=sys.stderr,
        )

    def prepare_sample(self):
        self.i2c.write_reg16_pointer(self.addr, REG_I2C_ACC_DATA_X)

    def read_prepared_sample(self):
        data = self.i2c.read_reg16_data(self.addr, 12)
        return self.decode_sample(data)

    def read_sample(self):
        self.prepare_sample()
        sleep_us(self.read_delay_us)
        return self.read_prepared_sample()

    def decode_sample(self, data):
        if len(data) != 12:
            raise RuntimeError(f"[0x{self.addr:02X}] short IMU read: got {len(data)} bytes, expected 12")

        ax = le_i16(data, 0)
        ay = le_i16(data, 2)
        az = le_i16(data, 4)
        gx = le_i16(data, 6)
        gy = le_i16(data, 8)
        gz = le_i16(data, 10)

        return ImuSample(
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


class Madgwick:
    def __init__(self, beta=0.1):
        self.w = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.beta = beta

    def normalize(self):
        norm = math.sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
        if norm <= 0.0 or not math.isfinite(norm):
            self.w, self.x, self.y, self.z = 1.0, 0.0, 0.0, 0.0
            return
        inv = 1.0 / norm
        self.w *= inv
        self.x *= inv
        self.y *= inv
        self.z *= inv

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.w, self.x, self.y, self.z

        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 0.0 and math.isfinite(norm):
            ax, ay, az = ax / norm, ay / norm, az / norm
            _2q0, _2q1, _2q2, _2q3 = 2.0 * q0, 2.0 * q1, 2.0 * q2, 2.0 * q3
            _4q0, _4q1, _4q2 = 4.0 * q0, 4.0 * q1, 4.0 * q2
            _8q1, _8q2 = 8.0 * q1, 8.0 * q2
            q0q0, q1q1, q2q2, q3q3 = q0 * q0, q1 * q1, q2 * q2, q3 * q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = (
                4.0 * q0q0 * q1
                - _2q0 * ay
                - _2q3 * ax
                + _4q1 * q3q3
                - _4q1
                + _8q1 * q1q1
                + _8q1 * q2q2
                + _4q1 * az
            )
            s2 = (
                4.0 * q0q0 * q2
                + _2q0 * ax
                + _4q2 * q3q3
                - _2q3 * ay
                - _4q2
                + _8q2 * q1q1
                + _8q2 * q2q2
                + _4q2 * az
            )
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
            sn = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if sn > 0.0 and math.isfinite(sn):
                s0, s1, s2, s3 = s0 / sn, s1 / sn, s2 / sn, s3 / sn
                q_dot0 -= self.beta * s0
                q_dot1 -= self.beta * s1
                q_dot2 -= self.beta * s2
                q_dot3 -= self.beta * s3

        self.w = q0 + q_dot0 * dt
        self.x = q1 + q_dot1 * dt
        self.y = q2 + q_dot2 * dt
        self.z = q3 + q_dot3 * dt
        self.normalize()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read two SEN0694 IMUs through Linux i2c-dev on RDK X5 H2 i2c-gpio."
    )
    parser.add_argument(
        "--dev",
        help=f"I2C device path; default auto-detects adapter named {DEFAULT_I2C_ADAPTER_NAME!r}",
    )
    parser.add_argument("--bus", type=int, help="Shortcut for --dev /dev/i2c-N")
    parser.add_argument("--addr1", type=lambda x: int(x, 0), default=SEN0694_DEFAULT_ADDR)
    parser.add_argument("--addr2", type=lambda x: int(x, 0), default=SEN0694_SECOND_DEFAULT_ADDR)
    parser.add_argument("--single", action="store_true", help="Read only IMU 1")
    parser.add_argument("--rate", type=float, default=50.0, help="Output rate in Hz")
    parser.add_argument("--samples", type=int, default=-1, help="Stop after N samples")
    parser.add_argument("--accel-range", type=int, choices=(2, 4, 8, 16), default=2)
    parser.add_argument("--gyro-range", type=int, choices=(125, 250, 500, 1000, 2000), default=250)
    parser.add_argument("--no-config", action="store_true", help="Do not write mode/range registers")
    parser.add_argument("--beta", type=float, default=0.10, help="Madgwick beta gain")
    parser.add_argument("--read-delay-us", type=int, default=0, help="Delay after register-address write")
    parser.add_argument("--combined", action="store_true", help="Use I2C_RDWR repeated-start reads")
    parser.add_argument("--no-header", action="store_true", help="Do not print CSV header")
    args = parser.parse_args()

    args.auto_dev = args.dev is None and args.bus is None
    if args.bus is not None:
        args.dev = f"/dev/i2c-{args.bus}"
    elif args.dev is None:
        args.dev = detect_i2c_dev_by_name() or DEFAULT_FALLBACK_I2C_DEV

    for name in ("addr1", "addr2"):
        address = getattr(args, name)
        if not (0 <= address <= 0x7F):
            parser.error(f"--{name} must be a 7-bit I2C address")
    if not args.single and args.addr1 == args.addr2:
        parser.error("--addr1 and --addr2 must be different")
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.samples == 0 or args.samples < -1:
        parser.error("--samples must be positive, or omitted for continuous output")
    if args.read_delay_us < 0:
        parser.error("--read-delay-us must be non-negative")
    if args.combined and args.read_delay_us:
        parser.error("--combined cannot be used together with --read-delay-us")
    return args


def imu_output_fields(prefix):
    return [
        f"{prefix}_addr",
        f"{prefix}_qw",
        f"{prefix}_qx",
        f"{prefix}_qy",
        f"{prefix}_qz",
        f"{prefix}_ax_g",
        f"{prefix}_ay_g",
        f"{prefix}_az_g",
        f"{prefix}_gx_dps",
        f"{prefix}_gy_dps",
        f"{prefix}_gz_dps",
    ]


def format_imu_output(imu, sample, quat):
    return [
        f"0x{imu.addr:02X}",
        f"{quat.w:.7f}",
        f"{quat.x:.7f}",
        f"{quat.y:.7f}",
        f"{quat.z:.7f}",
        f"{sample.accel_g.x:.6f}",
        f"{sample.accel_g.y:.6f}",
        f"{sample.accel_g.z:.6f}",
        f"{sample.gyro_dps.x:.6f}",
        f"{sample.gyro_dps.y:.6f}",
        f"{sample.gyro_dps.z:.6f}",
    ]


def run_loop(args, imus):
    quats = [Madgwick(args.beta) for _ in imus]
    period = 1.0 / args.rate
    next_t = time.monotonic()
    prev_t = next_t
    count = 0
    dps_to_rad = math.pi / 180.0

    if not args.no_header:
        header = ["t_ns", "hz"]
        for index in range(len(imus)):
            header.extend(imu_output_fields(f"imu{index + 1}"))
        print(",".join(header), flush=True)

    while not STOP and (args.samples < 0 or count < args.samples):
        now = time.monotonic()
        if count == 0:
            dt = period
            measured_hz = 0.0
        else:
            dt = now - prev_t
            measured_hz = 1.0 / dt if dt > 0.0 else 0.0
            if dt <= 0.0 or dt > 1.0:
                dt = period
                measured_hz = 0.0
        prev_t = now

        for imu in imus:
            imu.prepare_sample()
        sleep_us(max(imu.read_delay_us for imu in imus))
        samples = [imu.read_prepared_sample() for imu in imus]

        for quat, sample in zip(quats, samples):
            quat.update_imu(
                sample.gyro_dps.x * dps_to_rad,
                sample.gyro_dps.y * dps_to_rad,
                sample.gyro_dps.z * dps_to_rad,
                sample.accel_g.x,
                sample.accel_g.y,
                sample.accel_g.z,
                dt,
            )

        row = [str(time.monotonic_ns()), f"{measured_hz:.2f}"]
        for imu, sample, quat in zip(imus, samples, quats):
            row.extend(format_imu_output(imu, sample, quat))
        print(",".join(row), flush=True)

        count += 1
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.monotonic()


STOP = False


def handle_signal(signum, frame):
    del signum, frame
    global STOP
    STOP = True


def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    args = parse_args()

    try:
        bus = LinuxI2C(args.dev, combined=args.combined)
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"{args.dev} does not exist. The H2 kernel I2C adapter "
            f"{DEFAULT_I2C_ADAPTER_NAME!r} was not detected on this boot.\n"
            "Available I2C adapters:\n"
            f"{describe_i2c_adapters()}\n"
            "Run `i2cdetect -l` to confirm the bus number. If no `i2c-gpio-h2` adapter exists, "
            "check `/boot/config.txt`, `/boot/overlays/dtoverlay_h2_i2c_gpio.dtbo`, then reboot."
        ) from exc
    except PermissionError as exc:
        raise RuntimeError(f"Permission denied opening {args.dev}; run with sudo.") from exc

    print(
        f"I2C device: {args.dev} "
        f"({'auto-detected, ' if args.auto_dev else ''}"
        f"{'I2C_RDWR repeated-start' if args.combined else 'stop-then-read'} reads)",
        file=sys.stderr,
    )
    try:
        addresses = [args.addr1] if args.single else [args.addr1, args.addr2]
        imus = []
        for index, address in enumerate(addresses, start=1):
            imu = Sen0694(bus, address, args.read_delay_us, args.accel_range, args.gyro_range)
            try:
                imu.begin(not args.no_config, args.accel_range, args.gyro_range)
            except OSError as exc:
                raise RuntimeError(
                    f"IMU {index} at 0x{address:02X} did not respond during init: {exc}. "
                    "Check i2cdetect, address switches, power, GND, SCL, and SDA."
                ) from exc
            imus.append(imu)
        run_loop(args, imus)
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
