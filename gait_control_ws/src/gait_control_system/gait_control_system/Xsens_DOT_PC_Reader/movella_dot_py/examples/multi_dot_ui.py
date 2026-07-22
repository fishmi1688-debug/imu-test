import asyncio
import csv
import datetime as dt
import os
import sys
import threading
from typing import Optional
from pathlib import Path
from queue import Queue, Empty
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from bleak import BleakClient


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(REPO_ROOT)

from movella_dot_py.core.sensor import MovellaDOTSensor
from movella_dot_py.models.data_structures import SensorConfiguration
from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode


MAC_ADDRESSES = [
    "D4:22:CD:00:8A:5A",#左大腿
    "D4:22:CD:00:8A:5B",#脚
]
CSV_OUTPUT_DIR = "dot_csv"

OUTPUT_RATE = OutputRate.RATE_60    # 1 Hz  30/60
FILTER_PROFILE = FilterProfile.DYNAMIC   # 动态滤波配置
PAYLOAD_MODE = PayloadMode.CUSTOM_MODE_5   # 自定义模式 5，包含所有数据


CSV_COLUMNS = [
    "seq",
    "timestamp_us",
    "device_tag",
    "mac_address",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
    "euler_roll",
    "euler_pitch",
    "euler_yaw",
    "free_accel_x",
    "free_accel_y",
    "free_accel_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "ang_vel_x",
    "ang_vel_y",
    "ang_vel_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "status",
    "clipping_acc",
    "clipping_gyr",
]


def _vector_or_empty(vector):
    if vector is None:
        return ("", "", "")
    return (vector.x, vector.y, vector.z)


def _quat_or_empty(quat):
    if quat is None:
        return ("", "", "", "")
    return (quat.w, quat.x, quat.y, quat.z)


def _euler_or_empty(euler):
    if euler is None:
        return ("", "", "")
    return (euler.roll, euler.pitch, euler.yaw)


def export_sensor_data_to_csv(sensor: MovellaDOTSensor, output_dir: str) -> Optional[Path]:
    if not sensor.data_collector or not sensor.data_collector.data:
        return None

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    mac_address = sensor._device_address or "sensor"
    safe_id = mac_address.replace(":", "-")
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    csv_path = output_path / f"{safe_id}_{timestamp}.csv"

    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(CSV_COLUMNS)
        for idx, sample in enumerate(sensor.data_collector.data, start=1):
            quat_w, quat_x, quat_y, quat_z = _quat_or_empty(sample.quaternion)
            euler_roll, euler_pitch, euler_yaw = _euler_or_empty(sample.euler_angles)
            free_x, free_y, free_z = _vector_or_empty(sample.free_acceleration)
            acc_x, acc_y, acc_z = _vector_or_empty(sample.acceleration)
            gyr_x, gyr_y, gyr_z = _vector_or_empty(sample.angular_velocity)
            mag_x, mag_y, mag_z = _vector_or_empty(sample.magnetic_field)

            writer.writerow([
                idx,
                sample.timestamp.microseconds if sample.timestamp else "",
                sensor._device_tag or "",
                mac_address,
                quat_w,
                quat_x,
                quat_y,
                quat_z,
                euler_roll,
                euler_pitch,
                euler_yaw,
                free_x,
                free_y,
                free_z,
                acc_x,
                acc_y,
                acc_z,
                gyr_x,
                gyr_y,
                gyr_z,
                mag_x,
                mag_y,
                mag_z,
                sample.status.value if sample.status else "",
                sample.clipping_acc if sample.clipping_acc is not None else "",
                sample.clipping_gyr if sample.clipping_gyr is not None else "",
            ])

    return csv_path


class MultiDotController:
    def __init__(self, mac_addresses, config, log_queue: Optional[Queue] = None):
        self.mac_addresses = [mac.strip() for mac in mac_addresses if mac.strip()]
        self.config = config
        self.log_queue = log_queue
        self.sensors = {}
        self._lock = threading.Lock()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def log(self, message: str):
        if self.log_queue:
            self.log_queue.put(message)

    def run_coro(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def snapshot_sensors(self):
        with self._lock:
            return list(self.sensors.values())

    def _make_notification_handler(self, sensor: MovellaDOTSensor):
        def handler(sender, data: bytearray):
            try:
                if sensor.data_collector:
                    sensor.data_collector.add_data(data)
            except Exception as exc:
                self.log(f"{sensor._device_address} notification error: {exc}")

        return handler

    async def connect_all(self) -> bool:
        connected = {}
        for mac in self.mac_addresses:
            sensor = MovellaDOTSensor(self.config)
            sensor.client = BleakClient(mac)
            try:
                self.log(f"Connecting {mac}...")
                await sensor.client.connect()
                sensor.is_connected = True
                sensor._device_address = mac
                sensor._device_name = mac

                try:
                    info = await sensor.get_device_info()
                    sensor._device_tag = info.device_tag
                except Exception as exc:
                    self.log(f"{mac} device info error: {exc}")

                await sensor.configure_sensor()
                sensor.notification_handler = self._make_notification_handler(sensor)
                connected[mac] = sensor
                self.log(f"Connected {mac}")
            except Exception as exc:
                self.log(f"Failed to connect {mac}: {exc}")

        with self._lock:
            self.sensors = connected

        return bool(connected)

    async def start_receiving(self) -> bool:
        with self._lock:
            sensors = list(self.sensors.values())

        if not sensors:
            raise RuntimeError("No sensors connected")

        started_any = False
        for sensor in sensors:
            try:
                if sensor.data_collector:
                    sensor.data_collector.clear()
                await sensor.start_measurement()
                started_any = True
            except Exception as exc:
                self.log(f"{sensor._device_address} start error: {exc}")

        return started_any

    async def stop_receiving(self) -> bool:
        with self._lock:
            sensors = list(self.sensors.values())

        if not sensors:
            return False

        stopped_any = False
        for sensor in sensors:
            try:
                await sensor.stop_measurement()
                stopped_any = True
            except Exception as exc:
                self.log(f"{sensor._device_address} stop error: {exc}")

        return stopped_any

    async def disconnect_all(self):
        with self._lock:
            sensors = list(self.sensors.values())

        if sensors:
            await self.stop_receiving()
            await asyncio.gather(
                *(sensor.disconnect() for sensor in sensors),
                return_exceptions=True,
            )

        with self._lock:
            self.sensors = {}

    def shutdown(self):
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2)


class MultiDotUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Movella DOT Multi-Device")
        self.root.geometry("720x520")

        self.log_queue = Queue()
        config = SensorConfiguration(
            output_rate=OUTPUT_RATE,
            filter_profile=FILTER_PROFILE,
            payload_mode=PAYLOAD_MODE,
        )
        self.controller = MultiDotController(
            MAC_ADDRESSES,
            config,
            log_queue=self.log_queue,
        )

        self.status_var = tk.StringVar(value="Idle")
        self._build_ui()
        self._poll_logs()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.pack(fill=tk.BOTH, expand=True)

        info_frame = ttk.Frame(main_frame)
        info_frame.pack(fill=tk.X)

        ttk.Label(info_frame, text="MAC addresses (edit in this file):").pack(anchor=tk.W)
        ttk.Label(
            info_frame,
            text=", ".join(MAC_ADDRESSES),
            foreground="#555555",
            wraplength=680,
        ).pack(anchor=tk.W, pady=(0, 8))

        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(status_frame, text="Status:").pack(side=tk.LEFT)
        ttk.Label(status_frame, textvariable=self.status_var).pack(side=tk.LEFT, padx=(6, 0))

        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(0, 8))

        self.connect_btn = ttk.Button(button_frame, text="Connect", command=self._on_connect)
        self.start_btn = ttk.Button(
            button_frame,
            text="Start Receive",
            command=self._on_start,
            state=tk.DISABLED,
        )
        self.stop_btn = ttk.Button(
            button_frame,
            text="Stop Receive",
            command=self._on_stop,
            state=tk.DISABLED,
        )
        self.save_btn = ttk.Button(
            button_frame,
            text="Save CSV",
            command=self._on_save,
            state=tk.DISABLED,
        )
        self.disconnect_btn = ttk.Button(
            button_frame,
            text="Disconnect",
            command=self._on_disconnect,
            state=tk.DISABLED,
        )

        for button in (
            self.connect_btn,
            self.start_btn,
            self.stop_btn,
            self.save_btn,
            self.disconnect_btn,
        ):
            button.pack(side=tk.LEFT, padx=4)

        self.log_text = ScrolledText(main_frame, height=16, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _poll_logs(self):
        try:
            while True:
                message = self.log_queue.get_nowait()
                self._append_log(message)
        except Empty:
            pass

        self.root.after(200, self._poll_logs)

    def _append_log(self, message: str):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _run_async(self, coro, done_callback):
        future = self.controller.run_coro(coro)

        def _handle_done(fut):
            error = None
            result = None
            try:
                result = fut.result()
            except Exception as exc:
                error = exc

            if done_callback:
                self.root.after(0, done_callback, result, error)

        future.add_done_callback(_handle_done)

    def _has_connected_sensors(self) -> bool:
        return bool(self.controller.snapshot_sensors())

    def _has_collected_data(self) -> bool:
        for sensor in self.controller.snapshot_sensors():
            if sensor.data_collector and sensor.data_collector.data:
                return True
        return False

    def _set_buttons(self, connect, start, stop, save, disconnect):
        self.connect_btn.configure(state=tk.NORMAL if connect else tk.DISABLED)
        self.start_btn.configure(state=tk.NORMAL if start else tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL if stop else tk.DISABLED)
        self.save_btn.configure(state=tk.NORMAL if save else tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL if disconnect else tk.DISABLED)

    def _on_connect(self):
        self.status_var.set("Connecting...")
        self._set_buttons(False, False, False, False, False)

        def _done(result, error):
            if error or not result:
                self._append_log(f"Connect error: {error or 'no devices connected'}")
                self.status_var.set("Connect failed")
                self._set_buttons(True, False, False, False, False)
                return

            self.status_var.set("Connected")
            self._set_buttons(False, True, False, False, True)

        self._run_async(self.controller.connect_all(), _done)

    def _on_start(self):
        self.status_var.set("Receiving...")
        self._set_buttons(False, False, False, False, False)

        def _done(result, error):
            if error or not result:
                self._append_log(f"Start error: {error or 'no sensors started'}")
                self.status_var.set("Start failed")
                if self._has_connected_sensors():
                    self._set_buttons(False, True, False, False, True)
                else:
                    self._set_buttons(True, False, False, False, False)
                return

            self._append_log("Started receiving data.")
            self._set_buttons(False, False, True, False, True)

        self._run_async(self.controller.start_receiving(), _done)

    def _on_stop(self):
        self.status_var.set("Stopping...")
        self._set_buttons(False, False, False, False, False)

        def _done(result, error):
            if error or not result:
                self._append_log(f"Stop error: {error or 'no sensors stopped'}")
                self.status_var.set("Stop failed")
            else:
                self._append_log("Stopped receiving data.")
                self.status_var.set("Stopped")

            save_enabled = self._has_collected_data()
            if self._has_connected_sensors():
                self._set_buttons(False, True, False, save_enabled, True)
            else:
                self._set_buttons(True, False, False, False, False)

        self._run_async(self.controller.stop_receiving(), _done)

    def _on_save(self):
        sensors = self.controller.snapshot_sensors()
        if not sensors:
            self._append_log("No sensors connected for export.")
            return

        saved_any = False
        for sensor in sensors:
            csv_path = export_sensor_data_to_csv(sensor, CSV_OUTPUT_DIR)
            if csv_path:
                saved_any = True
                self._append_log(f"Saved {csv_path}")
            else:
                self._append_log(f"No data to save for {sensor._device_address}")

        if saved_any:
            self.status_var.set("CSV saved")
        else:
            self.status_var.set("No data to save")

    def _on_disconnect(self):
        self.status_var.set("Disconnecting...")
        self._set_buttons(False, False, False, False, False)

        def _done(result, error):
            if error:
                self._append_log(f"Disconnect error: {error}")
            else:
                self._append_log("Disconnected all sensors.")
            self.status_var.set("Disconnected")
            self._set_buttons(True, False, False, False, False)

        self._run_async(self.controller.disconnect_all(), _done)

    def _on_close(self):
        def _done(result, error):
            if error:
                self._append_log(f"Shutdown disconnect error: {error}")
            self.controller.shutdown()
            self.root.destroy()

        if self.controller.snapshot_sensors():
            self._run_async(self.controller.disconnect_all(), _done)
        else:
            self.controller.shutdown()
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    app = MultiDotUI()
    app.run()


if __name__ == "__main__":
    main()
