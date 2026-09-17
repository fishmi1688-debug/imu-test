#!/usr/bin/env python3
"""PC-side mock gait BLE Notify server for the Android APP."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
GAIT_SRC = os.path.join(REPO_ROOT, "gait_control_ws", "src", "gait_control_system")
if GAIT_SRC not in sys.path:
    sys.path.insert(0, GAIT_SRC)

from gait_control_system.bluetooth_server import (  # noqa: E402
    MODE_KEYS,
    StreamSettings,
    build_default_server,
    build_plot_frame_batch,
    build_state_frame,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mock gait BLE Notify server that sends binary plot/state frames to the Android APP.")
    parser.add_argument("--bind", default="any", help="Bluetooth adapter address to bind, or 'any'.")
    parser.add_argument("--name", default="GaitControl", help="BLE local name.")
    parser.add_argument("--alias", default="", help="Optional local Bluetooth alias, e.g. ubuntu.")
    parser.add_argument("--setup-adapter", action="store_true", help="Run bluetoothctl/rfkill commands to power on, pairable on, discoverable on, and optionally set alias.")
    parser.add_argument("--batch-size", type=int, default=5, help="Default plot samples per binary batch.")
    parser.add_argument("--sample-rate", type=float, default=50.0, help="Simulated sample rate in Hz.")
    parser.add_argument("--state-rate", type=float, default=1.0, help="State push rate in Hz.")
    parser.add_argument("--wave-hz", type=float, default=0.8, help="Simulated gait waveform frequency.")
    parser.add_argument("--amplitude", type=float, default=30.0, help="Simulated angle amplitude in degrees.")
    parser.add_argument("--notify-chunk", type=int, default=65535, help="BLE Notify chunk size; set large to avoid mock-side slicing.")
    parser.add_argument("--duration", type=float, default=0.0, help="Optional runtime limit per connection in seconds.")
    return parser.parse_args()


def setup_bluetooth_adapter(alias: str = "") -> None:
    import shutil
    import subprocess

    commands: list[list[str]] = []
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
    args = parse_args()
    os.environ["GAIT_BT_BIND_ADDR"] = args.bind
    os.environ["GAIT_BT_NAME"] = args.name
    os.environ["GAIT_BT_NOTIFY_CHUNK"] = str(max(1, int(args.notify_chunk)))
    if args.setup_adapter:
        setup_bluetooth_adapter(alias=args.alias)
    else:
        print("[hint] If the APP cannot find this PC, rerun with: --setup-adapter --alias ubuntu", flush=True)

    state = StreamSettings(batch_size=args.batch_size)
    lock = threading.Lock()
    handshake_ready = False
    session_started_at = 0.0
    next_plot_time = 0.0
    next_state_time = 0.0
    sample_index = 0

    def push_state(server) -> None:
        server.send_bytes(
            build_state_frame(state, seq=sample_index),
            payload_desc="state",
            soft_realtime=True,
        )

    def on_message(message: str):
        nonlocal handshake_ready, session_started_at, next_plot_time, next_state_time, sample_index
        text = str(message or "").strip()
        if not text:
            return None
        try:
            payload = json.loads(text)
        except Exception:
            payload = None

        msg_type = 1 if text.lower() == "ping" else int(payload.get("t", -1)) if isinstance(payload, dict) else -1
        server = server_holder["server"]

        with lock:
            if msg_type == 1:
                handshake_ready = True
                session_started_at = time.monotonic()
                next_plot_time = session_started_at
                next_state_time = session_started_at + 0.25
                push_state(server)
                return json.dumps({"t": 13, "o": 1}, ensure_ascii=False, separators=(",", ":"))

            if msg_type == 6 and isinstance(payload, dict):
                state.plot_format = int(payload.get("pf", state.plot_format))
                state.plot_mode = int(payload.get("pm", state.plot_mode))
                state.batch_size = max(1, min(255, int(payload.get("pn", state.batch_size))))
                state.every_n = max(1, int(payload.get("pe", state.every_n)))
                push_state(server)
                return json.dumps(
                    {
                        "t": 11,
                        "o": 1,
                        "a": 3,
                        "pf": state.plot_format,
                        "pm": state.plot_mode,
                        "pn": state.batch_size,
                        "pe": state.every_n,
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )

            if msg_type == 2:
                return json.dumps(
                    {"t": 8, "v": 1, "mi": state.mode_code, "gs": 2, "f": 0, "ds": 0.92},
                    ensure_ascii=False,
                    separators=(",", ":"),
                )

            if msg_type == 3 and isinstance(payload, dict):
                state.mode_code = max(0, min(len(MODE_KEYS) - 1, int(payload.get("m", 0))))
                push_state(server)
                return json.dumps(
                    {"t": 11, "o": 1, "a": 4, "m": state.mode_code},
                    ensure_ascii=False,
                    separators=(",", ":"),
                )

            if msg_type == 4:
                return json.dumps({"t": 11, "o": 1, "a": 5, "n": 0}, ensure_ascii=False, separators=(",", ":"))

            if msg_type == 5 and isinstance(payload, dict):
                command = int(payload.get("c", 0))
                action = 1 if command == 1 else 2 if command == 3 else command
                return json.dumps({"t": 11, "o": 1, "a": action}, ensure_ascii=False, separators=(",", ":"))

            if msg_type == 7 and isinstance(payload, dict):
                return json.dumps(
                    {
                        "t": 15,
                        "o": 1,
                        "s": int(payload.get("s", 0)),
                        "k": 1,
                        "q": 1,
                        "d": 1,
                        "z": 0,
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )

            return json.dumps({"t": 12, "o": 0, "ec": 1}, ensure_ascii=False, separators=(",", ":"))

    server_holder = {"server": build_default_server(on_message=on_message)}
    server_holder["server"].start()
    print("[ready] waiting for APP to subscribe to BLE Notify and send ping.", flush=True)

    try:
        while True:
            server = server_holder["server"]
            if not server.is_client_connected():
                time.sleep(0.1)
                continue

            now = time.monotonic()
            if args.duration > 0 and session_started_at > 0 and now - session_started_at >= args.duration:
                time.sleep(0.25)
                continue

            if handshake_ready and now >= next_plot_time:
                batch_size = max(1, min(255, int(state.batch_size)))
                frame = build_plot_frame_batch(
                    sample_index=sample_index,
                    count=batch_size,
                    sample_rate_hz=args.sample_rate,
                    wave_hz=args.wave_hz,
                    amplitude_deg=args.amplitude,
                )
                server.send_bytes(frame, payload_desc=f"plot_bin[{batch_size}]", soft_realtime=True)
                sample_index += batch_size
                next_plot_time = max(next_plot_time + batch_size / max(1.0, args.sample_rate), now + 0.01)

            if handshake_ready and now >= next_state_time:
                push_state(server)
                next_state_time = now + 1.0 / max(0.1, args.state_rate)

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[stop] interrupted", flush=True)
    finally:
        server_holder["server"].stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
