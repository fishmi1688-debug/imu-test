#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/mi1_can_phase.py"

CAN_SERIAL="${CAN_SERIAL:-/dev/ttyACM0}"
CAN_IFACE="${CAN_IFACE:-can0}"
SLCAN_SPEED="${SLCAN_SPEED:-s8}"
LEFT_ID="${LEFT_ID:-0x01}"
RIGHT_ID="${RIGHT_ID:-0x02}"
OUTPUT_RATE="${OUTPUT_RATE:-50}"
ENABLE_PLOT="${ENABLE_PLOT:-1}"
PRINT_STDOUT="${PRINT_STDOUT:-0}"
ANGLE_SOURCE="${ANGLE_SOURCE:-quat_y}"
ACC_LONG_AXIS="${ACC_LONG_AXIS:-x}"
ACC_SAGITTAL_AXIS="${ACC_SAGITTAL_AXIS:-z}"
GYRO_AXIS="${GYRO_AXIS:-y}"

if [[ ! -f "${PYTHON_SCRIPT}" ]]; then
    echo "error: ${PYTHON_SCRIPT} not found." >&2
    exit 1
fi

if ! command -v slcand >/dev/null 2>&1; then
    echo "error: slcand not found. Install can-utils first." >&2
    exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
    echo "error: ip command not found. Install iproute2 first." >&2
    exit 1
fi

modprobe slcan 2>/dev/null || true

if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
    if [[ ! -e "${CAN_SERIAL}" ]]; then
        echo "error: ${CAN_SERIAL} does not exist. Check the USB-CAN adapter device path." >&2
        exit 1
    fi

    echo "Starting CAN interface: slcand -o -c -${SLCAN_SPEED} ${CAN_SERIAL} ${CAN_IFACE}" >&2
    slcand -o -c "-${SLCAN_SPEED}" "${CAN_SERIAL}" "${CAN_IFACE}"

    for _ in $(seq 1 30); do
        if ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
            break
        fi
        sleep 0.1
    done
fi

if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
    echo "error: ${CAN_IFACE} was not created by slcand." >&2
    exit 1
fi

ip link set "${CAN_IFACE}" up

PYTHON_ARGS=(
    --interface "${CAN_IFACE}"
    --left-id "${LEFT_ID}"
    --right-id "${RIGHT_ID}"
    --rate "${OUTPUT_RATE}"
    --angle-source "${ANGLE_SOURCE}"
    --acc-long-axis "${ACC_LONG_AXIS}"
    --acc-sagittal-axis "${ACC_SAGITTAL_AXIS}"
    --gyro-axis "${GYRO_AXIS}"
)

if [[ "${ENABLE_PLOT}" != "0" ]]; then
    PYTHON_ARGS+=(--plot)
fi

if [[ "${PRINT_STDOUT}" == "0" ]]; then
    PYTHON_ARGS+=(--no-stdout)
fi

echo "Using ${CAN_IFACE} via ${CAN_SERIAL}; slcan speed ${SLCAN_SPEED} (s8 = 1 Mbit/s)." >&2
echo "MI1 mount: +X thigh-up, +Y left, +Z forward; angle=${ANGLE_SOURCE}, acc=atan2(${ACC_SAGITTAL_AXIS},${ACC_LONG_AXIS}), gyro=${GYRO_AXIS}." >&2
exec python3 "${PYTHON_SCRIPT}" "${PYTHON_ARGS[@]}" "$@"
