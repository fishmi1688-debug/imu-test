#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/mi1_left_hip_style_phase_live.py"

CAN_SERIAL="${CAN_SERIAL:-}"
CAN_IFACE="${CAN_IFACE:-can0}"
SLCAN_SPEED="${SLCAN_SPEED:-s8}"
NODE_ID="${NODE_ID:-${LEFT_ID:-0x01}}"
SIDE="${SIDE:-left}"
PROTOCOL="${PROTOCOL:-auto}"
OUTPUT_RATE="${OUTPUT_RATE:-50}"
DATA_TIMEOUT="${DATA_TIMEOUT:-0.5}"
ENABLE_PLOT="${ENABLE_PLOT:-1}"
PLOT_WINDOW="${PLOT_WINDOW:-12}"
PLOT_RATE="${PLOT_RATE:-20}"
PRINT_EVERY="${PRINT_EVERY:-10}"
PRINT_STDOUT="${PRINT_STDOUT:-1}"
ANGLE_SOURCE="${ANGLE_SOURCE:-quat_sagittal}"
ACC_ANGLE_NUM_AXIS="${ACC_ANGLE_NUM_AXIS:-z}"
ACC_ANGLE_DEN_AXIS="${ACC_ANGLE_DEN_AXIS:-x}"
ACC_ANGLE_NUM_SIGN="${ACC_ANGLE_NUM_SIGN:--1}"
ACC_ANGLE_DEN_SIGN="${ACC_ANGLE_DEN_SIGN:--1}"
GYRO_AXIS="${GYRO_AXIS:-y}"
QUAT_THIGH_AXIS="${QUAT_THIGH_AXIS:-x}"
QUAT_FORWARD_AXIS="${QUAT_FORWARD_AXIS:-x}"
QUAT_VERTICAL_AXIS="${QUAT_VERTICAL_AXIS:-z}"
ANGLE_SIGN="${ANGLE_SIGN:--1}"
GYRO_SIGN="${GYRO_SIGN:-1}"
SWING_THRESHOLD="${SWING_THRESHOLD:-25}"
MOTION_TIMEOUT="${MOTION_TIMEOUT:-0.45}"
STARTUP_WARMUP_STRIDES="${STARTUP_WARMUP_STRIDES:-0}"
HIP_PHASE_OFFSET="${HIP_PHASE_OFFSET:-${PHASE_OFFSET:-0}}"
STARTUP_SEED_ENABLED="${STARTUP_SEED_ENABLED:-1}"
STARTUP_SEED_MIN_WINDOW="${STARTUP_SEED_MIN_WINDOW:-0.10}"
STARTUP_SEED_MAX_WINDOW="${STARTUP_SEED_MAX_WINDOW:-0.20}"
STARTUP_SEED_GYRO_THRESHOLD="${STARTUP_SEED_GYRO_THRESHOLD:-15}"
STARTUP_SEED_ANGLE_THRESHOLD="${STARTUP_SEED_ANGLE_THRESHOLD:-3}"
STARTUP_SEED_POSITIVE_PHASE="${STARTUP_SEED_POSITIVE_PHASE:-0}"
STARTUP_SEED_NEGATIVE_PHASE="${STARTUP_SEED_NEGATIVE_PHASE:--3.141592653589793}"
STARTUP_SEED_RAMP="${STARTUP_SEED_RAMP:-0.30}"
STOP_WINDOW="${STOP_WINDOW:-0.30}"
STOP_HOLD="${STOP_HOLD:-0.12}"
STOP_GYRO_RMS_THRESHOLD="${STOP_GYRO_RMS_THRESHOLD:-8}"
STOP_ANGLE_RANGE_THRESHOLD="${STOP_ANGLE_RANGE_THRESHOLD:-3}"
STOP_PHASE_RATE_LIMIT="${STOP_PHASE_RATE_LIMIT:-4.0}"
STOP_UNSTABLE_HOLD="${STOP_UNSTABLE_HOLD:-0.25}"
SAMPLES="${SAMPLES:--1}"
CSV_PATH="${CSV_PATH:-}"
MAX_FIELD_AGE="${MAX_FIELD_AGE:-}"
REQUIRE_QUATERNION="${REQUIRE_QUATERNION:-0}"

find_acm_can_serial() {
    local devices=()
    shopt -s nullglob
    devices=(/dev/ttyACM*)
    shopt -u nullglob

    if (( ${#devices[@]} == 0 )); then
        return 1
    fi

    printf '%s\n' "${devices[@]}" | sort -V | head -n 1
}

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
    if [[ -z "${CAN_SERIAL}" ]]; then
        if ! CAN_SERIAL="$(find_acm_can_serial)"; then
            echo "error: no /dev/ttyACM* device found. Connect the USB-CAN adapter or set CAN_SERIAL manually." >&2
            exit 1
        fi
        echo "Auto-selected CAN serial: ${CAN_SERIAL}" >&2
    fi

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

if [[ -z "${CAN_SERIAL}" ]]; then
    CAN_SERIAL="existing-${CAN_IFACE}"
fi

PYTHON_ARGS=(
    --interface "${CAN_IFACE}"
    --node-id "${NODE_ID}"
    --side "${SIDE}"
    --protocol "${PROTOCOL}"
    --rate "${OUTPUT_RATE}"
    --data-timeout "${DATA_TIMEOUT}"
    --angle-source "${ANGLE_SOURCE}"
    --acc-angle-num-axis "${ACC_ANGLE_NUM_AXIS}"
    --acc-angle-den-axis "${ACC_ANGLE_DEN_AXIS}"
    --acc-angle-num-sign "${ACC_ANGLE_NUM_SIGN}"
    --acc-angle-den-sign "${ACC_ANGLE_DEN_SIGN}"
    --gyro-axis "${GYRO_AXIS}"
    --quat-thigh-axis "${QUAT_THIGH_AXIS}"
    --quat-forward-axis "${QUAT_FORWARD_AXIS}"
    --quat-vertical-axis "${QUAT_VERTICAL_AXIS}"
    --angle-sign "${ANGLE_SIGN}"
    --gyro-sign "${GYRO_SIGN}"
    --swing-threshold "${SWING_THRESHOLD}"
    --motion-timeout "${MOTION_TIMEOUT}"
    --startup-warmup-strides "${STARTUP_WARMUP_STRIDES}"
    --hip-phase-offset "${HIP_PHASE_OFFSET}"
    --startup-seed-min-window "${STARTUP_SEED_MIN_WINDOW}"
    --startup-seed-max-window "${STARTUP_SEED_MAX_WINDOW}"
    --startup-seed-gyro-threshold "${STARTUP_SEED_GYRO_THRESHOLD}"
    --startup-seed-angle-threshold "${STARTUP_SEED_ANGLE_THRESHOLD}"
    --startup-seed-positive-phase "${STARTUP_SEED_POSITIVE_PHASE}"
    --startup-seed-negative-phase "${STARTUP_SEED_NEGATIVE_PHASE}"
    --startup-seed-ramp "${STARTUP_SEED_RAMP}"
    --stop-window "${STOP_WINDOW}"
    --stop-hold "${STOP_HOLD}"
    --stop-gyro-rms-threshold "${STOP_GYRO_RMS_THRESHOLD}"
    --stop-angle-range-threshold "${STOP_ANGLE_RANGE_THRESHOLD}"
    --stop-phase-rate-limit "${STOP_PHASE_RATE_LIMIT}"
    --stop-unstable-hold "${STOP_UNSTABLE_HOLD}"
    --plot-window "${PLOT_WINDOW}"
    --plot-rate "${PLOT_RATE}"
    --samples "${SAMPLES}"
)

if [[ -n "${MAX_FIELD_AGE}" ]]; then
    PYTHON_ARGS+=(--max-field-age "${MAX_FIELD_AGE}")
fi

if [[ "${REQUIRE_QUATERNION}" != "0" ]]; then
    PYTHON_ARGS+=(--require-quaternion)
fi

if [[ "${STARTUP_SEED_ENABLED}" == "0" ]]; then
    PYTHON_ARGS+=(--disable-startup-seed)
fi

if [[ "${ENABLE_PLOT}" == "0" ]]; then
    PYTHON_ARGS+=(--no-plot)
fi

if [[ "${PRINT_STDOUT}" == "0" ]]; then
    PYTHON_ARGS+=(--print-every 0)
else
    PYTHON_ARGS+=(--print-every "${PRINT_EVERY}")
fi

if [[ -n "${CSV_PATH}" ]]; then
    PYTHON_ARGS+=(--csv "${CSV_PATH}")
fi

echo "Using ${CAN_IFACE} via ${CAN_SERIAL}; slcan speed ${SLCAN_SPEED} (s8 = 1 Mbit/s)." >&2
echo "MI1 hip-style phase test: node=${NODE_ID}, side=${SIDE}, protocol=${PROTOCOL}, rate=${OUTPUT_RATE}Hz." >&2
echo "Mount config: X down, Y left, Z back; angle=${ANGLE_SOURCE} (quat_sagittal is heading-free), quat_axes=thigh:${QUAT_THIGH_AXIS},fallback_forward:${QUAT_FORWARD_AXIS},vertical:${QUAT_VERTICAL_AXIS}, acc fallback=atan2(${ACC_ANGLE_NUM_SIGN}*${ACC_ANGLE_NUM_AXIS},${ACC_ANGLE_DEN_SIGN}*${ACC_ANGLE_DEN_AXIS}), gyro=${GYRO_AXIS}, signs=(${ANGLE_SIGN},${GYRO_SIGN})." >&2
echo "Hip phase offset: ${HIP_PHASE_OFFSET} cycles; startup warmup=${STARTUP_WARMUP_STRIDES} stride events; purple dashed phase is the old event-reference estimator." >&2
echo "Startup seed: enabled=${STARTUP_SEED_ENABLED}, window=${STARTUP_SEED_MIN_WINDOW}-${STARTUP_SEED_MAX_WINDOW}s, gyro_thr=${STARTUP_SEED_GYRO_THRESHOLD}deg/s, ramp=${STARTUP_SEED_RAMP}s." >&2
echo "Stop detect: window=${STOP_WINDOW}s, hold=${STOP_HOLD}s, gyro_rms_thr=${STOP_GYRO_RMS_THRESHOLD}deg/s, angle_range_thr=${STOP_ANGLE_RANGE_THRESHOLD}deg, phase_rate_limit=${STOP_PHASE_RATE_LIMIT}Hz, unstable_hold=${STOP_UNSTABLE_HOLD}s." >&2
echo "Plot: ${ENABLE_PLOT} window=${PLOT_WINDOW}s refresh=${PLOT_RATE}Hz; stdout every ${PRINT_EVERY} samples." >&2

exec python3 "${PYTHON_SCRIPT}" "${PYTHON_ARGS[@]}" "$@"
