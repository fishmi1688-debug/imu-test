#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    exec sudo -E "$0" "$@"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/rdkx5_sen0694_bitbang_i2c.py"

modprobe i2c-gpio
modprobe i2c-dev

bus=""
for _ in $(seq 1 30); do
    for adapter in /sys/class/i2c-dev/i2c-*; do
        [[ -r "${adapter}/name" ]] || continue
        if [[ "$(cat "${adapter}/name")" == "i2c-gpio-h2" ]]; then
            bus="${adapter##*/}"
            bus="${bus#i2c-}"
            break 2
        fi
    done
    sleep 0.1
done

if [[ -z "${bus}" ]]; then
    echo "error: i2c-gpio-h2 was not created after loading i2c-gpio/i2c-dev." >&2
    echo "Current I2C adapters:" >&2
    i2cdetect -l >&2 || true
    exit 1
fi

echo "Using i2c-gpio-h2 on /dev/i2c-${bus}" >&2
exec python3 "${PYTHON_SCRIPT}" --bus "${bus}" "$@"
