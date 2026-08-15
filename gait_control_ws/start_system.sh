#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

CAN_INTERFACE="${GAIT_CAN_INTERFACE:-can0}"
CAN_BITRATE="${GAIT_CAN_BITRATE:-1000000}"

# 工作区优先使用脚本所在目录，避免root用户下 ~ 指向 /root 导致路径错误
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAIT_WORKSPACE="${GAIT_WORKSPACE:-$SCRIPT_DIR}"
cd "$GAIT_WORKSPACE" || {
    echo -e "${RED}❌ 无法进入工作区目录: ${GAIT_WORKSPACE}${NC}"
    exit 1
}

# 保存每次运行的终端日志，便于回看现场启动与运行信息。
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
export GAIT_RUNTIME_LOG_DIR="${GAIT_RUNTIME_LOG_DIR:-$GAIT_WORKSPACE/runtime_logs}"
if [ -z "${GAIT_RUN_LOG_TIMESTAMP:-}" ]; then
    export GAIT_RUN_LOG_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
fi
if [ -z "${GAIT_RUN_LOG_FILE:-}" ]; then
    export GAIT_RUN_LOG_FILE="${GAIT_RUNTIME_LOG_DIR}/gait_runtime_${GAIT_RUN_LOG_TIMESTAMP}.log"
fi
if [ "${GAIT_TERMINAL_LOGGING_READY:-0}" != "1" ]; then
    mkdir -p "$GAIT_RUNTIME_LOG_DIR" || {
        echo -e "${RED}❌ 无法创建运行日志目录: ${GAIT_RUNTIME_LOG_DIR}${NC}"
        exit 1
    }
    ln -sfn "$(basename "$GAIT_RUN_LOG_FILE")" "${GAIT_RUNTIME_LOG_DIR}/latest.log" 2>/dev/null || true
    export GAIT_TERMINAL_LOGGING_READY=1
    exec > >(tee -a "$GAIT_RUN_LOG_FILE") 2>&1
fi

echo -e "${BLUE}🚀 启动ROS2步态控制系统...${NC}"
echo -e "${YELLOW}📝 本次终端日志: ${GAIT_RUN_LOG_FILE}${NC}"
echo -e "${YELLOW}📝 最近一次日志: ${GAIT_RUNTIME_LOG_DIR}/latest.log${NC}"

# 如需CAN配置权限，自动尝试以sudo重新运行脚本（保留环境变量）
if [ -z "$GAIT_SUDO_RERUN" ] && [ "$(id -u)" -ne 0 ]; then
    if command -v sudo >/dev/null 2>&1; then
        echo -e "${YELLOW}🔑 尝试使用sudo获取CAN配置权限（可能会提示输入密码）...${NC}"
        sudo -E \
            GAIT_SUDO_RERUN=1 \
            GAIT_TERMINAL_LOGGING_READY="${GAIT_TERMINAL_LOGGING_READY}" \
            GAIT_RUNTIME_LOG_DIR="${GAIT_RUNTIME_LOG_DIR}" \
            GAIT_RUN_LOG_TIMESTAMP="${GAIT_RUN_LOG_TIMESTAMP}" \
            GAIT_RUN_LOG_FILE="${GAIT_RUN_LOG_FILE}" \
            PYTHONUNBUFFERED="${PYTHONUNBUFFERED}" \
            "$0" "$@"
        SUDO_STATUS=$?
        if [ $SUDO_STATUS -eq 0 ]; then
            exit 0  # 提升权限后脚本已完成或正在运行
        else
            echo -e "${YELLOW}⚠️  sudo重新运行失败或被拒绝，继续以当前权限运行（CAN配置可能失败）${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️  未找到sudo，继续以当前权限运行（CAN配置可能失败）${NC}"
    fi
fi

# 检查是否在正确的目录
if [ ! -f "setup_env.sh" ] || [ ! -f "setup_can.sh" ]; then
    echo -e "${RED}❌ 错误: 请在工作空间根目录运行此脚本${NC}"
    WORKSPACE_HINT=${GAIT_WORKSPACE:-/home/sunrise/gait_control_ws}
    echo -e "${YELLOW}💡 提示: cd ${WORKSPACE_HINT} && ./start_system.sh${NC}"
    exit 1
fi

# 设置环境
echo -e "${YELLOW}⚙️  设置ROS2环境...${NC}"
source ./setup_env.sh
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 环境设置失败${NC}"
    exit 1
fi
# 确保Python可以找到源码包（使用相对import的模块）
export PYTHONPATH="$PWD/src:$PWD/src/gait_control_system:${PYTHONPATH}"
# 有线IMU和电机共用同一个SocketCAN接口，默认都走 can0。
export GAIT_IMU_PHASE_CAN_INTERFACE="${GAIT_IMU_PHASE_CAN_INTERFACE:-$CAN_INTERFACE}"
# 有线IMU不再阻塞APP BLE启动；开始助力时仍会按模式检查IMU数据流。
export GAIT_IMU_PHASE_PRECONNECT="${GAIT_IMU_PHASE_PRECONNECT:-0}"
export GAIT_IMU_PHASE_PRECONNECT_REQUIRED="${GAIT_IMU_PHASE_PRECONNECT_REQUIRED:-0}"
export GAIT_IMU_PHASE_PRECONNECT_TIMEOUT_SEC="${GAIT_IMU_PHASE_PRECONNECT_TIMEOUT_SEC:-60}"
export GAIT_IMU_PHASE_KEEP_STREAMING="${GAIT_IMU_PHASE_KEEP_STREAMING:-1}"
export GAIT_IMU_PHASE_SCAN_BEFORE_CONNECT="${GAIT_IMU_PHASE_SCAN_BEFORE_CONNECT:-1}"
export GAIT_IMU_PHASE_PRECONNECT_LEFT_ONLY="${GAIT_IMU_PHASE_PRECONNECT_LEFT_ONLY:-1}"

wait_for_condition() {
    local timeout_s="${1:-5}"
    shift
    local checks=$((timeout_s * 10))
    if [ "$checks" -lt 1 ]; then
        checks=1
    fi
    for ((i=0; i<checks; i++)); do
        if "$@"; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

is_can_interface_up() {
    ip link show "$CAN_INTERFACE" 2>/dev/null | grep -q -E 'state UP|<[^>]*UP[^>]*>'
}

is_bluetooth_compat_ready() {
    pgrep -a bluetoothd 2>/dev/null | grep -q -- '--compat'
}

is_bluetooth_service_ready() {
    bluetoothctl show >/dev/null 2>&1
}

bluetooth_power_on_best_effort() {
    local HCI_DEVS=("${GAIT_BT_HCI:-hci0}")
    if [ -n "${GAIT_IMU_BLE_ADAPTER:-}" ] && [ "${GAIT_IMU_BLE_ADAPTER}" != "${HCI_DEVS[0]}" ]; then
        HCI_DEVS+=("${GAIT_IMU_BLE_ADAPTER}")
    fi
    local SUDO_CMD=()
    if [ "$(id -u)" -ne 0 ]; then
        if command -v sudo >/dev/null 2>&1; then
            SUDO_CMD=(sudo -E)
        else
            echo -e "${YELLOW}⚠️  当前非root且未找到sudo，蓝牙上电命令可能失败${NC}"
        fi
    fi

    if command -v rfkill >/dev/null 2>&1; then
        "${SUDO_CMD[@]}" rfkill unblock bluetooth 2>/dev/null || true
    fi
    if command -v bluetoothctl >/dev/null 2>&1; then
        "${SUDO_CMD[@]}" bluetoothctl power on >/dev/null 2>&1 || true
    fi
    if command -v btmgmt >/dev/null 2>&1; then
        "${SUDO_CMD[@]}" btmgmt power on >/dev/null 2>&1 || true
        "${SUDO_CMD[@]}" btmgmt connectable on >/dev/null 2>&1 || true
        "${SUDO_CMD[@]}" btmgmt discov yes >/dev/null 2>&1 || true
    fi
    if command -v hciconfig >/dev/null 2>&1; then
        for HCI_DEV in "${HCI_DEVS[@]}"; do
            "${SUDO_CMD[@]}" hciconfig "$HCI_DEV" up >/dev/null 2>&1 || true
        done
    fi
}

are_core_ros_nodes_ready() {
    local nodes
    nodes="$(ros2 node list 2>/dev/null || true)"
    [[ "$nodes" == *"/gait_analysis_node"* && "$nodes" == *"/motor_controller"* ]]
}

wait_for_main_process_ready() {
    local pid="$1"
    local timeout_s="${2:-12}"
    local checks=$((timeout_s * 10))
    if [ "$checks" -lt 1 ]; then
        checks=1
    fi
    for ((i=0; i<checks; i++)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            return 1
        fi
        if are_core_ros_nodes_ready; then
            return 0
        fi
        sleep 0.1
    done
    return 2
}

ensure_bluetooth_discoverable() {
    if ! command -v bluetoothctl >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠️  未找到bluetoothctl，无法设置可发现/可配对${NC}"
        return 1
    fi
    bluetooth_power_on_best_effort
    local GAIT_BT_FORGET_BONDED_EFFECTIVE="${GAIT_BT_FORGET_BONDED:-1}"
    # 默认允许配对，避免Android RFCOMM握手在未配对场景下被远端直接拒绝。
    # 历史配对仍由 GAIT_BT_FORGET_BONDED=1 在开机时清空，不会“记忆旧设备”。
    local GAIT_BT_PAIRABLE_EFFECTIVE="${GAIT_BT_PAIRABLE:-1}"
    local BT_CMD=(bluetoothctl)
    if [ "$(id -u)" -ne 0 ]; then
        if command -v sudo >/dev/null 2>&1; then
            BT_CMD=(sudo -E bluetoothctl)
        else
            echo -e "${YELLOW}⚠️  未找到sudo，无法设置可发现/可配对${NC}"
            return 1
        fi
    fi
    if [ "$GAIT_BT_FORGET_BONDED_EFFECTIVE" = "1" ]; then
        echo -e "${YELLOW}🧹 清理主控板蓝牙配对记录...${NC}"
        mapfile -t _PAIRED_MACS < <("${BT_CMD[@]}" paired-devices 2>/dev/null | awk '/^Device / {print $2}')
        if [ "${#_PAIRED_MACS[@]}" -eq 0 ]; then
            echo -e "${GREEN}✅ 无历史配对记录${NC}"
        else
            for _mac in "${_PAIRED_MACS[@]}"; do
                if "${BT_CMD[@]}" remove "${_mac}" >/dev/null 2>&1; then
                    echo -e "${GREEN}✅ 已移除配对: ${_mac}${NC}"
                else
                    echo -e "${YELLOW}⚠️  移除配对失败: ${_mac}${NC}"
                fi
            done
        fi
    fi

    local PAIRABLE_LINE="pairable off"
    if [ "$GAIT_BT_PAIRABLE_EFFECTIVE" = "1" ]; then
        PAIRABLE_LINE="pairable on"
        echo -e "${YELLOW}📶 设置蓝牙可发现/可配对（允许配对）...${NC}"
    else
        echo -e "${YELLOW}📶 设置蓝牙可发现（禁止新配对）...${NC}"
    fi

    "${BT_CMD[@]}" <<EOF
power on
agent on
default-agent
discoverable on
${PAIRABLE_LINE}
discoverable-timeout 0
EOF

    bluetooth_power_on_best_effort

    local BT_SHOW
    BT_SHOW="$("${BT_CMD[@]}" show 2>/dev/null || true)"
    if ! grep -q "Powered: yes" <<< "$BT_SHOW"; then
        echo -e "${YELLOW}⚡ 检测到蓝牙控制器未上电，正在尝试重新上电...${NC}"
        bluetooth_power_on_best_effort
        sleep 0.5
        "${BT_CMD[@]}" <<EOF >/dev/null 2>&1 || true
power on
agent on
default-agent
discoverable on
${PAIRABLE_LINE}
discoverable-timeout 0
EOF
        sleep 0.5
        BT_SHOW="$("${BT_CMD[@]}" show 2>/dev/null || true)"
    fi
    if ! grep -q "Powered: yes" <<< "$BT_SHOW"; then
        echo -e "${YELLOW}⚠️  蓝牙控制器仍未上电(Powered != yes)，手机将无法发现设备${NC}"
        return 1
    fi
    if ! grep -q "Discoverable: yes" <<< "$BT_SHOW"; then
        echo -e "${YELLOW}📶 蓝牙已上电但不可发现，正在重新设置 discoverable...${NC}"
        "${BT_CMD[@]}" discoverable on >/dev/null 2>&1 || true
        if command -v btmgmt >/dev/null 2>&1; then
            if [ "$(id -u)" -eq 0 ]; then
                btmgmt discov yes >/dev/null 2>&1 || true
            elif command -v sudo >/dev/null 2>&1; then
                sudo -E btmgmt discov yes >/dev/null 2>&1 || true
            fi
        fi
        sleep 0.3
        BT_SHOW="$("${BT_CMD[@]}" show 2>/dev/null || true)"
    fi
    if ! grep -q "Discoverable: yes" <<< "$BT_SHOW"; then
        echo -e "${YELLOW}⚠️  蓝牙控制器未进入可发现状态(Discoverable != yes)${NC}"
        return 1
    fi
    echo -e "${GREEN}✅ 蓝牙控制器已上电并可发现${NC}"
}

stop_aux_bluetooth_services() {
    if [ "$(id -u)" -ne 0 ]; then
        return 0
    fi
    local GAIT_BT_MINIMAL_PROFILE_EFFECTIVE="${GAIT_BT_MINIMAL_PROFILE:-1}"
    if [ "$GAIT_BT_MINIMAL_PROFILE_EFFECTIVE" != "1" ]; then
        return 0
    fi
    echo -e "${YELLOW}🧹 停止非必要蓝牙辅助服务（OBEX等）...${NC}"
    systemctl stop obex.service obex.socket 2>/dev/null || true
    pkill -x obexd 2>/dev/null || true
}

restart_bluetooth_service() {
    if [ "$(id -u)" -ne 0 ]; then
        return 0
    fi
    if ! command -v systemctl >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠️  未找到 systemctl，跳过 bluetooth 服务重启${NC}"
        return 1
    fi
    echo -e "${YELLOW}🔄 重启 bluetooth 服务，清理旧 SDP/RFCOMM 状态...${NC}"
    systemctl restart bluetooth
    if wait_for_condition "${GAIT_BT_RESTART_TIMEOUT:-5}" is_bluetooth_service_ready; then
        echo -e "${GREEN}✅ bluetooth 服务重启完成${NC}"
        return 0
    fi
    echo -e "${YELLOW}⚠️  bluetooth 服务已重启，但未在超时内确认就绪${NC}"
    return 1
}

# 确保 bluetoothd 以 --compat 模式运行（sdptool 注册 SDP 记录的前提）
ensure_bluetooth_compat() {
    # 已经是 root 才有权改 systemd 服务文件
    if [ "$(id -u)" -ne 0 ]; then
        return 0
    fi
    local GAIT_BT_MINIMAL_PROFILE_EFFECTIVE="${GAIT_BT_MINIMAL_PROFILE:-1}"
    local GAIT_BT_NOPLUGIN_EFFECTIVE="${GAIT_BT_NOPLUGIN:-a2dp,avrcp,network,input,hog,sap}"
    local current_exec
    current_exec="$(systemctl show bluetooth.service --property=ExecStart 2>/dev/null || true)"
    local current_proc
    current_proc="$(pgrep -a bluetoothd 2>/dev/null | head -1 || true)"
    local service_has_compat=0
    local proc_has_compat=0
    local service_matches_minimal=0
    local proc_matches_minimal=0
    if [[ "$current_exec" == *"--compat"* ]]; then
        service_has_compat=1
    fi
    if [[ "$current_proc" == *"--compat"* ]]; then
        proc_has_compat=1
    fi
    if [ "$GAIT_BT_MINIMAL_PROFILE_EFFECTIVE" = "1" ]; then
        if [[ "$current_exec" == *"--noplugin=${GAIT_BT_NOPLUGIN_EFFECTIVE}"* ]]; then
            service_matches_minimal=1
        fi
        if [[ "$current_proc" == *"--noplugin=${GAIT_BT_NOPLUGIN_EFFECTIVE}"* ]]; then
            proc_matches_minimal=1
        fi
    else
        if [[ "$current_exec" != *"--noplugin="* ]]; then
            service_matches_minimal=1
        fi
        if [[ "$current_proc" != *"--noplugin="* ]]; then
            proc_matches_minimal=1
        fi
    fi
    if [ "$service_has_compat" = "1" ] && [ "$proc_has_compat" = "1" ] \
        && [ "$service_matches_minimal" = "1" ] && [ "$proc_matches_minimal" = "1" ]; then
        return 0
    fi
    echo -e "${YELLOW}🔧 正在刷新 bluetoothd 启动参数（compat/minimal profile）...${NC}"
    local OVERRIDE_DIR="/etc/systemd/system/bluetooth.service.d"
    local OVERRIDE_FILE="${OVERRIDE_DIR}/compat.conf"
    mkdir -p "$OVERRIDE_DIR"
    # 先找到 bluetoothd 的实际路径
    local BT_DAEMON
    BT_DAEMON="$(systemctl cat bluetooth.service 2>/dev/null \
        | grep '^ExecStart=' | head -1 \
        | sed 's/ExecStart=//' | awk '{print $1}')"
    BT_DAEMON="${BT_DAEMON:-/usr/libexec/bluetooth/bluetoothd}"
    if [ ! -x "$BT_DAEMON" ]; then
        BT_DAEMON="$(command -v bluetoothd 2>/dev/null || echo /usr/lib/bluetooth/bluetoothd)"
    fi
    # 写入 override：先清空默认 ExecStart 再设置带 --compat 的版本
    local BT_EXEC_ARGS="--compat"
    if [ "$GAIT_BT_MINIMAL_PROFILE_EFFECTIVE" = "1" ] && [ -n "$GAIT_BT_NOPLUGIN_EFFECTIVE" ]; then
        BT_EXEC_ARGS="${BT_EXEC_ARGS} --noplugin=${GAIT_BT_NOPLUGIN_EFFECTIVE}"
        echo -e "${YELLOW}🎛️  启用最小蓝牙配置，仅保留 SPP/RFCOMM 相关路径${NC}"
        echo -e "${YELLOW}   禁用插件: ${GAIT_BT_NOPLUGIN_EFFECTIVE}${NC}"
    fi
    cat > "$OVERRIDE_FILE" << EOF
[Service]
ExecStart=
ExecStart=${BT_DAEMON} ${BT_EXEC_ARGS}
EOF
    systemctl daemon-reload
    systemctl restart bluetooth
    if wait_for_condition "${GAIT_BT_COMPAT_TIMEOUT:-3}" is_bluetooth_compat_ready; then
        echo -e "${GREEN}✅ bluetoothd 已切换为 --compat 模式（sdptool 可正常注册 SDP）${NC}"
    else
        echo -e "${YELLOW}⚠️  bluetoothd 重启完成，但未确认 --compat 是否生效（继续执行）${NC}"
    fi
}

prepare_bluetooth_stack() {
    ensure_bluetooth_compat || true
    restart_bluetooth_service || true
    stop_aux_bluetooth_services || true
}

# 强制关闭调试模式（默认性能模式）
unset GAIT_DEBUG
echo -e "${BLUE}⚡ 性能模式已启用 (GAIT_DEBUG 已关闭)${NC}"

# 配置CAN接口
echo -e "${YELLOW}⚙️  配置CAN接口...${NC}"
CAN_AVAILABLE=false

# 检查系统默认CAN接口是否存在
if ! ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠️  未检测到默认CAN接口: ${CAN_INTERFACE}${NC}"
    CAN_AVAILABLE=false
else
    # 尝试配置CAN接口
    ./setup_can.sh
    if [ $? -ne 0 ]; then
        echo -e "${YELLOW}⚠️  CAN接口配置失败${NC}"
        CAN_AVAILABLE=false
    else
        # 轮询CAN接口状态，避免固定等待
        echo -e "${YELLOW}⏳ 等待CAN接口就绪...${NC}"
        if ! wait_for_condition "${GAIT_CAN_READY_TIMEOUT:-5}" is_can_interface_up; then
            echo -e "${YELLOW}⚠️  ${CAN_INTERFACE} 接口未正确配置${NC}"
            CAN_AVAILABLE=false
        else
            echo -e "${GREEN}✅ ${CAN_INTERFACE} 接口配置成功，bitrate=${CAN_BITRATE}${NC}"
            CAN_AVAILABLE=true
        fi
    fi
fi

# 构建系统（如果需要）
echo -e "${YELLOW}🔨 检查并构建系统...${NC}"
if [ ! -d "install/gait_control_system" ]; then
    echo -e "${YELLOW}📦 首次运行，正在构建系统...${NC}"
    colcon build --packages-select gait_control_system
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ 系统构建失败${NC}"
        exit 1
    fi
fi

if [ "$CAN_AVAILABLE" = false ]; then
    echo -e "${YELLOW}⚠️  CAN设备不可用，进入降级模式（仅蓝牙服务）${NC}"
    echo -e "${YELLOW}💡 连接CAN设备后重启可进入完整系统${NC}"
    if [ "${GAIT_BT_ENABLE:-1}" != "0" ]; then
        prepare_bluetooth_stack || true
        ensure_bluetooth_discoverable || true
        echo -e "${YELLOW}📡 启动蓝牙RFCOMM服务...${NC}"
        BT_CMD=(python3 -u -m gait_control_system.gait_control_system.bluetooth_server)
        if [ "$(id -u)" -ne 0 ]; then
            if command -v sudo >/dev/null 2>&1; then
                sudo -E "${BT_CMD[@]}" &
                BT_PID=$!
            else
                echo -e "${YELLOW}⚠️  未找到sudo，蓝牙服务可能无法启动（权限不足）${NC}"
                "${BT_CMD[@]}" &
                BT_PID=$!
            fi
        else
            "${BT_CMD[@]}" &
            BT_PID=$!
        fi
        echo -e "${YELLOW}💡 蓝牙服务 PID: $BT_PID${NC}"
    else
        echo -e "${YELLOW}🔕 蓝牙服务已禁用 (GAIT_BT_ENABLE=0)${NC}"
    fi
    echo -e "${YELLOW}💡 按 Ctrl+C 停止蓝牙服务${NC}"
    trap "echo -e '\n${YELLOW}🛑 停止蓝牙服务...${NC}'; if [ -n \"${BT_PID:-}\" ]; then kill $BT_PID 2>/dev/null; fi; exit" INT
    if [ -n "${BT_PID:-}" ]; then
        wait "$BT_PID"
    else
        echo -e "${YELLOW}⚠️ 未启动任何后台进程，脚本退出${NC}"
    fi
    exit 0
fi

echo -e "${BLUE}🔄 启动完整系统...${NC}"
# 在后台启动主程序
echo -e "${YELLOW}🤖 启动步态控制主程序...${NC}"
prepare_bluetooth_stack || true
ensure_bluetooth_discoverable || true
python3 -u -m gait_control_system.gait_control_system.last3_optimized &
MAIN_PID=$!
wait_for_main_process_ready "$MAIN_PID" "${GAIT_MAIN_READY_TIMEOUT:-12}"
READY_STATUS=$?
if [ $READY_STATUS -eq 0 ]; then
    echo -e "${GREEN}✅ 系统启动完成（核心ROS节点已就绪）${NC}"
elif [ $READY_STATUS -eq 1 ]; then
    echo -e "${RED}❌ 主程序启动失败（进程已退出）${NC}"
    exit 1
else
    echo -e "${YELLOW}⚠️ 主程序进程已启动，但核心ROS节点未在超时内就绪（继续运行）${NC}"
fi
echo -e "${YELLOW}💡 主程序PID: $MAIN_PID${NC}"
echo -e "${YELLOW}💡 按 Ctrl+C 停止系统${NC}"
trap "echo -e '\n${YELLOW}🛑 停止系统...${NC}'; kill $MAIN_PID 2>/dev/null; exit" INT
wait

echo -e "${GREEN}✅ 系统已退出${NC}"
