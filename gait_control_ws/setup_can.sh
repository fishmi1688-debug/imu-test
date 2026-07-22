#!/bin/bash
# CAN接口配置脚本 - 直接使用系统默认 can0

CAN_INTERFACE="${GAIT_CAN_INTERFACE:-can0}"
CAN_BITRATE="${GAIT_CAN_BITRATE:-1000000}"

# 确定是否需要sudo（在带有 no-new-privileges 的容器中sudo会失败）
SUDO_CMD=""
PRIV_READY=true
if [ "$(id -u)" -ne 0 ]; then
    if command -v sudo >/dev/null 2>&1; then
        SUDO_CHECK_OUTPUT=$(sudo -n true 2>&1)
        if [ $? -eq 0 ]; then
            SUDO_CMD="sudo"
        else
            SUDO_CMD="sudo"  # 仍然尝试，允许提示输入密码
            if echo "$SUDO_CHECK_OUTPUT" | grep -qi "no new privileges"; then
                PRIV_READY=false
                SUDO_CMD=""  # 不要再用sudo，以免持续失败
                echo "⚠️ sudo被 no-new-privileges 限制，将直接配置CAN接口（可能需要root/CAP_NET_ADMIN）"
            elif [ -n "$SUDO_CHECK_OUTPUT" ]; then
                echo "🔑 sudo可能需要密码，配置CAN时会提示输入"
            else
                echo "⚠️ sudo检查失败，将尝试直接配置CAN接口"
            fi
        fi
    else
        PRIV_READY=false
        echo "⚠️ 未找到sudo，将直接尝试配置CAN接口"
    fi
fi

run_can_cmd() {
    if [ -n "$SUDO_CMD" ]; then
        $SUDO_CMD "$@"
    else
        "$@"
    fi
}

# 检查系统默认CAN接口是否存在
if ! ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    echo "错误: 未找到默认CAN接口 ${CAN_INTERFACE}"
    exit 1
fi

echo "使用系统默认CAN接口: ${CAN_INTERFACE}"

# 设置CAN接口
echo "配置CAN接口 ${CAN_INTERFACE}，bitrate=${CAN_BITRATE}..."
run_can_cmd ip link set "$CAN_INTERFACE" down 2>/dev/null || true
if run_can_cmd ip link set "$CAN_INTERFACE" up type can bitrate "$CAN_BITRATE"; then
    echo "CAN接口配置成功"
else
    echo "CAN接口配置失败"
    if [ "$PRIV_READY" = false ]; then
        echo "❌ 失败原因可能是缺少网络配置权限。请以root运行，或在容器中授予CAP_NET_ADMIN/关闭 no-new-privileges。"
    fi
    exit 1
fi

# 检查接口状态
if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
    echo "✅ ${CAN_INTERFACE} 接口配置成功"
else
    echo "❌ ${CAN_INTERFACE} 接口配置失败"
    exit 1
fi
