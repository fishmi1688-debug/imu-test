#!/bin/bash
# ROS2环境配置脚本

# 自动识别工作区目录（优先 GAIT_WORKSPACE，其次脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAIT_WORKSPACE="${GAIT_WORKSPACE:-$SCRIPT_DIR}"

prepend_path() {
    local name="$1"
    local value="$2"
    if [ -z "$value" ] || [ ! -e "$value" ]; then
        return 0
    fi
    local current="${!name:-}"
    case ":$current:" in
        *":$value:"*) ;;
        *) export "$name=$value${current:+:$current}" ;;
    esac
}

# 基础ROS2环境
if [ "${GAIT_FAST_ROS_ENV:-1}" = "1" ] && [ -d "/opt/ros/humble" ]; then
    export ROS_VERSION=2
    export ROS_PYTHON_VERSION=3
    export ROS_DISTRO=humble

    PYTHON_BIN="${GAIT_PYTHON_BIN:-python3}"
    PYTHON_VERSION_SHORT="$("$PYTHON_BIN" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    prepend_path PATH "/opt/ros/humble/bin"
    prepend_path PYTHONPATH "/opt/ros/humble/lib/python${PYTHON_VERSION_SHORT}/site-packages"
    prepend_path PYTHONPATH "/opt/ros/humble/local/lib/python${PYTHON_VERSION_SHORT}/dist-packages"
    prepend_path LD_LIBRARY_PATH "/opt/ros/humble/lib"
    prepend_path LD_LIBRARY_PATH "/opt/ros/humble/lib/aarch64-linux-gnu"
    prepend_path LD_LIBRARY_PATH "/opt/ros/humble/lib/x86_64-linux-gnu"
    prepend_path AMENT_PREFIX_PATH "/opt/ros/humble"
    prepend_path CMAKE_PREFIX_PATH "/opt/ros/humble"

    if [ -d "${GAIT_WORKSPACE}/install/gait_control_system" ]; then
        WS_INSTALL="${GAIT_WORKSPACE}/install/gait_control_system"
        prepend_path PYTHONPATH "${WS_INSTALL}/lib/python${PYTHON_VERSION_SHORT}/site-packages"
        prepend_path LD_LIBRARY_PATH "${WS_INSTALL}/lib"
        prepend_path AMENT_PREFIX_PATH "${WS_INSTALL}"
        prepend_path CMAKE_PREFIX_PATH "${WS_INSTALL}"
        prepend_path COLCON_PREFIX_PATH "${WS_INSTALL}"
    fi
else
    # 完整ROS/colcon环境，兼容需要全部环境hook的调试场景。
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "⚠️ 未找到 /opt/ros/humble/setup.bash"
    fi

    if [ -f "${GAIT_WORKSPACE}/install/setup.bash" ]; then
        source "${GAIT_WORKSPACE}/install/setup.bash"
    elif [ -f "${GAIT_WORKSPACE}/install/local_setup.bash" ]; then
        source "${GAIT_WORKSPACE}/install/local_setup.bash"
    else
        echo "⚠️ 未找到工作区安装环境: ${GAIT_WORKSPACE}/install/setup.bash"
        echo "   首次部署请先执行: colcon build --symlink-install"
    fi
fi

# 域ID配置
export ROS_DOMAIN_ID=42

# DDS配置
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 默认只使用本机ROS通信，避免开机阶段等待或扫描网络接口。
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

echo "✅ ROS2环境配置完成"
echo "ROS_ENV模式: $([ "${GAIT_FAST_ROS_ENV:-1}" = "1" ] && echo fast || echo full)"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "工作空间: ${GAIT_WORKSPACE}"
