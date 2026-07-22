#!/bin/bash
# ROS2环境配置脚本

# 自动识别工作区目录（优先 GAIT_WORKSPACE，其次脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAIT_WORKSPACE="${GAIT_WORKSPACE:-$SCRIPT_DIR}"

# 基础ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "⚠️ 未找到 /opt/ros/humble/setup.bash"
fi

# 工作空间环境
if [ -f "${GAIT_WORKSPACE}/install/setup.bash" ]; then
    source "${GAIT_WORKSPACE}/install/setup.bash"
elif [ -f "${GAIT_WORKSPACE}/install/local_setup.bash" ]; then
    source "${GAIT_WORKSPACE}/install/local_setup.bash"
else
    echo "⚠️ 未找到工作区安装环境: ${GAIT_WORKSPACE}/install/setup.bash"
    echo "   首次部署请先执行: colcon build --symlink-install"
fi

# 域ID配置
export ROS_DOMAIN_ID=42

# DDS配置
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 网络配置
export ROS_LOCALHOST_ONLY=0

echo "✅ ROS2环境配置完成"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "工作空间: ${GAIT_WORKSPACE}"
