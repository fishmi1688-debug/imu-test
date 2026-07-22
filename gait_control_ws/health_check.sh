#!/bin/bash
echo "=== ROS2步态控制系统健康检查 ==="

CAN_INTERFACE="${GAIT_CAN_INTERFACE:-can0}"

# 自动识别工作区目录（优先 GAIT_WORKSPACE，其次脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GAIT_WORKSPACE="${GAIT_WORKSPACE:-$SCRIPT_DIR}"

# 检查ROS2环境
echo "1. ROS2环境检查"
source /opt/ros/humble/setup.bash
if [ -f "${GAIT_WORKSPACE}/install/setup.bash" ]; then
    source "${GAIT_WORKSPACE}/install/setup.bash"
elif [ -f "${GAIT_WORKSPACE}/install/local_setup.bash" ]; then
    source "${GAIT_WORKSPACE}/install/local_setup.bash"
else
    echo "⚠️ 未找到工作区安装环境: ${GAIT_WORKSPACE}/install/setup.bash"
fi
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 检查节点状态
echo "2. 节点状态检查"
nodes=$(ros2 node list 2>/dev/null)
if [[ $? -eq 0 ]]; then
    echo "活跃节点: $nodes"
else
    echo "❌ 无法获取节点列表"
fi

# 检查话题
echo "3. 话题检查"
topics=$(ros2 topic list 2>/dev/null | grep -E "(motor|gait)" | wc -l)
echo "相关话题数量: $topics"

# 检查CAN接口
echo "4. CAN接口检查"
if ip link show "$CAN_INTERFACE" &>/dev/null; then
    echo "✅ ${CAN_INTERFACE} 接口存在"
else
    echo "❌ ${CAN_INTERFACE} 接口未配置"
fi

echo "=== 检查完成 ==="
