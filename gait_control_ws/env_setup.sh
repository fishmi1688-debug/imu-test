#!/bin/bash
# 环境配置脚本 - Gait Control System
# 用法: source env_setup.sh

# 设置工作区根路径（如需更改，修改此变量或在 shell 中预先导出）
export GAIT_WORKSPACE="${GAIT_WORKSPACE:-/home/sunrise/gait_control_ws}"

echo "🔧 Gait Control System - 环境配置"
echo "======================================"
echo "工作区路径: $GAIT_WORKSPACE"

# 扩展 PYTHONPATH（包含构建输出和安装路径）
# 注意：实际 ROS 2 环境应优先 source install/local_setup.bash
export PYTHONPATH="${GAIT_WORKSPACE}/build/gait_control_system:${GAIT_WORKSPACE}/install/gait_control_system/lib/python3.10/site-packages:${PYTHONPATH}"

echo "✅ PYTHONPATH 已更新"
echo ""
echo "下一步操作："
echo "  1. 构建项目:"
echo "     cd $GAIT_WORKSPACE"
echo "     colcon build --symlink-install"
echo ""
echo "  2. Source ROS 2 环境:"
echo "     source install/local_setup.bash"
echo ""
echo "  3. 运行系统:"
echo "     ./start_system.sh"
echo ""
echo "  4. 或使用 ROS 2 launch:"
echo "     ros2 launch gait_control_system gait_control_launch.py"
echo ""
echo "💡 提示: 如需修改工作区路径，在 source 本脚本前设置："
echo "   export GAIT_WORKSPACE=/your/custom/path"
echo "   source env_setup.sh"
echo "======================================"
