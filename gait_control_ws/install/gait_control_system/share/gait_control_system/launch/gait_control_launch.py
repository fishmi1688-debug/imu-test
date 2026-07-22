import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAN_INTERFACE = os.environ.get("GAIT_CAN_INTERFACE", "can0")
CAN_BITRATE = os.environ.get("GAIT_CAN_BITRATE", "1000000")


def _detect_can_device(context):
    can_available = (
        'true'
        if subprocess.run(
            ["ip", "link", "show", CAN_INTERFACE],
            capture_output=True,
            text=True,
        ).returncode == 0
        else 'false'
    )
    message = (
        f"检测到默认CAN接口 {CAN_INTERFACE}，启动主控制系统（APP蓝牙控制）"
        if can_available == 'true'
        else f"未检测到默认CAN接口 {CAN_INTERFACE}，跳过主控制节点启动"
    )
    return [
        SetLaunchConfiguration('can_available', can_available),
        LogInfo(msg=message),
    ]

def generate_launch_description():
    # 声明启动参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='日志级别 (DEBUG, INFO, WARN, ERROR)'
    )

    return LaunchDescription([
        log_level_arg,
        OpaqueFunction(function=_detect_can_device),
        
        # 设置系统默认CAN接口
        ExecuteProcess(
            cmd=['bash', '-c', '''
            SUDO_CMD=""
            if [ "$(id -u)" -ne 0 ]; then
                if command -v sudo >/dev/null 2>&1; then
                    if sudo -n true >/dev/null 2>&1; then
                        SUDO_CMD="sudo"
                    else
                        echo "sudo不可用（可能被 no-new-privileges 限制），将尝试直接配置CAN接口"
                    fi
                else
                    echo "未找到sudo，将直接配置CAN接口"
                fi
            fi

            run_can_cmd() {
                if [ -n "$SUDO_CMD" ]; then
                    $SUDO_CMD "$@"
                else
                    "$@"
                fi
            }

            CAN_INTERFACE="''' + CAN_INTERFACE + '''"
            CAN_BITRATE="''' + CAN_BITRATE + '''"
            run_can_cmd ip link set "$CAN_INTERFACE" down 2>/dev/null || true
            run_can_cmd ip link set "$CAN_INTERFACE" up type can bitrate "$CAN_BITRATE"
            '''],
            name='setup_can_interface',
            condition=IfCondition(LaunchConfiguration('can_available'))
        ),
        
        # 启动主控制节点（包含步态分析和电机控制）
        Node(
            package='gait_control_system',
            executable='gait_control_node',
            name='gait_control_main',
            output='screen',
            condition=IfCondition(LaunchConfiguration('can_available')),
            parameters=[{
                'can_interface': CAN_INTERFACE,
                'update_rate': 100.0,
                'max_current': 27.0,
                'torque_constant': 0.87,
                'control_frequency': 100.0,
                'debug_mode': False
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])
