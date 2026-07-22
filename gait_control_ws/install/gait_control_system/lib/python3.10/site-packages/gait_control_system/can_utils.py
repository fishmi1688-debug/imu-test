"""Helper functions for configuring the CAN interface."""

import os
import shutil
import subprocess

from .gait_constants import CAN_INTERFACE

CAN_BITRATE = os.environ.get("GAIT_CAN_BITRATE", "1000000")


def _select_can_command_runner():
    """Decide how to run CAN-related commands (prefer sudo if available)."""
    if os.geteuid() == 0:
        # Already root inside some containers; sudo may be blocked by no-new-privileges
        return [], True

    sudo_path = shutil.which("sudo")
    if sudo_path:
        check = subprocess.run(["sudo", "-n", "true"], capture_output=True, text=True)
        reason = (check.stderr or check.stdout or "").strip()
        # 无密码sudo可用
        if check.returncode == 0:
            return ["sudo"], True

        # sudo需要密码或被no-new-privileges限制
        if "no new privileges" in reason.lower():
            print("⚠️ sudo被 no-new-privileges 限制，改用无sudo配置；"
                  "请在容器中关闭该标志或授予CAP_NET_ADMIN/使用root")
            return [], False
        if reason:
            print(f"🔑 sudo需要密码或权限受限（{reason}），后续命令将尝试提示输入密码")
        else:
            print("🔑 sudo可能需要密码，后续命令将尝试提示输入密码")
        return ["sudo"], True
    else:
        print("⚠️ 未找到sudo，将直接执行CAN配置命令")
    return [], False


def _run_can_command(cmd, runner_prefix, quiet=False):
    """Run a CAN setup command with optional sudo prefix and capture permission issues."""
    full_cmd = runner_prefix + cmd
    try:
        result = subprocess.run(full_cmd, capture_output=True, text=True)
        output = (result.stderr or result.stdout or "").strip()
        perm_denied = "permission" in output.lower() or "operation not permitted" in output.lower()
        if result.returncode != 0 and output and not quiet:
            print(f"命令{' '.join(full_cmd)}失败: {output}")
        return result.returncode == 0, perm_denied
    except FileNotFoundError as e:
        if not quiet:
            print(f"命令未找到: {e}")
        return False, False
    except PermissionError as e:
        if not quiet:
            print(f"权限不足: {e}")
        return False, True


def setup_can_interface():
    """程序启动时一次性初始化系统默认CAN接口。"""
    runner_prefix, privileged_available = _select_can_command_runner()
    if not privileged_available and os.geteuid() != 0:
        print("⚠️ 当前无法使用sudo，将直接尝试配置CAN接口；若失败请以root或具有CAP_NET_ADMIN的环境运行")

    if not shutil.which("ip"):
        raise Exception("未找到 ip 命令，无法配置CAN接口")

    exists = subprocess.run(
        ["ip", "link", "show", CAN_INTERFACE],
        capture_output=True,
        text=True,
    )
    if exists.returncode != 0:
        raise Exception(f"未找到默认CAN接口 {CAN_INTERFACE}，请检查系统网络接口配置")

    permission_blocked = False
    _run_can_command(["ip", "link", "set", CAN_INTERFACE, "down"], runner_prefix, quiet=True)
    result, perm_denied = _run_can_command(
        ["ip", "link", "set", CAN_INTERFACE, "up", "type", "can", "bitrate", CAN_BITRATE],
        runner_prefix,
    )
    permission_blocked = permission_blocked or perm_denied

    if not result:
        if permission_blocked and not privileged_available:
            error_msg = ("CAN接口配置失败：缺少权限（sudo不可用或被禁用）。"
                         "请以root运行，或在容器中授予CAP_NET_ADMIN/关闭no-new-privileges标志后重试")
        else:
            error_msg = f"默认CAN接口 {CAN_INTERFACE} 配置失败，请检查接口状态和bitrate设置"
        print(error_msg)
        raise Exception(error_msg)

    print(f"CAN接口 {CAN_INTERFACE} 初始化成功，bitrate={CAN_BITRATE}")

def cleanup_can_interface():
    """程序退出时关闭CAN接口"""
    try:
        runner_prefix, _ = _select_can_command_runner()
        _run_can_command(["ip", "link", "set", CAN_INTERFACE, "down"], runner_prefix, quiet=True)
        print("CAN接口已关闭")
    except Exception as e:
        print(f"关闭CAN接口失败: {e}")
