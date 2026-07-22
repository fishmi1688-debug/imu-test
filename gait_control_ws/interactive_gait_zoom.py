#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单个CSV步态数据可视化：支持区域放大/缩小
使用方法:
    python interactive_gait_zoom.py path/to/data.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.widgets import SpanSelector
from scipy import signal

# 为中文标签设置字体以避免乱码
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "AR PL UMing CN",
    "WenQuanYi Micro Hei",
    "SimHei",
    "Microsoft YaHei",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

# 支持的列别名，便于兼容不同导出格式
COLUMN_ALIASES: Dict[str, Sequence[str]] = {
    "time": ("时间戳", "timestamp", "time", "Time"),
    "left_angle": ("左髋角度", "left_angle", "left_hip_angle"),
    "right_angle": ("右髋角度", "right_angle", "right_hip_angle"),
    "left_phase": ("左腿相位", "phase_left", "left_phase"),
    "right_phase": ("右腿相位", "phase_right", "right_phase"),
    "gait_detect": ("运动检测", "gait_detected", "gait_state", "motion_detected"),
}


def resolve_columns(df: pd.DataFrame) -> Dict[str, str]:
    """根据别名解析实际列名。"""
    resolved: Dict[str, str] = {}
    for key, candidates in COLUMN_ALIASES.items():
        for name in candidates:
            if name in df.columns:
                resolved[key] = name
                break
    required = ("time", "left_angle", "right_angle")
    missing = [key for key in required if key not in resolved]
    if missing:
        raise ValueError(f"缺少必要列: {', '.join(missing)}")
    return resolved


def build_time_axis(time_series: pd.Series) -> np.ndarray:
    """生成以起始点为零的时间轴（秒），支持数字或日期时间字符串。"""
    numeric = pd.to_numeric(time_series, errors="coerce")
    if numeric.notna().any():
        values = numeric.to_numpy()
        first_idx = np.flatnonzero(~np.isnan(values))[0]
        return values - values[first_idx]

    dt = pd.to_datetime(time_series, errors="coerce", utc=False)
    if dt.notna().any():
        first_valid = dt.dropna().iloc[0]
        seconds = (dt - first_valid).dt.total_seconds().to_numpy()
        if np.isnan(seconds).any():
            raise ValueError("时间列存在无法解析的日期时间值")
        return seconds

    raise ValueError("时间列存在非数字或不可解析的日期时间数据")


def to_bool_array(series: pd.Series) -> np.ndarray:
    """将任意类型的布尔列转换为 bool 数组。"""
    if series.dtype == bool:
        return series.to_numpy()
    return series.astype(int).astype(bool).to_numpy()


def estimate_phase_hilbert(angle_signal: np.ndarray, dt: float) -> np.ndarray:
    """使用 Hilbert 估计相位，返回 0~2π。"""
    # 高通去除漂移
    try:
        sos = signal.butter(2, 0.1, btype="hp", fs=1 / dt, output="sos")
        filtered = signal.sosfiltfilt(sos, angle_signal)
    except Exception:
        filtered = angle_signal - np.mean(angle_signal)
    analytic = signal.hilbert(filtered)
    phase = np.angle(analytic)
    return np.mod(phase, 2 * np.pi)


def find_gait_spans(time: np.ndarray, mask: np.ndarray) -> List[Tuple[float, float]]:
    """根据 True 区间计算 (start, end) 列表。"""
    spans: List[Tuple[float, float]] = []
    start: Optional[float] = None
    for t, flag in zip(time, mask):
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, time[-1]))
    return spans


def add_spans(ax, spans: Iterable[Tuple[float, float]], label: str) -> None:
    """在给定坐标轴上叠加步行区间。"""
    for i, (xmin, xmax) in enumerate(spans):
        ax.axvspan(
            xmin,
            xmax,
            color="lightgreen",
            alpha=0.15,
            label=label if i == 0 else None,
        )


def compute_ylim(series_list: Sequence[np.ndarray]) -> Tuple[float, float]:
    """根据序列列表计算固定 y 轴范围。"""
    valid = [s for s in series_list if s is not None]
    y_min = min(float(np.min(s)) for s in valid)
    y_max = max(float(np.max(s)) for s in valid)
    margin = max((y_max - y_min) * 0.05, 1e-3)
    return y_min - margin, y_max + margin


def update_zoom_axes(
    axes,
    xmin: float,
    xmax: float,
    time: np.ndarray,
    series_list: Sequence[np.ndarray],
    y_limits: Optional[Tuple[float, float]] = None,
) -> None:
    """更新放大视图的 x 轴范围，可选固定 y 轴。"""
    mask = (time >= xmin) & (time <= xmax)
    if not mask.any():
        return
    axes.set_xlim(xmin, xmax)
    if y_limits is not None:
        axes.set_ylim(*y_limits)
    else:
        valid_series = [s[mask] for s in series_list if s is not None]
        y_min = min(np.min(s) for s in valid_series)
        y_max = max(np.max(s) for s in valid_series)
        margin = max((y_max - y_min) * 0.05, 1e-3)
        axes.set_ylim(y_min - margin, y_max + margin)


def create_plot(
    time: np.ndarray,
    left_angle: np.ndarray,
    right_angle: np.ndarray,
    left_phase: Optional[np.ndarray],
    right_phase: Optional[np.ndarray],
    left_phase_est: Optional[np.ndarray],
    right_phase_est: Optional[np.ndarray],
    gait_mask: Optional[np.ndarray],
    title: str,
    initial_window: float,
    initial_start: float,
) -> None:
    """构建交互式图像。"""
    fig, (ax_overview, ax_angle_zoom, ax_phase_zoom) = plt.subplots(
        3, 1, figsize=(12, 10), sharex=False
    )
    fig.suptitle(title, fontsize=14)

    # 总览
    ax_overview.plot(time, left_angle, label="左髋角度", color="#1f77b4")
    ax_overview.plot(time, right_angle, label="右髋角度", color="#d62728")
    ax_overview.set_ylabel("角度 (rad)")
    ax_overview.grid(True, alpha=0.3)
    if gait_mask is not None:
        spans = find_gait_spans(time, gait_mask)
        add_spans(ax_overview, spans, "步行区间")
    ax_overview.legend(loc="upper right")

    # 放大视图 - 角度
    ax_angle_zoom.plot(time, left_angle, label="左髋角度", color="#1f77b4")
    ax_angle_zoom.plot(time, right_angle, label="右髋角度", color="#d62728")
    ax_angle_zoom.set_ylabel("角度 (rad)")
    ax_angle_zoom.grid(True, alpha=0.3)
    if gait_mask is not None:
        spans = find_gait_spans(time, gait_mask)
        add_spans(ax_angle_zoom, spans, "步行区间")
    ax_angle_zoom.legend(loc="upper right")
    angle_ylim = compute_ylim((left_angle, right_angle))
    ax_angle_zoom.set_ylim(*angle_ylim)

    # 放大视图 - 相位
    phase_plotted = False
    phase_ylim: Optional[Tuple[float, float]] = None
    if left_phase is not None:
        ax_phase_zoom.plot(time, left_phase, label="左腿相位", color="#17becf")
        phase_plotted = True
    if right_phase is not None:
        ax_phase_zoom.plot(time, right_phase, label="右腿相位", color="#ff7f0e")
        phase_plotted = True
    if left_phase_est is not None:
        ax_phase_zoom.plot(
            time,
            left_phase_est,
            linestyle="--",
            color="#0a5c7a",
            alpha=0.9,
            label="左腿相位估计",
        )
        phase_plotted = True
    if right_phase_est is not None:
        ax_phase_zoom.plot(
            time,
            right_phase_est,
            linestyle="--",
            color="#c45a00",
            alpha=0.9,
            label="右腿相位估计",
        )
        phase_plotted = True
    ax_phase_zoom.set_ylabel("相位 (rad)")
    ax_phase_zoom.set_xlabel("时间 (s)")
    ax_phase_zoom.grid(True, alpha=0.3)
    if gait_mask is not None:
        spans = find_gait_spans(time, gait_mask)
        add_spans(ax_phase_zoom, spans, "步行区间")
    if phase_plotted:
        ax_phase_zoom.legend(loc="upper right")
        phase_ylim = compute_ylim(
            tuple(
                s
                for s in (
                    left_phase,
                    right_phase,
                    left_phase_est,
                    right_phase_est,
                )
                if s is not None
            )
        )
        ax_phase_zoom.set_ylim(*phase_ylim)
    else:
        ax_phase_zoom.text(
            0.5,
            0.5,
            "未找到相位列，跳过相位绘制",
            ha="center",
            va="center",
            transform=ax_phase_zoom.transAxes,
        )

    # 交互放大
    def on_select(xmin, xmax):
        update_zoom_axes(
            ax_angle_zoom,
            xmin,
            xmax,
            time,
            (left_angle, right_angle),
            y_limits=angle_ylim,
        )
        if phase_plotted:
            phase_series = tuple(
                s
                for s in (
                    left_phase,
                    right_phase,
                    left_phase_est,
                    right_phase_est,
                )
                if s is not None
            )
            update_zoom_axes(
                ax_phase_zoom,
                xmin,
                xmax,
                time,
                phase_series,
                y_limits=phase_ylim,
            )
        fig.canvas.draw_idle()

    SpanSelector(
        ax_overview,
        on_select,
        "horizontal",
        useblit=True,
        interactive=True,
        props=dict(alpha=0.2, facecolor="#aec7e8"),
    )

    # 初始化默认窗口
    start = max(0.0, initial_start)
    end = min(time[-1], start + initial_window)
    if end <= start:
        end = min(time[-1], start + initial_window + 1e-3)
    update_zoom_axes(
        ax_angle_zoom, start, end, time, (left_angle, right_angle), y_limits=angle_ylim
    )
    if phase_plotted:
        phase_series = tuple(s for s in (left_phase, right_phase) if s is not None)
        update_zoom_axes(
            ax_phase_zoom, start, end, time, phase_series, y_limits=phase_ylim
        )

    # 横向拖拽平移（在放大视图中按下左键拖拽）
    pan_state = {"press_x": None, "xlim": None}

    def on_press(event):
        if event.inaxes not in (ax_angle_zoom, ax_phase_zoom) or event.button != 1:
            return
        if event.xdata is None:
            return
        pan_state["press_x"] = event.xdata
        pan_state["xlim"] = ax_angle_zoom.get_xlim()

    def on_release(event):
        pan_state["press_x"] = None
        pan_state["xlim"] = None

    def on_motion(event):
        if pan_state["press_x"] is None:
            return
        if event.inaxes not in (ax_angle_zoom, ax_phase_zoom):
            return
        if event.xdata is None:
            return
        dx = event.xdata - pan_state["press_x"]
        x0, x1 = pan_state["xlim"]
        new_x0 = x0 - dx
        new_x1 = x1 - dx
        if new_x0 < time[0]:
            new_x1 += time[0] - new_x0
            new_x0 = time[0]
        if new_x1 > time[-1]:
            new_x0 -= new_x1 - time[-1]
            new_x1 = time[-1]
        if new_x1 - new_x0 <= 0:
            return
        ax_angle_zoom.set_xlim(new_x0, new_x1)
        if phase_plotted:
            ax_phase_zoom.set_xlim(new_x0, new_x1)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)

    fig.text(
        0.01,
        0.01,
        "提示：上方总览拖拽选区以缩小横轴；在下方两幅图按住左键拖动可沿横轴平移；工具栏的放大/缩放亦可使用",
        fontsize=9,
    )
    plt.tight_layout(rect=(0, 0.03, 1, 0.98))
    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="单个CSV步态数据可视化，支持拖拽选区放大查看细节"
    )
    parser.add_argument("csv_file", help="CSV 文件路径")
    parser.add_argument(
        "--start",
        type=float,
        default=0.0,
        help="初始放大视图起点时间（秒，默认0）",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=3.0,
        help="初始放大视图时间长度（秒，默认3秒）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        raise FileNotFoundError(f"未找到文件: {csv_path}")

    df = pd.read_csv(csv_path)
    resolved = resolve_columns(df)
    print("已识别列映射：")
    for k, v in resolved.items():
        print(f"  {k} -> {v}")

    df_sorted = df.sort_values(by=resolved["time"]).reset_index(drop=True)
    time = build_time_axis(df_sorted[resolved["time"]])
    left_angle = df_sorted[resolved["left_angle"]].to_numpy()
    right_angle = df_sorted[resolved["right_angle"]].to_numpy()
    left_phase = (
        df_sorted[resolved["left_phase"]].to_numpy()
        if "left_phase" in resolved
        else None
    )
    right_phase = (
        df_sorted[resolved["right_phase"]].to_numpy()
        if "right_phase" in resolved
        else None
    )
    # 基于左右角度差估计相位（Hilbert），右腿相位估计加 π
    if len(time) > 1:
        dt = float(np.median(np.diff(time)))
    else:
        dt = 0.01
    angle_diff = left_angle - right_angle
    base_phase_est = estimate_phase_hilbert(angle_diff, dt)
    left_phase_est = base_phase_est
    right_phase_est = np.mod(base_phase_est + np.pi, 2 * np.pi)
    gait_mask = (
        to_bool_array(df_sorted[resolved["gait_detect"]])
        if "gait_detect" in resolved
        else None
    )

    title = f"{csv_path.name} 步态可视化"
    create_plot(
        time=time,
        left_angle=left_angle,
        right_angle=right_angle,
        left_phase=left_phase,
        right_phase=right_phase,
        left_phase_est=left_phase_est,
        right_phase_est=right_phase_est,
        gait_mask=gait_mask,
        title=title,
        initial_window=args.window,
        initial_start=args.start,
    )


if __name__ == "__main__":
    main()
