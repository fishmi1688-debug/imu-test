#!/usr/bin/env python3
"""
单文件离线分析：计算左右腿角度差的频率（通过峰值计数估计周期）。
在文件开头修改 FILE_PATH 等配置即可切换要分析的CSV。

输出：将时序主频和图像存放到以输入文件名命名的子目录下。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# 限制并发线程，避免在受限环境下的 OMP 共享内存错误
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_MAIN_FREE", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("OMP_WAIT_POLICY", "PASSIVE")
os.environ.setdefault("KMP_AFFINITY", "disabled")
os.environ.setdefault("KMP_WARNINGS", "0")
os.environ.setdefault("KMP_INIT_AT_FORK", "FALSE")
os.environ.setdefault("KMP_BLOCKTIME", "0")
os.environ.setdefault("KMP_HW_SUBSET", "1C,1T")

import matplotlib

matplotlib.use("Agg")  # headless backend for saving figures
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 让脚本可直接导入 gait_control_system 包（不需要安装）
ROOT_DIR = Path(__file__).resolve().parent
SRC_DIR = ROOT_DIR / "src"
if SRC_DIR.exists():
    sys.path.insert(0, str(SRC_DIR))

try:
    from gait_control_system.gait_control_system.adaptive_oscillator_estimator import (
        AdaptiveOscillatorEstimator,
    )
    from gait_control_system.gait_control_system.gait_constants import AO_CONFIG
    AO_AVAILABLE = True
    AO_IMPORT_ERROR = None
except Exception as exc:  # noqa: BLE001
    AO_AVAILABLE = False
    AO_IMPORT_ERROR = exc

# 尝试使用常见中文字体，避免标签乱码
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "Noto Sans CJK HK",
    "AR PL UMing CN",
    "AR PL UKai CN",
    "SimHei",
    "WenQuanYi Micro Hei",
    "Arial Unicode MS",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

# === 用户可改的配置 ===
FILE_PATH = "gait_logs/gait_data_20251221_144423.csv"  # 要分析的CSV
BASE_OUTPUT_DIR = "analysis_results3"  # 结果根目录，实际输出在子目录中
TIME_COLUMN = None  # Optional explicit time column name
REQUIRE_TIMESTAMP = True  # Use timestamp from file; disable fallback if True
FALLBACK_DT = 0.05  # Sampling interval (s) when REQUIRE_TIMESTAMP is False
WINDOW_S = 6.0  # 滑窗长度 (秒)
STEP_S = 1.0  # 滑窗步长 (秒)
FMIN = 0.2  # 频率下限 (Hz)，用于过滤异常值
FMAX = 3.0  # 频率上限 (Hz)，用于过滤异常值
FREQ_METHOD = "peaks"  # 频率估计方法: "peaks" | "ao" | "fft"
# 峰值法参数（仅在 FREQ_METHOD == "peaks" 时使用）
PEAK_MIN_DISTANCE_S = None  # 峰间最小时间间隔 (秒)，None 时按 FMAX 自动推导
PEAK_MIN_HEIGHT_STD = 0.2  # 峰值最小高度(相对std)，<=0 表示不限制
PEAK_SMOOTH_S = 0.0  # 简单滑动平均平滑窗口 (秒)，0 表示不平滑

# 列名别名，兼容中英文
COLUMN_ALIASES: Dict[str, Sequence[str]] = {
    "time": ("时间戳", "timestamp", "time", "Time", "t", "time_s", "time_ms"),
    "left_angle": ("左髋角度", "left_angle", "left_hip_angle"),
    "right_angle": ("右髋角度", "right_angle", "right_hip_angle"),
}


def pick_column(df: pd.DataFrame, candidates: Iterable[str]) -> Optional[str]:
    for name in candidates:
        if name in df.columns:
            return name
    return None


def estimate_dt(time_series: pd.Series) -> Optional[float]:
    """估计采样间隔 (秒)。"""
    numeric = pd.to_numeric(time_series, errors="coerce").to_numpy()
    numeric = numeric[~np.isnan(numeric)]
    if numeric.size >= 2:
        diffs = np.diff(numeric)
        diffs = diffs[diffs > 0]
        if diffs.size:
            return float(np.median(diffs))

    dt_series = pd.to_datetime(time_series, errors="coerce", utc=False)
    dt_valid = dt_series.dropna()
    if len(dt_valid) >= 2:
        seconds = dt_valid.astype("int64").to_numpy() / 1e9
        diffs = np.diff(seconds)
        diffs = diffs[diffs > 0]
        if diffs.size:
            return float(np.median(diffs))

    return None


def build_time_axis(time_series: pd.Series, dt: float) -> Optional[np.ndarray]:
    """生成相对时间轴（秒）；若失败则返回 None。"""
    numeric = pd.to_numeric(time_series, errors="coerce").to_numpy()
    if np.isfinite(numeric).any():
        numeric = numeric[np.isfinite(numeric)]
        if numeric.size >= 1:
            return numeric - numeric[0]

    dt_series = pd.to_datetime(time_series, errors="coerce", utc=False)
    dt_valid = dt_series.dropna()
    if len(dt_valid) >= 1:
        seconds = dt_valid.astype("int64").to_numpy() / 1e9
        return seconds - seconds[0]

    if dt > 0:
        return np.arange(len(time_series)) * dt
    return None


def dominant_frequency_fft(
    angle_diff: np.ndarray, dt: float, fmin: float, fmax: float
) -> Optional[Dict[str, float]]:
    """主频估计：加窗 FFT，在指定频段内取峰值。"""
    if dt <= 0 or angle_diff.size < 8:
        return None

    fs = 1.0 / dt
    clean = angle_diff - np.nanmean(angle_diff)
    clean = clean[~np.isnan(clean)]
    if clean.size < 8:
        return None

    n = clean.size
    window = np.hanning(n)
    spec = np.fft.rfft(clean * window)
    psd = (np.abs(spec) ** 2) / (np.sum(window**2) * fs)
    freqs = np.fft.rfftfreq(n, d=dt)

    band = (freqs >= fmin) & (freqs <= fmax)
    if not band.any():
        return None

    freqs_band = freqs[band]
    psd_band = psd[band]
    idx = int(np.argmax(psd_band))
    peak_freq = float(freqs_band[idx])
    peak_power = float(psd_band[idx])
    band_power = float(np.trapezoid(psd_band, freqs_band))
    power_ratio = peak_power / band_power if band_power > 0 else np.nan

    return {
        "freq_hz": peak_freq,
        "peak_power": peak_power,
        "band_power": band_power,
        "power_ratio": power_ratio,
    }


def smooth_signal(signal: np.ndarray, dt: float, window_s: float) -> np.ndarray:
    """简单滑动平均平滑。"""
    if window_s <= 0:
        return signal
    win_n = int(round(window_s / dt))
    if win_n < 3:
        return signal
    kernel = np.ones(win_n, dtype=float) / float(win_n)
    return np.convolve(signal, kernel, mode="same")


def find_peaks_simple(
    signal: np.ndarray, min_distance_samples: int, min_height: Optional[float]
) -> np.ndarray:
    """在一维信号中寻找局部极大值（不依赖 SciPy）。"""
    if signal.size < 3:
        return np.array([], dtype=int)

    candidates = np.where((signal[1:-1] > signal[:-2]) & (signal[1:-1] >= signal[2:]))[0] + 1
    if min_height is not None:
        candidates = candidates[signal[candidates] >= min_height]
    if candidates.size == 0 or min_distance_samples <= 1:
        return candidates

    # 按幅值从大到小挑选，确保峰间距
    order = np.argsort(signal[candidates])[::-1]
    selected: List[int] = []
    taken = np.zeros(signal.size, dtype=bool)
    exclusion = max(0, min_distance_samples - 1)
    for idx in candidates[order]:
        if taken[idx]:
            continue
        selected.append(int(idx))
        lo = max(0, idx - exclusion)
        hi = min(signal.size, idx + exclusion + 1)
        taken[lo:hi] = True
    return np.array(sorted(selected), dtype=int)


def compute_ao_frequency_series(angle_diff: np.ndarray, dt: float) -> Optional[np.ndarray]:
    """使用AO算法计算每个采样点的频率（Hz）。"""
    if not AO_AVAILABLE:
        return None
    if dt <= 0 or angle_diff.size < 8:
        return None
    ao_config = AO_CONFIG.copy()
    ao = AdaptiveOscillatorEstimator(dt, ao_config)
    freqs = np.zeros_like(angle_diff, dtype=float)
    for i, sample in enumerate(angle_diff):
        try:
            ao.step(float(sample))
            freqs[i] = ao.para[1, 1] / (2 * np.pi)
        except Exception:
            freqs[i] = np.nan
    return freqs


def sliding_frequency_fft(
    angle_diff: np.ndarray,
    dt: float,
    window_s: float,
    step_s: float,
    fmin: float,
    fmax: float,
) -> List[Dict[str, float]]:
    """滑动窗口主频序列（FFT）。"""
    results: List[Dict[str, float]] = []
    if dt <= 0:
        return results

    win_n = int(round(window_s / dt))
    step_n = int(round(step_s / dt))
    if win_n < 8 or step_n < 1 or angle_diff.size < win_n:
        return results

    for start in range(0, angle_diff.size - win_n + 1, step_n):
        end = start + win_n
        segment = angle_diff[start:end]
        stats = dominant_frequency_fft(segment, dt, fmin, fmax)
        if stats:
            t_mid = (start + end) / 2 * dt
            results.append(
                {
                    "t_mid_s": t_mid,
                    "freq_hz": stats["freq_hz"],
                }
            )
    return results


def sliding_frequency_from_series(
    freq_series: np.ndarray,
    dt: float,
    window_s: float,
    step_s: float,
    fmin: float,
    fmax: float,
) -> List[Dict[str, float]]:
    """滑动窗口频率序列（AO频率的中位数）。"""
    results: List[Dict[str, float]] = []
    if dt <= 0 or freq_series.size < 8:
        return results
    win_n = int(round(window_s / dt))
    step_n = int(round(step_s / dt))
    if win_n < 8 or step_n < 1 or freq_series.size < win_n:
        return results
    for start in range(0, freq_series.size - win_n + 1, step_n):
        end = start + win_n
        segment = freq_series[start:end]
        segment = segment[np.isfinite(segment)]
        if fmin is not None and fmax is not None:
            segment = segment[(segment >= fmin) & (segment <= fmax)]
        if segment.size < 3:
            continue
        t_mid = (start + end) / 2 * dt
        results.append(
            {
                "t_mid_s": t_mid,
                "freq_hz": float(np.median(segment)),
            }
        )
    return results


def sliding_frequency_peaks(
    angle_diff: np.ndarray,
    time_axis: np.ndarray,
    dt: float,
    window_s: float,
    step_s: float,
    fmin: float,
    fmax: float,
    min_peak_distance_s: Optional[float],
    min_peak_height_std: float,
    smooth_s: float,
) -> List[Dict[str, float]]:
    """滑动窗口峰值计数频率（基于相邻峰在真实时间轴上的间隔中位数）。"""
    results: List[Dict[str, float]] = []
    time_axis = np.asarray(time_axis, dtype=float)
    if dt <= 0 or time_axis.size != angle_diff.size:
        return results

    win_n = int(round(window_s / dt))
    step_n = int(round(step_s / dt))
    if win_n < 8 or step_n < 1 or angle_diff.size < win_n:
        return results

    if not min_peak_distance_s or min_peak_distance_s <= 0:
        if fmax and fmax > 0:
            min_peak_distance_s = 0.5 / fmax
        else:
            min_peak_distance_s = 0.2
    min_distance_samples = max(1, int(round(min_peak_distance_s / dt)))

    for start in range(0, angle_diff.size - win_n + 1, step_n):
        end = start + win_n
        segment = angle_diff[start:end]
        if not np.isfinite(segment).all():
            continue
        segment = segment - float(np.mean(segment))
        if smooth_s > 0:
            segment = smooth_signal(segment, dt, smooth_s)

        min_height = None
        if min_peak_height_std and min_peak_height_std > 0:
            std = float(np.std(segment))
            if std > 0:
                min_height = min_peak_height_std * std

        peaks = find_peaks_simple(segment, min_distance_samples, min_height)
        if peaks.size < 2:
            continue
        time_segment = time_axis[start:end]
        if time_segment.size != segment.size or not np.isfinite(time_segment).all():
            continue
        intervals = np.diff(time_segment[peaks])
        intervals = intervals[intervals > 0]
        median_interval = float(np.median(intervals)) if intervals.size else 0.0
        if median_interval <= 0:
            continue
        freq = 1.0 / median_interval
        if fmin is not None and freq < fmin:
            continue
        if fmax is not None and freq > fmax:
            continue

        t_mid = (start + end) / 2 * dt
        results.append(
            {
                "t_mid_s": t_mid,
                "freq_hz": freq,
            }
        )
    return results


def analyze_single_file(
    csv_path: Path,
    fallback_dt: float,
    window_s: float,
    step_s: float,
    fmin: float,
    fmax: float,
) -> Optional[Tuple[pd.DataFrame, np.ndarray, np.ndarray, float, str]]:
    """返回时间序列频率结果，并绘制角度差+频率图。"""
    df = pd.read_csv(csv_path)

    left_col = pick_column(df, COLUMN_ALIASES["left_angle"])
    right_col = pick_column(df, COLUMN_ALIASES["right_angle"])
    if not left_col or not right_col:
        print(f"[skip] {csv_path.name}: missing left/right angle columns")
        return None

    left = pd.to_numeric(df[left_col], errors="coerce").to_numpy()
    right = pd.to_numeric(df[right_col], errors="coerce").to_numpy()
    valid = (~np.isnan(left)) & (~np.isnan(right))
    if valid.sum() < 8:
        print(f"[skip] {csv_path.name}: not enough valid angle samples")
        return None

    time_col = TIME_COLUMN if TIME_COLUMN else pick_column(df, COLUMN_ALIASES["time"])
    if TIME_COLUMN and time_col is None:
        print(f"[skip] {csv_path.name}: TIME_COLUMN '{TIME_COLUMN}' not found")
        return None
    if not time_col:
        if REQUIRE_TIMESTAMP:
            print(f"[skip] {csv_path.name}: missing time column (timestamp required)")
            return None
        dt = fallback_dt
        time_axis_full = np.arange(len(df)) * dt
    else:
        dt_est = estimate_dt(df[time_col])
        dt = dt_est if dt_est and dt_est > 0 else None
        if (dt is None or dt <= 0) and REQUIRE_TIMESTAMP:
            print(f"[skip] {csv_path.name}: cannot infer sampling interval from timestamp")
            return None
        if dt is None or dt <= 0:
            dt = fallback_dt
        time_axis_full = build_time_axis(df[time_col], 0.0 if REQUIRE_TIMESTAMP else dt)
        if time_axis_full is None:
            if REQUIRE_TIMESTAMP:
                print(f"[skip] {csv_path.name}: cannot build time axis from timestamp")
                return None
            time_axis_full = np.arange(len(df)) * dt

    angle_diff = (left - right)[valid]
    time_axis = time_axis_full[valid] if time_axis_full is not None else np.arange(len(angle_diff)) * dt
    freq_label = "角度差频率"
    rows: List[Dict[str, float]] = []
    if FREQ_METHOD == "peaks":
        rows = sliding_frequency_peaks(
            angle_diff=angle_diff,
            time_axis=time_axis,
            dt=dt,
            window_s=window_s,
            step_s=step_s,
            fmin=fmin,
            fmax=fmax,
            min_peak_distance_s=PEAK_MIN_DISTANCE_S,
            min_peak_height_std=PEAK_MIN_HEIGHT_STD,
            smooth_s=PEAK_SMOOTH_S,
        )
        freq_label = "角度差频率 (Peaks)"
    elif FREQ_METHOD == "ao":
        if AO_AVAILABLE:
            freq_series = compute_ao_frequency_series(angle_diff, dt)
            if freq_series is not None:
                rows = sliding_frequency_from_series(freq_series, dt, window_s, step_s, fmin, fmax)
                freq_label = "角度差频率 (AO)"
        else:
            print(f"[warn] AO导入失败，回退FFT: {AO_IMPORT_ERROR}")
    elif FREQ_METHOD == "fft":
        rows = sliding_frequency_fft(angle_diff, dt, window_s, step_s, fmin, fmax)
        freq_label = "角度差频率 (FFT)"
    else:
        print(f"[warn] Unknown FREQ_METHOD={FREQ_METHOD}, fallback to peaks")
        rows = sliding_frequency_peaks(
            angle_diff=angle_diff,
            dt=dt,
            window_s=window_s,
            step_s=step_s,
            fmin=fmin,
            fmax=fmax,
            min_peak_distance_s=PEAK_MIN_DISTANCE_S,
            min_peak_height_std=PEAK_MIN_HEIGHT_STD,
            smooth_s=PEAK_SMOOTH_S,
        )
        freq_label = "角度差频率 (Peaks)"
    if not rows and FREQ_METHOD == "ao" and not AO_AVAILABLE:
        rows = sliding_frequency_fft(angle_diff, dt, window_s, step_s, fmin, fmax)
        freq_label = "角度差频率 (FFT)"
    if not rows:
        print(f"[warn] {csv_path.name}: no sliding-window result (check window/step)")
        return None

    result_df = pd.DataFrame(rows)
    print(
        f"[ok] {csv_path.name}: {len(rows)} windows, "
        f"median {result_df['freq_hz'].median():.2f} Hz "
        f"[{freq_label}]"
    )
    return result_df, time_axis, angle_diff, dt, freq_label


def plot_results(
    time_axis: np.ndarray,
    angle_diff: np.ndarray,
    freq_df: pd.DataFrame,
    freq_label: str,
    plot_path: Path,
) -> None:
    """绘制左右角度差及频率随时间变化。"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    ax1.plot(time_axis, angle_diff, color="#1f77b4", linewidth=1.2, label="左-右角度差")
    ax1.set_ylabel("角度差 (rad)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2.plot(
        freq_df["t_mid_s"],
        freq_df["freq_hz"],
        color="#d62728",
        linewidth=1.4,
        marker="o",
        markersize=3,
        label=freq_label,
    )
    ax2.set_xlabel("时间 (s)")
    ax2.set_ylabel("频率 (Hz)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    fig.suptitle("左右角度差与频率随时间", fontsize=14)
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)


def main() -> None:
    csv_path = Path(FILE_PATH)
    if not csv_path.exists():
        print(f"Input file not found: {csv_path}")
        return

    output_dir = Path(BASE_OUTPUT_DIR) / csv_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = output_dir / "time_freq_angle_diff.csv"
    output_plot = output_dir / "time_freq_angle_diff.png"

    analysis = analyze_single_file(
        csv_path=csv_path,
        fallback_dt=FALLBACK_DT,
        window_s=WINDOW_S,
        step_s=STEP_S,
        fmin=FMIN,
        fmax=FMAX,
    )
    if analysis is None:
        return

    df_freq, time_axis, angle_diff, _dt, freq_label = analysis
    df_freq.to_csv(output_csv, index=False)
    print(f"Time-varying frequency saved to: {output_csv.resolve()}")

    plot_results(time_axis, angle_diff, df_freq, freq_label, output_plot)
    print(f"Plot saved to: {output_plot.resolve()}")


if __name__ == "__main__":
    main()
