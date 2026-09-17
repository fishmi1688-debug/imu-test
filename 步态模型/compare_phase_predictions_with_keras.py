#!/usr/bin/env python3
"""使用 .keras 模型重算相位，并与 phase_predictions.csv 对比绘图。

特性:
- 不读取/覆盖原会话目录，先复制输入到隔离目录。
- 使用 `.keras` 权重（非 onnx）做前向推理。
- 复现实时脚本同款窗口特征、标准化和相位限速逻辑。
- 生成对比 CSV、误差统计和 PNG 图。
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from model_io import load_feature_scaler, load_float_dense_weights

DEFAULT_MPL_CONFIG_DIR = Path("/tmp/matplotlib-cache")
DEFAULT_MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(DEFAULT_MPL_CONFIG_DIR))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_ROOT = BASE_DIR / "data"
DEFAULT_MODEL_PATH = BASE_DIR / "gait_phase_model.keras"
DEFAULT_SCALER_PATH = BASE_DIR / "feature_scaler.pkl"
DEFAULT_OUTPUT_ROOT = DEFAULT_DATA_ROOT / "keras_phase_compare"
DEFAULT_BATCH_SESSION_NAMES = [
    "20260402_125638",
    "20260402_125701",
    "20260402_130221",
    "20260402_130343",
]
REQUIRED_SESSION_FILES = ("left_foot_imu.csv", "right_foot_imu.csv", "phase_predictions.csv")

WINDOW_N = 27
SAMPLE_RATE_HZ = 30.0
STALE_TIMEOUT_SEC = 0.7

IMU_CHANNELS = [
    "left_imu_Euler_Y",
    "right_imu_Euler_Y",
    "left_imu_Euler_X",
    "right_imu_Euler_X",
    "left_imu_Gyr_X",
    "left_imu_Gyr_Y",
    "left_imu_Gyr_Z",
    "left_imu_Acc_X",
    "left_imu_Acc_Y",
    "left_imu_Acc_Z",
]
N_CHANNELS = len(IMU_CHANNELS)
N_FEATS_PER_CH = 7
INPUT_DIM = N_CHANNELS * N_FEATS_PER_CH
ROLE_ORDER = {"right_foot": 0, "left_foot": 1}


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def circular_delta(current: float, previous: float) -> float:
    return (current - previous + 0.5) % 1.0 - 0.5


def recover_phase(cos_value: float, sin_value: float) -> float:
    return (math.atan2(sin_value, cos_value) / (2.0 * math.pi)) % 1.0


def extract_window_features(window: np.ndarray) -> np.ndarray:
    mid = len(window) // 2
    return np.array(
        [
            window.max(),
            window.min(),
            window.mean(),
            window.std(),
            window[0],
            window[mid],
            window[-1],
        ],
        dtype=np.float32,
    )


def extract_model_feature_vector(window_rows: np.ndarray) -> np.ndarray:
    feats = [extract_window_features(window_rows[:, idx]) for idx in range(window_rows.shape[1])]
    return np.concatenate(feats).astype(np.float32)


def combine_samples_to_model_channels(left_row: pd.Series, right_row: pd.Series) -> np.ndarray:
    return np.array(
        [
            float(left_row["euler_y"]),
            float(right_row["euler_y"]),
            float(left_row["euler_x"]),
            float(right_row["euler_x"]),
            float(left_row["gyro_x"]),
            float(left_row["gyro_y"]),
            float(left_row["gyro_z"]),
            float(left_row["acc_x"]),
            float(left_row["acc_y"]),
            float(left_row["acc_z"]),
        ],
        dtype=np.float32,
    )


class KerasDensePhaseNet:
    """从 .keras 中读取全连接权重并用 numpy 做前向推理。"""

    def __init__(self, model_path: Path):
        if model_path.suffix.lower() != ".keras":
            raise ValueError(f"该脚本要求 .keras 模型，收到: {model_path}")
        params = load_float_dense_weights(model_path)
        self.w1 = params["w1"]
        self.b1 = params["b1"]
        self.w2 = params["w2"]
        self.b2 = params["b2"]
        self.w3 = params["w3"]
        self.b3 = params["b3"]
        self.input_dim = int(self.w1.shape[0])
        if int(self.w3.shape[1]) != 3:
            raise ValueError(f"输出层维度异常，预期 3，实际 {self.w3.shape}")

    def predict(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.input_dim:
            raise ValueError(f"模型输入应为 (N, {self.input_dim})，收到 {x.shape}")
        h1 = relu(x @ self.w1 + self.b1)
        h2 = relu(h1 @ self.w2 + self.b2)
        return h2 @ self.w3 + self.b3


class ReplayedPhasePredictor:
    def __init__(self, model_path: Path, scaler_path: Path, sample_rate_hz: float):
        self.model = KerasDensePhaseNet(model_path=model_path)
        self.scaler = load_feature_scaler(scaler_path)
        self.sample_rate_hz = float(sample_rate_hz)
        self.window_size = WINDOW_N
        self.buffer: deque[np.ndarray] = deque(maxlen=self.window_size)
        self.prev_phase: Optional[float] = None
        self.total_rows = 0
        self.total_predictions = 0

        scaler_dim = int(getattr(self.scaler, "n_features_in_", INPUT_DIM))
        if self.model.input_dim != INPUT_DIM:
            raise ValueError(
                f"当前通道配置输入维度应为 {INPUT_DIM}，但 .keras 模型输入为 {self.model.input_dim}"
            )
        if scaler_dim != INPUT_DIM:
            raise ValueError(
                f"当前通道配置输入维度应为 {INPUT_DIM}，但 scaler 输入为 {scaler_dim}"
            )

    def push_row(
        self,
        row: np.ndarray,
        timestamp_us: Optional[int],
        local_time: Optional[str],
        arrival_time: float,
    ) -> Optional[dict]:
        self.buffer.append(np.asarray(row, dtype=np.float32))
        self.total_rows += 1
        if len(self.buffer) < self.window_size:
            return None

        window = np.array(self.buffer, dtype=np.float32)
        feature_vector = extract_model_feature_vector(window).reshape(1, -1)
        scaled = self.scaler.transform(feature_vector).astype(np.float32)
        pred = self.model.predict(scaled)[0]

        cos_value = float(pred[0])
        sin_value = float(pred[1])
        stride_rate_hz = float(pred[2])
        raw_phase = recover_phase(cos_value, sin_value)
        phase = self._apply_phase_rate_limit_step(raw_phase, stride_rate_hz)

        self.total_predictions += 1
        return {
            "local_time": local_time,
            "arrival_time_monotonic": arrival_time,
            "timestamp_us": timestamp_us,
            "input_source": "左脚+右脚",
            "phase": phase,
            "raw_phase": raw_phase,
            "stride_rate_hz": stride_rate_hz,
            "cos_value": cos_value,
            "sin_value": sin_value,
            "window_size": self.window_size,
            "total_rows": self.total_rows,
            "total_predictions": self.total_predictions,
        }

    def _apply_phase_rate_limit_step(self, raw_phase: float, stride_rate_hz: float) -> float:
        if not np.isfinite(stride_rate_hz):
            stride_rate_hz = 1.0
        r_clamped = float(np.clip(stride_rate_hz, 0.3, 2.5))
        if self.prev_phase is None:
            self.prev_phase = raw_phase
            return raw_phase

        expected_delta = r_clamped / self.sample_rate_hz
        delta = circular_delta(raw_phase, self.prev_phase)
        min_delta = expected_delta * 0.2
        max_delta = expected_delta * 1.5
        if delta < min_delta:
            delta = expected_delta
        elif delta > max_delta:
            delta = max_delta
        phase = (self.prev_phase + delta) % 1.0
        self.prev_phase = phase
        return phase


def create_run_dir(output_root: Path) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base_name = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / base_name
    suffix = 1
    while run_dir.exists():
        run_dir = output_root / f"{base_name}_{suffix}"
        suffix += 1
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def prepare_session_workspace(
    run_dir: Path,
    session_dir: Path,
    model_path: Path,
    scaler_path: Path,
) -> tuple[Path, Path, Path]:
    session_run_dir = run_dir / session_dir.name
    input_dir = session_run_dir / "inputs"
    output_dir = session_run_dir / "outputs"
    input_dir.mkdir(parents=True, exist_ok=False)
    output_dir.mkdir(parents=True, exist_ok=False)

    files_to_copy = [
        session_dir / "left_foot_imu.csv",
        session_dir / "right_foot_imu.csv",
        session_dir / "phase_predictions.csv",
        model_path,
        scaler_path,
    ]
    for src in files_to_copy:
        if not src.exists():
            raise FileNotFoundError(f"文件不存在: {src}")
        shutil.copy2(src, input_dir / src.name)

    return session_run_dir, input_dir, output_dir


def load_and_stack_imu_events(input_dir: Path) -> pd.DataFrame:
    left = pd.read_csv(input_dir / "left_foot_imu.csv")
    right = pd.read_csv(input_dir / "right_foot_imu.csv")

    required_cols = [
        "local_time",
        "arrival_time_monotonic",
        "timestamp_us",
        "euler_x",
        "euler_y",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
    ]
    for name in required_cols:
        if name not in left.columns:
            raise ValueError(f"left_foot_imu.csv 缺少列: {name}")
        if name not in right.columns:
            raise ValueError(f"right_foot_imu.csv 缺少列: {name}")

    left = left[required_cols].copy()
    right = right[required_cols].copy()
    left["role"] = "left_foot"
    right["role"] = "right_foot"

    events = pd.concat([left, right], ignore_index=True)
    events["arrival_time_monotonic"] = pd.to_numeric(events["arrival_time_monotonic"], errors="coerce")
    events["timestamp_us"] = pd.to_numeric(events["timestamp_us"], errors="coerce")
    events = events.dropna(subset=["arrival_time_monotonic"])
    events["role_order"] = events["role"].map(ROLE_ORDER).fillna(99).astype(int)
    events = events.sort_values(["arrival_time_monotonic", "role_order"], kind="mergesort").reset_index(drop=True)
    return events


def run_keras_replay(events: pd.DataFrame, predictor: ReplayedPhasePredictor) -> pd.DataFrame:
    latest: dict[str, pd.Series] = {}
    rows: list[dict] = []

    for _, event in events.iterrows():
        role = str(event["role"])
        now = float(event["arrival_time_monotonic"])
        latest[role] = event

        if role != "left_foot":
            continue
        if "left_foot" not in latest or "right_foot" not in latest:
            continue

        left_row = latest["left_foot"]
        right_row = latest["right_foot"]
        left_age = now - float(left_row["arrival_time_monotonic"])
        right_age = now - float(right_row["arrival_time_monotonic"])
        if left_age > STALE_TIMEOUT_SEC or right_age > STALE_TIMEOUT_SEC:
            continue

        merged_row = combine_samples_to_model_channels(left_row, right_row)
        timestamp_us = left_row.get("timestamp_us")
        if pd.isna(timestamp_us):
            ts = None
        else:
            ts = int(timestamp_us)
        pred = predictor.push_row(
            row=merged_row,
            timestamp_us=ts,
            local_time=str(left_row.get("local_time")),
            arrival_time=float(left_row["arrival_time_monotonic"]),
        )
        if pred is not None:
            rows.append(pred)

    return pd.DataFrame(rows)


def build_comparison(
    input_dir: Path,
    keras_df: pd.DataFrame,
) -> tuple[pd.DataFrame, str]:
    ref = pd.read_csv(input_dir / "phase_predictions.csv")
    if "timestamp_us" not in ref.columns or "phase" not in ref.columns:
        raise ValueError("phase_predictions.csv 缺少必要列: timestamp_us / phase")
    if keras_df.empty:
        raise ValueError("未生成任何 keras 预测结果，请检查输入数据与模型。")

    ref = ref.copy()
    ref["timestamp_us"] = pd.to_numeric(ref["timestamp_us"], errors="coerce")
    keras_df = keras_df.copy()
    keras_df["timestamp_us"] = pd.to_numeric(keras_df["timestamp_us"], errors="coerce")
    ref = ref.dropna(subset=["timestamp_us"])
    keras_df = keras_df.dropna(subset=["timestamp_us"])
    ref["timestamp_us"] = ref["timestamp_us"].astype(np.int64)
    keras_df["timestamp_us"] = keras_df["timestamp_us"].astype(np.int64)

    ref = ref.sort_values("timestamp_us").drop_duplicates("timestamp_us", keep="first")
    keras_df = keras_df.sort_values("timestamp_us").drop_duplicates("timestamp_us", keep="first")

    ref_rename = {
        "phase": "phase_onnx",
        "raw_phase": "raw_phase_onnx",
        "stride_rate_hz": "stride_rate_hz_onnx",
        "cos_value": "cos_value_onnx",
        "sin_value": "sin_value_onnx",
        "total_predictions": "total_predictions_onnx",
    }
    for old, new in ref_rename.items():
        if old in ref.columns:
            ref[new] = ref[old]

    keras_rename = {
        "phase": "phase_keras",
        "raw_phase": "raw_phase_keras",
        "stride_rate_hz": "stride_rate_hz_keras",
        "cos_value": "cos_value_keras",
        "sin_value": "sin_value_keras",
        "total_predictions": "total_predictions_keras",
    }
    for old, new in keras_rename.items():
        if old in keras_df.columns:
            keras_df[new] = keras_df[old]

    ref_cols = ["timestamp_us"] + [v for v in ref_rename.values() if v in ref.columns]
    keras_cols = ["timestamp_us"] + [v for v in keras_rename.values() if v in keras_df.columns]
    merged = ref[ref_cols].merge(keras_df[keras_cols], on="timestamp_us", how="inner")
    alignment = "timestamp_us"

    if merged.empty:
        n = min(len(ref), len(keras_df))
        if n <= 0:
            raise ValueError("无法对齐 onnx 与 keras 的预测序列。")
        merged = pd.DataFrame(
            {
                "timestamp_us": np.arange(n, dtype=np.int64),
                "phase_onnx": ref["phase_onnx"].values[:n],
                "phase_keras": keras_df["phase_keras"].values[:n],
            }
        )
        alignment = "index_fallback"

    delta = ((merged["phase_keras"] - merged["phase_onnx"] + 0.5) % 1.0) - 0.5
    merged["phase_error_cycle"] = delta
    merged["phase_error_percent"] = delta * 100.0
    merged["phase_abs_error_percent"] = np.abs(delta) * 100.0
    return merged, alignment


def plot_comparison(compare_df: pd.DataFrame, out_png: Path, alignment: str) -> None:
    x = np.arange(len(compare_df), dtype=np.int64)
    phase_onnx = compare_df["phase_onnx"].to_numpy(dtype=np.float64)
    phase_keras = compare_df["phase_keras"].to_numpy(dtype=np.float64)
    marker_step = max(len(x) // 120, 1)

    fig, ax = plt.subplots(1, 1, figsize=(14, 5), constrained_layout=True)
    ax.plot(
        x,
        phase_onnx,
        label="ONNX phase",
        linewidth=1.8,
        color="#1f77b4",
        marker="o",
        markevery=slice(0, None, marker_step),
        markersize=3.2,
        markerfacecolor="white",
        markeredgewidth=0.7,
        zorder=2,
    )
    ax.plot(
        x,
        phase_keras,
        label="Keras phase",
        linewidth=1.3,
        color="#ff7f0e",
        alpha=0.9,
        linestyle=(0, (5, 2)),
        marker="s",
        markevery=slice(marker_step // 2, None, marker_step),
        markersize=3.0,
        markerfacecolor="white",
        markeredgewidth=0.7,
        zorder=3,
    )
    ax.set_xlabel("Aligned Sample Index")
    ax.set_ylabel("Phase (0~1)")
    ax.set_title(f"ONNX vs Keras Phase (alignment={alignment})")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.savefig(out_png, dpi=170)
    plt.close(fig)


def write_summary(
    out_path: Path,
    session_dir: Path,
    run_dir: Path,
    model_path: Path,
    scaler_path: Path,
    n_ref: int,
    n_keras: int,
    n_cmp: int,
    alignment: str,
    mae: float,
    rmse: float,
    max_abs: float,
) -> None:
    text = "\n".join(
        [
            "Keras vs phase_predictions 对比摘要",
            f"source_session_dir: {session_dir}",
            f"isolated_run_dir: {run_dir}",
            f"model: {model_path}",
            f"scaler: {scaler_path}",
            f"reference_rows(onnx_csv): {n_ref}",
            f"generated_rows(keras): {n_keras}",
            f"aligned_rows: {n_cmp}",
            f"alignment: {alignment}",
            f"mae_percent: {mae:.6f}",
            f"rmse_percent: {rmse:.6f}",
            f"max_abs_percent: {max_abs:.6f}",
        ]
    )
    out_path.write_text(text + "\n", encoding="utf-8")


def resolve_session_dir(data_root: Path, raw_session: str) -> Path:
    candidate = Path(raw_session).expanduser()
    if candidate.is_absolute():
        path = candidate.resolve()
    else:
        if candidate.exists():
            path = candidate.resolve()
        else:
            path = (data_root / candidate).resolve()
    if not path.exists() or not path.is_dir():
        raise FileNotFoundError(f"会话目录不存在: {raw_session} -> {path}")
    missing = [name for name in REQUIRED_SESSION_FILES if not (path / name).exists()]
    if missing:
        raise FileNotFoundError(f"会话目录缺少文件: {path} | 缺少: {', '.join(missing)}")
    return path


def resolve_session_dirs(args: argparse.Namespace, data_root: Path) -> list[Path]:
    # 每次运行都至少处理默认四个会话目录；外部参数只做追加。
    raw_sessions = list(DEFAULT_BATCH_SESSION_NAMES)
    if args.session_dirs:
        raw_sessions.extend(str(item) for item in args.session_dirs)
    if args.session_dir:
        raw_sessions.append(str(args.session_dir))

    resolved: list[Path] = []
    seen: set[str] = set()
    for item in raw_sessions:
        path = resolve_session_dir(data_root=data_root, raw_session=item)
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        resolved.append(path)
    return resolved


def process_one_session(
    run_dir: Path,
    session_dir: Path,
    model_path: Path,
    scaler_path: Path,
    sample_rate_hz: float,
) -> dict:
    session_run_dir, input_dir, output_dir = prepare_session_workspace(
        run_dir=run_dir,
        session_dir=session_dir,
        model_path=model_path,
        scaler_path=scaler_path,
    )

    predictor = ReplayedPhasePredictor(
        model_path=input_dir / model_path.name,
        scaler_path=input_dir / scaler_path.name,
        sample_rate_hz=sample_rate_hz,
    )
    events = load_and_stack_imu_events(input_dir)
    keras_df = run_keras_replay(events=events, predictor=predictor)
    if keras_df.empty:
        raise RuntimeError(f"{session_dir.name} 未得到 keras 预测结果，无法继续对比。")

    keras_csv = output_dir / "phase_predictions_from_keras.csv"
    keras_df.to_csv(keras_csv, index=False)

    compare_df, alignment = build_comparison(input_dir=input_dir, keras_df=keras_df)
    compare_csv = output_dir / "phase_compare_onnx_vs_keras.csv"
    compare_df.to_csv(compare_csv, index=False)

    plot_png = output_dir / "phase_compare_onnx_vs_keras.png"
    plot_comparison(compare_df=compare_df, out_png=plot_png, alignment=alignment)

    mae = float(compare_df["phase_abs_error_percent"].mean())
    rmse = float(np.sqrt(np.mean(np.square(compare_df["phase_error_percent"]))))
    max_abs = float(compare_df["phase_abs_error_percent"].max())

    n_ref = int(pd.read_csv(input_dir / "phase_predictions.csv").shape[0])
    n_keras = int(keras_df.shape[0])
    n_cmp = int(compare_df.shape[0])

    summary_path = output_dir / "summary.txt"
    write_summary(
        out_path=summary_path,
        session_dir=session_dir,
        run_dir=session_run_dir,
        model_path=model_path,
        scaler_path=scaler_path,
        n_ref=n_ref,
        n_keras=n_keras,
        n_cmp=n_cmp,
        alignment=alignment,
        mae=mae,
        rmse=rmse,
        max_abs=max_abs,
    )

    return {
        "session_name": session_dir.name,
        "session_dir": str(session_dir),
        "session_run_dir": str(session_run_dir),
        "input_dir": str(input_dir),
        "output_dir": str(output_dir),
        "keras_csv": str(keras_csv),
        "compare_csv": str(compare_csv),
        "plot_png": str(plot_png),
        "summary_path": str(summary_path),
        "alignment": alignment,
        "reference_rows": n_ref,
        "generated_rows": n_keras,
        "aligned_rows": n_cmp,
        "mae_percent": mae,
        "rmse_percent": rmse,
        "max_abs_percent": max_abs,
    }


def write_batch_summary(run_dir: Path, results: list[dict]) -> Path:
    rows = []
    for item in results:
        rows.append(
            {
                "session_name": item["session_name"],
                "session_dir": item["session_dir"],
                "session_run_dir": item["session_run_dir"],
                "alignment": item["alignment"],
                "reference_rows": item["reference_rows"],
                "generated_rows": item["generated_rows"],
                "aligned_rows": item["aligned_rows"],
                "mae_percent": item["mae_percent"],
                "rmse_percent": item["rmse_percent"],
                "max_abs_percent": item["max_abs_percent"],
                "plot_png": item["plot_png"],
            }
        )
    summary_csv = run_dir / "batch_summary.csv"
    pd.DataFrame(rows).to_csv(summary_csv, index=False)
    return summary_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="用 .keras 模型重算相位，并与 phase_predictions.csv 对比绘图（输入/输出写入隔离目录）。"
    )
    parser.add_argument(
        "--session-dir",
        type=Path,
        help="单个会话目录（兼容旧参数）。该目录会追加到默认四个会话中一起处理。",
    )
    parser.add_argument(
        "--session-dirs",
        nargs="+",
        help=(
            "批量会话目录列表，可写目录名或路径。"
            "传入目录会追加到默认批量会话中。默认批量处理: "
            "20260402_125638 20260402_125701 20260402_130221 20260402_130343"
        ),
    )
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT, help=f"数据根目录，默认: {DEFAULT_DATA_ROOT}")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help=f".keras 模型路径，默认: {DEFAULT_MODEL_PATH}")
    parser.add_argument("--scaler", type=Path, default=DEFAULT_SCALER_PATH, help=f"特征标准化器路径，默认: {DEFAULT_SCALER_PATH}")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT, help=f"隔离输出根目录，默认: {DEFAULT_OUTPUT_ROOT}")
    parser.add_argument("--sample-rate-hz", type=float, default=SAMPLE_RATE_HZ, help=f"相位限速采样率，默认: {SAMPLE_RATE_HZ}")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_root = args.data_root.expanduser().resolve()
    model_path = args.model.expanduser().resolve()
    scaler_path = args.scaler.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"模型文件不存在: {model_path}")
    if not scaler_path.exists():
        raise FileNotFoundError(f"scaler 文件不存在: {scaler_path}")

    session_dirs = resolve_session_dirs(args=args, data_root=data_root)
    run_dir = create_run_dir(output_root=output_root)

    print(f"批量运行目录: {run_dir}")
    print(f"会话数量: {len(session_dirs)}")

    results: list[dict] = []
    for idx, session_dir in enumerate(session_dirs, start=1):
        print(f"[{idx}/{len(session_dirs)}] 处理会话: {session_dir}")
        result = process_one_session(
            run_dir=run_dir,
            session_dir=session_dir,
            model_path=model_path,
            scaler_path=scaler_path,
            sample_rate_hz=args.sample_rate_hz,
        )
        results.append(result)
        print(
            f"  完成: {result['session_name']} | 对齐={result['alignment']} | 样本={result['aligned_rows']} "
            f"| MAE={result['mae_percent']:.4f}% | RMSE={result['rmse_percent']:.4f}% | MAX={result['max_abs_percent']:.4f}%"
        )
        print(f"  输出目录: {result['output_dir']}")

    batch_summary = write_batch_summary(run_dir=run_dir, results=results)
    print(f"批量摘要: {batch_summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
