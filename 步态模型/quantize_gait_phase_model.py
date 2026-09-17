#!/usr/bin/env python3
"""将步态相位全连接模型导出为简单的 int8 量化权重文件。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from model_io import MODEL_INPUT_DIM, load_float_dense_weights


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT = BASE_DIR / "gait_phase_model.onnx"
DEFAULT_OUTPUT = BASE_DIR / "gait_phase_model_int8.npz"


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def load_float_model(model_path: Path) -> dict[str, np.ndarray]:
    params = load_float_dense_weights(model_path)
    if params["w1"].shape != (MODEL_INPUT_DIM, 10):
        raise ValueError(f"不支持的模型结构: 第一层权重形状 {params['w1'].shape}")
    return params


def quantize_activation_int8(x: np.ndarray) -> tuple[np.ndarray, float]:
    x = np.asarray(x, dtype=np.float32)
    max_abs = float(np.max(np.abs(x)))
    if max_abs == 0.0 or not np.isfinite(max_abs):
        return np.zeros_like(x, dtype=np.int8), 1.0
    scale = max_abs / 127.0
    q = np.clip(np.round(x / scale), -127, 127).astype(np.int8)
    return q, float(scale)


def quantize_weights_per_output_channel(weights: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    weights = np.asarray(weights, dtype=np.float32)
    max_abs = np.max(np.abs(weights), axis=0)
    scales = np.where(max_abs == 0.0, 1.0, max_abs / 127.0).astype(np.float32)
    q = np.clip(np.round(weights / scales.reshape(1, -1)), -127, 127).astype(np.int8)
    return q, scales


def float_predict(params: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    h1 = relu(x @ params["w1"] + params["b1"])
    h2 = relu(h1 @ params["w2"] + params["b2"])
    return h2 @ params["w3"] + params["b3"]


def quantized_predict(qparams: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    def dense_dynamic(inp: np.ndarray, q_w: np.ndarray, w_scale: np.ndarray, bias: np.ndarray) -> np.ndarray:
        x_q, x_scale = quantize_activation_int8(inp)
        acc = x_q.astype(np.int32) @ q_w.astype(np.int32)
        return acc.astype(np.float32) * (x_scale * w_scale.reshape(1, -1)) + bias.reshape(1, -1)

    h1 = relu(dense_dynamic(x, qparams["w1_q"], qparams["w1_scale"], qparams["b1"]))
    h2 = relu(dense_dynamic(h1, qparams["w2_q"], qparams["w2_scale"], qparams["b2"]))
    return dense_dynamic(h2, qparams["w3_q"], qparams["w3_scale"], qparams["b3"])


def export_quantized_model(model_path: Path, output_path: Path) -> dict[str, np.ndarray]:
    params = load_float_model(model_path)
    w1_q, w1_scale = quantize_weights_per_output_channel(params["w1"])
    w2_q, w2_scale = quantize_weights_per_output_channel(params["w2"])
    w3_q, w3_scale = quantize_weights_per_output_channel(params["w3"])

    qparams = {
        "w1_q": w1_q,
        "w1_scale": w1_scale,
        "b1": params["b1"],
        "w2_q": w2_q,
        "w2_scale": w2_scale,
        "b2": params["b2"],
        "w3_q": w3_q,
        "w3_scale": w3_scale,
        "b3": params["b3"],
    }
    np.savez(output_path, **qparams)
    return qparams


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="导出步态相位 int8 量化模型。")
    parser.add_argument("--model", type=Path, default=DEFAULT_INPUT, help=f"输入 .onnx/.keras 路径，默认: {DEFAULT_INPUT}")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help=f"输出 .npz 路径，默认: {DEFAULT_OUTPUT}")
    parser.add_argument(
        "--compare-random",
        type=int,
        default=256,
        help="导出后用随机输入对比浮点/量化输出误差，默认 256 组。",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if not args.model.exists():
        raise SystemExit(f"输入模型不存在: {args.model}")

    qparams = export_quantized_model(args.model, args.output)
    print(f"量化模型已导出: {args.output}")

    if args.compare_random > 0:
        params = load_float_model(args.model)
        rng = np.random.default_rng(42)
        x = rng.normal(size=(args.compare_random, MODEL_INPUT_DIM)).astype(np.float32)
        y_float = float_predict(params, x)
        y_quant = quantized_predict(qparams, x)
        abs_err = np.abs(y_float - y_quant)
        print(f"随机输入对比: mean_abs_err={abs_err.mean():.6f}, max_abs_err={abs_err.max():.6f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
