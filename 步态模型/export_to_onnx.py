#!/usr/bin/env python3
"""根据 gait_phase_estimation.py 的配置导出 ONNX。"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import sys
from pathlib import Path
from typing import Optional


BASE_DIR = Path(__file__).resolve().parent
GAIT_PHASE_SCRIPT = BASE_DIR / "gait_phase_estimation.py"
FALLBACK_MODEL_NAME = "gait_phase_model.keras"
FALLBACK_OUTPUT_NAME = "gait_phase_model.onnx"
DEFAULT_OPSET = 11


def load_gait_phase_module():
    spec = importlib.util.spec_from_file_location(
        "gait_phase_estimation_for_export",
        GAIT_PHASE_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载脚本: {GAIT_PHASE_SCRIPT}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve_save_dir(gpe) -> Path:
    save_dir = getattr(gpe, "SAVE_DIR", "model_output")
    save_dir = Path(save_dir)
    if not save_dir.is_absolute():
        save_dir = BASE_DIR / save_dir
    return save_dir


def resolve_default_model_path(gpe) -> Path:
    candidates = [
        BASE_DIR / FALLBACK_MODEL_NAME,
        resolve_save_dir(gpe) / FALLBACK_MODEL_NAME,
    ]
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


def resolve_output_path(model_path: Path, output_path_arg: Optional[str]) -> Path:
    if output_path_arg:
        return Path(output_path_arg).expanduser().resolve()
    return model_path.with_suffix(".onnx").resolve()


def infer_input_dim_from_model(model) -> Optional[int]:
    try:
        input_shape = model.inputs[0].shape
    except Exception:
        return None

    if not input_shape:
        return None

    last_dim = input_shape[-1]
    if last_dim is None:
        return None
    try:
        return int(last_dim)
    except Exception:
        return None


def export_keras_to_onnx(
    model_path: Path,
    output_path: Path,
    opset: int = DEFAULT_OPSET,
    input_dim: Optional[int] = None,
):
    try:
        tf = importlib.import_module("tensorflow")
    except ImportError as exc:
        raise ImportError("未找到 tensorflow。请先安装 TensorFlow 再执行导出。") from exc

    try:
        tf2onnx = importlib.import_module("tf2onnx")
    except ImportError as exc:
        raise ImportError("未找到 tf2onnx。请先安装：pip install tf2onnx") from exc

    model = tf.keras.models.load_model(model_path)

    model_input_dim = infer_input_dim_from_model(model)
    if input_dim is None:
        input_dim = model_input_dim
    if input_dim is None:
        raise ValueError("无法自动推断输入维度，请显式传入 --input-dim。")
    if model_input_dim is not None and int(input_dim) != int(model_input_dim):
        raise ValueError(
            f"输入维度不一致: --input-dim={input_dim}, 但模型实际输入是 {model_input_dim}"
        )

    input_signature = [
        tf.TensorSpec(
            shape=[None, int(input_dim)],
            dtype=tf.float32,
            name="features",
        )
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tf2onnx.convert.from_keras(
        model,
        input_signature=input_signature,
        opset=opset,
        output_path=str(output_path),
    )
    return output_path


def validate_onnx(onnx_path: Path):
    try:
        onnx = importlib.import_module("onnx")
    except ImportError as exc:
        raise ImportError("未找到 onnx。若需要导出后校验，请先安装：pip install onnx") from exc

    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)
    return model


def build_parser():
    parser = argparse.ArgumentParser(description="将 gait_phase_estimation.py 训练得到的 Keras 模型导出为 ONNX。")
    parser.add_argument(
        "--model",
        help="Keras 模型路径。默认优先找当前目录下的 gait_phase_model.keras，其次找 SAVE_DIR 下的同名文件。",
    )
    parser.add_argument(
        "--output",
        help="导出的 ONNX 路径。默认与模型同目录同名，仅后缀改为 .onnx。",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=DEFAULT_OPSET,
        help=f"ONNX opset 版本，默认: {DEFAULT_OPSET}",
    )
    parser.add_argument(
        "--input-dim",
        type=int,
        help="输入特征维度。默认优先读取 gait_phase_estimation.py 的 INPUT_DIM，若不可用则从模型本身推断。",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="导出后使用 onnx.checker 做一次结构校验。",
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    gpe = load_gait_phase_module()
    default_input_dim = int(getattr(gpe, "INPUT_DIM"))
    default_save_dir = resolve_save_dir(gpe)

    model_path = Path(args.model).expanduser().resolve() if args.model else resolve_default_model_path(gpe).resolve()
    output_path = resolve_output_path(model_path, args.output)
    input_dim = args.input_dim if args.input_dim is not None else default_input_dim

    if not model_path.exists():
        print(f"模型文件不存在: {model_path}", file=sys.stderr)
        print(f"gait_phase_estimation.py 的 SAVE_DIR = {default_save_dir}", file=sys.stderr)
        return 1

    print(f"加载 gait_phase_estimation.py: {GAIT_PHASE_SCRIPT}")
    print(f"读取到 INPUT_DIM = {default_input_dim}")
    print(f"读取到 SAVE_DIR   = {default_save_dir}")
    print(f"加载 Keras 模型: {model_path}")
    print(f"开始导出 ONNX: output={output_path}, opset={args.opset}, input_dim={input_dim}")

    export_keras_to_onnx(
        model_path=model_path,
        output_path=output_path,
        opset=args.opset,
        input_dim=input_dim,
    )

    print(f"导出完成: {output_path}")

    if args.check:
        validate_onnx(output_path)
        print("ONNX 校验通过")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
