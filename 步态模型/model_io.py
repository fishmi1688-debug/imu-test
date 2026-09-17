from __future__ import annotations

from pathlib import Path

import h5py
import joblib
import numpy as np


MODEL_INPUT_DIM = 70


class FeatureScalerAdapter:
    """Small StandardScaler-compatible wrapper for dict-based scaler exports."""

    def __init__(self, mean: np.ndarray, scale: np.ndarray, n_features_in: int | None = None):
        mean = np.asarray(mean, dtype=np.float32)
        scale = np.asarray(scale, dtype=np.float32)
        if mean.ndim != 1 or scale.ndim != 1 or mean.shape != scale.shape:
            raise ValueError(
                f"scaler mean/scale 形状异常: mean={mean.shape}, scale={scale.shape}"
            )
        scale = np.where((scale == 0.0) | ~np.isfinite(scale), 1.0, scale)
        self.mean_ = mean
        self.scale_ = scale
        self.n_features_in_ = int(n_features_in or mean.shape[0])
        if self.n_features_in_ != mean.shape[0]:
            raise ValueError(
                f"scaler 输入维度异常: n_features_in={self.n_features_in_}, mean={mean.shape}"
            )

    def transform(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.n_features_in_:
            raise ValueError(f"scaler 输入应为 (N, {self.n_features_in_})，收到 {x.shape}")
        return ((x - self.mean_) / self.scale_).astype(np.float32)


def load_feature_scaler(scaler_path: Path):
    scaler = joblib.load(scaler_path)
    if hasattr(scaler, "transform"):
        return scaler
    if isinstance(scaler, dict):
        mean = scaler.get("mean", scaler.get("mean_"))
        scale = scaler.get("scale", scaler.get("scale_"))
        if mean is None or scale is None:
            raise ValueError(f"dict scaler 缺少 mean/scale: {scaler_path}")
        n_features_in = scaler.get("n_features_in", scaler.get("n_features_in_"))
        return FeatureScalerAdapter(mean=mean, scale=scale, n_features_in=n_features_in)
    raise ValueError(f"不支持的 scaler 格式: {type(scaler).__name__} ({scaler_path})")


def _read_varint(data: bytes, offset: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while True:
        if offset >= len(data):
            raise ValueError("protobuf varint 越界")
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            return value, offset
        shift += 7
        if shift > 70:
            raise ValueError("protobuf varint 非法")


def _iter_fields(message: bytes):
    offset = 0
    while offset < len(message):
        key, offset = _read_varint(message, offset)
        field_number = key >> 3
        wire_type = key & 0x07

        if wire_type == 0:
            value, offset = _read_varint(message, offset)
        elif wire_type == 1:
            if offset + 8 > len(message):
                raise ValueError("protobuf fixed64 越界")
            value = message[offset:offset + 8]
            offset += 8
        elif wire_type == 2:
            size, offset = _read_varint(message, offset)
            if offset + size > len(message):
                raise ValueError("protobuf length-delimited 越界")
            value = message[offset:offset + size]
            offset += size
        elif wire_type == 5:
            if offset + 4 > len(message):
                raise ValueError("protobuf fixed32 越界")
            value = message[offset:offset + 4]
            offset += 4
        else:
            raise ValueError(f"不支持的 protobuf wire_type: {wire_type}")

        yield field_number, wire_type, value


def _parse_packed_floats(raw: bytes) -> list[float]:
    if len(raw) % 4 != 0:
        raise ValueError("packed float_data 长度不是 4 的倍数")
    return np.frombuffer(raw, dtype="<f4").astype(np.float32).tolist()


def _parse_tensor_proto(tensor_bytes: bytes) -> tuple[str, np.ndarray]:
    dims: list[int] = []
    data_type: int | None = None
    name = ""
    raw_data: bytes | None = None
    float_data: list[float] = []
    int32_data: list[int] = []
    int64_data: list[int] = []

    for field_number, wire_type, value in _iter_fields(tensor_bytes):
        if field_number == 1 and wire_type == 0:
            dims.append(int(value))
        elif field_number == 2 and wire_type == 0:
            data_type = int(value)
        elif field_number == 4:
            if wire_type == 5:
                float_data.append(float(np.frombuffer(value, dtype="<f4")[0]))
            elif wire_type == 2:
                float_data.extend(_parse_packed_floats(value))
        elif field_number == 5 and wire_type == 0:
            int32_data.append(int(value))
        elif field_number == 7 and wire_type == 0:
            int64_data.append(int(value))
        elif field_number == 8 and wire_type == 2:
            name = value.decode("utf-8")
        elif field_number == 9 and wire_type == 2:
            raw_data = bytes(value)

    if data_type is None:
        raise ValueError(f"TensorProto 缺少 data_type: {name!r}")

    if data_type == 1:
        dtype = np.dtype("<f4")
        if raw_data is not None:
            array = np.frombuffer(raw_data, dtype=dtype).astype(np.float32, copy=True)
        else:
            array = np.asarray(float_data, dtype=np.float32)
    elif data_type == 6:
        dtype = np.dtype("<i4")
        if raw_data is not None:
            array = np.frombuffer(raw_data, dtype=dtype).astype(np.int32, copy=True)
        else:
            array = np.asarray(int32_data, dtype=np.int32)
    elif data_type == 7:
        dtype = np.dtype("<i8")
        if raw_data is not None:
            array = np.frombuffer(raw_data, dtype=dtype).astype(np.int64, copy=True)
        else:
            array = np.asarray(int64_data, dtype=np.int64)
    else:
        raise ValueError(f"当前仅支持 float/int32/int64 TensorProto，收到 data_type={data_type}")

    if dims:
        array = array.reshape(tuple(dims))
    return name, array


def _load_initializers_from_onnx(model_path: Path) -> dict[str, np.ndarray]:
    model_bytes = model_path.read_bytes()
    graph_bytes = None
    for field_number, wire_type, value in _iter_fields(model_bytes):
        if field_number == 7 and wire_type == 2:
            graph_bytes = value
            break
    if graph_bytes is None:
        raise ValueError(f"在 ONNX 模型中未找到 graph: {model_path}")

    initializers: dict[str, np.ndarray] = {}
    for field_number, wire_type, value in _iter_fields(graph_bytes):
        if field_number == 5 and wire_type == 2:
            name, array = _parse_tensor_proto(value)
            if name:
                initializers[name] = array
    if not initializers:
        raise ValueError(f"在 ONNX graph 中未找到 initializer: {model_path}")
    return initializers


def _pick_initializer(initializers: dict[str, np.ndarray], suffix: str) -> np.ndarray:
    matches = [tensor for name, tensor in initializers.items() if name.endswith(suffix)]
    if len(matches) != 1:
        names = ", ".join(sorted(initializers))
        raise ValueError(f"无法唯一匹配 initializer 后缀 {suffix!r}，当前可用: {names}")
    return matches[0].astype(np.float32, copy=False)


def load_onnx_dense_weights(model_path: Path) -> dict[str, np.ndarray]:
    initializers = _load_initializers_from_onnx(model_path)
    params = {
        "w1": _pick_initializer(initializers, "hidden_1/MatMul/ReadVariableOp:0"),
        "b1": _pick_initializer(initializers, "hidden_1/BiasAdd/ReadVariableOp:0"),
        "w2": _pick_initializer(initializers, "hidden_2/MatMul/ReadVariableOp:0"),
        "b2": _pick_initializer(initializers, "hidden_2/BiasAdd/ReadVariableOp:0"),
        "w3": _pick_initializer(initializers, "output/MatMul/ReadVariableOp:0"),
        "b3": _pick_initializer(initializers, "output/BiasAdd/ReadVariableOp:0"),
    }
    return params


def load_keras_dense_weights(model_path: Path) -> dict[str, np.ndarray]:
    with h5py.File(model_path, "r") as f:
        params = {
            "w1": np.array(f["model_weights/hidden_1/hidden_1/kernel:0"], dtype=np.float32),
            "b1": np.array(f["model_weights/hidden_1/hidden_1/bias:0"], dtype=np.float32),
            "w2": np.array(f["model_weights/hidden_2/hidden_2/kernel:0"], dtype=np.float32),
            "b2": np.array(f["model_weights/hidden_2/hidden_2/bias:0"], dtype=np.float32),
            "w3": np.array(f["model_weights/output/output/kernel:0"], dtype=np.float32),
            "b3": np.array(f["model_weights/output/output/bias:0"], dtype=np.float32),
        }
    return params


def load_float_dense_weights(model_path: Path) -> dict[str, np.ndarray]:
    suffix = model_path.suffix.lower()
    if suffix == ".onnx":
        return load_onnx_dense_weights(model_path)
    if suffix == ".keras":
        return load_keras_dense_weights(model_path)
    raise ValueError(f"不支持的浮点模型格式: {model_path}")
