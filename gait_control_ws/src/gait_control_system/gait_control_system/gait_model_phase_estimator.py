#!/usr/bin/env python3

"""Realtime ONNX gait phase estimator used by the model_phase mode."""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np

from .imu_phase_estimator import FirstOrderLowpass, quaternion_to_euler_xyz_degrees

try:
    import joblib
except Exception as exc:  # pragma: no cover - depends on runtime environment
    joblib = None
    _JOBLIB_IMPORT_ERROR = exc
else:
    _JOBLIB_IMPORT_ERROR = None


MODEL_INPUT_CHANNELS = (
    "left_imu_Euler_Y",
    "left_imu_Euler_X",
    "right_imu_Euler_Y",
    "right_imu_Euler_X",
    "left_imu_Gyr_X",
    "left_imu_Gyr_Y",
    "left_imu_Gyr_Z",
    "left_imu_Acc_X",
    "left_imu_Acc_Y",
    "left_imu_Acc_Z",
)
MODEL_PHASE_WINDOW_N = 27
MODEL_PHASE_SAMPLE_RATE_HZ = 30.0
MODEL_PHASE_FEATURES_PER_CHANNEL = 7
MODEL_PHASE_INPUT_DIM = len(MODEL_INPUT_CHANNELS) * MODEL_PHASE_FEATURES_PER_CHANNEL
MODEL_PHASE_MIN_STRIDE_RATE_HZ = 0.3
MODEL_PHASE_MAX_STRIDE_RATE_HZ = 2.5
MODEL_PHASE_MIN_RATE_FACTOR = 0.2
MODEL_PHASE_MAX_RATE_FACTOR = 1.5

DEFAULT_MODEL_PHASE_MODEL_PATH = (
    Path(__file__).resolve().parent / "model" / "gait_phase_model.onnx"
)
DEFAULT_MODEL_PHASE_SCALER_PATH = (
    Path(__file__).resolve().parent / "model" / "feature_scaler.pkl"
)


class FeatureScalerAdapter:
    """Small StandardScaler-compatible wrapper for dict-based scaler exports."""

    def __init__(
        self,
        mean: np.ndarray,
        scale: np.ndarray,
        n_features_in: int | None = None,
    ):
        mean = np.asarray(mean, dtype=np.float32)
        scale = np.asarray(scale, dtype=np.float32)
        if mean.ndim != 1 or scale.ndim != 1 or mean.shape != scale.shape:
            raise ValueError(
                f"invalid scaler mean/scale shapes: mean={mean.shape}, scale={scale.shape}"
            )
        scale = np.where((scale == 0.0) | ~np.isfinite(scale), 1.0, scale)
        self.mean_ = mean
        self.scale_ = scale
        self.n_features_in_ = int(n_features_in or mean.shape[0])
        if self.n_features_in_ != mean.shape[0]:
            raise ValueError(
                f"invalid scaler input dim: n_features_in={self.n_features_in_}, "
                f"mean={mean.shape}"
            )

    def transform(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.n_features_in_:
            raise ValueError(f"scaler input should be (N, {self.n_features_in_}), got {x.shape}")
        return ((x - self.mean_) / self.scale_).astype(np.float32)


def load_feature_scaler(scaler_path: Path):
    if joblib is None:
        raise RuntimeError(f"joblib is required to load feature scaler: {_JOBLIB_IMPORT_ERROR}")
    scaler_path = Path(scaler_path)
    if not scaler_path.exists():
        raise FileNotFoundError(f"feature scaler not found: {scaler_path}")

    sys.modules.setdefault("model_io", sys.modules[__name__])
    scaler = joblib.load(scaler_path)
    if hasattr(scaler, "transform"):
        return scaler
    if isinstance(scaler, dict):
        mean = scaler.get("mean", scaler.get("mean_"))
        scale = scaler.get("scale", scaler.get("scale_"))
        if mean is None or scale is None:
            raise ValueError(f"dict scaler missing mean/scale: {scaler_path}")
        n_features_in = scaler.get("n_features_in", scaler.get("n_features_in_"))
        return FeatureScalerAdapter(mean=mean, scale=scale, n_features_in=n_features_in)
    raise ValueError(f"unsupported scaler type: {type(scaler).__name__} ({scaler_path})")


def _read_varint(data: bytes, offset: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while True:
        if offset >= len(data):
            raise ValueError("protobuf varint out of range")
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            return value, offset
        shift += 7
        if shift > 70:
            raise ValueError("invalid protobuf varint")


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
                raise ValueError("protobuf fixed64 out of range")
            value = message[offset:offset + 8]
            offset += 8
        elif wire_type == 2:
            size, offset = _read_varint(message, offset)
            if offset + size > len(message):
                raise ValueError("protobuf length-delimited out of range")
            value = message[offset:offset + size]
            offset += size
        elif wire_type == 5:
            if offset + 4 > len(message):
                raise ValueError("protobuf fixed32 out of range")
            value = message[offset:offset + 4]
            offset += 4
        else:
            raise ValueError(f"unsupported protobuf wire type: {wire_type}")

        yield field_number, wire_type, value


def _parse_packed_floats(raw: bytes) -> list[float]:
    if len(raw) % 4 != 0:
        raise ValueError("packed float_data length is not divisible by 4")
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
        raise ValueError(f"TensorProto missing data_type: {name!r}")

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
        raise ValueError(f"unsupported TensorProto data_type: {data_type}")

    if dims:
        array = array.reshape(tuple(dims))
    return name, array


def _load_initializers_from_onnx(model_path: Path) -> dict[str, np.ndarray]:
    model_bytes = Path(model_path).read_bytes()
    graph_bytes = None
    for field_number, wire_type, value in _iter_fields(model_bytes):
        if field_number == 7 and wire_type == 2:
            graph_bytes = value
            break
    if graph_bytes is None:
        raise ValueError(f"ONNX graph not found: {model_path}")

    initializers: dict[str, np.ndarray] = {}
    for field_number, wire_type, value in _iter_fields(graph_bytes):
        if field_number == 5 and wire_type == 2:
            name, array = _parse_tensor_proto(value)
            if name:
                initializers[name] = array
    if not initializers:
        raise ValueError(f"ONNX graph has no initializers: {model_path}")
    return initializers


def _pick_initializer(initializers: dict[str, np.ndarray], suffix: str) -> np.ndarray:
    matches = [tensor for name, tensor in initializers.items() if name.endswith(suffix)]
    if len(matches) != 1:
        names = ", ".join(sorted(initializers))
        raise ValueError(
            f"could not uniquely match initializer suffix {suffix!r}; available: {names}"
        )
    return matches[0].astype(np.float32, copy=False)


def load_onnx_dense_weights(model_path: Path) -> dict[str, np.ndarray]:
    model_path = Path(model_path)
    if not model_path.exists():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")
    initializers = _load_initializers_from_onnx(model_path)
    return {
        "w1": _pick_initializer(initializers, "hidden_1/MatMul/ReadVariableOp:0"),
        "b1": _pick_initializer(initializers, "hidden_1/BiasAdd/ReadVariableOp:0"),
        "w2": _pick_initializer(initializers, "hidden_2/MatMul/ReadVariableOp:0"),
        "b2": _pick_initializer(initializers, "hidden_2/BiasAdd/ReadVariableOp:0"),
        "w3": _pick_initializer(initializers, "output/MatMul/ReadVariableOp:0"),
        "b3": _pick_initializer(initializers, "output/BiasAdd/ReadVariableOp:0"),
    }


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
    window_rows = np.asarray(window_rows, dtype=np.float32)
    if window_rows.ndim != 2 or window_rows.shape[1] != len(MODEL_INPUT_CHANNELS):
        raise ValueError(
            f"window rows should be (N, {len(MODEL_INPUT_CHANNELS)}), got {window_rows.shape}"
        )
    feats = [
        extract_window_features(window_rows[:, idx])
        for idx in range(window_rows.shape[1])
    ]
    return np.concatenate(feats).astype(np.float32)


class DenseGaitPhaseOnnxNet:
    def __init__(self, model_path: Path):
        params = load_onnx_dense_weights(Path(model_path))
        self.w1 = params["w1"]
        self.b1 = params["b1"]
        self.w2 = params["w2"]
        self.b2 = params["b2"]
        self.w3 = params["w3"]
        self.b3 = params["b3"]

        if self.w1.ndim != 2 or self.w2.ndim != 2 or self.w3.ndim != 2:
            raise ValueError("invalid dense model weight dimensions")
        if self.w3.shape[1] != 3:
            raise ValueError(f"output layer should have 3 units, got {self.w3.shape}")

        self.input_dim = int(self.w1.shape[0])
        self.model_path = Path(model_path)

    def predict(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != self.input_dim:
            raise ValueError(f"model input should be (N, {self.input_dim}), got {x.shape}")
        h1 = relu(x @ self.w1 + self.b1)
        h2 = relu(h1 @ self.w2 + self.b2)
        return h2 @ self.w3 + self.b3


@dataclass(frozen=True)
class GaitModelPhasePrediction:
    raw_phase: float
    phase_0_to_1: float
    phase_rad: float
    stride_rate_hz: float
    stride_rate_clamped_hz: float
    cos_value: float
    sin_value: float
    left_euler_x_deg: float
    left_euler_y_deg: float
    right_euler_x_deg: float
    right_euler_y_deg: float
    left_gyro_x_deg_s: float
    left_gyro_y_deg_s: float
    left_gyro_z_deg_s: float
    right_gyro_y_deg_s: float
    left_acc_x_g: float
    left_acc_y_g: float
    left_acc_z_g: float
    window_size: int
    total_rows: int
    total_predictions: int
    zero_event: bool
    zero_event_count: int
    previous_cycle_period_sec: float
    previous_cycle_frequency_hz: float
    motion_active: bool
    is_new_prediction: bool


class RealtimeOnnxGaitPhaseEstimator:
    """Convert MI1 left/right samples into a post-processed model gait phase."""

    def __init__(
        self,
        model_path: Path | str = DEFAULT_MODEL_PHASE_MODEL_PATH,
        scaler_path: Path | str = DEFAULT_MODEL_PHASE_SCALER_PATH,
        sample_rate_hz: float = MODEL_PHASE_SAMPLE_RATE_HZ,
        window_size: int = MODEL_PHASE_WINDOW_N,
        angle_lowpass_cutoff_hz: float = 6.0,
    ):
        self.model_path = Path(model_path)
        self.scaler_path = Path(scaler_path)
        self.sample_rate_hz = float(sample_rate_hz)
        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        self.sample_period_sec = 1.0 / self.sample_rate_hz
        self.window_size = int(window_size)
        if self.window_size <= 1:
            raise ValueError("window_size must be > 1")
        self.angle_lowpass_cutoff_hz = max(0.0, float(angle_lowpass_cutoff_hz))

        self.model = DenseGaitPhaseOnnxNet(self.model_path)
        self.scaler = load_feature_scaler(self.scaler_path)
        if self.model.input_dim != MODEL_PHASE_INPUT_DIM:
            raise ValueError(
                f"model input dim mismatch: expected {MODEL_PHASE_INPUT_DIM}, "
                f"got {self.model.input_dim}"
            )
        scaler_dim = int(getattr(self.scaler, "n_features_in_", MODEL_PHASE_INPUT_DIM))
        if scaler_dim != MODEL_PHASE_INPUT_DIM:
            raise ValueError(
                f"scaler input dim mismatch: expected {MODEL_PHASE_INPUT_DIM}, got {scaler_dim}"
            )

        self.buffer: deque[np.ndarray] = deque(maxlen=self.window_size)
        self.angle_filters = {
            "left_x": FirstOrderLowpass(self.angle_lowpass_cutoff_hz),
            "left_y": FirstOrderLowpass(self.angle_lowpass_cutoff_hz),
            "right_x": FirstOrderLowpass(self.angle_lowpass_cutoff_hz),
            "right_y": FirstOrderLowpass(self.angle_lowpass_cutoff_hz),
        }
        self.reset()

    def reset(self) -> None:
        self.buffer.clear()
        self.total_rows = 0
        self.total_predictions = 0
        self.prev_phase: Optional[float] = None
        self.last_zero_time: Optional[float] = None
        self.zero_event_count = 0
        self.previous_cycle_period_sec = 1.0
        self.latest_output: Optional[GaitModelPhasePrediction] = None
        self.next_row_time: Optional[float] = None
        for angle_filter in self.angle_filters.values():
            angle_filter.reset()

    def process_samples(
        self,
        left_sample,
        left_sample_time: float,
        right_sample,
        right_sample_time: float,
    ) -> Optional[GaitModelPhasePrediction]:
        sample_time = max(float(left_sample_time), float(right_sample_time))
        if not self._accept_row_at(sample_time):
            if self.latest_output is None:
                return None
            return replace(
                self.latest_output,
                zero_event=False,
                is_new_prediction=False,
            )

        row, sample_state = self._build_model_row(left_sample, right_sample, sample_time)
        self.buffer.append(row)
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
        phase, stride_rate_clamped_hz, zero_event = self._postprocess_phase(
            raw_phase,
            stride_rate_hz,
            sample_time,
        )

        self.total_predictions += 1
        output = GaitModelPhasePrediction(
            raw_phase=raw_phase,
            phase_0_to_1=phase,
            phase_rad=float(phase * 2.0 * math.pi),
            stride_rate_hz=stride_rate_hz,
            stride_rate_clamped_hz=stride_rate_clamped_hz,
            cos_value=cos_value,
            sin_value=sin_value,
            left_euler_x_deg=sample_state["left_euler_x_deg"],
            left_euler_y_deg=sample_state["left_euler_y_deg"],
            right_euler_x_deg=sample_state["right_euler_x_deg"],
            right_euler_y_deg=sample_state["right_euler_y_deg"],
            left_gyro_x_deg_s=sample_state["left_gyro_x_deg_s"],
            left_gyro_y_deg_s=sample_state["left_gyro_y_deg_s"],
            left_gyro_z_deg_s=sample_state["left_gyro_z_deg_s"],
            right_gyro_y_deg_s=sample_state["right_gyro_y_deg_s"],
            left_acc_x_g=sample_state["left_acc_x_g"],
            left_acc_y_g=sample_state["left_acc_y_g"],
            left_acc_z_g=sample_state["left_acc_z_g"],
            window_size=self.window_size,
            total_rows=self.total_rows,
            total_predictions=self.total_predictions,
            zero_event=zero_event,
            zero_event_count=self.zero_event_count,
            previous_cycle_period_sec=float(self.previous_cycle_period_sec),
            previous_cycle_frequency_hz=(
                1.0 / self.previous_cycle_period_sec
                if self.previous_cycle_period_sec > 0.0
                else stride_rate_clamped_hz
            ),
            motion_active=True,
            is_new_prediction=True,
        )
        self.latest_output = output
        return output

    def _accept_row_at(self, sample_time: float) -> bool:
        if self.next_row_time is None:
            self.next_row_time = float(sample_time) + self.sample_period_sec
            return True
        tolerance = self.sample_period_sec * 0.1
        if float(sample_time) + tolerance < self.next_row_time:
            return False
        while self.next_row_time <= float(sample_time):
            self.next_row_time += self.sample_period_sec
        return True

    def _build_model_row(
        self,
        left_sample,
        right_sample,
        sample_time: float,
    ) -> tuple[np.ndarray, dict[str, float]]:
        left = self._as_mi1_sample(left_sample, "left")
        right = self._as_mi1_sample(right_sample, "right")
        left_euler_x, left_euler_y, _left_euler_z = quaternion_to_euler_xyz_degrees(
            left[6],
            left[7],
            left[8],
            left[9],
        )
        right_euler_x, right_euler_y, _right_euler_z = quaternion_to_euler_xyz_degrees(
            right[6],
            right[7],
            right[8],
            right[9],
        )
        left_euler_x = self.angle_filters["left_x"].update(left_euler_x, sample_time)
        left_euler_y = self.angle_filters["left_y"].update(left_euler_y, sample_time)
        right_euler_x = self.angle_filters["right_x"].update(right_euler_x, sample_time)
        right_euler_y = self.angle_filters["right_y"].update(right_euler_y, sample_time)
        row = np.array(
            [
                left_euler_y,
                left_euler_x,
                right_euler_y,
                right_euler_x,
                left[3],
                left[4],
                left[5],
                left[0],
                left[1],
                left[2],
            ],
            dtype=np.float32,
        )
        state = {
            "left_euler_x_deg": float(left_euler_x),
            "left_euler_y_deg": float(left_euler_y),
            "right_euler_x_deg": float(right_euler_x),
            "right_euler_y_deg": float(right_euler_y),
            "left_gyro_x_deg_s": float(left[3]),
            "left_gyro_y_deg_s": float(left[4]),
            "left_gyro_z_deg_s": float(left[5]),
            "right_gyro_y_deg_s": float(right[4]),
            "left_acc_x_g": float(left[0]),
            "left_acc_y_g": float(left[1]),
            "left_acc_z_g": float(left[2]),
        }
        return row, state

    @staticmethod
    def _as_mi1_sample(sample, side: str) -> np.ndarray:
        values = np.asarray(sample, dtype=np.float32).reshape(-1)
        if values.size < 10:
            raise ValueError(f"{side} MI1 sample needs 10 values, got {values.size}")
        values = values[:10]
        if not np.isfinite(values).all():
            raise ValueError(f"{side} MI1 sample contains non-finite values")
        if float(np.linalg.norm(values[:3])) <= 1e-9:
            raise ValueError(f"{side} MI1 sample has invalid zero acceleration")
        if float(np.linalg.norm(values[6:10])) <= 1e-9:
            raise ValueError(f"{side} MI1 sample has invalid zero quaternion")
        return values

    def _postprocess_phase(
        self,
        raw_phase: float,
        stride_rate_hz: float,
        sample_time: float,
    ) -> tuple[float, float, bool]:
        if not np.isfinite(stride_rate_hz):
            stride_rate_hz = 1.0
        r_clamped = float(
            np.clip(
                stride_rate_hz,
                MODEL_PHASE_MIN_STRIDE_RATE_HZ,
                MODEL_PHASE_MAX_STRIDE_RATE_HZ,
            )
        )
        if self.prev_phase is None:
            self.prev_phase = float(raw_phase)
            return float(raw_phase), r_clamped, False

        prev_phase = float(self.prev_phase)
        expected_delta = r_clamped / self.sample_rate_hz
        delta = circular_delta(float(raw_phase), prev_phase)
        min_delta = expected_delta * MODEL_PHASE_MIN_RATE_FACTOR
        max_delta = expected_delta * MODEL_PHASE_MAX_RATE_FACTOR
        if delta < min_delta:
            delta = expected_delta
        elif delta > max_delta:
            delta = max_delta

        phase = (prev_phase + delta) % 1.0
        zero_event = bool(phase < prev_phase and (prev_phase - phase) > 0.5)
        if zero_event:
            self.zero_event_count += 1
            if self.last_zero_time is not None:
                period = max(0.3, min(3.3, float(sample_time) - self.last_zero_time))
                self.previous_cycle_period_sec = period
            self.last_zero_time = float(sample_time)
        self.prev_phase = phase
        return float(phase), r_clamped, zero_event
