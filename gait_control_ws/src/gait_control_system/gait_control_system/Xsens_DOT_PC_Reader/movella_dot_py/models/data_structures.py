from dataclasses import dataclass
from typing import Optional
import struct
import numpy as np
from .enums import FilterProfile, OutputRate, PayloadMode

@dataclass
class DeviceInfo:
    """设备信息结构。

    保存从设备信息特征读取的静态信息，如序列号和固件版本等。
    """
    mac_address: str
    firmware_version: str
    serial_number: int
    product_code: str
    device_tag: str
    output_rate: int
    filter_profile: FilterProfile

@dataclass
class SensorConfiguration:
    """传感器配置参数。

    用于设置输出率、滤波器与 payload 模式等运行参数。
    """
    output_rate: OutputRate = OutputRate.RATE_60
    filter_profile: FilterProfile = FilterProfile.GENERAL
    payload_mode: PayloadMode = PayloadMode.COMPLETE_EULER

@dataclass
class Timestamp:
    """时间戳（微秒）。"""
    microseconds: int

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Timestamp':
        """从 4 字节小端数据解析时间戳。"""
        return cls(struct.unpack('<I', data[:4])[0])

@dataclass
class Quaternion:
    """四元数姿态数据。"""
    w: float
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Quaternion':
        """从 16 字节小端浮点数组解析四元数。"""
        return cls(*struct.unpack('<4f', data[:16]))

    def to_numpy(self) -> np.ndarray:
        """转换为 numpy 数组 (w, x, y, z)。"""
        return np.array([self.w, self.x, self.y, self.z])

@dataclass
class EulerAngles:
    """欧拉角（单位：度）。"""
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'EulerAngles':
        """从 12 字节小端浮点数组解析欧拉角。"""
        return cls(*struct.unpack('<3f', data[:12]))

    def to_numpy(self) -> np.ndarray:
        """转换为 numpy 数组 (roll, pitch, yaw)。"""
        return np.array([self.roll, self.pitch, self.yaw])

@dataclass
class Vector3:
    """三维向量（用于加速度/角速度等）。"""
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Vector3':
        """从 12 字节小端浮点数组解析三维向量。"""
        return cls(*struct.unpack('<3f', data[:12]))

    def to_numpy(self) -> np.ndarray:
        """转换为 numpy 数组 (x, y, z)。"""
        return np.array([self.x, self.y, self.z])

@dataclass
class MagneticField:
    """磁场数据。"""
    x: float
    y: float
    z: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'MagneticField':
        """从 6 字节小端有符号整数解析磁场并进行缩放。"""
        TWO_POW_TWELVE = 2 ** 12
        if len(data) != 6:
            raise ValueError("Magnetic field data must be 6 bytes.")
        x, y, z = struct.unpack('<hhh', data)
        return cls(x / TWO_POW_TWELVE,
                  y / TWO_POW_TWELVE,
                  z / TWO_POW_TWELVE)

    def to_numpy(self) -> np.ndarray:
        """转换为 numpy 数组 (x, y, z)。"""
        return np.array([self.x, self.y, self.z])

@dataclass
class Status:
    """状态位信息。"""
    value: int

    def is_clipping_acc_x(self) -> bool:
        """判断加速度计 X 轴是否发生剪切。"""
        return bool(self.value & 0x0001)
    
    def is_clipping_acc_y(self) -> bool:
        """判断加速度计 Y 轴是否发生剪切。"""
        return bool(self.value & 0x0002)
    
    def is_clipping_acc_z(self) -> bool:
        """判断加速度计 Z 轴是否发生剪切。"""
        return bool(self.value & 0x0004)
    
    def is_clipping_gyr_x(self) -> bool:
        """判断陀螺仪 X 轴是否发生剪切。"""
        return bool(self.value & 0x0008)
    
    def is_clipping_gyr_y(self) -> bool:
        """判断陀螺仪 Y 轴是否发生剪切。"""
        return bool(self.value & 0x0010)
    
    def is_clipping_gyr_z(self) -> bool:
        """判断陀螺仪 Z 轴是否发生剪切。"""
        return bool(self.value & 0x0020)
    
    def is_mag_new(self) -> bool:
        """判断是否有新的磁场数据。"""
        return bool(self.value & 0x0200)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Status':
        """从 2 字节小端数据解析状态位。"""
        return cls(struct.unpack('<H', data[:2])[0])

@dataclass
class SensorData:
    """传感器数据容器。

    不同 payload 模式会填充不同字段，此结构用于统一承载解析结果。
    """
    timestamp: Optional[Timestamp] = None
    quaternion: Optional[Quaternion] = None
    euler_angles: Optional[EulerAngles] = None
    free_acceleration: Optional[Vector3] = None
    acceleration: Optional[Vector3] = None
    angular_velocity: Optional[Vector3] = None
    magnetic_field: Optional[MagneticField] = None
    delta_q: Optional[Quaternion] = None
    delta_v: Optional[Vector3] = None
    status: Optional[Status] = None
    clipping_acc: Optional[int] = None
    clipping_gyr: Optional[int] = None
