from typing import List
import numpy as np
from ..models.enums import PayloadMode
from ..models.data_structures import SensorData
from .parser import PayloadParser

class SensorDataCollector:
    """收集并缓存传感器数据。

    该类负责将原始 payload 解析为结构化数据，并提供
    以 numpy 数组形式导出的便利接口，便于后处理分析。
    """
    
    def __init__(self, payload_mode: PayloadMode, mac_address: str = None):  
        """初始化收集器并绑定解析器。

        根据 payload 模式创建对应解析器，同时保存可选的设备地址，
        便于多设备场景下标识数据来源。
        """
        self.parser = PayloadParser(payload_mode)
        self.data: List[SensorData] = []
        self.mac_address = mac_address
        
    def add_data(self, raw_data: bytes) -> SensorData:
        """解析并追加一条传感器数据记录。"""
        parsed_data = self.parser.parse(raw_data)
        self.data.append(parsed_data)
        return parsed_data
    
    def clear(self):
        """清空已缓存的采样数据。"""
        self.data.clear()
    
    def get_timestamps(self) -> np.ndarray:
        """返回时间戳数组（单位：微秒）。"""
        return np.array([d.timestamp.microseconds for d in self.data])
    
    def get_quaternions(self) -> np.ndarray:
        """返回四元数数组（w, x, y, z）。"""
        return np.array([d.quaternion.to_numpy() for d in self.data if d.quaternion])
    
    def get_euler_angles(self) -> np.ndarray:
        """返回欧拉角数组（roll, pitch, yaw，单位：度）。"""
        return np.array([d.euler_angles.to_numpy() for d in self.data if d.euler_angles])
    
    def get_accelerations(self) -> np.ndarray:
        """返回加速度数组（单位视设备定义而定）。"""
        return np.array([d.acceleration.to_numpy() for d in self.data if d.acceleration])
    
    def get_free_accelerations(self) -> np.ndarray:
        """返回去重力加速度数组。"""
        return np.array([d.free_acceleration.to_numpy() for d in self.data if d.free_acceleration])

    def get_status_values(self) -> np.ndarray:
        """返回状态位数组（原始 bitmask）。"""
        return np.array([d.status.value for d in self.data if d.status])
    
    def get_acc_clipping_counts(self) -> np.ndarray:
        """返回加速度计剪切计数数组。"""
        return np.array([d.clipping_acc for d in self.data if d.clipping_acc is not None])
    
    def get_gyr_clipping_counts(self) -> np.ndarray:
        """返回陀螺仪剪切计数数组。"""
        return np.array([d.clipping_gyr for d in self.data if d.clipping_gyr is not None])
