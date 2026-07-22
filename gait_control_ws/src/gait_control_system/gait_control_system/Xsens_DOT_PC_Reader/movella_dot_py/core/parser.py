from ..models.enums import PayloadMode
from ..models.data_structures import (SensorData, Timestamp, Quaternion, 
                                    EulerAngles, Vector3, MagneticField, Status)

class PayloadParser:
    """不同 payload 类型的解析器。

    该类根据当前 payload 模式选择对应的解析函数，
    将原始字节流转换为结构化的 SensorData 对象。
    """
    
    def __init__(self, payload_mode: PayloadMode):
        """初始化解析器并建立模式到解析函数的映射。"""
        self.payload_mode = payload_mode
        self.parse_map = {
            PayloadMode.EXTENDED_QUATERNION: self._parse_extended_quaternion,
            PayloadMode.COMPLETE_QUATERNION: self._parse_complete_quaternion,
            PayloadMode.ORIENTATION_EULER: self._parse_orientation_euler,
            PayloadMode.ORIENTATION_QUATERNION: self._parse_orientation_quaternion,
            PayloadMode.FREE_ACCELERATION: self._parse_free_acceleration,
            PayloadMode.EXTENDED_EULER: self._parse_extended_euler,
            PayloadMode.COMPLETE_EULER: self._parse_complete_euler,
            PayloadMode.DELTA_QUANTITIES: self._parse_delta_quantities,
            PayloadMode.DELTA_QUANTITIES_WITH_MAG: self._parse_delta_quantities_with_mag,
            PayloadMode.RATE_QUANTITIES: self._parse_rate_quantities,
            PayloadMode.RATE_QUANTITIES_WITH_MAG: self._parse_rate_quantities_with_mag,
            PayloadMode.CUSTOM_MODE_1: self._parse_custom_mode_1,
            PayloadMode.CUSTOM_MODE_2: self._parse_custom_mode_2,
            PayloadMode.CUSTOM_MODE_3: self._parse_custom_mode_3,
            PayloadMode.CUSTOM_MODE_5: self._parse_custom_mode_5,
        }

    def parse(self, data: bytes) -> SensorData:
        """根据当前 payload 模式解析一条数据。

        若模式不在支持列表中则抛出异常，避免错误解析。
        """
        if self.payload_mode not in self.parse_map:
            raise ValueError(f"Unsupported payload mode: {self.payload_mode}")
        return self.parse_map[self.payload_mode](data)

    def _parse_extended_quaternion(self, data: bytes) -> SensorData:
        """解析扩展四元数 payload（36 字节）。

        包含时间戳、四元数、去重力加速度、状态位与剪切计数。
        - Timestamp (4)
        - Quaternion (16)
        - Free acceleration (12)
        - Status (2)
        - Clipping Count Accelerometer (1)
        - Clipping Count Gyroscope (1)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            free_acceleration=Vector3.from_bytes(data[20:32]),
            status=Status.from_bytes(data[32:34]),
            clipping_acc=data[34],
            clipping_gyr=data[35]
        )

    def _parse_complete_quaternion(self, data: bytes) -> SensorData:
        """解析完整四元数 payload（32 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            free_acceleration=Vector3.from_bytes(data[20:32])
        )

    def _parse_orientation_euler(self, data: bytes) -> SensorData:
        """解析欧拉角姿态 payload（16 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16])
        )

    def _parse_orientation_quaternion(self, data: bytes) -> SensorData:
        """解析四元数姿态 payload（20 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20])
        )

    def _parse_free_acceleration(self, data: bytes) -> SensorData:
        """解析去重力加速度 payload（16 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            free_acceleration=Vector3.from_bytes(data[4:16])
        )

    def _parse_extended_euler(self, data: bytes) -> SensorData:
        """解析扩展欧拉角 payload（32 字节）。

        包含欧拉角、去重力加速度、状态位与剪切计数等扩展信息。
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Status (2)
        - Clipping Count Accelerometer (1)
        - Clipping Count Gyroscope (1)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            status=Status.from_bytes(data[28:30]),
            clipping_acc=data[30],
            clipping_gyr=data[31]
        )

    def _parse_complete_euler(self, data: bytes) -> SensorData:
        """解析完整欧拉角 payload（28 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28])
        )

    def _parse_delta_quantities(self, data: bytes) -> SensorData:
        """解析增量量（Delta Quantities）payload（32 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            delta_q=Quaternion.from_bytes(data[4:20]),
            delta_v=Vector3.from_bytes(data[20:32])
        )

    def _parse_delta_quantities_with_mag(self, data: bytes) -> SensorData:
        """解析带磁场的增量量 payload（38 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            delta_q=Quaternion.from_bytes(data[4:20]),
            delta_v=Vector3.from_bytes(data[20:32]),
            magnetic_field=MagneticField.from_bytes(data[32:38])
        )

    def _parse_rate_quantities(self, data: bytes) -> SensorData:
        """解析速率量（加速度/角速度）payload（28 字节）。"""
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            acceleration=Vector3.from_bytes(data[4:16]),
            angular_velocity=Vector3.from_bytes(data[16:28])
        )

    def _parse_rate_quantities_with_mag(self, data: bytes) -> SensorData:
        """解析带磁场的速率量 payload（34 字节）。

        同时包含加速度、角速度与磁场数据。
        - Timestamp (4)
        - Acceleration (12)
        - Angular velocity (12)
        - Magnetic field (6)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            acceleration=Vector3.from_bytes(data[4:16]),
            angular_velocity=Vector3.from_bytes(data[16:28]),
            magnetic_field=MagneticField.from_bytes(data[28:34])
        )

    def _parse_custom_mode_1(self, data: bytes) -> SensorData:
        """解析自定义模式 1 payload（40 字节）。

        包含欧拉角、去重力加速度与角速度。
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            angular_velocity=Vector3.from_bytes(data[28:40])
        )

    def _parse_custom_mode_2(self, data: bytes) -> SensorData:
        """解析自定义模式 2 payload（34 字节）。

        包含欧拉角、去重力加速度与磁场数据。
        - Timestamp (4)
        - Euler angles (12)
        - Free acceleration (12)
        - Magnetic field (6)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            euler_angles=EulerAngles.from_bytes(data[4:16]),
            free_acceleration=Vector3.from_bytes(data[16:28]),
            magnetic_field=MagneticField.from_bytes(data[28:34])
        )

    def _parse_custom_mode_3(self, data: bytes) -> SensorData:
        """解析自定义模式 3 payload（32 字节）。

        包含四元数与角速度数据。
        - Timestamp (4)
        - Quaternion (16)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            angular_velocity=Vector3.from_bytes(data[20:32])
        )

    def _parse_custom_mode_5(self, data: bytes) -> SensorData:
        """解析自定义模式 5 payload（44 字节）。

        同时包含四元数、加速度与角速度数据。
        Contains:
        - Timestamp (4)
        - Quaternion (16)
        - Acceleration (12)
        - Angular velocity (12)
        """
        return SensorData(
            timestamp=Timestamp.from_bytes(data[0:4]),
            quaternion=Quaternion.from_bytes(data[4:20]),
            acceleration=Vector3.from_bytes(data[20:32]),
            angular_velocity=Vector3.from_bytes(data[32:44])
        )
