from typing import Optional
import struct
import asyncio
from bleak import BleakClient, BleakScanner
from ..models.characteristics import MovellaDOTCharacteristics
from ..models.data_structures import (SensorConfiguration, DeviceInfo, 
                                    SensorData)
from ..models.enums import PayloadMode, FilterProfile
from .collector import SensorDataCollector
import time


class MovellaDOTSensor:
    def __init__(self, config: SensorConfiguration = None):
        """初始化传感器控制对象。

        该构造函数负责准备 BLE 客户端、默认配置以及数据收集器的占位，
        但不会发起任何 BLE 操作；真正的连接与配置在后续方法中完成。
        """
        self.client: Optional[BleakClient] = None
        self.chars = MovellaDOTCharacteristics()
        self.is_connected = False
        self.config = config or SensorConfiguration()
        self.config.payload_mode = self._validate_and_adjust_payload_mode(self.config.payload_mode)
        self.data_collector = None
        self._device_address = None
        self._device_name = None
        self._device_tag = None

    def _get_payload_characteristic(self, payload_mode: PayloadMode) -> str:
        """根据 payload 模式选择对应的特征值 UUID。

        Movella DOT 的不同数据长度会映射到不同的特征值，
        这里统一根据模式分类（短/中/长）以便后续订阅通知。
        """
        # Long payload (>40 bytes)
        if payload_mode in [
            PayloadMode.CUSTOM_MODE_5
        ]:
            return self.chars.LONG_PAYLOAD
            
        # Short payload (≤20 bytes)
        elif payload_mode in [
            PayloadMode.ORIENTATION_EULER,
            PayloadMode.ORIENTATION_QUATERNION,
            PayloadMode.FREE_ACCELERATION
        ]:
            return self.chars.SHORT_PAYLOAD
            
        # Medium payload (21-40 bytes)
        else:
            return self.chars.MEDIUM_PAYLOAD
        
    async def scan_and_connect(self, timeout=5.0):
        """扫描并连接到第一个可用的 Movella DOT 设备。

        该方法会进行 BLE 扫描，筛选名称包含 “Movella DOT” 的设备，
        然后与列表中的第一个设备建立连接并保存设备地址与名称。
        """
        print("Scanning for Movella DOT sensors...")
        devices = await BleakScanner.discover(timeout=timeout)
        dot_devices = [d for d in devices if d.name and "Movella DOT" in d.name]
        
        if not dot_devices:
            raise Exception("No Movella DOT sensors found")
            
        device = dot_devices[0]
        self._device_address = device.address
        self._device_name = device.name
        print(f"Connecting to {device.name} ({device.address})...")
        
        self.client = BleakClient(device.address)
        await self.client.connect()
        self.is_connected = True
        print("Connected successfully")

    async def reconnect(self):
        """使用上一次保存的地址进行重连。

        如果之前已经连接过并保存了 MAC 地址，则直接尝试重连；
        适用于连接中断或临时断开后的恢复。
        """
        if not self._device_address:
            raise Exception("No device address stored")
        
        print("Reconnecting...")
        try:
            self.client = BleakClient(self._device_address)
            await self.client.connect()
            self.is_connected = True
            print("Reconnected successfully")
        except Exception as e:
            print(f"Reconnection failed: {e}")
            raise
            
    async def configure_sensor(self):
        """按当前配置写入设备参数并初始化数据收集器。

        依次写入输出率、滤波器配置和 payload 模式，
        同时创建数据收集器以解析后续通知数据。
        """
        # Configure output rate
        rate_bytes = struct.pack('<H', self.config.output_rate)
        rate_config = bytearray([
            0x10,  # Visit Index with bit 4 set for output rate
            0,     # Identifying
            0,     # Power off options
            0,     # Power saving timeout X (minute)
            0,     # Power saving timeout X (second)
            0,     # Power saving timeout Y (minute)
            0,     # Power saving timeout Y (second)
            0,     # Device Tag length
            0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
            0, 0, 0, 0, 0, 0, 0, 0,
            rate_bytes[0], rate_bytes[1],  # Output rate (2 bytes)
            0,     # Filter profile index
            0, 0, 0, 0, 0  # Reserved
        ])
        
        await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, rate_config)
        print(f"Configured output rate: {self.config.output_rate}Hz")
        
        # Configure filter profile
        filter_config = bytearray([
            0x20,  # Visit Index with bit 5 set for filter profile
            0,     # Identifying
            0,     # Power off options
            0,     # Power saving timeout X (minute)
            0,     # Power saving timeout X (second)
            0,     # Power saving timeout Y (minute)
            0,     # Power saving timeout Y (second)
            0,     # Device Tag length
            0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 0,  # Output rate (2 bytes)
            self.config.filter_profile,  # Filter profile index
            0, 0, 0, 0, 0  # Reserved
        ])
        
        await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, filter_config)
        print(f"Configured filter profile: {self.config.filter_profile.name}")
        
        # Payload mode will be applied when start_measurement() writes control action=1.
        print(f"Configured payload mode: {self.config.payload_mode.name} (pending start)")
        
        # Initialize data collector
        self.data_collector = SensorDataCollector(
            self.config.payload_mode,
            self._device_address
        )

    def _validate_and_adjust_payload_mode(self, requested_mode: PayloadMode) -> PayloadMode:
        """验证 payload 模式并在不支持时降级。

        对于本仓库代码不支持的模式，会输出警告并退回到
        COMPLETE_EULER，以保证数据解析仍然可用。
        """
        unsupported_modes = [
            PayloadMode.HIGH_FIDELITY_WITH_MAG,
            PayloadMode.HIGH_FIDELITY,
            PayloadMode.CUSTOM_MODE_4
        ]
        
        if requested_mode in unsupported_modes:
            print(f"\nWARNING: Payload mode {requested_mode.name} is not supported by this code!")
            print("This mode can only be used with the official Movella SDK.")
            print("Falling back to COMPLETE_EULER mode, which provides:")
            print("- Timestamp")
            print("- Euler angles (roll, pitch, yaw)")
            print("- Free acceleration")
            return PayloadMode.COMPLETE_EULER
        
        return requested_mode

    def notification_handler(self, sender: int, data: bytearray):
        """处理 BLE 通知数据并解析输出。

        该回调会把原始数据交给解析器转换为结构化数据，
        同时将数据缓存到收集器中，并输出关键字段用于实时观察。
        """
        try:
            if self.data_collector:
                parsed_data = self.data_collector.add_data(data)
                
                print(f"\nReal-time Sensor Data from {self._device_tag} ({self._device_address}):")
                
                if parsed_data.quaternion:
                    print(f"Quaternion (w,x,y,z): {parsed_data.quaternion.w:.3f}, "
                          f"{parsed_data.quaternion.x:.3f}, {parsed_data.quaternion.y:.3f}, "
                          f"{parsed_data.quaternion.z:.3f}")
                    
                if parsed_data.euler_angles:
                    print(f"Euler (roll,pitch,yaw): {parsed_data.euler_angles.roll:.1f}°, "
                          f"{parsed_data.euler_angles.pitch:.1f}°, {parsed_data.euler_angles.yaw:.1f}°")
                    
                if parsed_data.acceleration:
                    print(f"Acceleration (x,y,z): {parsed_data.acceleration.x:.2f}, "
                          f"{parsed_data.acceleration.y:.2f}, {parsed_data.acceleration.z:.2f}")
                    
                if parsed_data.free_acceleration:
                    print(f"Free Acceleration (x,y,z): {parsed_data.free_acceleration.x:.2f}, "
                          f"{parsed_data.free_acceleration.y:.2f}, {parsed_data.free_acceleration.z:.2f}")
                    
                if parsed_data.angular_velocity:
                    print(f"Angular Velocity (x,y,z): {parsed_data.angular_velocity.x:.2f}, "
                          f"{parsed_data.angular_velocity.y:.2f}, {parsed_data.angular_velocity.z:.2f}")
                    
                if parsed_data.magnetic_field:
                    print(f"Magnetic Field (x,y,z): {parsed_data.magnetic_field.x:.2f}, "
                          f"{parsed_data.magnetic_field.y:.2f}, {parsed_data.magnetic_field.z:.2f}")
                    
                if parsed_data.status:
                    print("\nStatus Information:")
                    if parsed_data.status.is_clipping_acc_x(): print("- Accelerometer X clipping")
                    if parsed_data.status.is_clipping_acc_y(): print("- Accelerometer Y clipping")
                    if parsed_data.status.is_clipping_acc_z(): print("- Accelerometer Z clipping")
                    if parsed_data.status.is_clipping_gyr_x(): print("- Gyroscope X clipping")
                    if parsed_data.status.is_clipping_gyr_y(): print("- Gyroscope Y clipping")
                    if parsed_data.status.is_clipping_gyr_z(): print("- Gyroscope Z clipping")
                    if parsed_data.status.is_mag_new(): print("- New magnetic field data")
                
        except Exception as e:
            print(f"Error handling notification: {e}")

    async def start_measurement(self):
        """启动测量并订阅通知。

        根据 payload 模式选择对应特征值订阅通知，
        然后写入测量控制特征以启动数据流。
        """
        print("Starting measurement...")
        
        payload_char = self._get_payload_characteristic(self.config.payload_mode)
        
        await self.client.start_notify(
            payload_char,
            self.notification_handler
        )
        
        await self.client.write_gatt_char(
            self.chars.MEASUREMENT_CONTROL, 
            bytearray([1, 1, self.config.payload_mode])
        )

    async def stop_measurement(self):
        """停止测量并取消通知订阅。

        先写入测量控制特征通知设备停止输出，
        然后取消对 payload 特征值的通知订阅。
        """
        print("Stopping measurement...")
        
        await self.client.write_gatt_char(
            self.chars.MEASUREMENT_CONTROL, 
            bytearray([1, 0, self.config.payload_mode])
        )
        
        payload_char = self._get_payload_characteristic(self.config.payload_mode)
        await self.client.stop_notify(payload_char)

    async def start_recording(self, duration_seconds: int = 10):
        """启动设备端录制。

        按协议构造录制命令，包含当前时间和录制时长，
        写入 MESSAGE_CONTROL 以触发设备侧记录。
        """
        print(f"Starting recording for {duration_seconds} seconds...")
        current_time = int(time.time())
        message = bytearray([0x01, 0x07, 0x40]) + struct.pack("<I", current_time) + struct.pack("<H", duration_seconds)
        checksum = (256 - sum(message) % 256) % 256
        message.append(checksum)
        await self.client.write_gatt_char(self.chars.MESSAGE_CONTROL, message)

    async def stop_recording(self):
        """停止设备端录制。

        发送停止命令到 MESSAGE_CONTROL 以结束设备内部记录流程。
        """
        print("Stopping recording...")
        message = bytearray([0x01, 0x01, 0x41, 0xBD])
        await self.client.write_gatt_char(self.chars.MESSAGE_CONTROL, message)

    async def disconnect(self):
        """断开 BLE 连接并更新连接状态。"""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            print("Disconnected from sensor")

    def get_collected_data(self):
        """返回已收集的数据，格式化为 numpy 数组。

        若尚未初始化数据收集器则返回 None，
        否则返回包含时间戳、姿态、加速度等数组的字典。
        """
        if not self.data_collector:
            return None
        
        return {
            'device_tag': self._device_tag,
            'mac_address': self._device_address,
            'timestamps': self.data_collector.get_timestamps(),
            'quaternions': self.data_collector.get_quaternions(),
            'euler_angles': self.data_collector.get_euler_angles(),
            'accelerations': self.data_collector.get_accelerations()
        }

    async def get_device_info(self) -> DeviceInfo:
        """读取并解析设备信息特征值。

        读取设备信息、设备控制特征，并解析出 MAC、固件版本、
        序列号、产品码、设备标签、输出率和滤波器配置等信息。
        """
        try:
            info_data = await self.client.read_gatt_char(self.chars.BASE_UUID.format(0x1001))
            
            # Parse MAC Address
            mac_bytes = info_data[0:6][::-1]
            mac = ':'.join([f'{b:02X}' for b in mac_bytes])
            
            # Parse firmware version
            version_major = info_data[6]
            version_minor = info_data[7]
            version_revision = info_data[8]
            firmware_version = f"{version_major}.{version_minor}.{version_revision}"
            
            # Parse serial number
            serial_number_bytes = info_data[20:28]
            serial_number = ''.join(f'{byte:02X}' for byte in reversed(serial_number_bytes))
            
            # Parse product code
            try:
                product_code = bytes(info_data[28:34]).decode('ascii').rstrip('\x00')
            except:
                product_code = ' '.join([f'{b:02X}' for b in info_data[28:34]])
            
            # Read Device Control for current settings
            control_data = await self.client.read_gatt_char(self.chars.DEVICE_CONTROL)
            
            # Parse device tag
            tag_length = control_data[7]
            device_tag = bytes(control_data[8:8+tag_length]).decode('ascii')
            
            # Parse output rate and filter profile
            output_rate = int.from_bytes(control_data[24:26], byteorder='little')
            filter_profile = FilterProfile(control_data[26])
            
            return DeviceInfo(
                mac_address=mac,
                firmware_version=firmware_version,
                serial_number=serial_number,
                product_code=product_code,
                device_tag=device_tag,
                output_rate=output_rate,
                filter_profile=filter_profile
            )
        except Exception as e:
            print(f"Error getting device info: {e}")
            raise

    async def identify_sensor(self):
        """触发设备 LED 闪烁，用于物理识别传感器。"""
        try:
            identify_config = bytearray([
                0x01,  # Visit Index with bit 0 set for identifying
                0x01,  # Identifying set to 0x01
                0] + [0] * 29)  # Rest of the configuration bytes
            
            await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, identify_config)
            print("Sensor LED should blink 8 times in red")
        except Exception as e:
            print(f"Error identifying sensor: {e}")
            raise

    async def power_off_sensor(self):
        """按协议写入关机配置，关闭传感器电源。

        According to Table 7: Set bit 1 in Visit Index and set Power off bit to 1
        使用设备控制特征写入关机配置位，触发设备自关机。
        """
        try:
            power_off_config = bytearray([
                0x02,  # Visit Index with bit 1 set for power off
                0,     # Identifying
                0x01,  # Power off bit set to 1
                0,     # Power saving timeout X (minute)
                0,     # Power saving timeout X (second)
                0,     # Power saving timeout Y (minute)
                0,     # Power saving timeout Y (second)
                0,     # Device Tag length
                0, 0, 0, 0, 0, 0, 0, 0,  # Device Tag (16 bytes)
                0, 0, 0, 0, 0, 0, 0, 0,
                0, 0,  # Output rate (2 bytes)
                0,     # Filter profile index
                0, 0, 0, 0, 0  # Reserved
            ])
            
            await self.client.write_gatt_char(self.chars.DEVICE_CONTROL, power_off_config)
            print("Sensor powered off")
        except Exception as e:
            print(f"Error powering off sensor: {e}")
            raise
