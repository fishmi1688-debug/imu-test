# BLE 迁移验证检查清单

## 📋 代码变更验证

### Android 端

#### BluetoothService.kt
- [x] 从 RFCOMM 迁移到 BLE GATT Server
- [x] 移除 `BluetoothSocket` 相关代码
- [x] 实现 `BluetoothGattServer` 和 `BluetoothGattServerCallback`
- [x] 实现 BLE 广播（`BluetoothLeAdvertiser`）
- [x] 定义服务和特征 UUID
- [x] 实现 GATT 回调（连接、读取、描述符写入、通知发送）
- [x] 保持公共 API 兼容（`startServer()`, `stopServer()`, `sendIMUData()`）
- [x] 无编译错误
- [x] 移除未使用的导入

#### PermissionUtil.kt
- [x] 添加 `BLUETOOTH_ADVERTISE` 权限请求
- [x] 保留原有 `BLUETOOTH_CONNECT` 权限
- [x] 保留原有 `BLUETOOTH_SCAN` 权限

#### AndroidManifest.xml
- [ ] 确认包含 `<uses-permission android:name="android.permission.BLUETOOTH_ADVERTISE" />`
- [ ] 确认包含 `<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />`
- [ ] 确认包含 `<uses-feature android:name="android.hardware.bluetooth_le" android:required="true" />`

### PC 端

#### 新文件
- [x] `ble_receiver.py` - 完整功能接收器
- [x] `ble_simple_receiver.py` - 简化版接收器

#### 依赖
- [x] `requirements.txt` 更新为 `bleak>=0.22.1`

#### 功能验证
- [ ] `ble_receiver.py` 语法正确
- [ ] `ble_simple_receiver.py` 语法正确
- [ ] 导入 bleak 无错误（需先安装）

### 文档

#### 新文档
- [x] `BLE_QUICK_START.md` - BLE 快速开始
- [x] `BLE_MIGRATION.md` - 迁移指南
- [x] `INSTALLATION.md` - 安装配置
- [x] `SUMMARY.md` - 工作总结
- [x] `CHECKLIST.md` - 本文件

#### 更新文档
- [x] `README.md` - 更新协议、技术栈、使用说明
- [ ] 保留 `BLUETOOTH_GUIDE.md`（经典蓝牙参考）
- [ ] 保留 `QUICK_START.md`（经典蓝牙参考）

---

## 🧪 功能测试计划

### 第一步：环境准备

#### PC 端
```bash
# 1. 检查 Python 版本
python --version  # 应该 >= 3.7

# 2. 安装依赖
cd /home/zhang/android_APP/test/rokid-imu-motion-recorder-main
pip install -r requirements.txt

# 3. 验证安装
python -c "import bleak; print('OK')"

# 4. 测试蓝牙
bluetoothctl power on
```

**检查点：**
- [ ] Python 版本符合要求
- [ ] bleak 安装成功
- [ ] 蓝牙已开启

#### Android 端
```bash
# 1. 构建应用
./gradlew assembleDebug

# 2. 安装到设备
adb install -r app/build/outputs/apk/debug/app-debug.apk

# 3. 授予权限
adb shell pm grant com.imu.motionrecorder android.permission.BLUETOOTH_CONNECT
adb shell pm grant com.imu.motionrecorder android.permission.BLUETOOTH_ADVERTISE
adb shell pm grant com.imu.motionrecorder android.permission.BLUETOOTH_SCAN
adb shell pm grant com.imu.motionrecorder android.permission.BODY_SENSORS
```

**检查点：**
- [ ] 构建成功，无错误
- [ ] APK 安装成功
- [ ] 权限授予成功

---

### 第二步：BLE 服务测试

#### Android 端操作
1. [ ] 打开应用
2. [ ] 点击【蓝牙】按钮
3. [ ] 确认显示 "BLE服务已启动，等待连接..."
4. [ ] 查看日志：
   ```bash
   adb logcat | grep BluetoothService
   ```
   应该看到：
   - [ ] "BLE GATT server started"
   - [ ] "Advertising started"

#### PC 端扫描测试
```bash
# 运行 BLE 扫描测试
python -c "
import asyncio
from bleak import BleakScanner

async def scan():
    print('扫描 BLE 设备...')
    devices = await BleakScanner.discover(timeout=10.0)
    for d in devices:
        print(f'{d.name or \"Unknown\"}: {d.address}')
        if d.metadata and d.metadata.get('uuids'):
            for uuid in d.metadata['uuids']:
                if 'a0a0' in uuid.lower():
                    print('  → 找到 IMU 设备!')

asyncio.run(scan())
"
```

**检查点：**
- [ ] 扫描到 Android 设备
- [ ] 设备列表中显示服务 UUID `0000a0a0-...`

---

### 第三步：连接测试

#### 运行简化版接收器
```bash
# 方式1：自动扫描连接
python ble_simple_receiver.py

# 方式2：指定 MAC 地址
python ble_simple_receiver.py AC:86:D1:54:A4:A9
```

**检查点：**
- [ ] PC 端显示 "连接成功!"
- [ ] Android 端日志显示 "Device connected: XX:XX:XX:XX:XX:XX"
- [ ] PC 端列出设备服务
- [ ] 找到特征 `0000a0a1-...`

#### Android 端日志验证
```bash
adb logcat | grep -E "BluetoothService|GATT"
```

应该看到：
- [ ] "Device connected: XX:XX:XX:XX:XX:XX"
- [ ] "Notifications enabled for XX:XX:XX:XX:XX:XX"（如果 PC 端订阅了）

---

### 第四步：数据传输测试

#### Android 端操作
1. [ ] 确认 PC 端已连接
2. [ ] 点击【开始】按钮
3. [ ] 确认显示 "开始通过BLE传输数据"

#### PC 端验证
**预期输出：**
```
开始接收数据...

[     1] 速度(0.000,0.000,0.000) 加速度(0.123,-0.456,9.801) 姿态(0.5°,-1.2°,89.3°)
[     2] 速度(0.001,0.000,0.000) 加速度(0.125,-0.450,9.798) 姿态(0.6°,-1.1°,89.4°)
...
```

**检查点：**
- [ ] PC 端开始显示数据
- [ ] 数据格式正确（9个字段）
- [ ] 数据持续更新
- [ ] 无异常或错误

#### Android 端日志
```bash
adb logcat | grep "BluetoothService"
```

应该看到：
- [ ] "BLE data transmission enabled"
- [ ] （可能）"Notification to XX:XX:XX:XX:XX:XX failed" 如果有问题

---

### 第五步：完整接收器测试

#### 运行完整接收器
```bash
python ble_receiver.py
```

**检查点：**
- [ ] 自动扫描设备
- [ ] 自动连接
- [ ] 订阅通知成功
- [ ] 数据实时显示
- [ ] 创建 CSV 文件

#### 验证数据保存
```bash
ls -lh imu_data/
```

**检查点：**
- [ ] 生成 `ble_imu_YYYYMMDD_HHMMSS.csv` 文件
- [ ] 文件大小持续增长
- [ ] 文件可以正常打开

#### 检查 CSV 内容
```bash
head -20 imu_data/ble_imu_*.csv
```

**预期格式：**
```
timestamp,velocity_x,velocity_y,velocity_z,accel_x,accel_y,accel_z,pitch,roll,yaw
2023-12-02 15:30:00.123,0.000,0.000,0.000,0.123,-0.456,9.801,0.5,-1.2,89.3
2023-12-02 15:30:00.143,0.001,0.000,0.000,0.125,-0.450,9.798,0.6,-1.1,89.4
...
```

**检查点：**
- [ ] 表头正确
- [ ] 数据行格式正确
- [ ] 时间戳递增
- [ ] 数值在合理范围内

---

### 第六步：停止和重连测试

#### 测试停止
1. [ ] Android 端点击【停止】
2. [ ] 确认 PC 端不再收到新数据
3. [ ] Android 日志显示 "BLE data transmission disabled"

#### 测试重新开始
1. [ ] Android 端再次点击【开始】
2. [ ] 确认 PC 端恢复接收数据
3. [ ] 数据计数器继续累加

#### 测试断开重连
1. [ ] PC 端按 `Ctrl+C` 停止
2. [ ] Android 日志显示 "Device disconnected"
3. [ ] PC 端重新运行 `python ble_receiver.py`
4. [ ] 确认可以重新连接

**检查点：**
- [ ] 停止/开始功能正常
- [ ] 断开重连功能正常
- [ ] 无内存泄漏或崩溃

---

## 🔍 问题排查

### 问题 1: 构建失败

**检查：**
```bash
./gradlew clean
./gradlew assembleDebug --stacktrace
```

**常见原因：**
- SDK 版本不匹配
- 依赖下载失败
- 语法错误

### 问题 2: 扫描不到设备

**检查：**
1. Android 端是否显示 "BLE服务已启动"
2. 蓝牙是否开启（PC 和 Android）
3. 距离是否过远
4. 权限是否授予

**调试：**
```bash
# Android 日志
adb logcat | grep -E "BluetoothService|Advertis"

# 应该看到 "Advertising started"
```

### 问题 3: 连接失败

**检查：**
```bash
# Android 日志
adb logcat | grep GATT

# PC 端增加超时
python -c "
import asyncio
from bleak import BleakClient

async def test():
    async with BleakClient('AC:86:D1:54:A4:A9', timeout=20.0) as client:
        print('Connected:', client.is_connected)

asyncio.run(test())
"
```

### 问题 4: 收不到数据

**检查：**
1. Android 端是否点击【开始】
2. PC 端是否订阅通知成功
3. 特征 UUID 是否正确

**调试：**
```bash
# Android 日志
adb logcat | grep "notify"

# 应该看到 "Notifications enabled for XX:XX:XX:XX:XX:XX"
```

### 问题 5: 数据格式错误

**检查：**
- CSV 分隔符是否正确（应该是逗号）
- 换行符是否存在（`\n`）
- 编码是否为 UTF-8

**调试：**
```python
# 在 notification_handler 中添加：
print(f"Raw data: {data.hex()}")
print(f"Decoded: {data.decode('utf-8')}")
```

---

## ✅ 最终验收

### 功能完整性
- [ ] BLE 广播正常
- [ ] BLE 连接稳定
- [ ] 数据传输正确
- [ ] CSV 保存完整
- [ ] 停止/开始可控
- [ ] 断开重连正常

### 性能指标
- [ ] 连接延迟 < 3秒
- [ ] 数据延迟 < 100ms
- [ ] 无数据丢失
- [ ] 无内存泄漏
- [ ] CPU 使用率合理

### 文档完整性
- [ ] 所有新文档已创建
- [ ] README 已更新
- [ ] 代码注释清晰
- [ ] 示例代码可运行

### 跨平台测试（可选）
- [ ] Linux 测试通过
- [ ] Windows 测试通过
- [ ] macOS 测试通过

---

## 📊 测试报告模板

### 测试环境
- **日期**: ___________
- **Android 设备**: ___________
- **Android 版本**: ___________
- **PC 系统**: ___________
- **Python 版本**: ___________
- **Bleak 版本**: ___________

### 测试结果

| 测试项 | 状态 | 备注 |
|--------|------|------|
| 构建成功 | ☐ | |
| 权限授予 | ☐ | |
| BLE 服务启动 | ☐ | |
| BLE 广播 | ☐ | |
| PC 扫描到设备 | ☐ | |
| 连接成功 | ☐ | |
| 数据传输 | ☐ | |
| CSV 保存 | ☐ | |
| 停止/开始 | ☐ | |
| 断开重连 | ☐ | |

### 发现的问题
1. ___________
2. ___________
3. ___________

### 建议改进
1. ___________
2. ___________
3. ___________

---

## 🚀 下一步行动

### 立即执行
1. [ ] 在真实设备上构建并安装应用
2. [ ] 执行完整测试流程
3. [ ] 记录测试结果
4. [ ] 修复发现的问题

### 后续优化（可选）
1. [ ] 优化连接参数（MTU、间隔等）
2. [ ] 添加数据压缩
3. [ ] 实现数据校验
4. [ ] 添加配置选项
5. [ ] 实现图形化界面

---

**准备开始测试！** 🎯
