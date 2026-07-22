package com.imu.motionrecorder.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothGattServer
import android.bluetooth.BluetoothGattServerCallback
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.AdvertiseCallback
import android.bluetooth.le.AdvertiseData
import android.bluetooth.le.AdvertiseSettings
import android.bluetooth.le.BluetoothLeAdvertiser
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.ParcelUuid
import android.os.SystemClock
import android.util.Log
import androidx.core.app.ActivityCompat
import java.util.Collections
import java.util.UUID
import java.util.concurrent.ConcurrentHashMap

/**
 * BLE 服务，用于通过 GATT 通道向PC发送IMU数据
 */
class BluetoothService(private val context: Context) {

    companion object {
        private const val TAG = "BluetoothService"

        // 自定义 BLE Service 与 Characteristic UUID
        val SERVICE_UUID: UUID = UUID.fromString("0000A0A0-0000-1000-8000-00805F9B34FB")
        val DATA_CHAR_UUID: UUID = UUID.fromString("0000A0A1-0000-1000-8000-00805F9B34FB")
        private val CLIENT_CHARACTERISTIC_CONFIG_UUID: UUID = UUID.fromString("00002902-0000-1000-8000-00805F9B34FB")
    }

    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager.adapter

    private var gattServer: BluetoothGattServer? = null
    private var advertiser: BluetoothLeAdvertiser? = null
    private var advertiseCallback: AdvertiseCallback? = null
    private var dataCharacteristic: BluetoothGattCharacteristic? = null

    private val connectedDevices = Collections.newSetFromMap(ConcurrentHashMap<BluetoothDevice, Boolean>())
    
    // 存储每个设备的 CCCD (Client Characteristic Configuration Descriptor) 状态
    private val cccdMap = ConcurrentHashMap<String, ByteArray>()
    
    // 数据传输统计
    private var totalDataSent = 0L
    private var lastDataSentTime = 0L
    private var consecutiveFailures = 0
    private var lastNotifyTimestamp = 0L

    @Volatile
    private var isRunning = false

    @Volatile
    private var isTransmitting = false

    @Volatile
    private var lastPayload: ByteArray = ByteArray(0)

    var onConnectionStateChanged: ((Boolean) -> Unit)? = null

    fun isBluetoothAvailable(): Boolean = bluetoothAdapter != null

    fun isBluetoothEnabled(): Boolean = bluetoothAdapter?.isEnabled == true

    fun startServer() {
        if (!isBluetoothAvailable() || !isBluetoothEnabled()) {
            Log.e(TAG, "Bluetooth not available or not enabled")
            return
        }

        if (isRunning) {
            Log.d(TAG, "BLE server already running")
            return
        }

        if (!ensurePermissions()) {
            Log.e(TAG, "Missing Bluetooth permissions")
            return
        }

        setupGattServer()
        startAdvertising()
        isRunning = true
        Log.d(TAG, "BLE GATT server started")
    }

    fun stopServer() {
        isRunning = false
        isTransmitting = false
        stopAdvertising()
        shutdownGattServer()
        connectedDevices.clear()
        cccdMap.clear()
        Log.d(TAG, "BLE GATT server stopped")
    }

    /** 当用户点击"开始"按钮 */
    fun startTransmission() {
        isTransmitting = true
        totalDataSent = 0
        lastDataSentTime = System.currentTimeMillis()
        consecutiveFailures = 0
        Log.d(TAG, "BLE data transmission enabled")
    }

    /** 当用户点击"停止"按钮 */
    fun stopTransmission() {
        isTransmitting = false
        Log.d(TAG, "BLE data transmission disabled (total sent: $totalDataSent)")
    }

    fun isConnected(): Boolean = connectedDevices.isNotEmpty()

    /**
     * 发送IMU数据：ax,ay,az,gx,gy,gz\n
     */
    fun sendIMUData(
        accelX: Float, accelY: Float, accelZ: Float,
        gyroX: Float, gyroY: Float, gyroZ: Float
    ) {
        if (!isTransmitting || dataCharacteristic == null || !isConnected()) {
            return
        }

        val now = SystemClock.elapsedRealtime()
        // 限制通知频率，避免中心设备过载导致断开
        // 30Hz ≈ 33ms
        if (now - lastNotifyTimestamp < 33) {
            return
        }

        val payload = String.format(
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            accelX, accelY, accelZ,
            gyroX, gyroY, gyroZ
        ).toByteArray(Charsets.UTF_8)

        lastPayload = payload

        // 兼容性处理：API 33+ 使用新方法，旧版本使用 deprecated 方法
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
            @Suppress("DEPRECATION")
            dataCharacteristic?.value = payload
        }

        var sendSuccess = false
        connectedDevices.forEach { device ->
            try {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    gattServer?.notifyCharacteristicChanged(device, dataCharacteristic!!, false, payload)
                } else {
                    @Suppress("DEPRECATION")
                    gattServer?.notifyCharacteristicChanged(device, dataCharacteristic, false)
                }
                sendSuccess = true
                totalDataSent++
                lastDataSentTime = System.currentTimeMillis()
                lastNotifyTimestamp = now
                consecutiveFailures = 0
                
                // 每1000条数据记录一次
                if (totalDataSent % 1000 == 0L) {
                    Log.d(TAG, "BLE数据已发送: $totalDataSent 条")
                }
            } catch (e: Exception) {
                consecutiveFailures++
                Log.e(TAG, "Failed to notify device ${device.address} (failures: $consecutiveFailures)", e)
                
                // 连续失败10次，尝试重新连接
                if (consecutiveFailures >= 10) {
                    Log.e(TAG, "连续失败 $consecutiveFailures 次，设备可能已断开: ${device.address}")
                    connectedDevices.remove(device)
                    cccdMap.remove(device.address)
                    if (connectedDevices.isEmpty()) {
                        onConnectionStateChanged?.invoke(false)
                    }
                }
            }
        }

        if (!sendSuccess) {
            Log.w(TAG, "No connected device accepted notification at $now")
        }
    }

    private fun ensurePermissions(): Boolean {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val hasConnect = hasPermission(Manifest.permission.BLUETOOTH_CONNECT)
            val hasAdvertise = hasPermission(Manifest.permission.BLUETOOTH_ADVERTISE)
            if (!hasConnect || !hasAdvertise) {
                return false
            }
        }
        return true
    }

    private fun hasPermission(permission: String): Boolean {
        return ActivityCompat.checkSelfPermission(context, permission) == PackageManager.PERMISSION_GRANTED
    }

    private fun setupGattServer() {
        shutdownGattServer()

        val gatt = bluetoothManager.openGattServer(context, gattServerCallback)
        if (gatt == null) {
            Log.e(TAG, "Unable to open GATT server")
            return
        }

        val service = BluetoothGattService(SERVICE_UUID, BluetoothGattService.SERVICE_TYPE_PRIMARY)

        dataCharacteristic = BluetoothGattCharacteristic(
            DATA_CHAR_UUID,
            BluetoothGattCharacteristic.PROPERTY_NOTIFY or BluetoothGattCharacteristic.PROPERTY_READ,
            BluetoothGattCharacteristic.PERMISSION_READ
        )

        val configDescriptor = BluetoothGattDescriptor(
            CLIENT_CHARACTERISTIC_CONFIG_UUID,
            BluetoothGattDescriptor.PERMISSION_READ or BluetoothGattDescriptor.PERMISSION_WRITE
        )
        
        @Suppress("DEPRECATION")
        configDescriptor.value = BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE
        dataCharacteristic?.addDescriptor(configDescriptor)

        service.addCharacteristic(dataCharacteristic)
        gatt.addService(service)

        gattServer = gatt
    }

    private fun shutdownGattServer() {
        try {
            gattServer?.close()
        } catch (e: Exception) {
            Log.e(TAG, "Error closing GATT server", e)
        } finally {
            gattServer = null
            dataCharacteristic = null
        }
    }

    private fun startAdvertising() {
        val adapter = bluetoothAdapter ?: return
        advertiser = adapter.bluetoothLeAdvertiser

        if (advertiser == null) {
            Log.e(TAG, "BLE advertiser not available")
            return
        }

        val settings = AdvertiseSettings.Builder()
            .setAdvertiseMode(AdvertiseSettings.ADVERTISE_MODE_LOW_LATENCY)
            .setTxPowerLevel(AdvertiseSettings.ADVERTISE_TX_POWER_HIGH)
            .setConnectable(true)
            .setTimeout(0)
            .build()

        val data = AdvertiseData.Builder()
            .setIncludeDeviceName(true)
            .addServiceUuid(ParcelUuid(SERVICE_UUID))
            .build()

        advertiseCallback = object : AdvertiseCallback() {
            override fun onStartSuccess(settingsInEffect: AdvertiseSettings?) {
                Log.d(TAG, "Advertising started: $settingsInEffect")
            }

            override fun onStartFailure(errorCode: Int) {
                Log.e(TAG, "Advertising failed: $errorCode")
            }
        }

        advertiser?.startAdvertising(settings, data, advertiseCallback)
    }

    private fun stopAdvertising() {
        try {
            advertiser?.stopAdvertising(advertiseCallback)
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping advertising", e)
        } finally {
            advertiseCallback = null
            advertiser = null
        }
    }

    private val gattServerCallback = object : BluetoothGattServerCallback() {
        override fun onConnectionStateChange(device: BluetoothDevice, status: Int, newState: Int) {
            super.onConnectionStateChange(device, status, newState)

            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    connectedDevices.add(device)
                    Log.d(TAG, "Device connected: ${device.address}")
                    onConnectionStateChanged?.invoke(true)
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    connectedDevices.remove(device)
                    cccdMap.remove(device.address) // 清理断开连接设备的配置
                    Log.d(TAG, "Device disconnected: ${device.address}")
                    if (connectedDevices.isEmpty()) {
                        onConnectionStateChanged?.invoke(false)
                    }
                }
            }
        }

        override fun onCharacteristicReadRequest(
            device: BluetoothDevice,
            requestId: Int,
            offset: Int,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (characteristic.uuid == DATA_CHAR_UUID) {
                val value = if (offset >= lastPayload.size) {
                    ByteArray(0)
                } else {
                    lastPayload.copyOfRange(offset, lastPayload.size)
                }
                gattServer?.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, value)
            } else {
                gattServer?.sendResponse(device, requestId, BluetoothGatt.GATT_FAILURE, offset, null)
            }
        }

        override fun onDescriptorReadRequest(
            device: BluetoothDevice,
            requestId: Int,
            offset: Int,
            descriptor: BluetoothGattDescriptor
        ) {
            if (descriptor.uuid == CLIENT_CHARACTERISTIC_CONFIG_UUID) {
                // 使用 Map 获取状态，而不是 descriptor.value
                val value = cccdMap[device.address] ?: BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE
                gattServer?.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, value)
            } else {
                gattServer?.sendResponse(device, requestId, BluetoothGatt.GATT_FAILURE, offset, null)
            }
        }

        override fun onDescriptorWriteRequest(
            device: BluetoothDevice,
            requestId: Int,
            descriptor: BluetoothGattDescriptor,
            preparedWrite: Boolean,
            responseNeeded: Boolean,
            offset: Int,
            value: ByteArray
        ) {
            if (descriptor.uuid == CLIENT_CHARACTERISTIC_CONFIG_UUID) {
                // 更新 Map
                cccdMap[device.address] = value
                // 不再设置 descriptor.value = value
                
                val enabled = value.contentEquals(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
                Log.d(TAG, "Notifications ${if (enabled) "enabled" else "disabled"} for ${device.address}")
            }

            if (responseNeeded) {
                gattServer?.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, value)
            }
        }

        override fun onNotificationSent(device: BluetoothDevice, status: Int) {
            super.onNotificationSent(device, status)
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.w(TAG, "Notification to ${device.address} failed with status $status")
                consecutiveFailures++
            } else {
                // 发送成功，重置失败计数
                if (consecutiveFailures > 0) {
                    consecutiveFailures = 0
                }
            }
        }
    }
    
    /**
     * 获取数据传输统计信息
     */
    fun getTransmissionStats(): String {
        val timeSinceLastSent = if (lastDataSentTime > 0) {
            (System.currentTimeMillis() - lastDataSentTime) / 1000
        } else {
            0
        }
        return "已发送: $totalDataSent 条, 最后发送: ${timeSinceLastSent}秒前, 连续失败: $consecutiveFailures"
    }
}
