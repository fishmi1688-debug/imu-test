package com.example.mobile

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanRecord
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.util.LinkedHashMap
import java.util.Locale
import java.util.UUID

class InsoleV2Manager(
    context: Context,
    private val statusCallback: (String) -> Unit,
    private val sampleCallback: (Sample) -> Unit
) {
    companion object {
        private const val TAG = "InsoleV2Manager"

        private const val INSOLE_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
        private const val INSOLE_SERVICE_SHORT_UUID = "fff0"
        private const val INSOLE_READ_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
        private const val CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"
        private const val INSOLE_TARGET_MTU = 512

        private const val INSOLE_V2_LEFT_MAC_MARKER = "FF2502051A4B"
        private const val INSOLE_V2_RIGHT_MAC_MARKER = "FF25020518F3"
        private const val UNKNOWN_NAME = "Insole_V2"
    }

    data class DeviceItem(
        val address: String,
        val displayName: String,
        val rssi: Int,
        val connecting: Boolean,
        val connected: Boolean
    )

    data class Sample(
        val address: String,
        val displayName: String,
        val timestamp: Long,
        val footId: Int,
        val values: List<Int>
    )

    private data class DeviceState(
        val address: String,
        var displayName: String,
        var rssi: Int = Int.MIN_VALUE,
        var connecting: Boolean = false,
        var connected: Boolean = false,
        var gatt: BluetoothGatt? = null,
        var seenInCurrentScan: Boolean = false
    )

    private val appContext = context.applicationContext
    private val manager = appContext.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
    private val adapter: BluetoothAdapter?
        get() = manager?.adapter
    private val parser = InsoleV2Parser()
    private val mainHandler = Handler(Looper.getMainLooper())
    private val stateLock = Any()
    private val stateMap = LinkedHashMap<String, DeviceState>()

    @Volatile private var isScanning = false
    private var deviceListCallback: ((List<DeviceItem>) -> Unit)? = null

    private val bleScanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            val scanResult = result ?: return
            handleScanResult(scanResult)
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>?) {
            results.orEmpty().forEach(::handleScanResult)
        }

        override fun onScanFailed(errorCode: Int) {
            isScanning = false
            postStatus("鞋垫扫描失败，错误码: $errorCode")
            emitDeviceList()
        }
    }

    fun hasBluetoothAdapter(): Boolean = adapter != null

    fun isBluetoothEnabled(): Boolean = adapter?.isEnabled == true

    fun isScanning(): Boolean = isScanning

    fun startScanForSelection(onDevicesUpdated: (List<DeviceItem>) -> Unit): Boolean {
        if (!hasBluetoothAdapter()) {
            postStatus("当前设备不支持蓝牙")
            return false
        }
        if (!isBluetoothEnabled()) {
            postStatus("蓝牙未开启")
            return false
        }
        deviceListCallback = onDevicesUpdated
        synchronized(stateLock) {
            stateMap.values.forEach { it.seenInCurrentScan = false }
            emitDeviceListLocked()
        }

        val scanner = adapter?.bluetoothLeScanner
        if (scanner == null) {
            postStatus("鞋垫扫描器不可用")
            return false
        }

        if (isScanning) {
            runCatching { scanner.stopScan(bleScanCallback) }
        }

        return runCatching {
            val settings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            scanner.startScan(null, settings, bleScanCallback)
            isScanning = true
            postStatus("开始扫描鞋垫设备（Insoles_v2）")
            emitDeviceList()
            true
        }.getOrElse {
            isScanning = false
            postStatus("启动鞋垫扫描失败: ${it.message}")
            false
        }
    }

    fun stopScan(clearCallback: Boolean = true) {
        runCatching { adapter?.bluetoothLeScanner?.stopScan(bleScanCallback) }
        isScanning = false
        if (clearCallback) {
            deviceListCallback = null
        }
    }

    fun connect(address: String): Boolean {
        if (!hasBluetoothAdapter()) {
            postStatus("当前设备不支持蓝牙")
            return false
        }
        if (!isBluetoothEnabled()) {
            postStatus("蓝牙未开启")
            return false
        }
        val device = runCatching { adapter?.getRemoteDevice(address) }.getOrNull()
        if (device == null) {
            postStatus("无法获取鞋垫设备: $address")
            return false
        }

        synchronized(stateLock) {
            val existing = stateMap[address]
            if (existing?.connected == true || existing?.connecting == true) {
                emitDeviceListLocked()
                return true
            }
            val displayName = decorateDisplayName(device.name ?: UNKNOWN_NAME, address)
            val state = existing ?: DeviceState(address = address, displayName = displayName)
            state.displayName = displayName
            state.connecting = true
            state.connected = false
            stateMap[address] = state
            emitDeviceListLocked()
        }

        return runCatching {
            val callback = InsoleGattCallback(address)
            val gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                device.connectGatt(appContext, false, callback, BluetoothDevice.TRANSPORT_LE)
            } else {
                @Suppress("DEPRECATION")
                device.connectGatt(appContext, false, callback)
            }
            synchronized(stateLock) {
                stateMap[address]?.gatt = gatt
                emitDeviceListLocked()
            }
            true
        }.getOrElse {
            synchronized(stateLock) {
                val state = stateMap[address] ?: return@getOrElse false
                state.connecting = false
                state.connected = false
                state.gatt = null
                emitDeviceListLocked()
            }
            postStatus("连接鞋垫设备失败: ${it.message}")
            false
        }
    }

    fun disconnect(address: String) {
        val gatt = synchronized(stateLock) {
            val state = stateMap[address] ?: return
            state.connecting = false
            state.connected = false
            val holder = state.gatt
            state.gatt = null
            emitDeviceListLocked()
            holder
        }
        parser.clear(address)
        runCatching {
            gatt?.disconnect()
            gatt?.close()
        }
        postStatus("鞋垫设备已断开: $address")
    }

    fun release() {
        stopScan(clearCallback = true)
        val addresses = synchronized(stateLock) {
            stateMap.keys.toList()
        }
        addresses.forEach { disconnect(it) }
        parser.clearAll()
    }

    private fun handleScanResult(result: ScanResult) {
        val address = result.device.address ?: return
        val rawName = result.scanRecord?.deviceName ?: result.device.name ?: UNKNOWN_NAME
        if (!isInsoleV2(result.scanRecord, rawName)) {
            return
        }
        val displayName = decorateDisplayName(rawName, address)
        synchronized(stateLock) {
            val state = stateMap[address]
            if (state == null) {
                stateMap[address] = DeviceState(
                    address = address,
                    displayName = displayName,
                    rssi = result.rssi,
                    seenInCurrentScan = true
                )
            } else {
                state.displayName = displayName
                state.rssi = result.rssi
                state.seenInCurrentScan = true
            }
            emitDeviceListLocked()
        }
    }

    private fun isInsoleV2(record: ScanRecord?, nameRaw: String): Boolean {
        val trimmedName = nameRaw.trim()
        val isInsoleV2ByName = trimmedName.uppercase(Locale.US).startsWith("NB-")
        if (hasInsoleService(record) && isInsoleV2ByName) {
            return true
        }
        if (isInsoleV2ByName) {
            return true
        }
        return false
    }

    private fun hasInsoleService(record: ScanRecord?): Boolean {
        if (record == null) {
            return false
        }
        val shortUuid = normalizeUuid(INSOLE_SERVICE_SHORT_UUID)
        val fullPrefix = normalizeUuid(INSOLE_SERVICE_UUID).take(8)
        val hasServiceUuid = record.serviceUuids
            ?.any {
                val uuid = normalizeUuid(it.uuid.toString())
                uuid.contains(shortUuid) || uuid.startsWith(fullPrefix)
            }
            ?: false
        if (hasServiceUuid) {
            return true
        }
        val serviceDataKeys = record.serviceData?.keys ?: return false
        return serviceDataKeys.any {
            val uuid = normalizeUuid(it.uuid.toString())
            uuid.contains(shortUuid) || uuid.startsWith(fullPrefix)
        }
    }

    private fun decorateDisplayName(rawName: String, address: String): String {
        val side = inferSide(address)
        if (side == null || rawName.contains("左脚") || rawName.contains("右脚")) {
            return rawName
        }
        return if (side == "Left") {
            "$rawName 左脚"
        } else {
            "$rawName 右脚"
        }
    }

    private fun inferSide(address: String): String? {
        val mac = address.replace(":", "").uppercase(Locale.US)
        return when {
            mac.contains(INSOLE_V2_LEFT_MAC_MARKER) -> "Left"
            mac.contains(INSOLE_V2_RIGHT_MAC_MARKER) -> "Right"
            else -> null
        }
    }

    private fun normalizeUuid(raw: String): String {
        return raw.lowercase(Locale.US).replace("-", "")
    }

    private fun postStatus(message: String) {
        mainHandler.post { statusCallback(message) }
    }

    private fun emitDeviceList() {
        synchronized(stateLock) {
            emitDeviceListLocked()
        }
    }

    private fun emitDeviceListLocked() {
        val list = stateMap.values
            .filter { it.seenInCurrentScan }
            .sortedWith(
                compareByDescending<DeviceState> { it.connected }
                    .thenByDescending { it.connecting }
                    .thenByDescending { it.rssi }
                    .thenBy { it.displayName }
            )
            .map {
                DeviceItem(
                    address = it.address,
                    displayName = it.displayName,
                    rssi = it.rssi,
                    connecting = it.connecting,
                    connected = it.connected
                )
            }
        val callback = deviceListCallback ?: return
        mainHandler.post { callback(list) }
    }

    @Suppress("DEPRECATION")
    private fun enableInsoleNotification(gatt: BluetoothGatt, service: BluetoothGattService): Boolean {
        val characteristic = service.getCharacteristic(UUID.fromString(INSOLE_READ_UUID)) ?: return false
        if (!gatt.setCharacteristicNotification(characteristic, true)) {
            return false
        }
        val descriptor = characteristic.getDescriptor(UUID.fromString(CCCD_UUID)) ?: return false
        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        return gatt.writeDescriptor(descriptor)
    }

    private inner class InsoleGattCallback(
        private val address: String
    ) : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                synchronized(stateLock) {
                    val state = stateMap[address] ?: return
                    state.connecting = true
                    state.connected = false
                    state.gatt = gatt
                    emitDeviceListLocked()
                }
                postStatus("鞋垫已连接，发现服务中: $address")
                gatt.discoverServices()
                return
            }

            if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                synchronized(stateLock) {
                    val state = stateMap[address] ?: return
                    state.connecting = false
                    state.connected = false
                    state.gatt = null
                    emitDeviceListLocked()
                }
                parser.clear(address)
                postStatus("鞋垫设备已断开: $address")
                runCatching { gatt.close() }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                postStatus("鞋垫服务发现失败: $status")
                disconnect(address)
                return
            }
            val service = gatt.getService(UUID.fromString(INSOLE_SERVICE_UUID))
            if (service == null) {
                postStatus("鞋垫未找到服务: $address")
                disconnect(address)
                return
            }

            runCatching { gatt.requestMtu(INSOLE_TARGET_MTU) }
                .onFailure { Log.w(TAG, "requestMtu failed: ${it.message}") }

            val enabled = enableInsoleNotification(gatt, service)
            if (!enabled) {
                postStatus("鞋垫通知订阅失败: $address")
                disconnect(address)
                return
            }

            synchronized(stateLock) {
                val state = stateMap[address] ?: return
                state.connecting = false
                state.connected = true
                state.gatt = gatt
                emitDeviceListLocked()
            }
            postStatus("鞋垫已连接并开始接收数据: $address")
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            if (normalizeUuid(characteristic.uuid.toString()) != normalizeUuid(INSOLE_READ_UUID)) {
                return
            }
            handlePacket(address, value)
        }

        @Deprecated("Deprecated in API 33")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (normalizeUuid(characteristic.uuid.toString()) != normalizeUuid(INSOLE_READ_UUID)) {
                return
            }
            val packet = characteristic.value ?: return
            handlePacket(address, packet)
        }
    }

    private fun handlePacket(address: String, packet: ByteArray) {
        val parsed = parser.parse(address, packet) ?: return
        val name = synchronized(stateLock) {
            stateMap[address]?.displayName ?: UNKNOWN_NAME
        }
        sampleCallback(
            Sample(
                address = address,
                displayName = name,
                timestamp = parsed.timestamp,
                footId = parsed.count,
                values = parsed.values
            )
        )
    }
}
