package com.example.mobile

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanRecord
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import android.os.SystemClock
import android.util.Log
import androidx.core.content.ContextCompat
import com.google.gson.JsonArray
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.charset.CodingErrorAction
import java.util.ArrayDeque
import java.util.LinkedHashMap
import java.util.Locale
import java.util.UUID
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.ThreadPoolExecutor
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

class GaitBluetoothConnector(
    context: Context,
    private val statusCallback: (String) -> Unit
) {
    companion object {
        private const val TAG = "GaitBluetooth"
        val GAIT_SERVICE_UUID: UUID =
            UUID.fromString("f9c2d0b4-9c48-4d4a-925b-0c42f3a9b002")
        private val GAIT_NOTIFY_CHAR_UUID: UUID =
            UUID.fromString("f9c2d0b4-9c48-4d4a-925b-0c42f3a9b003")
        private val GAIT_CONTROL_CHAR_UUID: UUID =
            UUID.fromString("f9c2d0b4-9c48-4d4a-925b-0c42f3a9b004")
        private val CCCD_UUID: UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        private const val AUTO_RECONNECT_BASE_DELAY_MS = 1200L
        private const val AUTO_RECONNECT_MAX_DELAY_MS = 8000L
        private const val AUTO_RECONNECT_MAX_ATTEMPTS = 10
        private const val HANDSHAKE_TIMEOUT_MS = 4000L
        private const val HEARTBEAT_INTERVAL_MS = 3000L
        private const val LINK_IDLE_TIMEOUT_MS = 10000L
        private const val SERVICE_DISCOVERY_TIMEOUT_MS = 5000L
        private const val SERVICE_DISCOVERY_RETRY_DELAY_MS = 150L
        private const val SERVICE_DISCOVERY_MAX_ATTEMPTS = 5
        private const val MAX_PENDING_LINE_CALLBACKS = 256
        private const val MAX_BINARY_FRAME_PAYLOAD_BYTES = 8192
        private const val MAX_UNFRAMED_BUFFER_BYTES = 4096
        private const val BINARY_FRAME_HEADER_SIZE = 8
        private const val GAIT_TARGET_MTU = 247
        private const val UNKNOWN_DEVICE_NAME = "未命名设备"
        private const val CONTROL_KIND_REQUEST = 0x10
        private const val CONTROL_KIND_RESPONSE = 0x11
        private const val CONTROL_VERSION = 1
        private const val CONTROL_TYPE_INT32 = 1
        private const val CONTROL_TYPE_FLOAT32 = 2
        private const val CONTROL_TYPE_BOOL = 3
        private const val CONTROL_TYPE_BYTES = 4
        private const val CONTROL_FIELD_MODE = 1
        private const val CONTROL_FIELD_COMMAND = 2
        private const val CONTROL_FIELD_VALUE = 3
        private const val CONTROL_FIELD_PLOT_FORMAT = 4
        private const val CONTROL_FIELD_PLOT_MODE = 5
        private const val CONTROL_FIELD_PLOT_BATCH_SIZE = 6
        private const val CONTROL_FIELD_PLOT_EVERY_N = 7
        private const val CONTROL_FIELD_IMU_SLOT = 8
        private const val CONTROL_FIELD_IMU_CONNECT = 9
        private const val CONTROL_FIELD_IMU_MAC_BYTES = 10
        private const val CONTROL_FIELD_PARAM_UPDATES = 11
        private const val CONTROL_FIELD_ALLOW_OFFMODE = 12
        private const val CONTROL_FIELD_RESULT = 20
        private const val CONTROL_FIELD_ACTION = 21
        private const val CONTROL_FIELD_MODE_APPLIED = 22
        private const val CONTROL_FIELD_ENABLED = 23
        private const val CONTROL_FIELD_REASON = 24
        private const val CONTROL_FIELD_PLOT_FORMAT_APPLIED = 25
        private const val CONTROL_FIELD_PLOT_MODE_APPLIED = 26
        private const val CONTROL_FIELD_PLOT_BATCH_APPLIED = 27
        private const val CONTROL_FIELD_PLOT_EVERY_N_APPLIED = 28
        private const val CONTROL_FIELD_PARAM_COUNT = 29
        private const val CONTROL_FIELD_MODE_OVERRIDDEN = 30
        private const val CONTROL_FIELD_ERROR_CODE = 31
        private const val CONTROL_FIELD_IMU_CONNECTED = 32
        private const val CONTROL_FIELD_IMU_MEASURING = 33
        private const val CONTROL_FIELD_IMU_READY = 34
        private const val CONTROL_FIELD_IMU_STALE = 35
        private const val CONTROL_FIELD_STATE_VERSION = 40
        private const val CONTROL_FIELD_STATE_MODE = 41
        private const val CONTROL_FIELD_STATE_GAIT = 42
        private const val CONTROL_FIELD_STATE_FLAGS = 43
        private const val CONTROL_FIELD_STATE_SCORE = 44
        private const val CONTROL_FIELD_STATE_PARAMS = 45
        private val ALLOWED_GAIT_DEVICE_NAMES = setOf(
            "ubuntu",
            "zhang-Dell-G15-5520",
            "gaitcontrol",
        )
        private val BINARY_FRAME_MAGIC = byteArrayOf(0x47, 0x42, 0x46, 0x31) // "GBF1"
    }

    data class ScannedDevice(
        val address: String,
        val displayName: String,
        val bonded: Boolean
    )

    private val appContext = context.applicationContext
    private val manager = appContext.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
    private val adapter: BluetoothAdapter?
        get() = manager?.adapter
    private val mainHandler = Handler(Looper.getMainLooper())

    private val scanResultMap: LinkedHashMap<String, BluetoothDevice> = LinkedHashMap()
    private var scanListCallback: ((List<ScannedDevice>) -> Unit)? = null
    @Volatile private var isScanning = false
    @Volatile private var currentTargetAddress: String? = null

    @Volatile private var isConnecting = false
    @Volatile private var isAwaitingHandshake = false
    @Volatile private var linkReady = false
    @Volatile private var manualDisconnect = false
    @Volatile private var connectAttemptAutoReconnect = false

    private var gatt: BluetoothGatt? = null
    private var notifyCharacteristic: BluetoothGattCharacteristic? = null
    private var controlCharacteristic: BluetoothGattCharacteristic? = null
    private var readBuffer = ByteArray(0)
    private var pendingConnectCallback: ((Boolean) -> Unit)? = null

    private var handshakeTimeoutRunnable: Runnable? = null
    private var heartbeatRunnable: Runnable? = null
    private var serviceDiscoveryTimeoutRunnable: Runnable? = null
    private var serviceDiscoveryRunnable: Runnable? = null
    private var lineCallback: ((String) -> Unit)? = null
    private var binaryFrameCallback: ((ByteArray) -> Unit)? = null
    private val lineDispatchExecutor = ThreadPoolExecutor(
        1,
        1,
        30L,
        TimeUnit.SECONDS,
        ArrayBlockingQueue(MAX_PENDING_LINE_CALLBACKS),
        { runnable -> Thread(runnable, "gait-line-dispatch").apply { isDaemon = true } },
        ThreadPoolExecutor.DiscardOldestPolicy()
    )
    private val pendingLineCallbacks = AtomicInteger(0)
    private val droppedRealtimeLines = AtomicInteger(0)
    private var lastConnectedAddress: String? = null
    private var reconnectAttempt = 0
    private var reconnectRunnable: Runnable? = null
    private var serviceDiscoveryAttempt = 0
    @Volatile private var serviceDiscoveryInProgress = false
    @Volatile private var mtuRequestIssued = false
    @Volatile private var lastInboundElapsedMs = 0L

    private val controlWriteQueue = ArrayDeque<ByteArray>()
    private val controlWriteLock = Any()
    @Volatile private var writeInFlight = false

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
            postStatus("步态BLE扫描失败，错误码: $errorCode")
            emitScanList()
        }
    }

    fun hasBluetoothAdapter(): Boolean = adapter != null

    fun isBluetoothEnabled(): Boolean = adapter?.isEnabled == true

    fun isScanning(): Boolean = isScanning

    fun isConnecting(): Boolean = isConnecting || isAwaitingHandshake

    fun currentDeviceAddress(): String? = currentTargetAddress ?: lastConnectedAddress

    fun isConnected(): Boolean {
        val activeGatt = gatt ?: return false
        if (!linkReady) {
            return false
        }
        if (!isGattConnectedInSystem(activeGatt)) {
            handleSystemDisconnect(activeGatt)
            return false
        }
        return true
    }

    fun startScanForSelection(
        onScanDevicesUpdated: (List<ScannedDevice>) -> Unit
    ): Boolean {
        if (!hasRequiredPermissions()) {
            postStatus("权限不足，无法开始蓝牙扫描")
            return false
        }
        if (!isBluetoothEnabled()) {
            postStatus("蓝牙未开启")
            return false
        }
        stopScan()
        scanResultMap.clear()
        scanListCallback = onScanDevicesUpdated
        emitScanList()

        val scanner = adapter?.bluetoothLeScanner
        if (scanner == null) {
            postStatus("步态BLE扫描器不可用")
            return false
        }

        return runCatching {
            val settings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            scanner.startScan(null, settings, bleScanCallback)
            isScanning = true
            postStatus("开始扫描步态BLE设备")
            true
        }.getOrElse {
            isScanning = false
            postStatus("启动步态BLE扫描失败: ${it.message}")
            false
        }
    }

    fun stopScan() {
        runCatching { adapter?.bluetoothLeScanner?.stopScan(bleScanCallback) }
        isScanning = false
        scanListCallback = null
    }

    fun connectDeviceByAddress(address: String, onResult: (Boolean) -> Unit): Boolean {
        val device = resolveDeviceByAddress(address) ?: run {
            postStatus("未找到设备: $address")
            return false
        }
        currentTargetAddress = address
        manualDisconnect = false
        cancelAutoReconnect()
        return connectDeviceInternal(device = device, onResult = onResult, autoReconnect = false)
    }

    fun disconnect() {
        manualDisconnect = true
        currentTargetAddress = null
        cancelAutoReconnect()
        stopScan()
        closeGatt()
    }

    fun setLineCallback(callback: ((String) -> Unit)?) {
        lineCallback = callback
    }

    fun setBinaryFrameCallback(callback: ((ByteArray) -> Unit)?) {
        binaryFrameCallback = callback
    }

    fun sendLine(message: String): Boolean {
        val activeGatt = gatt
        val characteristic = controlCharacteristic
        if (activeGatt == null || characteristic == null || !linkReady) {
            postStatus("未连接设备，无法发送")
            return false
        }
        if (!isGattConnectedInSystem(activeGatt)) {
            handleSystemDisconnect(activeGatt)
            postStatus("未连接设备，无法发送")
            return false
        }
        val trimmed = message.trimEnd('\n', '\r')
        val binaryControl = encodeControlRequestFrameFromJson(trimmed)
        if (binaryControl == null) {
            postStatus("控制消息编码失败: 当前仅支持二进制控制帧")
            return false
        }
        val started = enqueueControlWrite(activeGatt, characteristic, binaryControl)
        if (!started) {
            failActiveConnection(
                activeGatt = activeGatt,
                userMessage = "步态BLE控制写入启动失败",
                reconnectReason = "检测到步态BLE控制写入启动失败",
            )
        }
        return started
    }

    @SuppressLint("MissingPermission")
    private fun connectDeviceInternal(
        device: BluetoothDevice,
        onResult: ((Boolean) -> Unit)? = null,
        autoReconnect: Boolean
    ): Boolean {
        if (isConnecting) {
            postStatus("蓝牙连接进行中，请稍后")
            onResult?.let { callback ->
                mainHandler.post { callback(false) }
            }
            return true
        }

        stopScan()
        cancelHandshakeTimeout()
        closeGatt()

        return runCatching {
            isConnecting = true
            isAwaitingHandshake = false
            linkReady = false
            manualDisconnect = false
            connectAttemptAutoReconnect = autoReconnect
            pendingConnectCallback = onResult
            readBuffer = ByteArray(0)
            if (!autoReconnect) {
                reconnectAttempt = 0
            }
            serviceDiscoveryAttempt = 0
            serviceDiscoveryInProgress = false
            mtuRequestIssued = false
            lastInboundElapsedMs = SystemClock.elapsedRealtime()
            currentTargetAddress = device.address
            lastConnectedAddress = device.address
            postStatus("开始连接步态BLE设备: ${device.name ?: device.address}")
            val callback = GaitGattCallback(device)
            gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                device.connectGatt(appContext, false, callback, BluetoothDevice.TRANSPORT_LE)
            } else {
                @Suppress("DEPRECATION")
                device.connectGatt(appContext, false, callback)
            }
            true
        }.getOrElse {
            isConnecting = false
            connectAttemptAutoReconnect = false
            pendingConnectCallback = null
            postStatus("连接失败: ${it.message ?: it::class.java.simpleName}")
            onResult?.let { callback ->
                mainHandler.post { callback(false) }
            }
            false
        }
    }

    private fun resolveDeviceByAddress(address: String): BluetoothDevice? {
        return scanResultMap[address]
            ?: runCatching { adapter?.getRemoteDevice(address) }.getOrNull()
    }

    private fun cancelAutoReconnect() {
        reconnectRunnable?.let { mainHandler.removeCallbacks(it) }
        reconnectRunnable = null
        reconnectAttempt = 0
    }

    private fun cancelHandshakeTimeout() {
        handshakeTimeoutRunnable?.let { mainHandler.removeCallbacks(it) }
        handshakeTimeoutRunnable = null
    }

    private fun cancelHeartbeat() {
        heartbeatRunnable?.let { mainHandler.removeCallbacks(it) }
        heartbeatRunnable = null
    }

    private fun cancelServiceDiscoveryTimeout() {
        serviceDiscoveryTimeoutRunnable?.let { mainHandler.removeCallbacks(it) }
        serviceDiscoveryTimeoutRunnable = null
    }

    private fun cancelServiceDiscovery() {
        serviceDiscoveryRunnable?.let { mainHandler.removeCallbacks(it) }
        serviceDiscoveryRunnable = null
        serviceDiscoveryAttempt = 0
        serviceDiscoveryInProgress = false
        cancelServiceDiscoveryTimeout()
    }

    private fun scheduleServiceDiscovery(activeGatt: BluetoothGatt) {
        if (serviceDiscoveryRunnable != null || this.gatt !== activeGatt || linkReady) {
            return
        }
        cancelServiceDiscoveryTimeout()
        val timeoutRunnable = Runnable {
            if (this.gatt !== activeGatt || linkReady || !serviceDiscoveryInProgress) {
                return@Runnable
            }
            failActiveConnection(
                activeGatt = activeGatt,
                userMessage = "步态BLE服务发现超时",
                reconnectReason = "步态BLE服务发现超时",
            )
        }
        serviceDiscoveryTimeoutRunnable = timeoutRunnable
        mainHandler.postDelayed(timeoutRunnable, SERVICE_DISCOVERY_TIMEOUT_MS)

        val runnable = Runnable {
            serviceDiscoveryRunnable = null
            if (this.gatt !== activeGatt || linkReady || !serviceDiscoveryInProgress) {
                return@Runnable
            }
            val started = runCatching { activeGatt.discoverServices() }.getOrDefault(false)
            if (started) {
                serviceDiscoveryAttempt = 0
                return@Runnable
            }
            serviceDiscoveryAttempt += 1
            Log.w(TAG, "discoverServices() failed to start, attempt=$serviceDiscoveryAttempt")
            if (serviceDiscoveryAttempt >= SERVICE_DISCOVERY_MAX_ATTEMPTS) {
                failActiveConnection(
                    activeGatt = activeGatt,
                    userMessage = "步态BLE服务发现启动失败",
                    reconnectReason = "步态BLE服务发现启动失败",
                )
                return@Runnable
            }
            scheduleServiceDiscovery(activeGatt)
        }
        serviceDiscoveryRunnable = runnable
        mainHandler.postDelayed(runnable, SERVICE_DISCOVERY_RETRY_DELAY_MS)
    }

    @SuppressLint("MissingPermission")
    private fun requestPreferredMtu(activeGatt: BluetoothGatt? = gatt) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.LOLLIPOP) {
            return
        }
        val targetGatt = activeGatt ?: return
        if (mtuRequestIssued || this.gatt !== targetGatt) {
            return
        }
        mtuRequestIssued = true
        val started = runCatching {
            targetGatt.requestMtu(GAIT_TARGET_MTU)
        }.onFailure {
            mtuRequestIssued = false
            Log.w(TAG, "requestMtu failed", it)
        }.getOrDefault(false)
        if (!started) {
            mtuRequestIssued = false
            Log.w(TAG, "requestMtu() did not start")
        }
    }

    private fun scheduleHandshakeTimeout(activeGatt: BluetoothGatt, device: BluetoothDevice) {
        cancelHandshakeTimeout()
        val runnable = Runnable {
            if (gatt !== activeGatt || !isAwaitingHandshake || linkReady) {
                return@Runnable
            }
            failActiveConnection(
                activeGatt = activeGatt,
                userMessage = "连接超时: 设备未返回响应 (${device.name ?: device.address})",
                reconnectReason = "检测到步态BLE握手失败",
            )
        }
        handshakeTimeoutRunnable = runnable
        mainHandler.postDelayed(runnable, HANDSHAKE_TIMEOUT_MS)
    }

    @SuppressLint("MissingPermission")
    private fun requestHighConnectionPriority(activeGatt: BluetoothGatt) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.LOLLIPOP || this.gatt !== activeGatt) {
            return
        }
        val started = runCatching {
            activeGatt.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
        }.onFailure {
            Log.w(TAG, "requestConnectionPriority failed", it)
        }.getOrDefault(false)
        if (!started) {
            Log.w(TAG, "requestConnectionPriority(CONNECTION_PRIORITY_HIGH) did not start")
        }
    }

    private fun buildPingFrame(): ByteArray {
        return buildControlFrame(
            kind = CONTROL_KIND_REQUEST,
            msgType = GaitProtocol.TYPE_PING,
            fields = emptyList(),
        )
    }

    private fun scheduleHeartbeat(activeGatt: BluetoothGatt, device: BluetoothDevice) {
        cancelHeartbeat()
        val runnable = object : Runnable {
            override fun run() {
                if (gatt !== activeGatt || !linkReady || manualDisconnect) {
                    heartbeatRunnable = null
                    return
                }
                val idleMs = SystemClock.elapsedRealtime() - lastInboundElapsedMs
                if (idleMs >= LINK_IDLE_TIMEOUT_MS) {
                    failActiveConnection(
                        activeGatt = activeGatt,
                        userMessage = "蓝牙心跳超时: ${idleMs}ms 未收到响应 (${device.name ?: device.address})",
                        reconnectReason = "检测到步态BLE心跳超时",
                    )
                    return
                }
                val characteristic = controlCharacteristic
                if (characteristic == null || !enqueueControlWrite(activeGatt, characteristic, buildPingFrame())) {
                    failActiveConnection(
                        activeGatt = activeGatt,
                        userMessage = "蓝牙心跳发送失败: ${device.name ?: device.address}",
                        reconnectReason = "检测到步态BLE心跳发送失败",
                    )
                    return
                }
                mainHandler.postDelayed(this, HEARTBEAT_INTERVAL_MS)
            }
        }
        heartbeatRunnable = runnable
        mainHandler.postDelayed(runnable, HEARTBEAT_INTERVAL_MS)
    }

    private fun failActiveConnection(
        activeGatt: BluetoothGatt?,
        userMessage: String,
        reconnectReason: String,
    ) {
        if (activeGatt != null && gatt !== activeGatt) {
            return
        }
        val wasConnecting = isConnecting || isAwaitingHandshake
        val wasAutoConnectAttempt = connectAttemptAutoReconnect
        postStatus(userMessage)
        closeGatt()
        if (wasConnecting) {
            completeConnectAttempt(false)
        }
        if (userMessage != "蓝牙连接已断开") {
            postStatus("蓝牙连接已断开")
        }
        if (!manualDisconnect && !wasAutoConnectAttempt) {
            scheduleAutoReconnect(reconnectReason)
        }
    }

    private fun scheduleAutoReconnect(reason: String) {
        if (manualDisconnect || isConnecting || isConnected()) {
            return
        }
        val address = lastConnectedAddress ?: return
        if (reconnectAttempt >= AUTO_RECONNECT_MAX_ATTEMPTS) {
            postStatus("自动重连停止：已达到最大重试次数")
            return
        }
        reconnectAttempt += 1
        reconnectRunnable?.let { mainHandler.removeCallbacks(it) }
        val delayMs = (AUTO_RECONNECT_BASE_DELAY_MS * reconnectAttempt)
            .coerceAtMost(AUTO_RECONNECT_MAX_DELAY_MS)
        postStatus("$reason，第${reconnectAttempt}次重连将在 ${delayMs}ms 后执行")
        val runnable = Runnable {
            if (manualDisconnect || isConnecting || isConnected()) {
                return@Runnable
            }
            val device = resolveDeviceByAddress(address)
            if (device == null) {
                postStatus("自动重连失败：设备不可用 ($address)")
                scheduleAutoReconnect("自动重连继续重试")
                return@Runnable
            }
            connectDeviceInternal(device = device, onResult = null, autoReconnect = true)
        }
        reconnectRunnable = runnable
        mainHandler.postDelayed(runnable, delayMs)
    }

    private fun handleScanResult(result: ScanResult) {
        val device = result.device
        if (!matchesGaitPeripheral(result.scanRecord, device.name)) {
            return
        }
        scanResultMap[device.address] = device
        emitScanList()
    }

    private fun matchesGaitPeripheral(record: ScanRecord?, deviceName: String?): Boolean {
        if (hasServiceUuid(record)) {
            return true
        }
        val normalizedName = deviceName.orEmpty().trim().lowercase(Locale.US)
        return normalizedName in ALLOWED_GAIT_DEVICE_NAMES
    }

    private fun hasServiceUuid(record: ScanRecord?): Boolean {
        if (record == null) {
            return false
        }
        val target = ParcelUuid(GAIT_SERVICE_UUID)
        if (record.serviceUuids?.any { it == target } == true) {
            return true
        }
        return record.serviceData.keys.any { it == target }
    }

    private fun emitScanList() {
        val callback = scanListCallback ?: return
        val devices = scanResultMap.values
            .sortedBy { it.name ?: UNKNOWN_DEVICE_NAME }
            .map { device ->
                ScannedDevice(
                    address = device.address,
                    displayName = device.name ?: UNKNOWN_DEVICE_NAME,
                    bonded = false,
                )
        }
        mainHandler.post { callback(devices) }
    }

    @SuppressLint("MissingPermission")
    private fun isGattConnectedInSystem(activeGatt: BluetoothGatt): Boolean {
        val device = activeGatt.device ?: return false
        if (!hasRequiredPermissions()) {
            return true
        }
        val bluetoothManager = manager ?: return true
        return runCatching {
            bluetoothManager.getConnectionState(device, BluetoothProfile.GATT) ==
                BluetoothProfile.STATE_CONNECTED
        }.getOrDefault(true)
    }

    private fun handleSystemDisconnect(activeGatt: BluetoothGatt) {
        if (gatt !== activeGatt) {
            return
        }
        closeGatt()
        postStatus("蓝牙连接已断开")
    }

    @Suppress("DEPRECATION")
    private fun enableNotification(
        gatt: BluetoothGatt,
        service: BluetoothGattService
    ): Boolean {
        val characteristic = service.getCharacteristic(GAIT_NOTIFY_CHAR_UUID) ?: return false
        if (!gatt.setCharacteristicNotification(characteristic, true)) {
            return false
        }
        val descriptor = characteristic.getDescriptor(CCCD_UUID) ?: return false
        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        return gatt.writeDescriptor(descriptor)
    }

    private fun enqueueControlWrite(
        activeGatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic,
        payload: ByteArray
    ): Boolean {
        synchronized(controlWriteLock) {
            controlWriteQueue.addLast(payload)
            if (writeInFlight) {
                return true
            }
            return startNextControlWriteLocked(activeGatt, characteristic)
        }
    }

    @SuppressLint("MissingPermission")
    private fun startNextControlWriteLocked(
        activeGatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic
    ): Boolean {
        while (controlWriteQueue.isNotEmpty()) {
            val payload = controlWriteQueue.removeFirst()
            val started = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                activeGatt.writeCharacteristic(
                    characteristic,
                    payload,
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT,
                ) == BluetoothGatt.GATT_SUCCESS
            } else {
                @Suppress("DEPRECATION")
                run {
                    characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                    characteristic.value = payload
                    activeGatt.writeCharacteristic(characteristic)
                }
            }
            if (started) {
                writeInFlight = true
                return true
            }
            Log.w(TAG, "Failed to start BLE control write, drop payload size=${payload.size}")
        }
        writeInFlight = false
        return false
    }

    private fun clearControlWriteState() {
        synchronized(controlWriteLock) {
            writeInFlight = false
            controlWriteQueue.clear()
        }
    }

    @SuppressLint("MissingPermission")
    private fun closeGatt() {
        cancelHandshakeTimeout()
        cancelHeartbeat()
        cancelServiceDiscovery()
        isAwaitingHandshake = false
        linkReady = false
        readBuffer = ByteArray(0)
        pendingLineCallbacks.set(0)
        droppedRealtimeLines.set(0)
        notifyCharacteristic = null
        controlCharacteristic = null
        clearControlWriteState()
        mtuRequestIssued = false
        val activeGatt = gatt
        gatt = null
        activeGatt?.let {
            runCatching { it.disconnect() }
            runCatching { it.close() }
        }
    }

    private fun completeConnectAttempt(success: Boolean) {
        val callback = pendingConnectCallback
        val shouldRetry = connectAttemptAutoReconnect && !success && !manualDisconnect
        pendingConnectCallback = null
        connectAttemptAutoReconnect = false
        isConnecting = false
        callback?.let {
            mainHandler.post { it(success) }
        }
        if (shouldRetry) {
            scheduleAutoReconnect("自动重连将在稍后重试")
        }
    }

    @Synchronized
    private fun processIncomingBytes(
        chunk: ByteArray,
        device: BluetoothDevice
    ) {
        if (chunk.isEmpty()) {
            return
        }
        lastInboundElapsedMs = SystemClock.elapsedRealtime()
        readBuffer += chunk
        while (true) {
            if (readBuffer.isEmpty()) {
                return
            }
            val header = parseBinaryFrameHeader(readBuffer)
            if (header != null) {
                if (readBuffer.size < header.totalLength) {
                    return
                }
                val frame = readBuffer.copyOfRange(0, header.totalLength)
                readBuffer = readBuffer.copyOfRange(header.totalLength, readBuffer.size)
                markLinkReady(device)
                val controlResponseLine = decodeControlResponseFrameToJson(frame)
                if (controlResponseLine != null) {
                    dispatchLine(controlResponseLine)
                } else {
                    dispatchBinaryFrame(frame)
                }
                continue
            }
            if (matchesBinaryMagic(readBuffer) && readBuffer.size >= BINARY_FRAME_HEADER_SIZE) {
                Log.w(TAG, "Invalid BLE binary frame header, drop one byte for resync")
                readBuffer = readBuffer.copyOfRange(1, readBuffer.size)
                continue
            }
            if (looksLikeBinaryPrefix(readBuffer) && readBuffer.size < BINARY_FRAME_HEADER_SIZE) {
                return
            }
            val magicIndex = indexOfBinaryMagic(readBuffer, startIndex = 1)
            if (magicIndex > 0) {
                Log.w(TAG, "Drop ${magicIndex} unframed BLE bytes before binary frame magic")
                readBuffer = readBuffer.copyOfRange(magicIndex, readBuffer.size)
                continue
            }
            val newlineIndex = indexOfByte(readBuffer, '\n'.code.toByte())
            if (newlineIndex < 0) {
                if (readBuffer.size > MAX_UNFRAMED_BUFFER_BYTES && !looksLikeTextTransportPrefix(readBuffer)) {
                    Log.w(TAG, "Drop oversized unframed BLE buffer: size=${readBuffer.size}")
                    readBuffer = trailingBinaryMagicPrefix(readBuffer)
                }
                return
            }
            val lineBytes = readBuffer.copyOfRange(0, newlineIndex)
            readBuffer = readBuffer.copyOfRange(newlineIndex + 1, readBuffer.size)
            val line = decodeUtf8Strict(lineBytes)?.trim()
            if (line == null) {
                Log.w(TAG, "Drop non-UTF8 BLE text fragment: size=${lineBytes.size}")
                continue
            }
            if (line.isEmpty()) {
                continue
            }
            val decoded = decodeTransportLine(line)
            if (decoded == null) {
                Log.w(TAG, "Invalid transport frame, ignore: $line")
                continue
            }
            markLinkReady(device)
            when (decoded) {
                is IncomingPayload.BinaryFrame -> dispatchBinaryFrame(decoded.bytes)
                is IncomingPayload.TextLine -> {
                    if (!isStructuredJsonLine(decoded.text) && isHumanReadablePlainStatus(decoded.text)) {
                        postStatus("收到设备消息: ${decoded.text}")
                    }
                    if (isStructuredJsonLine(decoded.text) || isHumanReadablePlainStatus(decoded.text)) {
                        dispatchLine(decoded.text)
                    } else {
                        Log.w(TAG, "Drop suspicious BLE text fragment after resync: size=${decoded.text.length}")
                    }
                }
            }
        }
    }

    private fun markLinkReady(device: BluetoothDevice) {
        if (linkReady) {
            return
        }
        linkReady = true
        isAwaitingHandshake = false
        reconnectAttempt = 0
        cancelHandshakeTimeout()
        postStatus("已连接: ${device.name ?: device.address}")
        completeConnectAttempt(true)
        val activeGatt = gatt
        if (activeGatt != null) {
            scheduleHeartbeat(activeGatt, device)
        }
        mainHandler.post { requestPreferredMtu(activeGatt) }
    }

    private fun dispatchLine(line: String) {
        val cb = lineCallback ?: return
        val isSoftRealtime = isSoftRealtimePayloadLine(line)
        if (isSoftRealtime && pendingLineCallbacks.get() >= MAX_PENDING_LINE_CALLBACKS) {
            val dropped = droppedRealtimeLines.incrementAndGet()
            if (dropped == 1 || dropped % 100 == 0) {
                Log.w(
                    TAG,
                    "Drop realtime lines due callback backlog: dropped=$dropped, pending=${pendingLineCallbacks.get()}"
                )
            }
            return
        }
        pendingLineCallbacks.incrementAndGet()
        try {
            lineDispatchExecutor.execute {
                try {
                    cb(line)
                } catch (t: Throwable) {
                    Log.w(TAG, "line callback failed", t)
                } finally {
                    pendingLineCallbacks.decrementAndGet()
                }
            }
        } catch (e: RejectedExecutionException) {
            pendingLineCallbacks.decrementAndGet()
            Log.w(TAG, "line callback rejected", e)
        }
    }

    private fun dispatchBinaryFrame(frame: ByteArray) {
        val cb = binaryFrameCallback ?: return
        if (pendingLineCallbacks.get() >= MAX_PENDING_LINE_CALLBACKS) {
            val dropped = droppedRealtimeLines.incrementAndGet()
            if (dropped == 1 || dropped % 100 == 0) {
                Log.w(
                    TAG,
                    "Drop realtime binary frames due callback backlog: dropped=$dropped, pending=${pendingLineCallbacks.get()}"
                )
            }
            return
        }
        pendingLineCallbacks.incrementAndGet()
        try {
            lineDispatchExecutor.execute {
                try {
                    cb(frame)
                } catch (t: Throwable) {
                    Log.w(TAG, "binary callback failed", t)
                } finally {
                    pendingLineCallbacks.decrementAndGet()
                }
            }
        } catch (e: RejectedExecutionException) {
            pendingLineCallbacks.decrementAndGet()
            Log.w(TAG, "binary callback rejected", e)
        }
    }

    private data class BinaryFrameHeader(
        val kind: Int,
        val count: Int,
        val payloadLength: Int,
        val totalLength: Int,
    )

    private fun parseBinaryFrameHeader(bytes: ByteArray): BinaryFrameHeader? {
        if (bytes.size < BINARY_FRAME_HEADER_SIZE) {
            return null
        }
        if (!matchesBinaryMagic(bytes)) {
            return null
        }
        val kind = bytes[4].toInt() and 0xFF
        val count = bytes[5].toInt() and 0xFF
        val payloadLength = (bytes[6].toInt() and 0xFF) or ((bytes[7].toInt() and 0xFF) shl 8)
        if (payloadLength < 0 || payloadLength > MAX_BINARY_FRAME_PAYLOAD_BYTES) {
            return null
        }
        val totalLength = BINARY_FRAME_HEADER_SIZE + payloadLength
        return BinaryFrameHeader(
            kind = kind,
            count = count,
            payloadLength = payloadLength,
            totalLength = totalLength,
        )
    }

    private fun matchesBinaryMagic(bytes: ByteArray): Boolean {
        if (bytes.size < BINARY_FRAME_MAGIC.size) {
            return false
        }
        for (i in BINARY_FRAME_MAGIC.indices) {
            if (bytes[i] != BINARY_FRAME_MAGIC[i]) {
                return false
            }
        }
        return true
    }

    private fun looksLikeBinaryPrefix(bytes: ByteArray): Boolean {
        val size = minOf(bytes.size, BINARY_FRAME_MAGIC.size)
        if (size <= 0) {
            return false
        }
        for (i in 0 until size) {
            if (bytes[i] != BINARY_FRAME_MAGIC[i]) {
                return false
            }
        }
        return true
    }

    private fun indexOfByte(bytes: ByteArray, target: Byte): Int {
        for (i in bytes.indices) {
            if (bytes[i] == target) {
                return i
            }
        }
        return -1
    }

    private fun indexOfBinaryMagic(bytes: ByteArray, startIndex: Int = 0): Int {
        if (bytes.size < BINARY_FRAME_MAGIC.size) {
            return -1
        }
        val start = startIndex.coerceAtLeast(0)
        val last = bytes.size - BINARY_FRAME_MAGIC.size
        for (idx in start..last) {
            var matched = true
            for (magicIdx in BINARY_FRAME_MAGIC.indices) {
                if (bytes[idx + magicIdx] != BINARY_FRAME_MAGIC[magicIdx]) {
                    matched = false
                    break
                }
            }
            if (matched) {
                return idx
            }
        }
        return -1
    }

    private fun decodeUtf8Strict(bytes: ByteArray): String? {
        return runCatching {
            Charsets.UTF_8
                .newDecoder()
                .onMalformedInput(CodingErrorAction.REPORT)
                .onUnmappableCharacter(CodingErrorAction.REPORT)
                .decode(ByteBuffer.wrap(bytes))
                .toString()
        }.getOrNull()
    }

    private fun looksLikeTextTransportPrefix(bytes: ByteArray): Boolean {
        if (bytes.isEmpty()) {
            return false
        }
        return when (bytes[0].toInt().toChar()) {
            '{', '[', 'H', 'h', 'p', 'P' -> true
            else -> false
        }
    }

    private fun trailingBinaryMagicPrefix(bytes: ByteArray): ByteArray {
        val maxKeep = minOf(BINARY_FRAME_MAGIC.size - 1, bytes.size)
        for (keep in maxKeep downTo 1) {
            var matched = true
            val start = bytes.size - keep
            for (idx in 0 until keep) {
                if (bytes[start + idx] != BINARY_FRAME_MAGIC[idx]) {
                    matched = false
                    break
                }
            }
            if (matched) {
                return bytes.copyOfRange(start, bytes.size)
            }
        }
        return ByteArray(0)
    }

    private fun isHumanReadablePlainStatus(text: String): Boolean {
        val trimmed = text.trim()
        if (trimmed.isEmpty() || trimmed.length > 160) {
            return false
        }
        var hasLetterOrDigit = false
        for (ch in trimmed) {
            if (ch.code < 0x20 || ch.code > 0x7E) {
                return false
            }
            if (ch.isLetterOrDigit()) {
                hasLetterOrDigit = true
            }
        }
        return hasLetterOrDigit
    }

    private sealed interface IncomingPayload {
        data class TextLine(val text: String) : IncomingPayload
        data class BinaryFrame(val bytes: ByteArray) : IncomingPayload
    }

    private data class ControlField(
        val id: Int,
        val type: Int,
        val value: ByteArray,
    )

    private fun decodeTransportLine(line: String): IncomingPayload? {
        val text = line.trim()
        if (text.startsWith("HX:", ignoreCase = true)) {
            val hex = text.substring(3)
            val payload = hex.hexToBytes() ?: return null
            val decoded = payload.toString(Charsets.UTF_8).trim()
            if (decoded.isEmpty()) {
                return null
            }
            return IncomingPayload.TextLine(decoded)
        }
        if (text.startsWith("HB:", ignoreCase = true)) {
            val hex = text.substring(3)
            val payload = hex.hexToBytes() ?: return null
            return IncomingPayload.BinaryFrame(payload)
        }
        return IncomingPayload.TextLine(text)
    }

    private fun String.hexToBytes(): ByteArray? {
        val clean = trim()
        if (clean.length % 2 != 0) {
            return null
        }
        val output = ByteArray(clean.length / 2)
        var idx = 0
        while (idx < clean.length) {
            val hi = clean[idx].hexNibble() ?: return null
            val lo = clean[idx + 1].hexNibble() ?: return null
            output[idx / 2] = ((hi shl 4) or lo).toByte()
            idx += 2
        }
        return output
    }

    private fun Char.hexNibble(): Int? {
        return when (this) {
            in '0'..'9' -> this - '0'
            in 'a'..'f' -> this - 'a' + 10
            in 'A'..'F' -> this - 'A' + 10
            else -> null
        }
    }

    private fun isStructuredJsonLine(line: String): Boolean {
        if (!line.startsWith("{")) {
            return false
        }
        return runCatching {
            val obj = JsonParser.parseString(line).asJsonObject
            val typeCode = runCatching { obj.get("t")?.asInt }.getOrNull()
            if (typeCode != null && typeCode > 0) {
                return@runCatching true
            }
            val legacyType = runCatching { obj.get("type")?.asString?.trim().orEmpty() }.getOrNull().orEmpty()
            legacyType.isNotEmpty()
        }.getOrDefault(false)
    }

    private fun encodeControlRequestFrameFromJson(jsonText: String): ByteArray? {
        val obj = runCatching { JsonParser.parseString(jsonText).asJsonObject }.getOrNull() ?: return null
        val msgType = runCatching { obj.get("t")?.asInt }.getOrNull() ?: return null
        if (msgType !in GaitProtocol.TYPE_PING..GaitProtocol.TYPE_IMU_MANAGE) {
            return null
        }
        val fields = ArrayList<ControlField>()
        addIntField(obj, "m", CONTROL_FIELD_MODE, fields)
        addIntField(obj, "c", CONTROL_FIELD_COMMAND, fields)
        addBoolLikeField(obj, "v", CONTROL_FIELD_VALUE, fields)
        addIntField(obj, "pf", CONTROL_FIELD_PLOT_FORMAT, fields)
        addIntField(obj, "pm", CONTROL_FIELD_PLOT_MODE, fields)
        addIntField(obj, "pn", CONTROL_FIELD_PLOT_BATCH_SIZE, fields)
        addIntField(obj, "pe", CONTROL_FIELD_PLOT_EVERY_N, fields)
        addIntField(obj, "s", CONTROL_FIELD_IMU_SLOT, fields)
        addBoolLikeField(obj, "x", CONTROL_FIELD_IMU_CONNECT, fields)
        addBoolLikeField(obj, "ao", CONTROL_FIELD_ALLOW_OFFMODE, fields)

        val macBytes = parseMacBytesField(obj)
        if (macBytes != null && macBytes.isNotEmpty()) {
            fields.add(
                ControlField(
                    id = CONTROL_FIELD_IMU_MAC_BYTES,
                    type = CONTROL_TYPE_BYTES,
                    value = macBytes,
                )
            )
        }

        val paramUpdates = encodeParamUpdatesField(obj)
        if (paramUpdates != null && paramUpdates.isNotEmpty()) {
            fields.add(
                ControlField(
                    id = CONTROL_FIELD_PARAM_UPDATES,
                    type = CONTROL_TYPE_BYTES,
                    value = paramUpdates,
                )
            )
        }

        return buildControlFrame(
            kind = CONTROL_KIND_REQUEST,
            msgType = msgType,
            fields = fields,
        )
    }

    private fun decodeControlResponseFrameToJson(frame: ByteArray): String? {
        val header = parseBinaryFrameHeader(frame) ?: return null
        if (header.kind != CONTROL_KIND_RESPONSE) {
            return null
        }
        if (frame.size < BINARY_FRAME_HEADER_SIZE + 3) {
            return null
        }
        val payload = ByteBuffer.wrap(frame, BINARY_FRAME_HEADER_SIZE, header.payloadLength)
            .order(ByteOrder.LITTLE_ENDIAN)
        val version = payload.get().toInt() and 0xFF
        if (version != CONTROL_VERSION) {
            return null
        }
        val msgType = payload.get().toInt() and 0xFF
        val fieldCount = payload.get().toInt() and 0xFF
        val obj = JsonObject().apply { addProperty("t", msgType) }
        repeat(fieldCount) {
            if (payload.remaining() < 4) {
                return null
            }
            val fieldId = payload.get().toInt() and 0xFF
            val valueType = payload.get().toInt() and 0xFF
            val length = payload.short.toInt() and 0xFFFF
            if (payload.remaining() < length) {
                return null
            }
            val value = ByteArray(length)
            payload.get(value)
            applyControlResponseField(obj, fieldId, valueType, value)
        }
        return obj.toString()
    }

    private fun applyControlResponseField(
        obj: JsonObject,
        fieldId: Int,
        valueType: Int,
        value: ByteArray,
    ) {
        fun asInt(): Int? {
            if (valueType != CONTROL_TYPE_INT32 || value.size != 4) return null
            return ByteBuffer.wrap(value).order(ByteOrder.LITTLE_ENDIAN).int
        }

        fun asFloat(): Float? {
            if (valueType != CONTROL_TYPE_FLOAT32 || value.size != 4) return null
            return ByteBuffer.wrap(value).order(ByteOrder.LITTLE_ENDIAN).float
        }

        fun asBoolInt(): Int? {
            if (valueType != CONTROL_TYPE_BOOL || value.isEmpty()) return null
            return if ((value[0].toInt() and 0xFF) != 0) 1 else 0
        }

        when (fieldId) {
            CONTROL_FIELD_RESULT -> asBoolInt()?.let { obj.addProperty("o", it) }
            CONTROL_FIELD_ACTION -> asInt()?.let { obj.addProperty("a", it) }
            CONTROL_FIELD_MODE_APPLIED -> asInt()?.let { obj.addProperty("m", it) }
            CONTROL_FIELD_ENABLED -> asBoolInt()?.let { obj.addProperty("e", it) }
            CONTROL_FIELD_REASON -> asInt()?.let { obj.addProperty("r", it) }
            CONTROL_FIELD_PLOT_FORMAT_APPLIED -> asInt()?.let { obj.addProperty("pf", it) }
            CONTROL_FIELD_PLOT_MODE_APPLIED -> asInt()?.let { obj.addProperty("pm", it) }
            CONTROL_FIELD_PLOT_BATCH_APPLIED -> asInt()?.let { obj.addProperty("pn", it) }
            CONTROL_FIELD_PLOT_EVERY_N_APPLIED -> asInt()?.let { obj.addProperty("pe", it) }
            CONTROL_FIELD_PARAM_COUNT -> asInt()?.let { obj.addProperty("n", it) }
            CONTROL_FIELD_MODE_OVERRIDDEN -> asBoolInt()?.let { obj.addProperty("ov", it) }
            CONTROL_FIELD_ERROR_CODE -> asInt()?.let { obj.addProperty("ec", it) }
            CONTROL_FIELD_IMU_SLOT -> asInt()?.let { obj.addProperty("s", it) }
            CONTROL_FIELD_IMU_CONNECTED -> asBoolInt()?.let { obj.addProperty("k", it) }
            CONTROL_FIELD_IMU_MEASURING -> asBoolInt()?.let { obj.addProperty("q", it) }
            CONTROL_FIELD_IMU_READY -> asBoolInt()?.let { obj.addProperty("d", it) }
            CONTROL_FIELD_IMU_STALE -> asBoolInt()?.let { obj.addProperty("z", it) }
            CONTROL_FIELD_STATE_VERSION -> asInt()?.let { obj.addProperty("v", it) }
            CONTROL_FIELD_STATE_MODE -> asInt()?.let { obj.addProperty("mi", it) }
            CONTROL_FIELD_STATE_GAIT -> asInt()?.let { obj.addProperty("gs", it) }
            CONTROL_FIELD_STATE_FLAGS -> asInt()?.let { obj.addProperty("f", it) }
            CONTROL_FIELD_STATE_SCORE -> asFloat()?.let { obj.addProperty("ds", it) }
            CONTROL_FIELD_STATE_PARAMS -> {
                if (valueType == CONTROL_TYPE_BYTES) {
                    decodeParamPairsBytes(value)?.let { obj.add("u", it) }
                }
            }
            CONTROL_FIELD_COMMAND -> asInt()?.let { obj.addProperty("c", it) }
        }
    }

    private fun decodeParamPairsBytes(bytes: ByteArray): JsonArray? {
        if (bytes.isEmpty() || bytes.size % 5 != 0) return null
        val payload = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        val array = JsonArray()
        while (payload.remaining() >= 5) {
            val code = payload.get().toInt() and 0xFF
            val value = payload.float
            val row = JsonArray()
            row.add(code)
            row.add(value.toDouble())
            array.add(row)
        }
        return array
    }

    private fun parseMacBytesField(obj: JsonObject): ByteArray? {
        val rawArray = runCatching { obj.getAsJsonArray("ma") }.getOrNull()
        if (rawArray != null) {
            val values = ArrayList<Int>(6)
            for (element in rawArray) {
                val value = runCatching { element.asInt }.getOrNull() ?: return null
                if (value !in 0..255) return null
                values.add(value)
            }
            if (values.size == 6) {
                return ByteArray(6) { idx -> values[idx].toByte() }
            }
        }
        val legacyMac = runCatching { obj.get("mac")?.asString?.trim().orEmpty() }.getOrNull().orEmpty()
        if (legacyMac.isBlank()) {
            return null
        }
        val parts = legacyMac.split(":")
        if (parts.size != 6) return null
        val bytes = ByteArray(6)
        for (idx in parts.indices) {
            val value = runCatching { parts[idx].toInt(16) }.getOrNull() ?: return null
            if (value !in 0..255) return null
            bytes[idx] = value.toByte()
        }
        return bytes
    }

    private fun encodeParamUpdatesField(obj: JsonObject): ByteArray? {
        val updatesOut = ByteArrayOutputStream()
        val updateArray = runCatching { obj.getAsJsonArray("u") }.getOrNull()
        if (updateArray != null) {
            for (entry in updateArray) {
                val row = runCatching { entry.asJsonArray }.getOrNull() ?: continue
                if (row.size() < 2) continue
                val code = runCatching { row[0].asInt }.getOrNull() ?: continue
                val value = runCatching { row[1].asFloat }.getOrNull() ?: continue
                if (code !in 1..255) continue
                updatesOut.write(code)
                updatesOut.write(packFloat32(value))
            }
        }
        val paramsObj = runCatching { obj.getAsJsonObject("p") }.getOrNull()
        if (paramsObj != null) {
            for ((rawKey, element) in paramsObj.entrySet()) {
                val code = rawKey.toIntOrNull() ?: GaitProtocol.paramCodeFromKey(rawKey) ?: continue
                val value = runCatching { element.asFloat }.getOrNull() ?: continue
                if (code !in 1..255) continue
                updatesOut.write(code)
                updatesOut.write(packFloat32(value))
            }
        }
        val bytes = updatesOut.toByteArray()
        return if (bytes.isNotEmpty()) bytes else null
    }

    private fun addIntField(
        obj: JsonObject,
        key: String,
        fieldId: Int,
        out: MutableList<ControlField>,
    ) {
        val element = obj.get(key) ?: return
        val value = runCatching { element.asInt }.getOrNull() ?: return
        out.add(ControlField(fieldId, CONTROL_TYPE_INT32, packInt32(value)))
    }

    private fun addBoolLikeField(
        obj: JsonObject,
        key: String,
        fieldId: Int,
        out: MutableList<ControlField>,
    ) {
        val element = obj.get(key) ?: return
        val value = GaitProtocol.boolElementOrNull(element) ?: return
        out.add(ControlField(fieldId, CONTROL_TYPE_BOOL, byteArrayOf(if (value) 1 else 0)))
    }

    private fun buildControlFrame(
        kind: Int,
        msgType: Int,
        fields: List<ControlField>,
    ): ByteArray {
        val payloadOut = ByteArrayOutputStream()
        payloadOut.write(CONTROL_VERSION)
        payloadOut.write(msgType and 0xFF)
        payloadOut.write(fields.size and 0xFF)
        for (field in fields) {
            payloadOut.write(field.id and 0xFF)
            payloadOut.write(field.type and 0xFF)
            val len = field.value.size.coerceAtMost(0xFFFF)
            payloadOut.write(len and 0xFF)
            payloadOut.write((len ushr 8) and 0xFF)
            payloadOut.write(field.value, 0, len)
        }
        val payload = payloadOut.toByteArray()
        val frame = ByteArray(BINARY_FRAME_HEADER_SIZE + payload.size)
        frame[0] = BINARY_FRAME_MAGIC[0]
        frame[1] = BINARY_FRAME_MAGIC[1]
        frame[2] = BINARY_FRAME_MAGIC[2]
        frame[3] = BINARY_FRAME_MAGIC[3]
        frame[4] = (kind and 0xFF).toByte()
        frame[5] = 1
        frame[6] = (payload.size and 0xFF).toByte()
        frame[7] = ((payload.size ushr 8) and 0xFF).toByte()
        payload.copyInto(frame, BINARY_FRAME_HEADER_SIZE)
        return frame
    }

    private fun packInt32(value: Int): ByteArray {
        return ByteBuffer.allocate(4)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putInt(value)
            .array()
    }

    private fun packFloat32(value: Float): ByteArray {
        return ByteBuffer.allocate(4)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putFloat(value)
            .array()
    }

    private fun isSoftRealtimePayloadLine(line: String): Boolean {
        val text = line.trimStart()
        if (!text.startsWith("{")) {
            return false
        }
        return runCatching {
            val obj = JsonParser.parseString(text).asJsonObject
            when (runCatching { obj.get("t")?.asInt }.getOrNull()) {
                GaitProtocol.TYPE_STATE,
                GaitProtocol.TYPE_PLOT,
                GaitProtocol.TYPE_PLOT_BATCH -> true
                else -> {
                    val legacyType = runCatching { obj.get("type")?.asString?.trim().orEmpty() }
                        .getOrNull()
                        .orEmpty()
                    legacyType == "state" || legacyType == "plot" || legacyType == "plot_batch"
                }
            }
        }.getOrDefault(false)
    }

    private fun hasRequiredPermissions(): Boolean {
        val permissions = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            permissions.add(Manifest.permission.BLUETOOTH)
            permissions.add(Manifest.permission.BLUETOOTH_ADMIN)
        }
        permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)
        return permissions.all { permission ->
            ContextCompat.checkSelfPermission(appContext, permission) == PackageManager.PERMISSION_GRANTED
        }
    }

    private fun postStatus(message: String) {
        mainHandler.post { statusCallback(message) }
    }

    private inner class GaitGattCallback(
        private val device: BluetoothDevice
    ) : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (status != BluetoothGatt.GATT_SUCCESS && newState != BluetoothGatt.STATE_CONNECTED) {
                Log.w(TAG, "Connection state change failed: status=$status newState=$newState")
            }
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                postStatus("BLE已连接，发现服务中: ${device.name ?: device.address}")
                requestHighConnectionPriority(gatt)
                serviceDiscoveryAttempt = 0
                serviceDiscoveryInProgress = true
                scheduleServiceDiscovery(gatt)
                return
            }
            if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "蓝牙连接已断开",
                    reconnectReason = "检测到步态BLE断连",
                )
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.i(TAG, "Gait BLE MTU negotiated: $mtu")
            } else {
                Log.w(TAG, "Gait BLE MTU request failed: status=$status mtu=$mtu")
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            serviceDiscoveryInProgress = false
            cancelServiceDiscoveryTimeout()
            if (status != BluetoothGatt.GATT_SUCCESS) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE服务发现失败: $status",
                    reconnectReason = "检测到步态BLE服务发现失败",
                )
                return
            }
            val service = gatt.getService(GAIT_SERVICE_UUID)
            if (service == null) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "未找到步态BLE服务: ${device.address}",
                    reconnectReason = "检测到步态BLE服务缺失",
                )
                return
            }
            notifyCharacteristic = service.getCharacteristic(GAIT_NOTIFY_CHAR_UUID)
            controlCharacteristic = service.getCharacteristic(GAIT_CONTROL_CHAR_UUID)
            if (notifyCharacteristic == null || controlCharacteristic == null) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE特征不完整: ${device.address}",
                    reconnectReason = "检测到步态BLE特征不完整",
                )
                return
            }
            if (!enableNotification(gatt, service)) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE通知订阅失败: ${device.address}",
                    reconnectReason = "检测到步态BLE通知订阅失败",
                )
                return
            }
            isAwaitingHandshake = true
            postStatus("BLE Notify已订阅，等待设备响应: ${device.name ?: device.address}")
        }

        override fun onDescriptorWrite(
            gatt: BluetoothGatt,
            descriptor: BluetoothGattDescriptor,
            status: Int
        ) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (descriptor.uuid != CCCD_UUID) {
                return
            }
            if (status != BluetoothGatt.GATT_SUCCESS) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE通知订阅失败: $status",
                    reconnectReason = "检测到步态BLE通知订阅失败",
                )
                return
            }
            val characteristic = controlCharacteristic
            if (characteristic == null) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE控制特征不可用",
                    reconnectReason = "检测到步态BLE控制特征不可用",
                )
                return
            }
            if (!enqueueControlWrite(gatt, characteristic, buildPingFrame())) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "握手 ping 发送失败",
                    reconnectReason = "检测到步态BLE握手发送失败",
                )
                return
            }
            scheduleHandshakeTimeout(gatt, device)
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (characteristic.uuid != GAIT_CONTROL_CHAR_UUID) {
                return
            }
            var shouldFailConnection = false
            synchronized(controlWriteLock) {
                writeInFlight = false
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    Log.w(TAG, "BLE control write failed: status=$status")
                    controlWriteQueue.clear()
                    shouldFailConnection = true
                } else {
                    startNextControlWriteLocked(gatt, characteristic)
                }
            }
            if (shouldFailConnection) {
                failActiveConnection(
                    activeGatt = gatt,
                    userMessage = "步态BLE控制写入失败: $status",
                    reconnectReason = "检测到步态BLE控制写入失败",
                )
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (characteristic.uuid != GAIT_NOTIFY_CHAR_UUID) {
                return
            }
            processIncomingBytes(value, device)
        }

        @Deprecated("Deprecated in API 33")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (this@GaitBluetoothConnector.gatt !== gatt) {
                return
            }
            if (characteristic.uuid != GAIT_NOTIFY_CHAR_UUID) {
                return
            }
            val value = characteristic.value ?: return
            processIncomingBytes(value, device)
        }
    }
}
