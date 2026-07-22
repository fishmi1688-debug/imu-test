package com.example.mobile

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.util.Log
import com.example.mobile.xsens.IMU
import com.example.mobile.xsens.JointAngleCalculator
import com.example.mobile.xsens.JointAngleChangedCallback
import com.example.mobile.xsens.XsensDotLogger
import com.xsens.dot.android.sdk.XsensDotSdk
import com.xsens.dot.android.sdk.events.XsensDotData
import com.xsens.dot.android.sdk.interfaces.XsensDotDeviceCallback
import com.xsens.dot.android.sdk.interfaces.XsensDotMeasurementCallback
import com.xsens.dot.android.sdk.interfaces.XsensDotScannerCallback
import com.xsens.dot.android.sdk.interfaces.XsensDotSyncCallback
import com.xsens.dot.android.sdk.models.FilterProfileInfo
import com.xsens.dot.android.sdk.models.XsensDotDevice
import com.xsens.dot.android.sdk.models.XsensDotSyncManager
import com.xsens.dot.android.sdk.utils.XsensDotParser
import com.xsens.dot.android.sdk.utils.XsensDotScanner
import java.io.File
import java.util.ArrayList
import java.util.HashMap

class XsensDotManager(
    context: Context,
    private val listener: Listener,
) {
    companion object {
        private const val TAG = "XsensDotManager"
        const val FILTER_PROFILE_GENERAL = 0
        const val FILTER_PROFILE_DYNAMIC = 1
        const val LOGGER_FLAG_DEFAULT = 0
        const val LOGGER_FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER = 6
        const val LOGGER_FLAG_ACC_GYR_ONLY = 7

        private const val PAYLOAD_TYPE_CUSTOM_MODE_4 = 25
        private const val LOGGER_TYPE_CUSTOM_MODE = 27
        private const val SYNC_REQUEST_CODE = 0x45
    }

    enum class SyncStatus {
        NOT_SYNCHRONIZED,
        SYNCHRONIZING,
        SYNCHRONIZED,
    }

    data class XsensTelemetry(
        val freeAcc: DoubleArray,
        val acc: DoubleArray,
        val gyr: DoubleArray,
        val mag: DoubleArray,
        val euler: DoubleArray,
        val angle: DoubleArray?,
        val timestamp: Long,
        val sampleTimeFine: Long,
    )

    interface Listener {
        fun onDeviceFound(address: String, name: String, rssi: Int)
        fun onConnectionChanged(address: String, state: Int)
        fun onBatteryChanged(address: String, battery: Int)
        fun onTelemetry(address: String, telemetry: XsensTelemetry)
        fun onSyncProgress(progress: Int)
        fun onSyncDone(success: Boolean)
        fun onJointAngles(side: String, hip: DoubleArray?, knee: DoubleArray?, ankle: DoubleArray?)
        fun onStatus(message: String)
    }

    private val appContext = context.applicationContext
    private val bluetoothAdapter by lazy {
        (appContext.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter
    }
    private var scanner: XsensDotScanner? = null
    private val deviceMap: MutableMap<String, XsensDotDevice> = LinkedHashMap()
    private val stateMap: MutableMap<String, Int> = LinkedHashMap()
    private val loggerMap: MutableMap<String, XsensDotLogger> = LinkedHashMap()
    private var labelModeEnabledForRecording: Boolean = false
    private var labelProviderForRecording: (() -> String?)? = null
    private var leftJointCalculator: JointAngleCalculator? = null
    private var rightJointCalculator: JointAngleCalculator? = null

    var syncStatus: SyncStatus = SyncStatus.NOT_SYNCHRONIZED
        private set

    private val callback = object :
        XsensDotDeviceCallback,
        XsensDotScannerCallback,
        XsensDotSyncCallback,
        XsensDotMeasurementCallback,
        JointAngleChangedCallback {

        override fun onXsensDotConnectionChanged(address: String, state: Int) {
            stateMap[address] = state
            if (state == XsensDotDevice.CONN_STATE_DISCONNECTED) {
                loggerMap.remove(address)?.stop()
            }
            listener.onConnectionChanged(address, state)
        }

        override fun onXsensDotServicesDiscovered(address: String, status: Int) = Unit

        override fun onXsensDotFirmwareVersionRead(address: String, firmwareVersion: String) = Unit

        override fun onXsensDotTagChanged(address: String, tag: String) = Unit

        override fun onXsensDotBatteryChanged(address: String, isCharging: Int, battery: Int) {
            listener.onBatteryChanged(address, battery)
        }

        override fun onXsensDotDataChanged(address: String, data: XsensDotData) {
            val label = if (labelModeEnabledForRecording) {
                labelProviderForRecording?.invoke()
            } else {
                null
            }
            loggerMap[address]?.update(data, label)
            updateSensorData(address, data)

            val telemetry = XsensTelemetry(
                freeAcc = readCalFreeAcc(data),
                acc = data.acc ?: doubleArrayOf(),
                gyr = data.gyr ?: doubleArrayOf(),
                mag = data.mag ?: doubleArrayOf(),
                euler = XsensDotParser.quaternion2Euler(data.quat),
                angle = getAngles(address, data),
                timestamp = System.currentTimeMillis(),
                sampleTimeFine = data.sampleTimeFine,
            )
            listener.onTelemetry(address, telemetry)
        }

        override fun onXsensDotInitDone(address: String) {
            val device = deviceMap[address] ?: return
            runCatching {
                val battery = device.batteryPercentage
                if (battery >= 0) {
                    listener.onBatteryChanged(address, battery)
                }
                device.readRssi()
            }
        }

        override fun onXsensDotButtonClicked(address: String, timestamp: Long) = Unit

        override fun onXsensDotPowerSavingTriggered(address: String) = Unit

        override fun onReadRemoteRssi(address: String, rssi: Int) = Unit

        override fun onXsensDotOutputRateUpdate(address: String, rate: Int) = Unit

        override fun onXsensDotFilterProfileUpdate(address: String, profile: Int) = Unit

        override fun onXsensDotGetFilterProfileInfo(address: String, profiles: ArrayList<FilterProfileInfo>) = Unit

        override fun onSyncStatusUpdate(address: String, synced: Boolean) = Unit

        @SuppressLint("MissingPermission")
        override fun onXsensDotScanned(device: BluetoothDevice, rssi: Int) {
            val address = device.address ?: return
            val name = device.name ?: "XsensDot"
            listener.onDeviceFound(address, name, rssi)
        }

        override fun onSyncingStarted(address: String, isSuccess: Boolean, requestCode: Int) {
            syncStatus = if (isSuccess) SyncStatus.SYNCHRONIZING else SyncStatus.NOT_SYNCHRONIZED
        }

        override fun onSyncingProgress(progress: Int, requestCode: Int) {
            listener.onSyncProgress(progress)
        }

        override fun onSyncingResult(address: String, isSuccess: Boolean, requestCode: Int) = Unit

        override fun onSyncingDone(syncingResultMap: HashMap<String, Boolean>, isSuccess: Boolean, requestCode: Int) {
            syncStatus = if (isSuccess) SyncStatus.SYNCHRONIZED else SyncStatus.NOT_SYNCHRONIZED
            listener.onSyncDone(isSuccess)
        }

        override fun onSyncingStopped(address: String, isSuccess: Boolean, requestCode: Int) {
            syncStatus = SyncStatus.NOT_SYNCHRONIZED
            listener.onStatus("Xsens 同步已停止")
        }

        override fun onXsensDotHeadingChanged(address: String, status: Int, result: Int) = Unit

        override fun onXsensDotRotLocalRead(address: String, rotLocal: FloatArray) = Unit

        override fun onJointAngleChanged(side: String, angHip: DoubleArray?, angKnee: DoubleArray?, angAnkle: DoubleArray?) {
            listener.onJointAngles(side, angHip, angKnee, angAnkle)
        }
    }

    init {
        runCatching {
            XsensDotSdk.getSdkVersion()
            XsensDotSdk.setDebugEnabled(false)
            XsensDotSdk.setReconnectEnabled(true)
        }.onFailure {
            Log.e(TAG, "Xsens SDK init failed", it)
            listener.onStatus("Xsens SDK 初始化失败: ${it.message}")
        }
    }

    fun startScan(): Boolean {
        if (scanner == null) {
            scanner = XsensDotScanner(appContext, callback).also {
                it.setScanMode(ScanSettings.SCAN_MODE_BALANCED)
            }
        }
        return runCatching { scanner?.startScan() == true }
            .onFailure { Log.e(TAG, "startScan failed", it) }
            .getOrDefault(false)
    }

    fun stopScan(): Boolean {
        return runCatching { scanner?.stopScan() == true }
            .onFailure { Log.e(TAG, "stopScan failed", it) }
            .getOrDefault(false)
    }

    @SuppressLint("MissingPermission")
    fun connect(address: String): Boolean {
        return runCatching {
            val bleDevice = bluetoothAdapter?.getRemoteDevice(address)
                ?: return false
            val dotDevice = XsensDotDevice(appContext, bleDevice, callback)
            dotDevice.connect()
            deviceMap[address] = dotDevice
            true
        }.onFailure {
            Log.e(TAG, "connect failed: $address", it)
            listener.onStatus("Xsens 连接失败: ${it.message}")
        }.getOrDefault(false)
    }

    fun disconnect(address: String): Boolean {
        val device = deviceMap[address] ?: return false
        return runCatching {
            device.cancelReconnecting()
            device.disconnect()
            true
        }.onFailure {
            Log.e(TAG, "disconnect failed: $address", it)
        }.getOrDefault(false)
    }

    fun disconnectAll() {
        deviceMap.keys.toList().forEach { address ->
            disconnect(address)
        }
    }

    fun connectedAddresses(): List<String> {
        return stateMap.filterValues { it == XsensDotDevice.CONN_STATE_CONNECTED }.keys.toList()
    }

    fun startSynchronization(): Boolean {
        val devices = connectedAddresses().mapNotNull { deviceMap[it] }
        if (devices.isEmpty()) {
            listener.onStatus("没有已连接的 Xsens 设备")
            return false
        }
        return runCatching {
            syncStatus = SyncStatus.SYNCHRONIZING
            devices[0].setRootDevice(true)
            XsensDotSyncManager.getInstance(callback).startSyncing(ArrayList(devices), SYNC_REQUEST_CODE)
        }.onFailure {
            syncStatus = SyncStatus.NOT_SYNCHRONIZED
            Log.e(TAG, "startSynchronization failed", it)
            listener.onStatus("Xsens 同步启动失败: ${it.message}")
        }.getOrDefault(false)
    }

    fun stopSynchronization(): Boolean {
        val devices = connectedAddresses().mapNotNull { deviceMap[it] }
        if (devices.isEmpty()) {
            syncStatus = SyncStatus.NOT_SYNCHRONIZED
            return false
        }
        return runCatching {
            devices[0].setRootDevice(true)
            val ret = XsensDotSyncManager.getInstance(callback).stopSyncing(ArrayList(devices))
            syncStatus = SyncStatus.NOT_SYNCHRONIZED
            ret
        }.onFailure {
            Log.e(TAG, "stopSynchronization failed", it)
        }.getOrDefault(false)
    }

    fun applyOutputConfigToConnected(rate: Int, filterProfile: Int) {
        connectedAddresses().forEach { address ->
            val device = deviceMap[address] ?: return@forEach
            runCatching {
                device.setOutputRate(rate)
                device.setFilterProfile(filterProfile)
            }
        }
    }

    fun resetBodyPosition() {
        leftJointCalculator = null
        rightJointCalculator = null
    }

    fun setBodyPosition(side: String, thighAddress: String?, lowerLegAddress: String?, footAddress: String?) {
        if (side == JointAngleCalculator.SIDE_LEFT) {
            leftJointCalculator = JointAngleCalculator(side, thighAddress, lowerLegAddress, footAddress, callback)
        } else if (side == JointAngleCalculator.SIDE_RIGHT) {
            rightJointCalculator = JointAngleCalculator(side, thighAddress, lowerLegAddress, footAddress, callback)
        }
    }

    fun setLoggerFlag(flag: Int) {
        XsensDotLogger.setFlag(flag)
    }

    fun startRecording(
        logDir: File,
        subjectName: String,
        gaitType: String,
        recordingTimeTag: String,
        outputRate: Int,
        filterProfile: Int,
        sensorDisplayNames: Map<String, String>,
        appVersion: String,
        loggerFlag: Int,
        labelModeEnabled: Boolean = false,
        labelProvider: (() -> String?)? = null,
    ): Int {
        labelModeEnabledForRecording = labelModeEnabled
        labelProviderForRecording = if (labelModeEnabled) labelProvider else null
        setLoggerFlag(loggerFlag)
        val startedAt = System.currentTimeMillis()
        var startedCount = 0

        connectedAddresses().forEach { address ->
            val device = deviceMap[address] ?: return@forEach

            runCatching {
                device.setMeasurementMode(PAYLOAD_TYPE_CUSTOM_MODE_4)
                device.setOutputRate(outputRate)
                device.setFilterProfile(filterProfile)
                device.setXsensDotMeasurementCallback(callback)

                var success = false
                repeat(20) {
                    if (success) {
                        return@repeat
                    }
                    Thread.sleep(40)
                    success = device.startMeasuring()
                }

                if (!success) {
                    listener.onStatus("Xsens 设备未进入测量状态，跳过写文件: $address")
                    return@runCatching
                }

                startedCount += 1

                val sensorName = sensorDisplayNames[address] ?: address
                val file = File(
                    logDir,
                    "${sanitize(sensorName)}+${sanitize(subjectName)}+${sanitize(gaitType)}+${sanitize(recordingTimeTag)}.csv"
                )
                val logger = XsensDotLogger(
                    appContext,
                    XsensDotLogger.TYPE_CSV,
                    LOGGER_TYPE_CUSTOM_MODE,
                    file.absolutePath,
                    sensorName,
                    runCatching { device.firmwareVersion }.getOrNull() ?: "0.0.0",
                    syncStatus == SyncStatus.SYNCHRONIZED,
                    outputRate,
                    filterProfileLabel(filterProfile),
                    appVersion,
                    startedAt,
                    labelModeEnabledForRecording,
                )
                loggerMap[address] = logger
            }.onFailure {
                Log.e(TAG, "startRecording failed: $address", it)
                listener.onStatus("Xsens 开始采集失败: ${it.message}")
            }
        }

        return startedCount
    }

    fun stopRecording() {
        connectedAddresses().forEach { address ->
            runCatching {
                deviceMap[address]?.stopMeasuring()
            }
        }
        loggerMap.values.forEach { logger ->
            runCatching { logger.stop() }
        }
        loggerMap.clear()
        labelModeEnabledForRecording = false
        labelProviderForRecording = null
    }

    fun resetHeading(address: String): Boolean {
        val device = deviceMap[address] ?: return false
        return runCatching {
            if (device.measurementState == XsensDotDevice.MEASUREMENT_STATE_OFF) {
                return false
            }
            if (device.headingStatus == XsensDotDevice.HEADING_STATUS_XRM_HEADING) {
                device.revertHeading()
            } else {
                device.resetHeading()
            }
        }.getOrDefault(false)
    }

    fun resetHeadingForConnected() {
        connectedAddresses().forEach { address ->
            resetHeading(address)
        }
    }

    fun release() {
        stopRecording()
        stopScan()
        disconnectAll()
        resetBodyPosition()
    }

    private fun updateSensorData(address: String, data: XsensDotData) {
        var imu = leftJointCalculator?.getIMU(address)
        if (imu == null) {
            imu = rightJointCalculator?.getIMU(address)
        }
        imu?.data = data
    }

    private fun getAngles(address: String, data: XsensDotData): DoubleArray? {
        var imu: IMU? = leftJointCalculator?.getIMU(address)
        var side = -1
        if (imu != null) {
            side = JointAngleCalculator.Side_Left
        }
        if (imu == null) {
            imu = rightJointCalculator?.getIMU(address)
            if (imu != null) {
                side = JointAngleCalculator.Side_Right
            }
        }
        if (imu == null) {
            return null
        }

        return when (imu.part) {
            IMU.Thigh -> JointAngleCalculator.calculateHip(data)
            IMU.Foot -> JointAngleCalculator.calculateAnkle(data)
            IMU.LowerLeg -> {
                if (side == JointAngleCalculator.Side_Left) {
                    val thigh = leftJointCalculator?.getIMU(IMU.Thigh)?.data
                    JointAngleCalculator.calculateKnee(thigh, data)
                } else {
                    val thigh = rightJointCalculator?.getIMU(IMU.Thigh)?.data
                    JointAngleCalculator.calculateKnee(thigh, data)
                }
            }
            else -> null
        }
    }

    private fun sanitize(raw: String): String {
        return raw.trim()
            .replace("\\s+".toRegex(), "_")
            .replace("[\\\\/:*?\"<>|]".toRegex(), "_")
            .trim('_')
            .ifBlank { "unknown" }
    }

    private fun filterProfileLabel(profile: Int): String {
        return when (profile) {
            FILTER_PROFILE_GENERAL -> "General"
            FILTER_PROFILE_DYNAMIC -> "Dynamic"
            else -> "Unknown"
        }
    }

    private fun readCalFreeAcc(data: XsensDotData): DoubleArray {
        val result = runCatching {
            data.javaClass.getMethod("getCalFreeAcc").invoke(data) as? FloatArray
        }.getOrNull() ?: runCatching {
            data.javaClass.getMethod("getFreeAcc").invoke(data) as? FloatArray
        }.getOrNull()

        return result?.map { it.toDouble() }?.toDoubleArray() ?: doubleArrayOf()
    }
}
