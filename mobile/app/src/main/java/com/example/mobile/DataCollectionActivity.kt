package com.example.mobile

import android.Manifest
import android.app.Activity
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
import android.content.Intent
import android.content.pm.PackageManager
import android.content.ContentValues
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.Environment
import android.provider.MediaStore
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ArrayAdapter
import android.widget.BaseAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.LinearLayout
import android.widget.ListView
import android.widget.Spinner
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.content.ContextCompat
import com.xsens.dot.android.sdk.models.XsensDotDevice
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.LinkedHashMap
import java.util.Locale
import java.util.UUID

class DataCollectionActivity : AppCompatActivity(), XsensDotManager.Listener {

    companion object {
        private const val TAG = "DataCollection"
        private const val REQUEST_CODE_PERMISSIONS = 120
        const val EXTRA_ENABLE_LABEL_MODE = "collection.enable_label_mode"
        private const val DEFAULT_LABEL = "未标注"

        private const val MOVELLA_COMPANY_ID = 0x0886
        private const val SCAN_WINDOW_MS = 5_000L

        private const val INSOLE_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
        private const val INSOLE_SERVICE_SHORT_UUID = "fff0"
        private const val INSOLE_READ_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
        private const val CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"
        private const val INSOLE_TARGET_MTU = 512
        private const val INSOLE_V2_LEFT_MAC_MARKER = "FF2502051A4B"
        private const val INSOLE_V2_RIGHT_MAC_MARKER = "FF25020518F3"
    }

    private enum class SensorType {
        XSENS_DOT,
        INSOLE,
        INSOLE_V2,
    }

    private data class CollectionSensor(
        val address: String,
        var name: String,
        val type: SensorType,
        var rssi: Int,
        var connecting: Boolean = false,
        var connected: Boolean = false,
        var battery: Int? = null,
        var side: String? = null,
        var bodyPart: String? = null,
        var gatt: BluetoothGatt? = null,
    )

    private val requiredPermissions: Array<String>
        get() = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION).apply {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                add(Manifest.permission.BLUETOOTH_SCAN)
                add(Manifest.permission.BLUETOOTH_CONNECT)
            } else {
                add(Manifest.permission.BLUETOOTH)
                add(Manifest.permission.BLUETOOTH_ADMIN)
            }
            if (Build.VERSION.SDK_INT <= Build.VERSION_CODES.P) {
                add(Manifest.permission.WRITE_EXTERNAL_STORAGE)
            }
        }.toTypedArray()

    private val bluetoothAdapter: BluetoothAdapter?
        get() = (getSystemService(BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private lateinit var statusView: TextView
    private lateinit var titleView: TextView
    private lateinit var subtitleView: TextView
    private lateinit var packetCountView: TextView
    private lateinit var syncStatusView: TextView
    private lateinit var selectedUserView: TextView
    private lateinit var labelSummaryView: TextView

    private lateinit var scanButton: Button
    private lateinit var startCollectButton: Button
    private lateinit var stopCollectButton: Button
    private lateinit var openRealtimePlotButton: Button
    private lateinit var createLabelButton: Button
    private lateinit var openSensorConfigButton: Button
    private lateinit var disconnectAllButton: Button
    private lateinit var syncStartButton: Button
    private lateinit var syncStopButton: Button
    private lateinit var resetHeadingButton: Button
    private lateinit var selectUserButton: Button
    private lateinit var createUserButton: Button
    private lateinit var userListButton: Button

    private lateinit var deviceListView: ListView

    private lateinit var listAdapter: CollectionDeviceAdapter
    private val displayLabels = mutableListOf<String>()
    private val displaySensors = mutableListOf<CollectionSensor>()
    private val sensorMap = LinkedHashMap<String, CollectionSensor>()

    private val scanHandler = Handler(Looper.getMainLooper())
    private val scanStopRunnable = Runnable { stopScanInternal(announce = true) }
    private val popupHandler = Handler(Looper.getMainLooper())
    @Volatile
    private var isScanning = false

    private val insoleV1Parser = InsoleV1Parser()
    private val insoleV2Parser = InsoleV2Parser()
    private val loggerMap = LinkedHashMap<String, CsvBufferedLogger>()

    private lateinit var settingsStore: CollectionSettingsStore
    private lateinit var userDb: CollectionUserDbHelper
    private lateinit var xsensManager: XsensDotManager

    private var isCollecting = false
    private var currentLogDir: File? = null
    private var packetCount = 0L
    private var labelModeEnabled = false
    private val labels = mutableListOf<String>()
    @Volatile
    private var activeLabel: String = DEFAULT_LABEL

    private var selectedUser: CollectionUser? = null
    private var selectedSensorAddress: String? = null
    private var lastSubjectName: String = ""
    private var lastGaitType: String = ""
    private var savePathDialog: AlertDialog? = null
    private val dismissSavePathDialogRunnable = Runnable {
        savePathDialog?.dismiss()
        savePathDialog = null
    }

    private val outputRates = intArrayOf(1, 4, 10, 12, 15, 20, 30, 60, 120)
    private val filterProfiles = intArrayOf(
        XsensDotManager.FILTER_PROFILE_GENERAL,
        XsensDotManager.FILTER_PROFILE_DYNAMIC,
    )
    private val loggerFlags = intArrayOf(
        XsensDotManager.LOGGER_FLAG_DEFAULT,
        1, 2, 3, 4, 5,
        XsensDotManager.LOGGER_FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER,
        XsensDotManager.LOGGER_FLAG_ACC_GYR_ONLY,
    )

    private var pendingPermissionAction: (() -> Unit)? = null
    private var pendingBluetoothEnableAction: (() -> Unit)? = null

    private val requestBluetoothEnable = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            showStatus("蓝牙已打开")
            pendingBluetoothEnableAction?.invoke()
        } else {
            showStatus("蓝牙未打开")
        }
        pendingBluetoothEnableAction = null
    }

    private val sensorConfigLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode != Activity.RESULT_OK) {
            return@registerForActivityResult
        }
        val data = result.data ?: return@registerForActivityResult
        val outputRate = data.getIntExtra(
            CollectionSensorConfigActivity.EXTRA_OUTPUT_RATE,
            settingsStore.getOutputRate()
        )
        val filterProfile = data.getIntExtra(
            CollectionSensorConfigActivity.EXTRA_FILTER_PROFILE,
            settingsStore.getFilterProfile()
        )
        val loggerFlag = data.getIntExtra(
            CollectionSensorConfigActivity.EXTRA_LOGGER_FLAG,
            settingsStore.getLoggerFlag()
        )

        settingsStore.setOutputRate(outputRate)
        settingsStore.setFilterProfile(filterProfile)
        settingsStore.setLoggerFlag(loggerFlag)

        xsensManager.applyOutputConfigToConnected(outputRate, filterProfile)
        xsensManager.setLoggerFlag(loggerFlag)
        showStatus(
            "采集参数已应用: ${outputRate}Hz / ${filterProfileLabel(filterProfile)} / ${loggerFlagLabel(loggerFlag)}"
        )
    }

    private val bleScanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            val scanResult = result ?: return
            handleBleScanResult(scanResult)
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>?) {
            results.orEmpty().forEach(::handleBleScanResult)
        }

        override fun onScanFailed(errorCode: Int) {
            isScanning = false
            runOnUiThread {
                showStatus("扫描失败，错误码: $errorCode")
                updateScanButtonText()
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_data_collection)
        labelModeEnabled = intent?.getBooleanExtra(EXTRA_ENABLE_LABEL_MODE, false) == true

        settingsStore = CollectionSettingsStore(this)
        userDb = CollectionUserDbHelper(this)
        xsensManager = XsensDotManager(applicationContext, this)

        statusView = findViewById(R.id.collectionStatusView)
        titleView = findViewById(R.id.collectionTitleView)
        subtitleView = findViewById(R.id.collectionSubtitleView)
        packetCountView = findViewById(R.id.packetCountView)
        syncStatusView = findViewById(R.id.syncStatusView)
        selectedUserView = findViewById(R.id.selectedUserView)
        labelSummaryView = findViewById(R.id.labelSummaryView)

        scanButton = findViewById(R.id.scanCollectionDeviceButton)
        startCollectButton = findViewById(R.id.startCollectButton)
        stopCollectButton = findViewById(R.id.stopCollectButton)
        openRealtimePlotButton = findViewById(R.id.openRealtimePlotButton)
        createLabelButton = findViewById(R.id.createLabelButton)
        openSensorConfigButton = findViewById(R.id.openSensorConfigButton)
        disconnectAllButton = findViewById(R.id.disconnectAllButton)
        syncStartButton = findViewById(R.id.syncStartButton)
        syncStopButton = findViewById(R.id.syncStopButton)
        resetHeadingButton = findViewById(R.id.resetHeadingButton)
        selectUserButton = findViewById(R.id.selectUserButton)
        createUserButton = findViewById(R.id.createUserButton)
        userListButton = findViewById(R.id.userListButton)

        deviceListView = findViewById(R.id.collectionDeviceListView)

        findViewById<ImageButton>(R.id.backToPreviousButton).setOnClickListener {
            onBackPressedDispatcher.onBackPressed()
        }
        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            finish()
        }

        listAdapter = CollectionDeviceAdapter()
        deviceListView.adapter = listAdapter
        deviceListView.setOnItemClickListener { _, _, position, _ ->
            val sensor = displaySensors.getOrNull(position) ?: return@setOnItemClickListener
            onSensorClicked(sensor)
        }
        deviceListView.setOnItemLongClickListener { _, _, position, _ ->
            val sensor = displaySensors.getOrNull(position) ?: return@setOnItemLongClickListener false
            showSensorConfigDialog(sensor)
            true
        }

        scanButton.setOnClickListener {
            ensureBluetoothReady {
                if (isScanning) {
                    stopScanInternal(announce = true)
                } else {
                    startScanInternal()
                }
            }
        }

        disconnectAllButton.setOnClickListener {
            disconnectAllSensors()
        }

        startCollectButton.setOnClickListener {
            showStartCollectionDialog()
        }

        stopCollectButton.setOnClickListener {
            stopCollection("手动停止", showSavePathPopup = true)
        }

        openRealtimePlotButton.setOnClickListener {
            if (!isCollecting) {
                showStatus("请先开始采集")
                return@setOnClickListener
            }
            openRealtimePlotPage()
        }

        createLabelButton.setOnClickListener {
            showCreateLabelDialog()
        }

        openSensorConfigButton.setOnClickListener {
            openSensorConfigPage()
        }

        syncStartButton.setOnClickListener {
            val ok = xsensManager.startSynchronization()
            if (!ok) {
                showStatus("Xsens 同步启动失败")
            } else {
                syncStatusView.text = getString(R.string.collection_sync_status_syncing)
            }
        }

        syncStopButton.setOnClickListener {
            xsensManager.stopSynchronization()
            syncStatusView.text = getString(R.string.collection_sync_status_not)
        }

        resetHeadingButton.setOnClickListener {
            val selected = selectedSensorAddress?.let { sensorMap[it] }
            if (selected?.type == SensorType.XSENS_DOT && selected.connected) {
                xsensManager.resetHeading(selected.address)
            } else {
                xsensManager.resetHeadingForConnected()
            }
            showStatus("已发送 heading reset")
        }

        selectUserButton.setOnClickListener {
            showUserPickerDialog()
        }

        createUserButton.setOnClickListener {
            showUserEditorDialog(null)
        }

        userListButton.setOnClickListener {
            showUserManagerDialog()
        }

        if (labelModeEnabled) {
            titleView.text = getString(R.string.entry_data_collection_v2)
            subtitleView.text = getString(R.string.collection_subtitle_v2)
            createLabelButton.visibility = View.VISIBLE
            labelSummaryView.visibility = View.VISIBLE
            updateLabelSummaryView()
        } else {
            createLabelButton.visibility = View.GONE
            labelSummaryView.visibility = View.GONE
        }
        CollectionRealtimeBridge.configureLabelMode(false, emptyList(), null)

        refreshDeviceList()
        showStatus("可扫描设备: xsens_dot / insole_v2")
        updateScanButtonText()
        updateCollectButtons()
        updatePacketCountView()
        syncStatusView.text = getString(R.string.collection_sync_status_not)
        updateSelectedUserView()
    }

    override fun onDestroy() {
        stopScanInternal(announce = false)
        stopCollection("页面销毁")
        disconnectAllSensors()
        xsensManager.release()
        popupHandler.removeCallbacks(dismissSavePathDialogRunnable)
        savePathDialog?.dismiss()
        savePathDialog = null
        super.onDestroy()
    }

    private fun onSensorClicked(sensor: CollectionSensor) {
        selectedSensorAddress = sensor.address
        showStatus("已选中设备: ${sensor.name}")
    }

    private fun onSensorToggleRequested(sensor: CollectionSensor, shouldConnect: Boolean) {
        selectedSensorAddress = sensor.address
        if (sensor.connecting) {
            refreshDeviceList()
            return
        }

        if (shouldConnect) {
            if (sensor.connected) {
                refreshDeviceList()
                return
            }
            ensureBluetoothReady {
                when (sensor.type) {
                    SensorType.XSENS_DOT -> {
                        sensor.connecting = true
                        refreshDeviceList()
                        val ok = xsensManager.connect(sensor.address)
                        if (!ok) {
                            sensor.connecting = false
                            refreshDeviceList()
                        }
                    }

                    SensorType.INSOLE,
                    SensorType.INSOLE_V2,
                    -> {
                        connectInsoleSensor(sensor)
                    }
                }
            }
            if (!sensor.connecting && !sensor.connected) {
                refreshDeviceList()
            }
            return
        }

        if (sensor.connected) {
            disconnectSensor(sensor.address, "用户断开")
        } else {
            refreshDeviceList()
        }
    }

    private fun ensureBluetoothReady(action: () -> Unit) {
        if (!hasAllRequiredPermissions()) {
            pendingPermissionAction = action
            requestPermissions(requiredPermissions, REQUEST_CODE_PERMISSIONS)
            return
        }

        val adapter = bluetoothAdapter
        if (adapter == null) {
            showStatus("当前设备不支持蓝牙")
            return
        }

        if (adapter.isEnabled) {
            action()
            return
        }

        pendingBluetoothEnableAction = action
        requestBluetoothEnable.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
    }

    private fun startScanInternal() {
        val scanner = bluetoothAdapter?.bluetoothLeScanner
        if (scanner == null) {
            showStatus("蓝牙扫描器不可用")
            return
        }

        clearScannedHistoryForNewScan()
        isScanning = true
        updateScanButtonText()
        showStatus("开始扫描（5秒）")

        runCatching {
            val settings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()
            scanner.startScan(null, settings, bleScanCallback)
        }.onFailure {
            isScanning = false
            updateScanButtonText()
            showStatus("蓝牙扫描启动失败: ${it.message}")
            return
        }

        xsensManager.startScan()

        scanHandler.removeCallbacks(scanStopRunnable)
        scanHandler.postDelayed(scanStopRunnable, SCAN_WINDOW_MS)
    }

    private fun clearScannedHistoryForNewScan() {
        val iterator = sensorMap.entries.iterator()
        while (iterator.hasNext()) {
            val sensor = iterator.next().value
            if (!sensor.connected && !sensor.connecting) {
                iterator.remove()
            }
        }
        if (selectedSensorAddress?.let { sensorMap[it] } == null) {
            selectedSensorAddress = null
        }
        refreshDeviceList()
    }

    private fun stopScanInternal(announce: Boolean) {
        if (!isScanning) {
            return
        }
        runCatching {
            bluetoothAdapter?.bluetoothLeScanner?.stopScan(bleScanCallback)
        }
        xsensManager.stopScan()
        isScanning = false
        scanHandler.removeCallbacks(scanStopRunnable)
        updateScanButtonText()
        if (announce) {
            showStatus("扫描结束")
        }
    }

    private fun handleBleScanResult(result: ScanResult) {
        val address = result.device.address ?: return
        val nameRaw = readDeviceName(result)
        val type = classifyType(result, nameRaw) ?: return
        val displayName = decorateDisplayNameForType(nameRaw, type, address)
        upsertSensor(address, displayName, type, result.rssi)
    }

    private fun classifyType(result: ScanResult, nameRaw: String): SensorType? {
        val record = result.scanRecord
        val manufacturerData = record?.manufacturerSpecificData
        if (manufacturerData != null && manufacturerData.indexOfKey(MOVELLA_COMPANY_ID) >= 0) {
            return SensorType.XSENS_DOT
        }

        val hasInsoleService = hasInsoleService(record)
        val trimmedName = nameRaw.trim()
        val isInsoleV2ByName = trimmedName.uppercase(Locale.US).startsWith("NB-")

        // Only keep xsens_dot and insole_v2 in scan results.
        if (hasInsoleService && isInsoleV2ByName) {
            return SensorType.INSOLE_V2
        }
        if (isInsoleV2ByName) {
            return SensorType.INSOLE_V2
        }
        return null
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

    private fun decorateDisplayNameForType(rawName: String, type: SensorType, address: String): String {
        if (type != SensorType.INSOLE_V2) {
            return rawName
        }
        val side = inferInsoleV2Side(address) ?: return rawName
        if (rawName.contains("左脚") || rawName.contains("右脚")) {
            return rawName
        }
        val suffix = if (side == "Left") "左脚" else "右脚"
        return "$rawName $suffix"
    }

    private fun inferInsoleV2Side(address: String): String? {
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

    private fun readDeviceName(result: ScanResult): String {
        return result.scanRecord?.deviceName
            ?: result.device.name
            ?: getString(R.string.collection_device_unknown)
    }

    private fun upsertSensor(address: String, rawName: String, type: SensorType, rssi: Int) {
        val existing = sensorMap[address]
        if (existing == null) {
            val name = settingsStore.getSensorName(address, rawName)
            val sideHint = if (type == SensorType.INSOLE_V2) inferInsoleV2Side(address) else null
            sensorMap[address] = CollectionSensor(
                address = address,
                name = name,
                type = type,
                rssi = rssi,
                side = settingsStore.getSensorSide(address) ?: sideHint,
                bodyPart = settingsStore.getSensorBodyPart(address),
            )
        } else {
            existing.rssi = rssi
            if (existing.side.isNullOrBlank() && existing.type == SensorType.INSOLE_V2) {
                existing.side = inferInsoleV2Side(address)
            }
            if (existing.name.isBlank() || existing.name == getString(R.string.collection_device_unknown)) {
                existing.name = settingsStore.getSensorName(address, rawName)
            }
        }
        refreshDeviceList()
    }

    private fun connectInsoleSensor(sensor: CollectionSensor) {
        if (sensor.connecting || sensor.connected) {
            return
        }
        val adapter = bluetoothAdapter
        if (adapter == null) {
            showStatus("蓝牙不可用")
            return
        }

        val device = runCatching { adapter.getRemoteDevice(sensor.address) }.getOrNull()
        if (device == null) {
            showStatus("无法获取设备: ${sensor.address}")
            return
        }

        sensor.connecting = true
        refreshDeviceList()
        showStatus("连接设备: ${sensor.name}")

        val callback = InsoleGattCallback(sensor.address)
        val gatt = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            device.connectGatt(this, false, callback, BluetoothDevice.TRANSPORT_LE)
        } else {
            @Suppress("DEPRECATION")
            device.connectGatt(this, false, callback)
        }
        sensor.gatt = gatt
    }

    private fun disconnectAllSensors() {
        val addresses = sensorMap.keys.toList()
        addresses.forEach { address ->
            disconnectSensor(address, "断开所有设备")
        }
    }

    private fun disconnectSensor(address: String, reason: String) {
        val sensor = sensorMap[address] ?: return

        if (sensor.type == SensorType.XSENS_DOT) {
            xsensManager.disconnect(address)
            sensor.connecting = false
            sensor.connected = false
            sensor.battery = null
        } else {
            sensor.connecting = false
            sensor.connected = false
            val gatt = sensor.gatt
            sensor.gatt = null
            runCatching {
                gatt?.disconnect()
                gatt?.close()
            }
            if (isCollecting) {
                loggerMap.remove(address)?.flush()
            }
            insoleV1Parser.clear(address)
            insoleV2Parser.clear(address)
        }

        if (isCollecting) {
            CollectionRealtimeBridge.removeDevice(address)
        }
        refreshDeviceList()
        showStatus("${sensor.name} 已断开 ($reason)")
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

    private fun startCollection(subjectNameRaw: String, gaitTypeRaw: String) {
        if (isCollecting) {
            showStatus("采集已在进行中")
            return
        }

        val user = selectedUser
        if (user == null) {
            showStatus("请先选择录制用户")
            return
        }

        val subject = sanitizeFilePart(subjectNameRaw.ifBlank { user.name })
        val gait = sanitizeFilePart(gaitTypeRaw)
        if (gait.isBlank() || gait == "unknown") {
            showStatus("请填写步态类型")
            return
        }

        val activeInsoleSensors = sensorMap.values.filter {
            it.connected && (it.type == SensorType.INSOLE || it.type == SensorType.INSOLE_V2)
        }
        val activeXsens = sensorMap.values.filter { it.connected && it.type == SensorType.XSENS_DOT }
        if (activeInsoleSensors.isEmpty() && activeXsens.isEmpty()) {
            showStatus("请先连接至少一个传感器")
            return
        }

        if (labelModeEnabled && labels.isEmpty()) {
            showStatus("请先创建至少一个标签")
            return
        }
        if (labelModeEnabled && activeLabel.isBlank()) {
            activeLabel = labels.firstOrNull() ?: DEFAULT_LABEL
        }

        val now = System.currentTimeMillis()
        val timeTag = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(Date(now))
        val dirName = "${subject}+${gait}+${timeTag}"
        val dir = File(getExternalFilesDir(null), "collection/$dirName")
        if (!dir.exists() && !dir.mkdirs()) {
            showStatus("创建日志目录失败: ${dir.absolutePath}")
            return
        }

        currentLogDir = dir
        packetCount = 0
        loggerMap.clear()

        activeInsoleSensors.forEach { sensor ->
            val safeName = sanitizeFilePart(sensor.name)
            val file = File(dir, "${safeName}+${subject}+${gait}+${timeTag}.csv")
            val logger = CsvBufferedLogger(file)
            logger.append("Side:,${sensor.side.orEmpty()},,,,,,,,,,,,,,,,,,\r\n")
            logger.append("StartTime:,${now},,,,,,,,,,,,,,,,,,\r\n")
            logger.append("\r\n")
            if (labelModeEnabled) {
                logger.append("Label,Timestamp,Channel,Count,Payload\r\n")
            } else {
                logger.append("Timestamp,Channel,Count,Payload\r\n")
            }
            logger.flush()
            loggerMap[sensor.address] = logger
        }

        configureXsensBodyPosition()

        val outputRate = settingsStore.getOutputRate().takeIf { outputRates.contains(it) } ?: 60
        val filterProfile = settingsStore.getFilterProfile().takeIf { filterProfiles.contains(it) }
            ?: XsensDotManager.FILTER_PROFILE_GENERAL
        val loggerFlag = settingsStore.getLoggerFlag().takeIf { loggerFlags.contains(it) }
            ?: XsensDotManager.LOGGER_FLAG_DEFAULT

        val appVersion = runCatching {
            packageManager.getPackageInfo(packageName, 0).versionName ?: "0.0.0"
        }.getOrDefault("0.0.0")

        val sensorNames = sensorMap.values.associate { it.address to it.name }
        val xsensStarted = xsensManager.startRecording(
            logDir = dir,
            subjectName = subject,
            gaitType = gait,
            recordingTimeTag = timeTag,
            outputRate = outputRate,
            filterProfile = filterProfile,
            sensorDisplayNames = sensorNames,
            appVersion = appVersion,
            loggerFlag = loggerFlag,
            labelModeEnabled = labelModeEnabled,
            labelProvider = { resolveCurrentLabelForLogging() },
        )

        if (loggerMap.isEmpty() && xsensStarted <= 0) {
            showStatus("没有可用传感器开始采集")
            return
        }

        isCollecting = true
        CollectionRealtimeBridge.configureCollectionState(active = true, startedAtMillis = now)
        updateCollectButtons()
        updatePacketCountView()
        if (labelModeEnabled) {
            syncLabelStateToBridgeIfNeeded()
            updateLabelSummaryView()
        }
        showStatus(
            "开始采集: insole=${activeInsoleSensors.size}, xsens=$xsensStarted, 目录=${dir.absolutePath}"
        )
        syncRealtimePlotDevices()
        openRealtimePlotPage()
    }

    private fun showStartCollectionDialog() {
        if (isCollecting) {
            showStatus("采集已在进行中")
            return
        }

        val user = selectedUser
        if (user == null) {
            showStatus("请先选择录制用户")
            return
        }

        val view = LayoutInflater.from(this).inflate(R.layout.dialog_collection_test_info, null)
        val subjectInput = view.findViewById<EditText>(R.id.collectSubjectInput)
        val gaitTypeInput = view.findViewById<EditText>(R.id.collectGaitTypeInput)

        val defaultSubject = if (lastSubjectName.isNotBlank()) lastSubjectName else user.name
        subjectInput.setText(defaultSubject)
        subjectInput.setSelection(subjectInput.text.length)
        gaitTypeInput.setText(lastGaitType)
        gaitTypeInput.setSelection(gaitTypeInput.text.length)

        val dialog = AlertDialog.Builder(this)
            .setTitle(getString(R.string.collection_test_info_title))
            .setView(view)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton(getString(R.string.collection_start), null)
            .create()

        dialog.setOnShowListener {
            val positive = dialog.getButton(AlertDialog.BUTTON_POSITIVE)
            positive.setOnClickListener {
                val subjectName = subjectInput.text.toString().trim()
                val gaitType = gaitTypeInput.text.toString().trim()
                if (subjectName.isBlank()) {
                    Toast.makeText(this, "请输入测试者姓名", Toast.LENGTH_SHORT).show()
                    return@setOnClickListener
                }
                if (gaitType.isBlank()) {
                    Toast.makeText(this, "请输入步态类型", Toast.LENGTH_SHORT).show()
                    return@setOnClickListener
                }

                lastSubjectName = subjectName
                lastGaitType = gaitType
                dialog.dismiss()
                startCollection(subjectNameRaw = subjectName, gaitTypeRaw = gaitType)
            }
        }
        dialog.show()
    }

    private fun configureXsensBodyPosition() {
        xsensManager.resetBodyPosition()

        val leftSensors = sensorMap.values.filter {
            it.connected && it.type == SensorType.XSENS_DOT && it.side == "Left"
        }
        val rightSensors = sensorMap.values.filter {
            it.connected && it.type == SensorType.XSENS_DOT && it.side == "Right"
        }

        if (leftSensors.isNotEmpty()) {
            xsensManager.setBodyPosition(
                side = "left",
                thighAddress = leftSensors.firstOrNull { it.bodyPart == "Thigh" }?.address,
                lowerLegAddress = leftSensors.firstOrNull { it.bodyPart == "Lower_Leg" }?.address,
                footAddress = leftSensors.firstOrNull { it.bodyPart == "Foot" }?.address,
            )
        }

        if (rightSensors.isNotEmpty()) {
            xsensManager.setBodyPosition(
                side = "right",
                thighAddress = rightSensors.firstOrNull { it.bodyPart == "Thigh" }?.address,
                lowerLegAddress = rightSensors.firstOrNull { it.bodyPart == "Lower_Leg" }?.address,
                footAddress = rightSensors.firstOrNull { it.bodyPart == "Foot" }?.address,
            )
        }
    }

    private fun stopCollection(reason: String, showSavePathPopup: Boolean = false) {
        if (!isCollecting && loggerMap.isEmpty()) {
            return
        }
        val savedDirPath = currentLogDir?.absolutePath
        isCollecting = false
        loggerMap.values.forEach { it.flush() }
        loggerMap.clear()
        insoleV1Parser.clearAll()
        insoleV2Parser.clearAll()
        xsensManager.stopRecording()
        CollectionRealtimeBridge.configureCollectionState(active = false, startedAtMillis = null)
        CollectionRealtimeBridge.configureLabelMode(false, emptyList(), null)
        CollectionRealtimeBridge.clear()
        updateCollectButtons()
        showStatus("已停止采集($reason)，累计写入: $packetCount")
        var displayPath = savedDirPath
        if (!savedDirPath.isNullOrBlank()) {
            val exportedPath = exportSessionCsvToDownloads(File(savedDirPath))
            if (!exportedPath.isNullOrBlank()) {
                displayPath = exportedPath
            }
        }
        if (showSavePathPopup && !displayPath.isNullOrBlank()) {
            showSavePathDialog(displayPath)
        }
        currentLogDir = null
    }

    private fun exportSessionCsvToDownloads(sessionDir: File): String? {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            showStatus("当前系统不支持自动导出到 Download，请使用弹窗路径手动查找")
            return null
        }
        val csvFiles = sessionDir.listFiles()
            ?.filter { it.isFile && it.name.endsWith(".csv", ignoreCase = true) }
            .orEmpty()
        if (csvFiles.isEmpty()) {
            return null
        }

        var successCount = 0
        csvFiles.forEach { file ->
            if (copyCsvToDownloads(sessionDir.name, file)) {
                successCount += 1
            }
        }
        val exportDisplayPath = "Download/collection/${sessionDir.name}"
        showStatus(
            "采集文件已导出: $successCount/${csvFiles.size} -> $exportDisplayPath"
        )
        return if (successCount > 0) exportDisplayPath else null
    }

    private fun copyCsvToDownloads(sessionDirName: String, source: File): Boolean {
        val resolver = applicationContext.contentResolver
        val relativePath = "${Environment.DIRECTORY_DOWNLOADS}/collection/$sessionDirName"
        val values = ContentValues().apply {
            put(MediaStore.Downloads.DISPLAY_NAME, source.name)
            put(MediaStore.Downloads.MIME_TYPE, "text/csv")
            put(MediaStore.Downloads.RELATIVE_PATH, relativePath)
            put(MediaStore.Downloads.IS_PENDING, 1)
        }
        val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values) ?: return false
        return runCatching {
            resolver.openOutputStream(uri)?.use { output ->
                source.inputStream().use { input ->
                    input.copyTo(output)
                }
            } ?: error("打开 Download 输出流失败")
            val doneValues = ContentValues().apply {
                put(MediaStore.Downloads.IS_PENDING, 0)
            }
            resolver.update(uri, doneValues, null, null)
            true
        }.getOrElse {
            resolver.delete(uri, null, null)
            false
        }
    }

    private fun handleIncomingPacket(address: String, packet: ByteArray) {
        if (!isCollecting) {
            return
        }
        val sensor = sensorMap[address] ?: return
        val logger = loggerMap[address] ?: return

        try {
            when (sensor.type) {
                SensorType.INSOLE -> {
                    val parsed = insoleV1Parser.parse(address, packet) ?: return
                    val label = resolveCurrentLabelForLogging()
                    parsed.values.forEach { frame ->
                        val body = frame.joinToString(",") { value ->
                            value.toString(16).padStart(2, '0')
                        }
                        if (labelModeEnabled) {
                            logger.append("${escapeCsvField(label)},${parsed.timestamp},0,${parsed.count},$body\r\n")
                        } else {
                            logger.append("${parsed.timestamp},0,${parsed.count},$body\r\n")
                        }
                        packetCount += 1
                    }
                    val lastFrame = parsed.values.lastOrNull().orEmpty()
                    if (lastFrame.isNotEmpty()) {
                        val pressureSum = lastFrame.sum().toFloat()
                        publishRealtimeSample(
                            address = sensor.address,
                            values = floatArrayOf(pressureSum),
                            legends = listOf("PressureSum"),
                        )
                    }
                }

                SensorType.INSOLE_V2 -> {
                    val parsed = insoleV2Parser.parse(address, packet) ?: return
                    val label = resolveCurrentLabelForLogging()
                    if (labelModeEnabled) {
                        logger.append("${escapeCsvField(label)},${parsed.timestamp},0,${parsed.count},${parsed.values.joinToString(",")}\r\n")
                    } else {
                        logger.append("${parsed.timestamp},0,${parsed.count},${parsed.values.joinToString(",")}\r\n")
                    }
                    packetCount += 1
                    val values = parsed.values.map { it.toFloat() }.toFloatArray()
                    val legends = List(parsed.values.size) { index -> "P${index + 1}" }
                    publishRealtimeSample(
                        address = sensor.address,
                        values = values,
                        legends = legends,
                    )
                }

                SensorType.XSENS_DOT -> Unit
            }
        } catch (e: Exception) {
            Log.e(TAG, "Write packet failed: ${sensor.address}", e)
            showStatus("写入失败: ${e.message}")
        }

        if (packetCount % 20L == 0L) {
            runOnUiThread { updatePacketCountView() }
        }
    }

    private fun updatePacketCountView() {
        packetCountView.text = getString(R.string.collection_packet_count, packetCount)
    }

    private fun updateScanButtonText() {
        scanButton.text = if (isScanning) {
            getString(R.string.collection_stop_scan)
        } else {
            getString(R.string.collection_start_scan)
        }
    }

    private fun updateCollectButtons() {
        startCollectButton.isEnabled = !isCollecting
        stopCollectButton.isEnabled = isCollecting
        openRealtimePlotButton.isEnabled = isCollecting
    }

    private fun openRealtimePlotPage() {
        startActivity(Intent(this, CollectionRealtimePlotActivity::class.java))
    }

    private fun showCreateLabelDialog() {
        if (!labelModeEnabled) {
            return
        }
        val view = LayoutInflater.from(this).inflate(R.layout.dialog_collection_label_manager, null)
        val labelListContainer = view.findViewById<LinearLayout>(R.id.labelListContainer)
        val emptyHintView = view.findViewById<TextView>(R.id.labelEmptyHintView)
        val addLabelButton = view.findViewById<Button>(R.id.addLabelButton)

        fun refreshLabelRows() {
            labelListContainer.removeAllViews()
            if (labels.isEmpty()) {
                emptyHintView.visibility = View.VISIBLE
                return
            }
            emptyHintView.visibility = View.GONE
            labels.forEach { label ->
                val rowView = LayoutInflater.from(this)
                    .inflate(R.layout.item_collection_label_manage, labelListContainer, false)
                val labelNameView = rowView.findViewById<TextView>(R.id.labelNameView)
                val editButton = rowView.findViewById<ImageButton>(R.id.editLabelButton)
                val deleteButton = rowView.findViewById<ImageButton>(R.id.deleteLabelButton)

                val isCurrent = label == activeLabel
                labelNameView.text = if (isCurrent) {
                    "$label (${getString(R.string.collection_label_current_suffix)})"
                } else {
                    label
                }

                editButton.setOnClickListener {
                    showLabelNameInputDialog(
                        title = getString(R.string.collection_label_edit_title),
                        initialName = label,
                    ) { updatedName ->
                        if (labels.any { it.equals(updatedName, ignoreCase = true) && it != label }) {
                            Toast.makeText(this, getString(R.string.collection_label_name_exists), Toast.LENGTH_SHORT).show()
                            return@showLabelNameInputDialog
                        }
                        val index = labels.indexOf(label)
                        if (index >= 0) {
                            labels[index] = updatedName
                            if (activeLabel == label) {
                                activeLabel = updatedName
                            }
                            updateLabelSummaryView()
                            syncLabelStateToBridgeIfNeeded()
                            refreshLabelRows()
                        }
                    }
                }

                deleteButton.setOnClickListener {
                    AlertDialog.Builder(this)
                        .setTitle(getString(R.string.collection_label_delete_title))
                        .setMessage(getString(R.string.collection_label_delete_message, label))
                        .setNegativeButton(android.R.string.cancel, null)
                        .setPositiveButton("删除") { _, _ ->
                            labels.remove(label)
                            if (activeLabel == label) {
                                activeLabel = labels.firstOrNull() ?: DEFAULT_LABEL
                            }
                            updateLabelSummaryView()
                            syncLabelStateToBridgeIfNeeded()
                            refreshLabelRows()
                        }
                        .show()
                }
                labelListContainer.addView(rowView)
            }
        }

        addLabelButton.setOnClickListener {
            showLabelNameInputDialog(
                title = getString(R.string.collection_label_add_title),
                initialName = "",
            ) { newLabel ->
                if (labels.any { it.equals(newLabel, ignoreCase = true) }) {
                    Toast.makeText(this, getString(R.string.collection_label_name_exists), Toast.LENGTH_SHORT).show()
                    return@showLabelNameInputDialog
                }
                labels.add(newLabel)
                if (activeLabel == DEFAULT_LABEL || activeLabel.isBlank()) {
                    activeLabel = newLabel
                }
                updateLabelSummaryView()
                syncLabelStateToBridgeIfNeeded()
                refreshLabelRows()
            }
        }

        refreshLabelRows()

        AlertDialog.Builder(this)
            .setTitle(getString(R.string.collection_create_label))
            .setView(view)
            .setPositiveButton("完成", null)
            .show()
    }

    private fun showLabelNameInputDialog(
        title: String,
        initialName: String,
        onConfirm: (String) -> Unit,
    ) {
        val input = EditText(this).apply {
            hint = getString(R.string.collection_label_name_hint)
            setText(initialName)
            setSelection(text.length)
        }
        AlertDialog.Builder(this)
            .setTitle(title)
            .setView(input)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton("保存") { _, _ ->
                val value = input.text.toString().trim()
                if (value.isBlank()) {
                    Toast.makeText(this, getString(R.string.collection_label_name_empty), Toast.LENGTH_SHORT).show()
                    return@setPositiveButton
                }
                onConfirm(value)
            }
            .show()
    }

    private fun syncLabelStateToBridgeIfNeeded() {
        if (!labelModeEnabled || !isCollecting) {
            return
        }
        CollectionRealtimeBridge.configureLabelMode(
            enabled = true,
            labels = labels.toList(),
            currentLabel = activeLabel,
        )
    }

    private fun updateLabelSummaryView() {
        if (!labelModeEnabled) {
            return
        }
        val summary = if (labels.isEmpty()) {
            getString(R.string.collection_label_summary_empty)
        } else {
            getString(
                R.string.collection_label_summary,
                labels.joinToString(" | "),
                activeLabel.ifBlank { DEFAULT_LABEL },
            )
        }
        labelSummaryView.text = summary
    }

    private fun resolveCurrentLabelForLogging(): String {
        if (!labelModeEnabled) {
            return ""
        }
        val bridgeLabel = CollectionRealtimeBridge.getCurrentLabel()
        if (!bridgeLabel.isNullOrBlank()) {
            activeLabel = bridgeLabel
        } else if (labels.isNotEmpty() && labels.none { it == activeLabel }) {
            activeLabel = labels.first()
        } else if (activeLabel.isBlank()) {
            activeLabel = labels.firstOrNull() ?: DEFAULT_LABEL
        }
        return activeLabel
    }

    private fun escapeCsvField(raw: String): String {
        if (raw.isEmpty()) return ""
        val needsQuote = raw.contains(',') || raw.contains('"') || raw.contains('\n') || raw.contains('\r')
        if (!needsQuote) return raw
        return "\"" + raw.replace("\"", "\"\"") + "\""
    }

    private fun showSavePathDialog(path: String) {
        if (isFinishing || isDestroyed) {
            return
        }
        popupHandler.removeCallbacks(dismissSavePathDialogRunnable)
        savePathDialog?.dismiss()
        savePathDialog = AlertDialog.Builder(this)
            .setTitle("文件保存位置")
            .setMessage(path)
            .create()
        savePathDialog?.setCanceledOnTouchOutside(false)
        savePathDialog?.show()
        popupHandler.postDelayed(dismissSavePathDialogRunnable, 3_000L)
    }

    private fun openSensorConfigPage() {
        val intent = Intent(this, CollectionSensorConfigActivity::class.java).apply {
            putExtra(CollectionSensorConfigActivity.EXTRA_OUTPUT_RATE, settingsStore.getOutputRate())
            putExtra(CollectionSensorConfigActivity.EXTRA_FILTER_PROFILE, settingsStore.getFilterProfile())
            putExtra(CollectionSensorConfigActivity.EXTRA_LOGGER_FLAG, settingsStore.getLoggerFlag())
        }
        sensorConfigLauncher.launch(intent)
    }

    private inner class CollectionDeviceAdapter : BaseAdapter() {
        override fun getCount(): Int = displayLabels.size

        override fun getItem(position: Int): Any? = displaySensors.getOrNull(position)

        override fun getItemId(position: Int): Long = position.toLong()

        override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
            val view = convertView ?: LayoutInflater.from(parent.context)
                .inflate(R.layout.item_collection_device, parent, false)
            val infoView = view.findViewById<TextView>(R.id.deviceInfoView)
            val toggleView = view.findViewById<SwitchCompat>(R.id.deviceConnectSwitch)

            infoView.text = displayLabels.getOrNull(position).orEmpty()
            val sensor = displaySensors.getOrNull(position)
            if (sensor == null) {
                toggleView.setOnCheckedChangeListener(null)
                view.setOnLongClickListener(null)
                infoView.setOnLongClickListener(null)
                toggleView.setOnLongClickListener(null)
                toggleView.isChecked = false
                toggleView.isEnabled = false
                toggleView.alpha = 0.35f
                toggleView.visibility = View.INVISIBLE
                return view
            }

            val openConfigByLongPress = View.OnLongClickListener {
                showSensorConfigDialog(sensor)
                true
            }
            view.setOnLongClickListener(openConfigByLongPress)
            infoView.setOnLongClickListener(openConfigByLongPress)
            toggleView.setOnLongClickListener(openConfigByLongPress)

            toggleView.visibility = View.VISIBLE
            toggleView.setOnCheckedChangeListener(null)
            toggleView.isChecked = sensor.connected
            toggleView.isEnabled = !sensor.connecting
            toggleView.alpha = if (sensor.connecting) 0.5f else 1f
            toggleView.text = when {
                sensor.connecting -> "连接中"
                sensor.connected -> "已连接"
                else -> "连接"
            }
            toggleView.setOnCheckedChangeListener { _, isChecked ->
                onSensorToggleRequested(sensor, isChecked)
            }
            return view
        }
    }

    private fun refreshDeviceList() {
        val sensors = sensorMap.values.sortedWith(
            compareBy<CollectionSensor> { it.type.ordinal }
                .thenBy { it.name }
                .thenBy { it.address }
        )

        displaySensors.clear()
        displayLabels.clear()

        if (sensors.isEmpty()) {
            displayLabels.add(getString(R.string.collection_empty_list))
            listAdapter.notifyDataSetChanged()
            deviceListView.post { updateDeviceListHeight() }
            return
        }

        sensors.forEach { sensor ->
            displaySensors.add(sensor)
            displayLabels.add(buildDisplayLabel(sensor))
        }
        listAdapter.notifyDataSetChanged()
        deviceListView.post { updateDeviceListHeight() }
    }

    private fun updateDeviceListHeight() {
        val adapter = deviceListView.adapter ?: return
        if (adapter.count <= 0) {
            return
        }

        val width = if (deviceListView.width > 0) {
            deviceListView.width
        } else {
            resources.displayMetrics.widthPixels - deviceListView.paddingLeft - deviceListView.paddingRight
        }
        val widthSpec = View.MeasureSpec.makeMeasureSpec(width, View.MeasureSpec.AT_MOST)
        val heightSpec = View.MeasureSpec.makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED)

        var totalHeight = 0
        for (index in 0 until adapter.count) {
            val itemView = adapter.getView(index, null, deviceListView)
            itemView.measure(widthSpec, heightSpec)
            totalHeight += itemView.measuredHeight
        }

        val dividerHeight = deviceListView.dividerHeight * (adapter.count - 1).coerceAtLeast(0)
        val targetHeight = totalHeight + dividerHeight
        if (targetHeight <= 0) {
            return
        }

        val params = deviceListView.layoutParams
        if (params.height != targetHeight) {
            params.height = targetHeight
            deviceListView.layoutParams = params
            deviceListView.requestLayout()
        }
    }

    private fun buildDisplayLabel(sensor: CollectionSensor): String {
        val typeText = sensorTypeLabel(sensor.type)
        val statusText = when {
            sensor.connecting -> "连接中"
            sensor.connected -> "已连接"
            else -> "未连接"
        }
        val meta = buildString {
            append("RSSI ${sensor.rssi}")
            sensor.battery?.let { append(" | 电量 ${it}%") }
            sensor.side?.let { append(" | Side=$it") }
            sensor.bodyPart?.let { append(" | Body=$it") }
        }
        return "[$typeText] ${sensor.name}\n${sensor.address}\n$meta | $statusText"
    }

    private fun sensorTypeLabel(type: SensorType): String {
        return when (type) {
            SensorType.XSENS_DOT -> "xsens_dot"
            SensorType.INSOLE -> "insole"
            SensorType.INSOLE_V2 -> "insole_v2"
        }
    }

    private fun syncRealtimePlotDevices() {
        if (!isCollecting) {
            return
        }
        val connectedDevices = sensorMap.values.filter { it.connected }.map { sensor ->
            CollectionRealtimeBridge.DeviceMeta(
                address = sensor.address,
                name = sensor.name,
                type = sensorTypeLabel(sensor.type),
                side = sensor.side,
            )
        }
        CollectionRealtimeBridge.setConnectedDevices(connectedDevices)
    }

    private fun publishRealtimeSample(address: String, values: FloatArray, legends: List<String>) {
        if (!isCollecting || values.isEmpty()) {
            return
        }
        val sensor = sensorMap[address] ?: return
        CollectionRealtimeBridge.publishSample(
            address = sensor.address,
            name = sensor.name,
            type = sensorTypeLabel(sensor.type),
            side = sensor.side,
            values = values,
            legends = legends,
        )
    }

    private fun sanitizeFilePart(raw: String): String {
        return raw.trim()
            .replace("\\s+".toRegex(), "_")
            .replace("[\\\\/:*?\"<>|]".toRegex(), "_")
            .trim('_')
            .ifBlank { "unknown" }
    }

    private fun showStatus(text: String) {
        runOnUiThread {
            statusView.text = "状态: $text"
        }
    }

    private fun hasAllRequiredPermissions(): Boolean {
        return requiredPermissions.all { permission ->
            ContextCompat.checkSelfPermission(this, permission) == PackageManager.PERMISSION_GRANTED
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQUEST_CODE_PERMISSIONS) {
            return
        }
        val granted = grantResults.isNotEmpty() && grantResults.all { it == PackageManager.PERMISSION_GRANTED }
        if (granted) {
            pendingPermissionAction?.invoke()
        } else {
            Toast.makeText(this, "缺少蓝牙权限，无法扫描采集设备", Toast.LENGTH_SHORT).show()
            showStatus("权限未授予")
        }
        pendingPermissionAction = null
    }

    override fun onDeviceFound(address: String, name: String, rssi: Int) {
        runOnUiThread {
            upsertSensor(address, name, SensorType.XSENS_DOT, rssi)
        }
    }

    override fun onConnectionChanged(address: String, state: Int) {
        runOnUiThread {
            val sensor = sensorMap[address] ?: return@runOnUiThread
            when (state) {
                XsensDotDevice.CONN_STATE_CONNECTED -> {
                    sensor.connecting = false
                    sensor.connected = true
                }
                XsensDotDevice.CONN_STATE_CONNECTING,
                XsensDotDevice.CONN_STATE_RECONNECTING,
                XsensDotDevice.CONN_STATE_START_RECONNECTING,
                -> {
                    sensor.connecting = true
                    sensor.connected = false
                }
                else -> {
                    sensor.connecting = false
                    sensor.connected = false
                    sensor.battery = null
                }
            }
            refreshDeviceList()
            if (isCollecting) {
                syncRealtimePlotDevices()
            }
        }
    }

    override fun onBatteryChanged(address: String, battery: Int) {
        runOnUiThread {
            sensorMap[address]?.battery = battery
            refreshDeviceList()
        }
    }

    override fun onTelemetry(address: String, telemetry: XsensDotManager.XsensTelemetry) {
        if (isCollecting) {
            packetCount += 1
            val freeAcc = telemetry.freeAcc
            if (freeAcc.size >= 3) {
                publishRealtimeSample(
                    address = address,
                    values = floatArrayOf(freeAcc[0].toFloat(), freeAcc[1].toFloat(), freeAcc[2].toFloat()),
                    legends = listOf("FreeAccX", "FreeAccY", "FreeAccZ"),
                )
            }
            if (packetCount % 20L == 0L) {
                runOnUiThread { updatePacketCountView() }
            }
        }
    }

    override fun onSyncProgress(progress: Int) {
        runOnUiThread {
            syncStatusView.text = getString(R.string.collection_sync_progress, progress)
        }
    }

    override fun onSyncDone(success: Boolean) {
        runOnUiThread {
            syncStatusView.text = if (success) {
                getString(R.string.collection_sync_status_done)
            } else {
                getString(R.string.collection_sync_status_not)
            }
        }
    }

    override fun onJointAngles(side: String, hip: DoubleArray?, knee: DoubleArray?, ankle: DoubleArray?) {
        // Data preview block removed from this page; keep callback for compatibility.
    }

    override fun onStatus(message: String) {
        showStatus(message)
    }

    private inner class InsoleGattCallback(private val address: String) : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            val sensor = sensorMap[address] ?: return
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                runOnUiThread {
                    sensor.connecting = true
                    sensor.connected = false
                    sensor.gatt = gatt
                    refreshDeviceList()
                    showStatus("${sensor.name} 已连接，发现服务中")
                }
                gatt.discoverServices()
                return
            }

            if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                runOnUiThread {
                    sensor.connecting = false
                    sensor.connected = false
                    sensor.gatt = null
                    loggerMap.remove(address)?.flush()
                    insoleV1Parser.clear(address)
                    insoleV2Parser.clear(address)
                    refreshDeviceList()
                    if (isCollecting) {
                        syncRealtimePlotDevices()
                    }
                    showStatus("${sensor.name} 已断开")
                }
                runCatching { gatt.close() }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            val sensor = sensorMap[address] ?: return
            if (status != BluetoothGatt.GATT_SUCCESS) {
                runOnUiThread {
                    showStatus("${sensor.name} 服务发现失败: $status")
                    disconnectSensor(address, "服务发现失败")
                }
                return
            }

            val insoleService = gatt.getService(UUID.fromString(INSOLE_SERVICE_UUID))
            if (insoleService == null) {
                runOnUiThread {
                    showStatus("${sensor.name} 未找到 insole 服务")
                    disconnectSensor(address, "缺少FFF0服务")
                }
                return
            }

            if (sensor.type == SensorType.INSOLE_V2) {
                runCatching { gatt.requestMtu(INSOLE_TARGET_MTU) }
                    .onFailure { Log.w(TAG, "requestMtu failed for ${sensor.address}: ${it.message}") }
            }

            val enabled = enableInsoleNotification(gatt, insoleService)
            runOnUiThread {
                if (enabled) {
                    sensor.connecting = false
                    sensor.connected = true
                    sensor.gatt = gatt
                    refreshDeviceList()
                    if (isCollecting) {
                        syncRealtimePlotDevices()
                    }
                    showStatus("${sensor.name} 已订阅数据通知")
                } else {
                    showStatus("${sensor.name} 订阅通知失败")
                    disconnectSensor(address, "通知订阅失败")
                }
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
        ) {
            if (normalizeUuid(characteristic.uuid.toString()) != normalizeUuid(INSOLE_READ_UUID)) {
                return
            }
            handleIncomingPacket(address, value)
        }

        @Deprecated("Deprecated in API 33")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
        ) {
            if (normalizeUuid(characteristic.uuid.toString()) != normalizeUuid(INSOLE_READ_UUID)) {
                return
            }
            val packet = characteristic.value ?: return
            handleIncomingPacket(address, packet)
        }
    }

    private fun showSensorConfigDialog(sensor: CollectionSensor) {
        val view = LayoutInflater.from(this).inflate(R.layout.dialog_sensor_config, null)
        val nameInput = view.findViewById<EditText>(R.id.configNameInput)
        val sideSpinner = view.findViewById<Spinner>(R.id.configSideSpinner)
        val bodyPartSpinner = view.findViewById<Spinner>(R.id.configBodyPartSpinner)

        val sideValues = listOf("", "Left", "Right")
        val bodyValues = listOf("", "Thigh", "Lower_Leg", "Foot")

        nameInput.setText(sensor.name)

        sideSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            listOf("未设置", "Left", "Right")
        )
        bodyPartSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            listOf("未设置", "Thigh", "Lower_Leg", "Foot")
        )

        sideSpinner.setSelection(sideValues.indexOf(sensor.side).coerceAtLeast(0))
        bodyPartSpinner.setSelection(bodyValues.indexOf(sensor.bodyPart).coerceAtLeast(0))
        bodyPartSpinner.isEnabled = sensor.type == SensorType.XSENS_DOT

        AlertDialog.Builder(this)
            .setTitle("设备配置")
            .setView(view)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton("保存") { _, _ ->
                val newName = nameInput.text.toString().trim().ifBlank { sensor.name }
                val newSide = sideValues.getOrNull(sideSpinner.selectedItemPosition).orEmpty().ifBlank { null }
                val newBody = bodyValues.getOrNull(bodyPartSpinner.selectedItemPosition).orEmpty().ifBlank { null }

                sensor.name = newName
                sensor.side = newSide
                sensor.bodyPart = if (sensor.type == SensorType.XSENS_DOT) newBody else null

                settingsStore.setSensorName(sensor.address, sensor.name)
                settingsStore.setSensorSide(sensor.address, sensor.side)
                if (sensor.type == SensorType.XSENS_DOT) {
                    settingsStore.setSensorBodyPart(sensor.address, sensor.bodyPart)
                } else {
                    settingsStore.setSensorBodyPart(sensor.address, null)
                }

                if (sensor.connected && isCollecting) {
                    CollectionRealtimeBridge.upsertDevice(
                        address = sensor.address,
                        name = sensor.name,
                        type = sensorTypeLabel(sensor.type),
                        side = sensor.side,
                    )
                }

                refreshDeviceList()
                showStatus("已保存设备配置: ${sensor.name}")
            }
            .show()
    }

    private fun showUserPickerDialog() {
        val users = userDb.getAllUsers()
        if (users.isEmpty()) {
            Toast.makeText(this, "暂无用户，请先创建", Toast.LENGTH_SHORT).show()
            return
        }

        val labels = users.map {
            "${it.name} | ${if (it.gender == "M") "男" else "女"} | ${it.height}cm ${it.weight}kg"
        }.toTypedArray()

        AlertDialog.Builder(this)
            .setTitle("选择录制用户")
            .setItems(labels) { _, which ->
                selectedUser = users.getOrNull(which)
                selectedUser?.let { user ->
                    lastSubjectName = user.name
                }
                updateSelectedUserView()
            }
            .show()
    }

    private fun showUserManagerDialog() {
        val users = userDb.getAllUsers()
        if (users.isEmpty()) {
            Toast.makeText(this, "暂无用户，请先创建", Toast.LENGTH_SHORT).show()
            return
        }

        val labels = users.map {
            "${it.name} | ${if (it.gender == "M") "男" else "女"} | ${it.birthYear}/${it.birthMonth}"
        }.toTypedArray()

        AlertDialog.Builder(this)
            .setTitle("用户列表")
            .setItems(labels) { _, which ->
                val user = users.getOrNull(which) ?: return@setItems
                AlertDialog.Builder(this)
                    .setTitle(user.name)
                    .setItems(arrayOf("选择为录制用户", "编辑", "删除")) { _, action ->
                        when (action) {
                            0 -> {
                                selectedUser = user
                                lastSubjectName = user.name
                                updateSelectedUserView()
                            }
                            1 -> showUserEditorDialog(user)
                            2 -> {
                                AlertDialog.Builder(this)
                                    .setTitle("确认删除")
                                    .setMessage("确定删除用户 ${user.name} ?")
                                    .setNegativeButton(android.R.string.cancel, null)
                                    .setPositiveButton("删除") { _, _ ->
                                        userDb.deleteUser(user.id)
                                        if (selectedUser?.id == user.id) {
                                            selectedUser = null
                                            updateSelectedUserView()
                                        }
                                    }
                                    .show()
                            }
                        }
                    }
                    .show()
            }
            .show()
    }

    private fun showUserEditorDialog(editingUser: CollectionUser?) {
        val view = LayoutInflater.from(this).inflate(R.layout.dialog_user_form, null)
        val nameInput = view.findViewById<EditText>(R.id.userNameInput)
        val genderSpinner = view.findViewById<Spinner>(R.id.userGenderSpinner)
        val birthYearInput = view.findViewById<EditText>(R.id.userBirthYearInput)
        val birthMonthInput = view.findViewById<EditText>(R.id.userBirthMonthInput)
        val heightInput = view.findViewById<EditText>(R.id.userHeightInput)
        val weightInput = view.findViewById<EditText>(R.id.userWeightInput)

        genderSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            listOf("男", "女")
        )

        editingUser?.let { u ->
            nameInput.setText(u.name)
            genderSpinner.setSelection(if (u.gender == "M") 0 else 1)
            birthYearInput.setText(u.birthYear.toString())
            birthMonthInput.setText(u.birthMonth.toString())
            heightInput.setText(u.height.toString())
            weightInput.setText(u.weight.toString())
        }

        AlertDialog.Builder(this)
            .setTitle(if (editingUser == null) "创建用户" else "编辑用户")
            .setView(view)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton("保存") { _, _ ->
                val name = nameInput.text.toString().trim()
                val gender = if (genderSpinner.selectedItemPosition == 0) "M" else "W"
                val birthYear = birthYearInput.text.toString().toIntOrNull() ?: 0
                val birthMonth = birthMonthInput.text.toString().toIntOrNull() ?: 0
                val height = heightInput.text.toString().toIntOrNull() ?: 0
                val weight = weightInput.text.toString().toIntOrNull() ?: 0

                if (name.isBlank() || birthYear !in 1900..2100 || birthMonth !in 1..12 || height <= 0 || weight <= 0) {
                    Toast.makeText(this, "请填写合法用户信息", Toast.LENGTH_SHORT).show()
                    return@setPositiveButton
                }

                runCatching {
                    if (editingUser == null) {
                        userDb.insertUser(name, gender, birthYear, birthMonth, height, weight)
                    } else {
                        userDb.updateUser(
                            editingUser.copy(
                                name = name,
                                gender = gender,
                                birthYear = birthYear,
                                birthMonth = birthMonth,
                                height = height,
                                weight = weight,
                            )
                        )
                    }
                    Toast.makeText(this, "用户保存成功", Toast.LENGTH_SHORT).show()
                }.onFailure {
                    Toast.makeText(this, "用户保存失败: ${it.message}", Toast.LENGTH_SHORT).show()
                }
            }
            .show()
    }

    private fun updateSelectedUserView() {
        selectedUserView.text = selectedUser?.let {
            getString(R.string.collection_selected_user_value, it.name)
        } ?: getString(R.string.collection_selected_user_empty)
    }

    private fun filterProfileLabel(profile: Int): String {
        return when (profile) {
            XsensDotManager.FILTER_PROFILE_GENERAL -> "General"
            XsensDotManager.FILTER_PROFILE_DYNAMIC -> "Dynamic"
            else -> "Unknown"
        }
    }

    private fun loggerFlagLabel(flag: Int): String {
        return collectionLoggerFlagLabel(flag, includePrefix = true)
    }
}
