package com.example.mobile

import android.Manifest
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.BitmapFactory
import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
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
import android.widget.TextView
import android.widget.Toast
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.lifecycle.MutableLiveData
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import com.rokid.cxr.client.utils.ValueUtil

class MainActivity : AppCompatActivity() {
    companion object {
        private const val TAG = "MainActivity"
        private const val REQUEST_CODE_PERMISSIONS = 100

        private const val RELAY_WIDTH = 320
        private const val RELAY_HEIGHT = 240
        private const val RELAY_QUALITY = 70
        private const val RELAY_INTERVAL_MS = 1000L
        private const val BATTERY_REFRESH_INTERVAL_MS = 15000L

        private const val CONTROL_CHANNEL = "imu_capture_control"
        private const val RESULT_CHANNEL = "imu_capture_result"

        private const val IMU_SLOT_COUNT = 2
        private const val IMU_SETTINGS_PREFS = "imu_connection_manager"
        private const val IMU_CONNECT_TIMEOUT_MS = 12000L
        private const val IMU_SLOT_WALKING = "walking"
        private const val IMU_SLOT_CYCLING = "cycling"
        private const val STATE_FLAG_IMU_WALK_CONNECTED = 16
        private const val STATE_FLAG_IMU_CYCLE_CONNECTED = 20
    }

    private data class ImuSlotState(
        val index: Int,
        var name: String,
        var address: String? = null,
        var connecting: Boolean = false,
        var connected: Boolean = false,
        var battery: Int? = null,
    )

    private data class ImuSlotViews(
        val nameInput: EditText,
        val connectSwitch: SwitchCompat,
        val addressView: TextView,
    )

    private data class InlineDeviceRow(
        val label: String,
        val checked: Boolean,
        val enabled: Boolean,
        val toggleText: String,
        val onToggleRequested: (Boolean) -> Unit,
    )

    private val requiredPermissions: Array<String>
        get() = mutableListOf(
            Manifest.permission.ACCESS_FINE_LOCATION
        ).apply {
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

    private val permissionGrantedResult = MutableLiveData<Boolean?>()

    private lateinit var statusView: TextView
    private lateinit var batteryPercentView: TextView
    private lateinit var glassesScanButton: Button
    private lateinit var gaitScanButton: Button
    private lateinit var insoleScanButton: Button
    private lateinit var imuManageButton: Button
    private lateinit var gaitDevicesEmptyView: TextView
    private lateinit var gaitDevicesContainer: LinearLayout
    private lateinit var glassesDevicesEmptyView: TextView
    private lateinit var glassesDevicesContainer: LinearLayout
    private lateinit var insoleDevicesEmptyView: TextView
    private lateinit var insoleDevicesContainer: LinearLayout

    private lateinit var rokidBluetoothConnector: RokidBluetoothConnector
    private lateinit var gaitBluetoothConnector: GaitBluetoothConnector
    private lateinit var insoleV2Manager: InsoleV2Manager
    private lateinit var roadConditionAnalyzer: RoadConditionAnalyzer

    private var scanDialog: AlertDialog? = null
    private var scanDialogAdapter: ArrayAdapter<String>? = null
    private val scanDialogLabels = mutableListOf<String>()
    private val scanDialogDevices = mutableListOf<RokidBluetoothConnector.ScannedDevice>()
    private var gaitScanDialog: AlertDialog? = null
    private var gaitScanDialogAdapter: ArrayAdapter<String>? = null
    private val gaitScanDialogLabels = mutableListOf<String>()
    private val gaitScanDialogDevices = mutableListOf<GaitBluetoothConnector.ScannedDevice>()
    private var imuManageDialog: AlertDialog? = null
    private var imuManageViews: List<ImuSlotViews>? = null
    private val imuConnectTimeoutTasks = mutableMapOf<Int, Runnable>()
    private var insoleConnectDialog: AlertDialog? = null
    private var insoleConnectDialogAdapter: InsoleConnectDeviceAdapter? = null
    private val insoleConnectDevices = mutableListOf<InsoleV2Manager.DeviceItem>()
    private var insoleConnectEmptyView: TextView? = null
    private var pendingBluetoothEnableAction: (() -> Unit)? = null
    private val rokidInlineDevices = mutableListOf<RokidBluetoothConnector.ScannedDevice>()
    private val gaitInlineDevices = mutableListOf<GaitBluetoothConnector.ScannedDevice>()
    private val insoleInlineDevices = mutableListOf<InsoleV2Manager.DeviceItem>()
    private var hasRequestedGlassesScan = false
    private var hasRequestedGaitScan = false
    private var hasRequestedInsoleScan = false

    private val imuSlots = mutableListOf<ImuSlotState>()
    private val imuLastErrorNotice = mutableMapOf<Int, String>()
    private val imuPrefs by lazy { getSharedPreferences(IMU_SETTINGS_PREFS, MODE_PRIVATE) }

    private val relayHandler = Handler(Looper.getMainLooper())
    private val batteryRefreshHandler = Handler(Looper.getMainLooper())
    @Volatile private var isRelayRunning = false
    @Volatile private var isCaptureInFlight = false
    @Volatile private var isAnalysisInFlight = false
    private var frameIndex = 0L

    private val relayRunnable = object : Runnable {
        override fun run() {
            if (!isRelayRunning) {
                return
            }
            if (!rokidBluetoothConnector.isBluetoothConnected()) {
                showStatus("蓝牙通信断开，等待重连后继续")
                relayHandler.postDelayed(this, RELAY_INTERVAL_MS)
                return
            }
            if (isCaptureInFlight || isAnalysisInFlight) {
                relayHandler.postDelayed(this, 200L)
                return
            }

            isCaptureInFlight = true
            val requestStatus = rokidBluetoothConnector.takePhoto(
                RELAY_WIDTH,
                RELAY_HEIGHT,
                RELAY_QUALITY
            ) { status, photo ->
                isCaptureInFlight = false
                if (!isRelayRunning) {
                    return@takePhoto
                }
                if (status == ValueUtil.CxrStatus.RESPONSE_SUCCEED && photo != null && photo.isNotEmpty()) {
                    frameIndex += 1
                    analyzeAndSendResult(photo, frameIndex)
                    showStatus("采集中: 第${frameIndex}帧 ${RELAY_WIDTH}x${RELAY_HEIGHT} @1Hz")
                } else {
                    showStatus("拍照回调失败: ${status?.name ?: "UNKNOWN"}")
                }
            }

            if (requestStatus != ValueUtil.CxrStatus.REQUEST_SUCCEED) {
                isCaptureInFlight = false
                showStatus("拍照请求失败: ${requestStatus.name}")
            }

            relayHandler.postDelayed(this, RELAY_INTERVAL_MS)
        }
    }

    private val batteryRefreshRunnable = object : Runnable {
        override fun run() {
            if (rokidBluetoothConnector.isBluetoothConnected()) {
                requestBatteryInfo()
            } else {
                updateBatteryIndicator(null)
            }
            batteryRefreshHandler.postDelayed(this, BATTERY_REFRESH_INTERVAL_MS)
        }
    }

    private val requestBluetoothEnable = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            showStatus("蓝牙已打开")
            pendingBluetoothEnableAction?.invoke()
        } else {
            showStatus("蓝牙未打开，无法连接设备")
        }
        pendingBluetoothEnableAction = null
    }

    private val gaitStatusListener: (String) -> Unit = { message ->
        showStatus(message)
        Log.i(TAG, message)
        if (message.startsWith("已连接:") || message.startsWith("BLE已连接")) {
            runOnUiThread {
                stopOtherBluetoothScansForGait()
            }
        }
        runOnUiThread {
            renderGaitDeviceList()
        }
    }
    private val gaitLineListener: (String) -> Unit = { line ->
        handleGaitImuMessage(line)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_main)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }

        statusView = findViewById(R.id.statusView)
        batteryPercentView = findViewById(R.id.batteryPercentView)
        glassesScanButton = findViewById(R.id.glassesScanButton)
        gaitScanButton = findViewById(R.id.gaitScanButton)
        insoleScanButton = findViewById(R.id.insoleScanButton)
        imuManageButton = findViewById(R.id.imuManageButton)
        gaitDevicesEmptyView = findViewById(R.id.gaitDevicesEmptyView)
        gaitDevicesContainer = findViewById(R.id.gaitDevicesContainer)
        glassesDevicesEmptyView = findViewById(R.id.glassesDevicesEmptyView)
        glassesDevicesContainer = findViewById(R.id.glassesDevicesContainer)
        insoleDevicesEmptyView = findViewById(R.id.insoleDevicesEmptyView)
        insoleDevicesContainer = findViewById(R.id.insoleDevicesContainer)
        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            ExoBottomNav.openHome(this)
        }
        updateBatteryIndicator(null)
        ExoBottomNav.setup(this, ExoDestination.BLUETOOTH)

        rokidBluetoothConnector = RokidBluetoothConnector(this) { message ->
            handleRokidStatusMessage(message)
        }
        rokidBluetoothConnector.setCustomMessageListener { channel, payload ->
            if (channel != CONTROL_CHANNEL || payload == null) {
                return@setCustomMessageListener
            }
            val raw = payload.toString(Charsets.UTF_8).trim()
            if (raw.isEmpty()) {
                return@setCustomMessageListener
            }
            Log.i(TAG, "Receive control message channel=$channel value=$raw")
            runOnUiThread {
                handleControlMessage(raw)
            }
        }
        gaitBluetoothConnector = GaitBluetoothBridge.getConnector(this)
        insoleV2Manager = InsoleV2Manager(
            context = applicationContext,
            statusCallback = { message ->
                showStatus(message)
                Log.i(TAG, message)
                runOnUiThread {
                    renderInsoleDeviceList()
                }
            },
            sampleCallback = { sample ->
                GaitBluetoothBridge.onInsoleV2Sample(
                    address = sample.address,
                    deviceName = sample.displayName,
                    insoleTimestamp = sample.timestamp,
                    footId = sample.footId,
                    values = sample.values
                )
            }
        )
        roadConditionAnalyzer = RoadConditionAnalyzer(applicationContext)
        initializeImuSlots()

        glassesScanButton.setOnClickListener {
            toggleGlassesScan()
        }
        gaitScanButton.setOnClickListener {
            toggleGaitScan()
        }
        insoleScanButton.setOnClickListener {
            toggleInsoleScan()
        }
        imuManageButton.setOnClickListener {
            openImuManageDialog()
        }
        setActionButtonsEnabled(false)
        renderRokidDeviceList()
        renderGaitDeviceList()
        renderInsoleDeviceList()

        permissionGrantedResult.observe(this) { granted ->
            when (granted) {
                true -> {
                    Log.i(TAG, "All required permissions granted.")
                    showStatus("权限已通过，请点击右侧扫描按钮连接设备")
                    setActionButtonsEnabled(true)
                }
                false -> {
                    setActionButtonsEnabled(false)
                    stopBatteryRefreshLoop()
                    updateBatteryIndicator(null)
                    Toast.makeText(
                        this,
                        "缺少必要权限，蓝牙与传感器功能将不可用",
                        Toast.LENGTH_SHORT
                    ).show()
                }
                null -> Unit
            }
        }

        requestCxrPermissions()
    }

    override fun onStart() {
        super.onStart()
        GaitBluetoothBridge.addStatusListener(gaitStatusListener)
        GaitBluetoothBridge.addLineListener(gaitLineListener)
    }

    override fun onStop() {
        GaitBluetoothBridge.removeStatusListener(gaitStatusListener)
        GaitBluetoothBridge.removeLineListener(gaitLineListener)
        super.onStop()
    }

    override fun onDestroy() {
        scanDialog?.dismiss()
        rokidBluetoothConnector.stopScan()
        gaitScanDialog?.dismiss()
        gaitBluetoothConnector.stopScan()
        insoleConnectDialog?.dismiss()
        insoleV2Manager.release()
        imuManageDialog?.dismiss()
        imuConnectTimeoutTasks.values.forEach { relayHandler.removeCallbacks(it) }
        imuConnectTimeoutTasks.clear()
        super.onDestroy()
        stopRelayPipeline("页面销毁")
        stopBatteryRefreshLoop()
        rokidBluetoothConnector.setCustomMessageListener(null)
        rokidBluetoothConnector.release()
        rokidBluetoothConnector.deinitBluetooth()
    }

    private fun stopOtherBluetoothScansForGait() {
        if (rokidBluetoothConnector.isScanning()) {
            rokidBluetoothConnector.stopScan()
        }
        if (gaitBluetoothConnector.isScanning()) {
            gaitBluetoothConnector.stopScan()
        }
        if (insoleV2Manager.isScanning()) {
            insoleV2Manager.stopScan(clearCallback = false)
        }
    }

    private fun requestCxrPermissions() {
        if (hasAllRequiredPermissions()) {
            permissionGrantedResult.postValue(true)
            return
        }
        permissionGrantedResult.postValue(null)
        requestPermissions(requiredPermissions, REQUEST_CODE_PERMISSIONS)
    }

    private fun ensureBluetoothEnabledThenScanGlasses() {
        if (!hasAllRequiredPermissions()) {
            showStatus("请先授予蓝牙相关权限")
            requestCxrPermissions()
            return
        }
        startGlassesScan()
    }

    private fun ensureBluetoothEnabledThenScanGait() {
        if (!hasAllRequiredPermissions()) {
            showStatus("请先授予蓝牙相关权限")
            requestCxrPermissions()
            return
        }
        if (!gaitBluetoothConnector.hasBluetoothAdapter()) {
            showStatus("当前设备不支持蓝牙")
            return
        }
        if (gaitBluetoothConnector.isBluetoothEnabled()) {
            startGaitScan()
            return
        }
        pendingBluetoothEnableAction = { startGaitScan() }
        requestBluetoothEnable.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
    }

    private fun ensureBluetoothEnabledThenScanInsole() {
        if (!hasAllRequiredPermissions()) {
            showStatus("请先授予蓝牙相关权限")
            requestCxrPermissions()
            return
        }
        if (!insoleV2Manager.hasBluetoothAdapter()) {
            showStatus("当前设备不支持蓝牙")
            return
        }
        if (insoleV2Manager.isBluetoothEnabled()) {
            startInsoleScan()
            return
        }
        pendingBluetoothEnableAction = { startInsoleScan() }
        requestBluetoothEnable.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
    }

    private fun toggleGlassesScan() {
        if (rokidBluetoothConnector.isScanning()) {
            rokidBluetoothConnector.stopScan()
            showStatus("已停止扫描眼镜设备")
            renderRokidDeviceList()
            return
        }
        ensureBluetoothEnabledThenScanGlasses()
    }

    private fun toggleGaitScan() {
        if (gaitBluetoothConnector.isScanning()) {
            gaitBluetoothConnector.stopScan()
            showStatus("已停止扫描步态设备")
            renderGaitDeviceList()
            return
        }
        ensureBluetoothEnabledThenScanGait()
    }

    private fun toggleInsoleScan() {
        if (insoleV2Manager.isScanning()) {
            insoleV2Manager.stopScan(clearCallback = true)
            showStatus("已停止扫描压力鞋垫设备")
            renderInsoleDeviceList()
            return
        }
        ensureBluetoothEnabledThenScanInsole()
    }

    private fun startGlassesScan() {
        stopCompetingScans(target = "glasses")
        hasRequestedGlassesScan = true
        rokidInlineDevices.clear()
        renderRokidDeviceList()
        val started = rokidBluetoothConnector.startScanForSelection { devices ->
            runOnUiThread {
                rokidInlineDevices.clear()
                rokidInlineDevices.addAll(devices)
                renderRokidDeviceList()
            }
        }
        if (!started) {
            renderRokidDeviceList()
        }
    }

    private fun startGaitScan() {
        stopCompetingScans(target = "gait")
        hasRequestedGaitScan = true
        gaitInlineDevices.clear()
        renderGaitDeviceList()
        val started = gaitBluetoothConnector.startScanForSelection { devices ->
            runOnUiThread {
                gaitInlineDevices.clear()
                gaitInlineDevices.addAll(devices)
                renderGaitDeviceList()
            }
        }
        if (!started) {
            renderGaitDeviceList()
        }
    }

    private fun startInsoleScan() {
        stopCompetingScans(target = "insole")
        hasRequestedInsoleScan = true
        insoleInlineDevices.clear()
        renderInsoleDeviceList()
        val started = insoleV2Manager.startScanForSelection { devices ->
            runOnUiThread {
                insoleInlineDevices.clear()
                insoleInlineDevices.addAll(devices)
                renderInsoleDeviceList()
            }
        }
        if (!started) {
            renderInsoleDeviceList()
        }
    }

    private fun stopCompetingScans(target: String) {
        if (target != "glasses" && rokidBluetoothConnector.isScanning()) {
            rokidBluetoothConnector.stopScan()
        }
        if (target != "gait" && gaitBluetoothConnector.isScanning()) {
            gaitBluetoothConnector.stopScan()
        }
        if (target != "insole" && insoleV2Manager.isScanning()) {
            insoleV2Manager.stopScan(clearCallback = true)
        }
    }

    private fun initializeImuSlots() {
        imuSlots.clear()
        repeat(IMU_SLOT_COUNT) { index ->
            val defaultName = defaultImuName(index)
            val name = imuPrefs.getString("slot.$index.name", defaultName)?.ifBlank { defaultName } ?: defaultName
            val address = imuPrefs.getString("slot.$index.address", null)?.ifBlank { null }
            imuSlots.add(
                ImuSlotState(
                    index = index,
                    name = name,
                    address = address,
                )
            )
        }
    }

    private fun defaultImuName(index: Int): String {
        return when (index) {
            0 -> getString(R.string.imu_manage_slot_default_1)
            1 -> getString(R.string.imu_manage_slot_default_2)
            else -> "IMU-${index + 1}"
        }
    }

    private fun openImuManageDialog() {
        if (!gaitBluetoothConnector.isConnected()) {
            showStatus(getString(R.string.imu_manage_connection_required))
            return
        }
        if (!hasAllRequiredPermissions()) {
            showStatus("请先授予蓝牙相关权限")
            requestCxrPermissions()
            return
        }
        if (!gaitBluetoothConnector.isBluetoothEnabled()) {
            pendingBluetoothEnableAction = { openImuManageDialog() }
            requestBluetoothEnable.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
            return
        }
        if (imuManageDialog?.isShowing == true) {
            refreshImuManageDialogViews()
            return
        }

        val view = LayoutInflater.from(this).inflate(R.layout.dialog_imu_connection_manager, null)
        val slotViews = listOf(
            ImuSlotViews(
                nameInput = view.findViewById(R.id.imuNameInput1),
                connectSwitch = view.findViewById(R.id.imuConnectSwitch1),
                addressView = view.findViewById(R.id.imuAddressView1),
            ),
            ImuSlotViews(
                nameInput = view.findViewById(R.id.imuNameInput2),
                connectSwitch = view.findViewById(R.id.imuConnectSwitch2),
                addressView = view.findViewById(R.id.imuAddressView2),
            )
        )

        imuManageViews = slotViews
        refreshImuConnectionSnapshot()

        slotViews.forEachIndexed { index, slotView ->
            val slot = imuSlots[index]
            slotView.nameInput.setText(slot.name)
            slotView.nameInput.setOnFocusChangeListener { _, hasFocus ->
                if (!hasFocus) {
                    persistImuSlotName(index, slotView.nameInput.text.toString())
                }
            }
            slotView.addressView.setOnClickListener {
                persistImuSlotName(index, slotView.nameInput.text.toString())
                promptImuAddressDialog(index)
            }
            slotView.connectSwitch.setOnCheckedChangeListener { _, isChecked ->
                persistImuSlotName(index, slotView.nameInput.text.toString())
                onImuToggleRequested(index, isChecked)
            }
        }
        refreshImuManageDialogViews()

        val dialog = AlertDialog.Builder(this)
            .setTitle(getString(R.string.imu_manage_title))
            .setView(view)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton(getString(R.string.imu_manage_save)) { _, _ ->
                persistAllImuSlotNames()
            }
            .create()

        dialog.setOnDismissListener {
            persistAllImuSlotNames()
            imuManageDialog = null
            imuManageViews = null
        }
        imuManageDialog = dialog
        dialog.show()
    }

    private fun persistAllImuSlotNames() {
        imuManageViews?.forEachIndexed { index, slotView ->
            persistImuSlotName(index, slotView.nameInput.text.toString())
        }
        refreshImuManageDialogViews()
    }

    private fun persistImuSlotName(index: Int, rawName: String) {
        val slot = imuSlots.getOrNull(index) ?: return
        val fallback = defaultImuName(index)
        val normalized = rawName.trim().ifBlank { fallback }
        if (slot.name == normalized) {
            return
        }
        slot.name = normalized
        imuPrefs.edit().putString("slot.$index.name", normalized).apply()
    }

    private fun persistImuSlotAddress(index: Int, address: String?) {
        imuPrefs.edit().apply {
            if (address.isNullOrBlank()) {
                remove("slot.$index.address")
            } else {
                putString("slot.$index.address", address)
            }
        }.apply()
    }

    private fun imuSlotKey(index: Int): String {
        return when (index) {
            0 -> IMU_SLOT_WALKING
            1 -> IMU_SLOT_CYCLING
            else -> IMU_SLOT_WALKING
        }
    }

    private fun imuSlotIndex(slot: String?): Int? {
        return when (slot?.trim()?.lowercase()) {
            IMU_SLOT_WALKING -> 0
            IMU_SLOT_CYCLING -> 1
            else -> null
        }
    }

    private fun onImuToggleRequested(index: Int, requestedConnected: Boolean) {
        val slot = imuSlots.getOrNull(index) ?: return
        if (slot.connecting) {
            refreshImuManageDialogViews()
            return
        }
        if (requestedConnected) {
            requestImuConnect(index)
        } else {
            disconnectImuSlot(index, "用户断开")
        }
    }

    private fun requestImuConnect(index: Int) {
        val slot = imuSlots.getOrNull(index) ?: return
        val address = slot.address
        slot.connecting = true
        slot.connected = false
        imuLastErrorNotice.remove(index)
        refreshImuManageDialogViews()
        val started = sendImuManageCommand(index = index, connect = true)
        if (!started) {
            cancelImuConnectTimeout(index)
            slot.connecting = false
            slot.connected = false
            refreshImuManageDialogViews()
            showStatus("IMU 控制命令发送失败: ${slot.name}")
            return
        }
        scheduleImuConnectTimeout(index)
        if (address.isNullOrBlank()) {
            showStatus("已请求 gait 连接 IMU: ${slot.name}（使用 gait 默认 MAC）")
        } else {
            showStatus("已请求 gait 连接 IMU: ${slot.name}")
        }
    }

    private fun disconnectImuSlot(index: Int, reason: String) {
        val slot = imuSlots.getOrNull(index) ?: return
        cancelImuConnectTimeout(index)
        slot.connecting = false
        slot.connected = false
        imuLastErrorNotice.remove(index)
        slot.battery = null
        sendImuManageCommand(index = index, connect = false)
        refreshImuManageDialogViews()
        showStatus("已请求 gait 断开 IMU: ${slot.name} ($reason)")
    }

    private fun sendImuManageCommand(index: Int, connect: Boolean): Boolean {
        val slot = imuSlots.getOrNull(index) ?: return false
        val slotCode = GaitProtocol.imuSlotCodeFromText(imuSlotKey(index)) ?: index
        val payload = LinkedHashMap<String, Any>()
        payload["t"] = GaitProtocol.TYPE_IMU_MANAGE
        payload["s"] = slotCode
        payload["x"] = connect
        val macBytes = parseMacBytes(slot.address)
        if (macBytes != null) {
            payload["ma"] = macBytes
        }
        return gaitBluetoothConnector.sendLine(toJsonString(payload))
    }

    private fun handleGaitImuMessage(line: String) {
        val obj = runCatching { JsonParser.parseString(line).asJsonObject }.getOrNull() ?: return
        val type = GaitProtocol.resolveMessageType(obj) ?: return
        when (type) {
            "imu_manage_ack" -> applyImuStatusFromAck(obj)
            "state", "telemetry", "imu_manage_status" -> applyImuStatusFromTelemetry(obj)
            "error" -> {
                val errorCode = obj.intOrNull("ec")
                showStatus("gait 命令失败: ${GaitProtocol.formatReasonCode(errorCode)}")
            }
            else -> Unit
        }
    }

    private fun applyImuStatusFromAck(obj: JsonObject) {
        val slotText = obj.intOrNull("s")?.let { GaitProtocol.imuSlotTextFromCode(it) }
            ?: obj.stringOrNull("slot")
        val slot = imuSlotIndex(slotText) ?: return
        val ok = obj.boolOrNull("ok") ?: obj.boolOrNull("o") ?: false
        if (!ok) {
            cancelImuConnectTimeout(slot)
            val state = imuSlots.getOrNull(slot) ?: return
            state.connecting = false
            state.connected = false
            state.battery = null
            refreshImuManageDialogViews()
            val reasonCode = obj.intOrNull("r") ?: -1
            showStatus("gait IMU控制失败(${state.name}): ${GaitProtocol.formatReasonCode(reasonCode)}")
            return
        }
        val connected = obj.boolOrNull("k") ?: obj.boolOrNull("connected")
        val measuring = obj.boolOrNull("q") ?: obj.boolOrNull("measuring")
        if (connected != null) {
            applyRemoteImuStatus(slot, connected, null)
            val state = imuSlots.getOrNull(slot)
            if (state != null) {
                val measureText = if (measuring == true) ", measuring=true" else ""
                showStatus("gait IMU应答: ${state.name} connected=$connected$measureText")
            }
        } else {
            refreshImuManageDialogViews()
        }
    }

    private fun applyImuStatusFromTelemetry(obj: JsonObject) {
        val compactFlags = obj.intOrNull("f")
        val walkConnected = obj.boolOrNull("imu_walk_connected")
            ?: compactFlags?.hasStateFlag(STATE_FLAG_IMU_WALK_CONNECTED)
        if (walkConnected != null) {
            applyRemoteImuStatus(0, walkConnected, null)
        }
        val cycleConnected = obj.boolOrNull("imu_cycle_connected")
            ?: compactFlags?.hasStateFlag(STATE_FLAG_IMU_CYCLE_CONNECTED)
        if (cycleConnected != null) {
            applyRemoteImuStatus(1, cycleConnected, null)
        }
    }

    private fun applyRemoteImuStatus(index: Int, connected: Boolean, mac: String?, lastError: String? = null) {
        val slot = imuSlots.getOrNull(index) ?: return
        if (connected) {
            cancelImuConnectTimeout(index)
            slot.connected = true
            slot.connecting = false
            imuLastErrorNotice.remove(index)
            if (!mac.isNullOrBlank()) {
                val current = normalizeMac(slot.address)
                val incoming = normalizeMac(mac)
                if (incoming != null && current != incoming) {
                    slot.address = mac
                    persistImuSlotAddress(index, mac)
                }
            }
        } else {
            slot.connected = false
            val normalizedError = lastError?.trim().orEmpty()
            if (slot.connecting && normalizedError.isNotBlank()) {
                cancelImuConnectTimeout(index)
                slot.connecting = false
                val prev = imuLastErrorNotice[index]
                if (prev != normalizedError) {
                    imuLastErrorNotice[index] = normalizedError
                    showStatus("gait侧${slot.name}连接失败: $normalizedError")
                }
            }
            if (!slot.connecting) {
                slot.battery = null
            }
        }
        refreshImuManageDialogViews()
    }

    private fun cancelImuConnectTimeout(index: Int) {
        val pending = imuConnectTimeoutTasks.remove(index) ?: return
        relayHandler.removeCallbacks(pending)
    }

    private fun scheduleImuConnectTimeout(index: Int) {
        cancelImuConnectTimeout(index)
        val task = Runnable {
            val slot = imuSlots.getOrNull(index) ?: return@Runnable
            if (slot.connected || !slot.connecting) {
                return@Runnable
            }
            slot.connecting = false
            slot.connected = false
            slot.battery = null
            refreshImuManageDialogViews()
            showStatus("IMU 连接超时: ${slot.name}，请检查 gait 端日志和 IMU 电源")
        }
        imuConnectTimeoutTasks[index] = task
        relayHandler.postDelayed(task, IMU_CONNECT_TIMEOUT_MS)
    }

    private fun promptImuAddressDialog(slotIndex: Int) {
        val slot = imuSlots.getOrNull(slotIndex) ?: return
        val input = EditText(this).apply {
            setText(slot.address.orEmpty())
            setSelection(text.length)
            hint = "AA:BB:CC:DD:EE:FF"
            inputType = android.text.InputType.TYPE_CLASS_TEXT
        }
        AlertDialog.Builder(this)
            .setTitle("设置${slot.name} MAC")
            .setView(input)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton(getString(R.string.imu_manage_save)) { _, _ ->
                val raw = input.text?.toString()?.trim().orEmpty()
                if (raw.isBlank()) {
                    slot.address = null
                    slot.connected = false
                    slot.connecting = false
                    slot.battery = null
                    persistImuSlotAddress(slotIndex, null)
                    refreshImuManageDialogViews()
                    return@setPositiveButton
                }
                if (!isValidMac(raw)) {
                    showStatus("MAC 地址格式无效: $raw")
                    return@setPositiveButton
                }
                val normalized = raw.uppercase()
                slot.address = normalized
                slot.connected = false
                slot.connecting = false
                slot.battery = null
                persistImuSlotAddress(slotIndex, normalized)
                refreshImuManageDialogViews()
            }
            .show()
    }

    private fun refreshImuConnectionSnapshot() {
        imuSlots.forEach { slot ->
            if (!slot.connecting) {
                slot.battery = null
            }
        }
    }

    private fun refreshImuManageDialogViews() {
        val slotViews = imuManageViews ?: return
        refreshImuConnectionSnapshot()
        slotViews.forEachIndexed { index, slotView ->
            val slot = imuSlots[index]
            if (!slotView.nameInput.hasFocus()) {
                val currentText = slotView.nameInput.text?.toString().orEmpty()
                if (currentText != slot.name) {
                    slotView.nameInput.setText(slot.name)
                }
            }

            slotView.connectSwitch.setOnCheckedChangeListener(null)
            slotView.connectSwitch.isChecked = slot.connected || slot.connecting
            // 连接中时禁止再次点击，但保持正常透明度，确保"连接中"文字清晰可见
            slotView.connectSwitch.isEnabled = !slot.connecting
            slotView.connectSwitch.alpha = 1f
            // 同时更新 track 内部文字（textOn）和外部标签（text），保持状态一致：
            // 连接中 → track 内也显示"连接中"，不误导用户看到"已连接"
            val statusText = when {
                slot.connecting -> getString(R.string.imu_manage_status_connecting)
                slot.connected -> getString(R.string.imu_manage_status_connected)
                else -> getString(R.string.imu_manage_status_connect)
            }
            slotView.connectSwitch.textOn = if (slot.connecting)
                getString(R.string.imu_manage_status_connecting)
            else
                getString(R.string.imu_manage_status_connected)
            slotView.connectSwitch.text = statusText
            slotView.connectSwitch.setOnCheckedChangeListener { _, isChecked ->
                persistImuSlotName(index, slotView.nameInput.text.toString())
                onImuToggleRequested(index, isChecked)
            }
            slotView.addressView.text = buildImuAddressLine(slot)
        }
    }

    private fun buildImuAddressLine(slot: ImuSlotState): String {
        val target = slot.address
        if (target.isNullOrBlank()) {
            return getString(R.string.imu_manage_unbound)
        }
        return getString(R.string.imu_manage_device_line, target)
    }

    private fun isValidMac(address: String): Boolean {
        val value = address.trim()
        val regex = Regex("^(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")
        return regex.matches(value)
    }

    private fun normalizeMac(address: String?): String? {
        return address
            ?.lowercase()
            ?.replace("-", "")
            ?.replace(":", "")
            ?.ifBlank { null }
    }

    private fun parseMacBytes(address: String?): List<Int>? {
        val normalized = address?.trim()?.takeIf { isValidMac(it) } ?: return null
        return normalized.split(":").mapNotNull { token ->
            runCatching { token.toInt(16) }.getOrNull()
        }.takeIf { it.size == 6 }
    }

    private fun toJsonString(payload: Map<String, Any>): String {
        val entries = payload.entries.joinToString(",") { (key, value) ->
            val encoded = encodeJsonValue(value)
            "\"$key\":$encoded"
        }
        return "{$entries}"
    }

    private fun encodeJsonValue(value: Any?): String {
        return when (value) {
            null -> "null"
            is Number, is Boolean -> value.toString()
            is String -> "\"" + value
                .replace("\\", "\\\\")
                .replace("\"", "\\\"") + "\""
            is List<*> -> value.joinToString(prefix = "[", postfix = "]") { item ->
                encodeJsonValue(item)
            }
            is Map<*, *> -> value.entries.joinToString(prefix = "{", postfix = "}") { (k, v) ->
                val key = k?.toString()?.replace("\\", "\\\\")?.replace("\"", "\\\"").orEmpty()
                "\"$key\":" + encodeJsonValue(v)
            }
            else -> "\"" + value.toString()
                .replace("\\", "\\\\")
                .replace("\"", "\\\"") + "\""
        }
    }

    private fun JsonObject.boolOrNull(key: String): Boolean? {
        return GaitProtocol.boolOrNull(this, key)
    }

    private fun JsonObject.intOrNull(key: String): Int? {
        val element = get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asInt }.getOrNull()
    }

    private fun Int.hasStateFlag(bit: Int): Boolean {
        return (this and (1 shl bit)) != 0
    }

    private fun JsonObject.stringOrNull(key: String): String? {
        val element = get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asString }.getOrNull()
    }

    private fun renderRokidDeviceList() {
        updateScanButtonLabels()
        val bondedTag = getString(R.string.dialog_device_bonded)
        val scannedTag = getString(R.string.dialog_device_scanned)
        val currentAddress = rokidBluetoothConnector.currentDeviceAddress()
        val connectedAddress = if (rokidBluetoothConnector.isBluetoothConnected()) {
            currentAddress
        } else {
            null
        }
        val connectingAddress = if (rokidBluetoothConnector.isConnecting()) {
            currentAddress
        } else {
            null
        }
        val buttonsEnabled = hasAllRequiredPermissions()

        val rows = rokidInlineDevices.map { device ->
            val sourceTag = if (device.bonded) bondedTag else scannedTag
            val isConnected = connectedAddress == device.address
            val isConnecting = connectingAddress == device.address
            InlineDeviceRow(
                label = "${device.displayName} [$sourceTag]\n${device.address}",
                checked = isConnected || isConnecting,
                enabled = buttonsEnabled && !rokidBluetoothConnector.isConnecting(),
                toggleText = when {
                    isConnecting -> "连接中"
                    isConnected -> "已连接"
                    else -> "连接"
                },
                onToggleRequested = { shouldConnect ->
                    if (shouldConnect) {
                        startBatteryRefreshLoop()
                        if (!rokidBluetoothConnector.connectDeviceByAddress(device.address)) {
                            stopBatteryRefreshLoop()
                            renderRokidDeviceList()
                        } else {
                            renderRokidDeviceList()
                        }
                    } else {
                        stopBatteryRefreshLoop()
                        updateBatteryIndicator(null)
                        rokidBluetoothConnector.deinitBluetooth()
                        renderRokidDeviceList()
                    }
                }
            )
        }

        renderInlineDeviceRows(
            container = glassesDevicesContainer,
            emptyView = glassesDevicesEmptyView,
            rows = rows,
            emptyText = when {
                rokidBluetoothConnector.isScanning() -> getString(R.string.dialog_scanning)
                !hasRequestedGlassesScan -> getString(R.string.exo_empty_idle)
                else -> getString(R.string.dialog_no_devices)
            }
        )
    }

    private fun renderGaitDeviceList() {
        updateScanButtonLabels()
        val bondedTag = getString(R.string.dialog_device_bonded)
        val scannedTag = getString(R.string.dialog_device_scanned)
        val currentAddress = gaitBluetoothConnector.currentDeviceAddress()
        val connectedAddress = if (gaitBluetoothConnector.isConnected()) {
            currentAddress
        } else {
            null
        }
        val connectingAddress = if (gaitBluetoothConnector.isConnecting()) {
            currentAddress
        } else {
            null
        }
        val buttonsEnabled = hasAllRequiredPermissions()

        val rows = gaitInlineDevices.map { device ->
            val sourceTag = if (device.bonded) bondedTag else scannedTag
            val isConnected = connectedAddress == device.address
            val isConnecting = connectingAddress == device.address
            InlineDeviceRow(
                label = "${device.displayName} [$sourceTag]\n${device.address}",
                checked = isConnected || isConnecting,
                enabled = buttonsEnabled && !gaitBluetoothConnector.isConnecting(),
                toggleText = when {
                    isConnecting -> "连接中"
                    isConnected -> "已连接"
                    else -> "连接"
                },
                onToggleRequested = { shouldConnect ->
                    if (shouldConnect) {
                        showStatus("正在连接步态设备: ${device.displayName}")
                        val started = gaitBluetoothConnector.connectDeviceByAddress(device.address) { _ ->
                            runOnUiThread {
                                renderGaitDeviceList()
                            }
                        }
                        if (!started) {
                            renderGaitDeviceList()
                        } else {
                            renderGaitDeviceList()
                        }
                    } else {
                        gaitBluetoothConnector.disconnect()
                        renderGaitDeviceList()
                    }
                }
            )
        }

        renderInlineDeviceRows(
            container = gaitDevicesContainer,
            emptyView = gaitDevicesEmptyView,
            rows = rows,
            emptyText = when {
                gaitBluetoothConnector.isScanning() -> getString(R.string.dialog_scanning)
                !hasRequestedGaitScan -> getString(R.string.exo_empty_idle)
                else -> getString(R.string.dialog_no_gait_devices)
            }
        )
    }

    private fun renderInsoleDeviceList() {
        updateScanButtonLabels()
        val buttonsEnabled = hasAllRequiredPermissions()
        val rows = insoleInlineDevices.map { device ->
            val rssiText = if (device.rssi == Int.MIN_VALUE) "--" else device.rssi.toString()
            InlineDeviceRow(
                label = getString(
                    R.string.insole_device_line,
                    device.displayName,
                    device.address,
                    rssiText
                ),
                checked = device.connected || device.connecting,
                enabled = buttonsEnabled && !device.connecting,
                toggleText = when {
                    device.connecting -> getString(R.string.insole_action_connecting)
                    device.connected -> "已连接"
                    else -> getString(R.string.insole_action_connect)
                },
                onToggleRequested = { shouldConnect ->
                    if (shouldConnect) {
                        insoleV2Manager.connect(device.address)
                    } else {
                        insoleV2Manager.disconnect(device.address)
                    }
                    renderInsoleDeviceList()
                }
            )
        }

        renderInlineDeviceRows(
            container = insoleDevicesContainer,
            emptyView = insoleDevicesEmptyView,
            rows = rows,
            emptyText = when {
                insoleV2Manager.isScanning() -> getString(R.string.insole_dialog_scanning)
                !hasRequestedInsoleScan -> getString(R.string.exo_empty_idle)
                else -> getString(R.string.insole_dialog_empty)
            }
        )
    }

    private fun renderInlineDeviceRows(
        container: LinearLayout,
        emptyView: TextView,
        rows: List<InlineDeviceRow>,
        emptyText: String,
    ) {
        container.removeAllViews()
        if (rows.isEmpty()) {
            emptyView.visibility = View.VISIBLE
            emptyView.text = emptyText
            return
        }

        emptyView.visibility = View.GONE
        val inflater = LayoutInflater.from(this)
        val itemSpacing = dpToPx(8)
        rows.forEachIndexed { index, row ->
            val itemView = inflater.inflate(R.layout.item_collection_device, container, false)
            val infoView = itemView.findViewById<TextView>(R.id.deviceInfoView)
            val toggleView = itemView.findViewById<SwitchCompat>(R.id.deviceConnectSwitch)

            infoView.text = row.label
            toggleView.setOnCheckedChangeListener(null)
            toggleView.isChecked = row.checked
            toggleView.isEnabled = row.enabled
            toggleView.alpha = if (row.enabled) 1f else 0.5f
            toggleView.text = row.toggleText
            toggleView.setOnCheckedChangeListener { _, isChecked ->
                row.onToggleRequested(isChecked)
            }

            val params = LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
            )
            if (index > 0) {
                params.topMargin = itemSpacing
            }
            itemView.layoutParams = params
            container.addView(itemView)
        }
    }

    private fun updateScanButtonLabels() {
        gaitScanButton.text = if (gaitBluetoothConnector.isScanning()) {
            getString(R.string.exo_scanning_button)
        } else {
            getString(R.string.exo_scan_button)
        }
        glassesScanButton.text = if (rokidBluetoothConnector.isScanning()) {
            getString(R.string.exo_scanning_button)
        } else {
            getString(R.string.exo_scan_button)
        }
        insoleScanButton.text = if (insoleV2Manager.isScanning()) {
            getString(R.string.exo_scanning_button)
        } else {
            getString(R.string.exo_scan_button)
        }
    }

    private fun dpToPx(dp: Int): Int {
        val density = resources.displayMetrics.density
        return (dp * density).toInt()
    }

    private fun showScanDeviceDialog() {
        if (scanDialog?.isShowing == true) {
            return
        }
        scanDialogDevices.clear()
        scanDialogLabels.clear()
        scanDialogLabels.add(getString(R.string.dialog_scanning))
        val adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, scanDialogLabels)
        scanDialogAdapter = adapter

        val dialog = AlertDialog.Builder(this)
            .setTitle(getString(R.string.dialog_select_device_title))
            .setAdapter(adapter) { _, which ->
                val selected = scanDialogDevices.getOrNull(which)
                if (selected == null) {
                    Toast.makeText(this, getString(R.string.dialog_scanning), Toast.LENGTH_SHORT).show()
                    return@setAdapter
                }
                val connected = rokidBluetoothConnector.connectDeviceByAddress(selected.address)
                if (!connected) {
                    Toast.makeText(this, getString(R.string.dialog_connect_failed), Toast.LENGTH_SHORT).show()
                }
                scanDialog?.dismiss()
            }
            .setNegativeButton(android.R.string.cancel) { _, _ ->
                rokidBluetoothConnector.stopScan()
            }
            .create()

        dialog.setOnDismissListener {
            scanDialog = null
            scanDialogAdapter = null
            scanDialogLabels.clear()
            scanDialogDevices.clear()
            rokidBluetoothConnector.stopScan()
        }

        scanDialog = dialog
        dialog.show()

        val started = rokidBluetoothConnector.startScanForSelection { devices ->
            runOnUiThread {
                if (scanDialog?.isShowing != true) {
                    return@runOnUiThread
                }
                scanDialogDevices.clear()
                scanDialogDevices.addAll(devices)
                scanDialogLabels.clear()
                if (devices.isEmpty()) {
                    scanDialogLabels.add(getString(R.string.dialog_no_devices))
                } else {
                    val bondedTag = getString(R.string.dialog_device_bonded)
                    val scannedTag = getString(R.string.dialog_device_scanned)
                    devices.forEach { device ->
                        val sourceTag = if (device.bonded) bondedTag else scannedTag
                        scanDialogLabels.add("${device.displayName} [$sourceTag]\\n${device.address}")
                    }
                }
                scanDialogAdapter?.notifyDataSetChanged()
            }
        }
        if (!started) {
            dialog.dismiss()
        }
    }

    private fun showGaitScanDeviceDialog() {
        if (gaitScanDialog?.isShowing == true) {
            return
        }
        gaitScanDialogDevices.clear()
        gaitScanDialogLabels.clear()
        gaitScanDialogLabels.add(getString(R.string.dialog_scanning))
        val adapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, gaitScanDialogLabels)
        gaitScanDialogAdapter = adapter

        val dialog = AlertDialog.Builder(this)
            .setTitle(getString(R.string.dialog_select_device_title))
            .setAdapter(adapter) { _, which ->
                val selected = gaitScanDialogDevices.getOrNull(which)
                if (selected == null) {
                    Toast.makeText(this, getString(R.string.dialog_scanning), Toast.LENGTH_SHORT).show()
                    return@setAdapter
                }
                showStatus("正在连接步态设备: ${selected.displayName}")
                val started = gaitBluetoothConnector.connectDeviceByAddress(selected.address) { success ->
                    if (!success) {
                        Toast.makeText(this, getString(R.string.dialog_connect_failed), Toast.LENGTH_SHORT).show()
                    }
                }
                if (!started) {
                    Toast.makeText(this, getString(R.string.dialog_connect_failed), Toast.LENGTH_SHORT).show()
                }
                gaitScanDialog?.dismiss()
            }
            .setNegativeButton(android.R.string.cancel) { _, _ ->
                gaitBluetoothConnector.stopScan()
            }
            .create()

        dialog.setOnDismissListener {
            gaitScanDialog = null
            gaitScanDialogAdapter = null
            gaitScanDialogLabels.clear()
            gaitScanDialogDevices.clear()
            gaitBluetoothConnector.stopScan()
        }

        gaitScanDialog = dialog
        dialog.show()

        val started = gaitBluetoothConnector.startScanForSelection { devices ->
            runOnUiThread {
                if (gaitScanDialog?.isShowing != true) {
                    return@runOnUiThread
                }
                gaitScanDialogDevices.clear()
                gaitScanDialogDevices.addAll(devices)
                gaitScanDialogLabels.clear()
                if (devices.isEmpty()) {
                    gaitScanDialogLabels.add(getString(R.string.dialog_no_gait_devices))
                } else {
                    val bondedTag = getString(R.string.dialog_device_bonded)
                    val scannedTag = getString(R.string.dialog_device_scanned)
                    devices.forEach { device ->
                        val sourceTag = if (device.bonded) bondedTag else scannedTag
                        gaitScanDialogLabels.add("${device.displayName} [$sourceTag]\\n${device.address}")
                    }
                }
                gaitScanDialogAdapter?.notifyDataSetChanged()
            }
        }
        if (!started) {
            dialog.dismiss()
        }
    }

    private fun openInsoleConnectDialog() {
        if (insoleConnectDialog?.isShowing == true) {
            return
        }
        if (!hasAllRequiredPermissions()) {
            showStatus("请先授予蓝牙相关权限")
            requestCxrPermissions()
            return
        }
        if (!insoleV2Manager.hasBluetoothAdapter()) {
            showStatus("当前设备不支持蓝牙")
            return
        }
        if (!insoleV2Manager.isBluetoothEnabled()) {
            pendingBluetoothEnableAction = { openInsoleConnectDialog() }
            requestBluetoothEnable.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
            return
        }

        val view = LayoutInflater.from(this).inflate(R.layout.dialog_insole_connect, null)
        val listView = view.findViewById<ListView>(R.id.insoleDeviceListView)
        val emptyView = view.findViewById<TextView>(R.id.insoleEmptyView)
        insoleConnectEmptyView = emptyView

        insoleConnectDialogAdapter = InsoleConnectDeviceAdapter()
        listView.adapter = insoleConnectDialogAdapter
        listView.emptyView = emptyView
        updateInsoleEmptyView()

        val dialog = AlertDialog.Builder(this)
            .setTitle(getString(R.string.insole_dialog_title))
            .setView(view)
            .setNegativeButton(android.R.string.cancel) { _, _ ->
                insoleV2Manager.stopScan(clearCallback = true)
            }
            .create()

        dialog.setOnDismissListener {
            insoleConnectDialog = null
            insoleConnectDialogAdapter = null
            insoleConnectDevices.clear()
            insoleConnectEmptyView = null
            insoleV2Manager.stopScan(clearCallback = true)
        }

        insoleConnectDialog = dialog
        dialog.show()
        dialog.window?.setBackgroundDrawable(ColorDrawable(Color.WHITE))

        val started = insoleV2Manager.startScanForSelection { devices ->
            runOnUiThread {
                if (insoleConnectDialog?.isShowing != true) {
                    return@runOnUiThread
                }
                insoleConnectDevices.clear()
                insoleConnectDevices.addAll(devices)
                insoleConnectDialogAdapter?.notifyDataSetChanged()
                updateInsoleEmptyView()
            }
        }
        if (!started) {
            updateInsoleEmptyView()
        }
    }

    private fun updateInsoleEmptyView() {
        val emptyView = insoleConnectEmptyView ?: return
        emptyView.text = if (insoleV2Manager.isScanning()) {
            getString(R.string.insole_dialog_scanning)
        } else {
            getString(R.string.insole_dialog_empty)
        }
    }

    private inner class InsoleConnectDeviceAdapter : BaseAdapter() {
        override fun getCount(): Int = insoleConnectDevices.size

        override fun getItem(position: Int): Any? = insoleConnectDevices.getOrNull(position)

        override fun getItemId(position: Int): Long = position.toLong()

        override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
            val view = convertView ?: LayoutInflater.from(parent.context)
                .inflate(R.layout.item_insole_connect_device, parent, false)
            val infoView = view.findViewById<TextView>(R.id.insoleDeviceInfoView)
            val actionSwitch = view.findViewById<SwitchCompat>(R.id.insoleConnectActionButton)

            val device = insoleConnectDevices.getOrNull(position)
            if (device == null) {
                infoView.text = ""
                actionSwitch.setOnCheckedChangeListener(null)
                actionSwitch.isChecked = false
                actionSwitch.isEnabled = false
                return view
            }

            val rssiText = if (device.rssi == Int.MIN_VALUE) "--" else device.rssi.toString()
            infoView.text = getString(
                R.string.insole_device_line,
                device.displayName,
                device.address,
                rssiText
            )

            actionSwitch.setOnCheckedChangeListener(null)
            actionSwitch.text = if (device.connecting) {
                getString(R.string.insole_action_connecting)
            } else {
                getString(R.string.insole_action_connect)
            }
            actionSwitch.isChecked = device.connected || device.connecting
            actionSwitch.isEnabled = !device.connecting
            actionSwitch.setOnCheckedChangeListener { _, checked ->
                if (checked) {
                    insoleV2Manager.connect(device.address)
                } else {
                    insoleV2Manager.disconnect(device.address)
                }
            }
            return view
        }
    }

    private fun setActionButtonsEnabled(enabled: Boolean) {
        glassesScanButton.isEnabled = enabled
        gaitScanButton.isEnabled = enabled
        insoleScanButton.isEnabled = enabled
        imuManageButton.isEnabled = enabled
        updateScanButtonLabels()
    }

    private fun showApiConfigDialog() {
        val view = LayoutInflater.from(this).inflate(R.layout.dialog_api_config, null)
        val baseUrlInput = view.findViewById<EditText>(R.id.baseUrlInput)
        val apiKeyInput = view.findViewById<EditText>(R.id.apiKeyInput)
        val apiModelInput = view.findViewById<EditText>(R.id.apiModelInput)
        val currentConfig = roadConditionAnalyzer.getApiConfig()
        baseUrlInput.setText(currentConfig.baseUrl)
        apiKeyInput.setText(currentConfig.apiKey)
        apiModelInput.setText(currentConfig.model)

        val dialog = AlertDialog.Builder(this)
            .setTitle(R.string.api_config_title)
            .setView(view)
            .setNegativeButton(android.R.string.cancel, null)
            .setPositiveButton(R.string.api_config_save, null)
            .create()

        dialog.setOnShowListener {
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener {
                val baseUrl = baseUrlInput.text?.toString()?.trim().orEmpty()
                val apiKey = apiKeyInput.text?.toString()?.trim().orEmpty()
                val model = apiModelInput.text?.toString()?.trim().orEmpty()
                if (model.isBlank()) {
                    apiModelInput.error = getString(R.string.api_config_model_required)
                    return@setOnClickListener
                }
                roadConditionAnalyzer.updateApiConfig(
                    apiKey = apiKey,
                    model = model,
                    baseUrl = baseUrl,
                )
                val savedConfig = roadConditionAnalyzer.getApiConfig()
                showStatus(
                    "API参数已更新: base=${savedConfig.baseUrl}, " +
                        "model=${savedConfig.model}, key=${maskApiKey(savedConfig.apiKey)}"
                )
                Toast.makeText(this, getString(R.string.api_config_saved), Toast.LENGTH_SHORT).show()
                dialog.dismiss()
            }
        }
        dialog.show()
    }

    private fun maskApiKey(apiKey: String): String {
        if (apiKey.isBlank()) {
            return "(empty)"
        }
        return if (apiKey.length <= 8) "****" else "${apiKey.take(4)}****${apiKey.takeLast(4)}"
    }

    private fun showStatus(text: String) {
        runOnUiThread {
            statusView.text = "状态: $text"
        }
    }

    private fun startBatteryRefreshLoop() {
        batteryRefreshHandler.removeCallbacks(batteryRefreshRunnable)
        batteryRefreshHandler.post(batteryRefreshRunnable)
    }

    private fun stopBatteryRefreshLoop() {
        batteryRefreshHandler.removeCallbacks(batteryRefreshRunnable)
    }

    private fun requestBatteryInfo() {
        if (!isGlassesConnected()) {
            updateBatteryIndicator(null)
            return
        }
        rokidBluetoothConnector.getGlassesInfo { status, info ->
            if (status == ValueUtil.CxrStatus.RESPONSE_SUCCEED) {
                updateBatteryIndicator(info?.batteryLevel)
            }
        }
    }

    private fun updateBatteryIndicator(level: Int?) {
        val normalized = level?.coerceIn(0, 100)
        runOnUiThread {
            batteryPercentView.text = normalized?.let { "$it%" } ?: "--%"
        }
    }

    private fun runWhenBluetoothConnected(action: () -> Unit) {
        if (!isGlassesConnected()) {
            showStatus("请先在首页完成 Rokid AI 授权并连接眼镜")
            return
        }
        action()
    }

    private fun handleControlMessage(raw: String) {
        if (!isGlassesConnected()) {
            Log.w(TAG, "Ignore control message while glasses disconnected: $raw")
            return
        }
        val command = raw.trim().lowercase()
        when (command) {
            "record_start", "start", "capture_start" -> {
                runWhenBluetoothConnected {
                    startRelayPipeline("眼镜请求启动")
                }
            }
            "record_stop", "stop", "capture_stop" -> {
                stopRelayPipeline("眼镜请求停止")
            }
            else -> {
                Log.w(TAG, "Unknown control command: $raw")
            }
        }
    }

    private fun startRelayPipeline(trigger: String) {
        if (!isGlassesConnected()) {
            showStatus("眼镜未连接，已跳过眼镜数据收发")
            return
        }
        if (isRelayRunning) {
            showStatus("图像采集已在运行")
            return
        }
        frameIndex = 0L
        isCaptureInFlight = false
        isAnalysisInFlight = false
        isRelayRunning = true

        val cameraStatus = rokidBluetoothConnector.aiOpenCamera(RELAY_WIDTH, RELAY_HEIGHT, RELAY_QUALITY)
        showStatus("开始采集($trigger): 1Hz ${RELAY_WIDTH}x${RELAY_HEIGHT}, open=${cameraStatus.name}")

        relayHandler.removeCallbacks(relayRunnable)
        relayHandler.post(relayRunnable)
    }

    private fun stopRelayPipeline(reason: String) {
        val wasRunning = isRelayRunning || isCaptureInFlight || isAnalysisInFlight
        isRelayRunning = false
        isCaptureInFlight = false
        isAnalysisInFlight = false
        relayHandler.removeCallbacks(relayRunnable)
        if (wasRunning) {
            showStatus("已停止图像采集: $reason")
        }
    }

    private fun sendResultToGlasses(payload: String) {
        if (!isGlassesConnected()) {
            return
        }
        val success = rokidBluetoothConnector.sendCustomMessage(RESULT_CHANNEL, payload)
        if (success) {
            Log.i(TAG, "[$RESULT_CHANNEL] send success payload=$payload")
        } else {
            Log.e(TAG, "[$RESULT_CHANNEL] send failed payload=$payload")
        }
    }

    private fun analyzeAndSendResult(photo: ByteArray, frame: Long) {
        val sizeText = decodeImageSize(photo)?.let { "${it.first}x${it.second}" }
            ?: "${RELAY_WIDTH}x${RELAY_HEIGHT}"
        isAnalysisInFlight = true
        roadConditionAnalyzer.analyzeRoadCondition(photo) { roadCondition ->
            isAnalysisInFlight = false
            if (!isRelayRunning) {
                return@analyzeRoadCondition
            }
            val scene = roadCondition ?: "路况分析失败"
            val payload = "RESULT|frame=$frame|scene=$scene|size=$sizeText"
            GaitBluetoothBridge.notifyRoadCondition(scene)
            sendResultToGlasses(payload)
            if (roadCondition != null) {
                Log.i(TAG, "路况分析结果: $roadCondition")
            } else {
                Log.w(TAG, "路况分析失败")
            }
        }
    }

    private fun decodeImageSize(imageBytes: ByteArray): Pair<Int, Int>? {
        val options = BitmapFactory.Options().apply {
            inJustDecodeBounds = true
        }
        BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size, options)
        if (options.outWidth <= 0 || options.outHeight <= 0) {
            return null
        }
        return options.outWidth to options.outHeight
    }

    private fun handleRokidStatusMessage(message: String) {
        showStatus(message)
        Log.i(TAG, message)
        if (!isGlassesConnected()) {
            stopBatteryRefreshLoop()
            updateBatteryIndicator(null)
            stopRelayPipeline("眼镜链路断开")
        }
        runOnUiThread {
            renderRokidDeviceList()
        }
    }

    private fun isGlassesConnected(): Boolean = rokidBluetoothConnector.isBluetoothConnected()

    private fun hasAllRequiredPermissions(): Boolean =
        requiredPermissions.all { permission ->
            ContextCompat.checkSelfPermission(this, permission) == PackageManager.PERMISSION_GRANTED
        }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQUEST_CODE_PERMISSIONS) {
            val allGranted = grantResults.isNotEmpty() &&
                grantResults.all { it == PackageManager.PERMISSION_GRANTED }
            permissionGrantedResult.postValue(allGranted)
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        if (rokidBluetoothConnector.handleActivityResult(requestCode, resultCode, data)) {
            return
        }
        super.onActivityResult(requestCode, resultCode, data)
    }
}
