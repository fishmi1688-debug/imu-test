package com.example.mobile

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.InputMethodManager
import android.widget.ArrayAdapter
import android.widget.AutoCompleteTextView
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import com.rokid.cxr.client.utils.ValueUtil
import java.math.BigDecimal
import java.math.RoundingMode
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

class GaitParameterActivity : AppCompatActivity() {

    companion object {
        private const val BATTERY_REFRESH_INTERVAL_MS = 15000L
        private const val STATE_FLAG_STAIRS_DOWN_MANUAL_ASSIST = 2
        private val COMPACT_MODE_KEYS = listOf(
            "walking",
            "stairs_up",
            "stairs_down",
            "test",
            "walking_test",
            "cycling",
            "uphill",
            "downhill",
            "imu_phase",
            "imu_left_phase",
        )
        private val IMU_PHASE_MODE_KEYS = setOf("imu_phase", "imu_left_phase")
    }

    private data class ParamSpec(
        val min: Double,
        val max: Double,
        val step: Double,
        val decimals: Int
    )

    private data class ParamViews(
        val minusButton: Button,
        val plusButton: Button,
        val valueView: TextView
    )

    private val paramSpecs = mapOf(
        "ext_t0" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "ext_tf" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "ext_p" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "ext_Tmax" to ParamSpec(min = 0.0, max = 17.0, step = 0.1, decimals = 1),
        "flex_t0" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "flex_tf" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "flex_p" to ParamSpec(min = 0.0, max = 1.0, step = 0.01, decimals = 3),
        "flex_Tmax" to ParamSpec(min = 0.0, max = 17.0, step = 0.1, decimals = 1),
        "phase_bias" to ParamSpec(min = -1.0, max = 1.0, step = 0.01, decimals = 3),
        "phase_bias_at_0p6" to ParamSpec(min = -1.0, max = 1.0, step = 0.01, decimals = 3),
        "phase_bias_slope" to ParamSpec(min = -10.0, max = 10.0, step = 0.05, decimals = 3),
        "event_prob_threshold" to ParamSpec(min = 0.0, max = 1.0, step = 0.05, decimals = 2),
        "swing_threshold" to ParamSpec(min = 0.0, max = 90.0, step = 1.0, decimals = 1)
    )

    private val paramValues = mutableMapOf<String, Double>()
    private val paramViews = mutableMapOf<String, ParamViews>()
    private val modeParamDrafts = mutableMapOf<String, MutableMap<String, Double>>()
    private var currentModeKey: String = "walking"

    private lateinit var statusView: TextView
    private lateinit var batteryPercentView: TextView
    private lateinit var modeDescriptionView: TextView
    private lateinit var modeDropdown: AutoCompleteTextView
    private lateinit var paramsScrollView: ScrollView
    private lateinit var stairsDownToggleCard: View
    private lateinit var stairsDownToggleButton: Button
    private lateinit var assistCurveEditorView: AssistCurveEditorView
    private lateinit var assistCurveSummaryView: TextView

    private lateinit var rokidBluetoothConnector: RokidBluetoothConnector
    private lateinit var gaitBluetoothConnector: GaitBluetoothConnector
    private val gson = Gson()
    private var lastPhaseBiasUiSyncSec = 0.0
    private var stairsDownAssistEnabled = false
    private var pendingModeKey: String? = null
    private var lastStateUiSignature: String? = null
    private val gaitStatusListener: (String) -> Unit = { message ->
        showStatus(message)
    }
    private val gaitLineListener: (String) -> Unit = { line ->
        handleGaitLine(line)
    }

    private val batteryRefreshHandler = Handler(Looper.getMainLooper())
    private val batteryRefreshRunnable = object : Runnable {
        override fun run() {
            fetchCurrentInfo(silentWhenDisconnected = true)
            batteryRefreshHandler.postDelayed(this, BATTERY_REFRESH_INTERVAL_MS)
        }
    }

    private fun isManualToggleMode(modeKey: String): Boolean {
        return modeKey == "stairs_down" ||
            modeKey == "test" ||
            modeKey == "walking_test" ||
            modeKey in IMU_PHASE_MODE_KEYS
    }

    private fun manualToggleModeLabel(modeKey: String): String {
        return when (modeKey) {
            "stairs_down" -> "下楼梯"
            "test" -> "骑车测试模式"
            "walking_test" -> "步行测试模式"
            "imu_phase" -> "有线IMU相位模式"
            "imu_left_phase" -> "左有线IMU相位模式"
            else -> "手动模式"
        }
    }

    private fun resolveStateModeKey(obj: JsonObject): String? {
        obj.stringOrNull("motion_mode")?.let { return it }
        GaitProtocol.resolveModeKey(obj)?.let { return it }
        val modeIndex = obj.intOrNull("mi") ?: return null
        return COMPACT_MODE_KEYS.getOrNull(modeIndex)
    }

    private fun resolveManualAssistFromState(obj: JsonObject): Boolean {
        obj.boolOrNull("stairs_down_manual_assist")?.let { return it }
        val compactFlags = obj.intOrNull("f") ?: return false
        return compactFlags.hasStateFlag(STATE_FLAG_STAIRS_DOWN_MANUAL_ASSIST)
    }

    private fun resolveAckAction(obj: JsonObject): Int? {
        return GaitProtocol.resolveAckActionCode(obj)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_gait_parameter)
        ExoBottomNav.setup(this, ExoDestination.GAIT_PARAMS)

        statusView = findViewById(R.id.gaitStatusView)
        batteryPercentView = findViewById(R.id.gaitBatteryPercentView)
        modeDescriptionView = findViewById(R.id.gaitModeDescriptionView)
        modeDropdown = findViewById(R.id.gaitModeDropdown)
        paramsScrollView = findViewById(R.id.gaitParamsScrollView)
        stairsDownToggleCard = findViewById(R.id.stairsDownToggleCard)
        stairsDownToggleButton = findViewById(R.id.stairsDownToggleButton)
        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            ExoBottomNav.openHome(this)
        }

        rokidBluetoothConnector = RokidBluetoothConnector(this) { message ->
            showStatus(message)
        }
        gaitBluetoothConnector = GaitBluetoothBridge.getConnector(this)

        findViewById<Button>(R.id.startAssistButton).setOnClickListener {
            sendStartAssist()
        }
        findViewById<Button>(R.id.emergencyStopButton).setOnClickListener {
            sendEmergencyStop()
        }
        stairsDownToggleButton.setOnClickListener {
            sendStairsDownAssistToggle()
        }

        bindParameterViews()
        setupModeDropdown()
        setupAssistCurveEditor()
        findViewById<Button>(R.id.resetModeDefaultsButton).setOnClickListener {
            applyModeDefaults(currentModeKey)
        }
        findViewById<Button>(R.id.applyGaitParamsButton).setOnClickListener {
            val modeName = GaitMotionModes.byKey[currentModeKey]?.name ?: currentModeKey
            val sent = sendParamsToDevice()
            val statusTag = if (sent) "已发送" else "发送失败"
            showStatus("参数已更新（$statusTag）: $modeName")
            val toastText = if (sent) "步态参数已更新" else "步态参数发送失败"
            Toast.makeText(this, toastText, Toast.LENGTH_SHORT).show()
        }

        applyModeDefaults(currentModeKey)
        updateBatteryIndicator(null)
        startBatteryRefreshLoop()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopBatteryRefreshLoop()
        rokidBluetoothConnector.release()
    }

    override fun onStart() {
        super.onStart()
        GaitBluetoothBridge.addStatusListener(gaitStatusListener)
        GaitBluetoothBridge.addLineListener(gaitLineListener)
        requestDeviceState(silentWhenDisconnected = true)
    }

    override fun onStop() {
        GaitBluetoothBridge.removeStatusListener(gaitStatusListener)
        GaitBluetoothBridge.removeLineListener(gaitLineListener)
        super.onStop()
    }

    private fun handleGaitLine(line: String) {
        val obj = runCatching { JsonParser.parseString(line).asJsonObject }.getOrNull() ?: return
        val type = GaitProtocol.resolveMessageType(obj) ?: return

        if (type.equals("state", ignoreCase = true)) {
            applyDeviceState(obj)
            return
        }

        if (type.equals("error", ignoreCase = true)) {
            val errorCode = obj.intOrNull("ec")
            showStatus("主板拒绝命令: ${GaitProtocol.formatReasonCode(errorCode)}")
            return
        }

        if (type.equals("ack", ignoreCase = true)) {
            val action = resolveAckAction(obj) ?: return
            val ok = obj.boolOrNull("ok") ?: obj.boolOrNull("o") ?: false
            if (action == GaitProtocol.ACTION_START_ASSIST) {
                if (!ok) {
                    val reasonCode = obj.intOrNull("r") ?: -1
                    val detail = obj.stringOrNull("err")?.takeIf { it.isNotBlank() }
                        ?: if (reasonCode == 19) {
                            "有线IMU数据频率未达到50Hz，请检查IMU设备供电、CAN连接、终端电阻、can接口状态和帧ID配置"
                        } else {
                            null
                        }
                    val detailText = if (detail != null) "，$detail" else ""
                    showStatus(
                        "开始命令执行失败: ${GaitProtocol.formatReasonCode(reasonCode)}$detailText"
                    )
                    return
                }
                val ackMode = obj.intOrNull("m")?.let(GaitProtocol::modeKeyFromCode) ?: currentModeKey
                if (isManualToggleMode(ackMode)) {
                    val enabled = obj.boolOrNull("e") ?: obj.boolOrNull("enabled") ?: false
                    runOnUiThread {
                        setStairsDownAssistEnabled(enabled)
                    }
                }
                requestDeviceState(silentWhenDisconnected = true)
                return
            }
            if (action == GaitProtocol.ACTION_STAIRS_DOWN_ASSIST) {
                val ackMode = obj.intOrNull("m")?.let(GaitProtocol::modeKeyFromCode) ?: currentModeKey
                val modeLabel = manualToggleModeLabel(ackMode)
                if (!ok) {
                    val reasonCode = obj.intOrNull("r") ?: -1
                    showStatus("$modeLabel 手动启停失败: ${GaitProtocol.formatReasonCode(reasonCode)}")
                    return
                }
                val enabled = obj.boolOrNull("e") ?: obj.boolOrNull("enabled") ?: return
                runOnUiThread {
                    setStairsDownAssistEnabled(enabled)
                }
                requestDeviceState(silentWhenDisconnected = true)
                return
            }
            if (action == GaitProtocol.ACTION_SET_MODE) {
                if (!ok) {
                    pendingModeKey = null
                    runOnUiThread {
                        val currentModeName =
                            GaitMotionModes.byKey[currentModeKey]?.name ?: currentModeKey
                        modeDropdown.setText(currentModeName, false)
                    }
                    val reasonCode = obj.intOrNull("r")
                    showStatus("模式切换失败: ${GaitProtocol.formatReasonCode(reasonCode)}")
                    return
                }
                val ackMode = obj.intOrNull("m")?.let(GaitProtocol::modeKeyFromCode) ?: currentModeKey
                val modeInfo = GaitMotionModes.byKey[ackMode]
                val modeName = modeInfo?.name ?: ackMode
                val modeDescription = modeInfo?.description ?: ackMode
                val resolvedParams = resolveModeParams(
                    modeKey = ackMode,
                    paramsObj = obj.jsonObjectOrNull("p"),
                    paramPairs = obj.jsonArrayOrNull("u")
                )
                runOnUiThread {
                    pendingModeKey = null
                    applyModeSnapshot(
                        modeKey = ackMode,
                        modeName = modeName,
                        modeDescription = modeDescription,
                        values = resolvedParams,
                        manualAssistEnabled = false,
                        statusText = "已切换模式: $modeName"
                    )
                }
                requestDeviceState(silentWhenDisconnected = true)
                return
            }
            if (action == GaitProtocol.ACTION_SET_PARAMS) {
                if (!ok) {
                    val reasonCode = obj.intOrNull("r")
                    showStatus("步态参数更新失败: ${GaitProtocol.formatReasonCode(reasonCode)}")
                    return
                }
                requestDeviceState(silentWhenDisconnected = true)
                return
            }
            return
        }

        if (!type.equals("telemetry", ignoreCase = true)) {
            return
        }
        val mode = resolveStateModeKey(obj) ?: return
        if (mode != currentModeKey) {
            return
        }

        if (isManualToggleMode(mode)) {
            val enabled = obj.boolOrNull("stairs_down_manual_assist")
                ?: obj.boolOrNull("assist_enabled")
                ?: obj.intOrNull("f")?.hasStateFlag(STATE_FLAG_STAIRS_DOWN_MANUAL_ASSIST)
                ?: return
            runOnUiThread {
                setStairsDownAssistEnabled(enabled)
            }
            return
        }

        if (mode != "walking" && mode != "cycling") {
            return
        }
        val phaseBias = obj.doubleOrNull("phase_bias") ?: return
        val timestampSec = obj.doubleOrNull("ts") ?: 0.0

        runOnUiThread {
            val current = paramValues["phase_bias"] ?: return@runOnUiThread
            if (abs(current - phaseBias) < 0.001) {
                return@runOnUiThread
            }
            if (timestampSec > 0.0 && timestampSec < lastPhaseBiasUiSyncSec) {
                return@runOnUiThread
            }
            if (timestampSec > 0.0) {
                lastPhaseBiasUiSyncSec = timestampSec
            }
            setParamValue("phase_bias", phaseBias, updatePreview = true)
        }
    }

    private fun applyDeviceState(obj: JsonObject) {
        val modeKey = resolveStateModeKey(obj) ?: return
        val modeInfo = GaitMotionModes.byKey[modeKey]
        val modeName = obj.stringOrNull("mode_name") ?: modeInfo?.name ?: modeKey
        val modeDescription =
            obj.stringOrNull("mode_description") ?: modeInfo?.description ?: modeKey
        val paramsObj = obj.jsonObjectOrNull("params") ?: obj.jsonObjectOrNull("p")
        val paramPairs = obj.jsonArrayOrNull("u")
        val manualAssistEnabled = resolveManualAssistFromState(obj)
        val stateSignature = buildString {
            append(modeKey)
            append('|')
            append(manualAssistEnabled)
            append('|')
            append(paramsObj?.toString().orEmpty())
            append('|')
            append(paramPairs?.toString().orEmpty())
        }
        if (stateSignature == lastStateUiSignature) {
            return
        }
        lastStateUiSignature = stateSignature
        val resolvedParams = resolveModeParams(
            modeKey = modeKey,
            paramsObj = paramsObj,
            paramPairs = paramPairs
        )

        runOnUiThread {
            pendingModeKey = null
            applyModeSnapshot(
                modeKey = modeKey,
                modeName = modeName,
                modeDescription = modeDescription,
                values = resolvedParams,
                manualAssistEnabled = manualAssistEnabled,
                statusText = "已同步设备状态: $modeName"
            )
        }
    }

    private fun resolveModeParams(
        modeKey: String,
        paramsObj: JsonObject?,
        paramPairs: com.google.gson.JsonArray? = null,
    ): Map<String, Double> {
        val modeInfo = GaitMotionModes.byKey[modeKey]
        val defaults = modeInfo?.let(::defaultParamsForMode) ?: mutableMapOf()
        val merged = defaults.toMutableMap()
        modeParamDrafts[modeKey]?.let { merged.putAll(it) }
        if (paramPairs != null) {
            for (entry in paramPairs) {
                val row = runCatching { entry.asJsonArray }.getOrNull() ?: continue
                if (row.size() < 2) continue
                val code = runCatching { row[0].asInt }.getOrNull() ?: continue
                val value = runCatching { row[1].asDouble }.getOrNull() ?: continue
                val key = GaitProtocol.paramKeyFromCode(code) ?: continue
                merged[key] = value
            }
        }
        if (paramsObj != null) {
            for (key in paramSpecs.keys) {
                paramsObj.doubleOrNull(key)?.let { merged[key] = it }
            }
            for ((name, _) in paramSpecs) {
                val code = GaitProtocol.paramCodeFromKey(name) ?: continue
                paramsObj.doubleOrNull(code.toString())?.let { merged[name] = it }
            }
        }
        return paramSpecs.keys.associateWith { key -> merged[key] ?: 0.0 }
    }

    private fun bindParameterViews() {
        paramViews["ext_t0"] = ParamViews(
            minusButton = findViewById(R.id.extT0MinusButton),
            plusButton = findViewById(R.id.extT0PlusButton),
            valueView = findViewById(R.id.extT0ValueView)
        )
        paramViews["ext_tf"] = ParamViews(
            minusButton = findViewById(R.id.extTfMinusButton),
            plusButton = findViewById(R.id.extTfPlusButton),
            valueView = findViewById(R.id.extTfValueView)
        )
        paramViews["ext_p"] = ParamViews(
            minusButton = findViewById(R.id.extPMinusButton),
            plusButton = findViewById(R.id.extPPlusButton),
            valueView = findViewById(R.id.extPValueView)
        )
        paramViews["ext_Tmax"] = ParamViews(
            minusButton = findViewById(R.id.extTmaxMinusButton),
            plusButton = findViewById(R.id.extTmaxPlusButton),
            valueView = findViewById(R.id.extTmaxValueView)
        )
        paramViews["flex_t0"] = ParamViews(
            minusButton = findViewById(R.id.flexT0MinusButton),
            plusButton = findViewById(R.id.flexT0PlusButton),
            valueView = findViewById(R.id.flexT0ValueView)
        )
        paramViews["flex_tf"] = ParamViews(
            minusButton = findViewById(R.id.flexTfMinusButton),
            plusButton = findViewById(R.id.flexTfPlusButton),
            valueView = findViewById(R.id.flexTfValueView)
        )
        paramViews["flex_p"] = ParamViews(
            minusButton = findViewById(R.id.flexPMinusButton),
            plusButton = findViewById(R.id.flexPPlusButton),
            valueView = findViewById(R.id.flexPValueView)
        )
        paramViews["flex_Tmax"] = ParamViews(
            minusButton = findViewById(R.id.flexTmaxMinusButton),
            plusButton = findViewById(R.id.flexTmaxPlusButton),
            valueView = findViewById(R.id.flexTmaxValueView)
        )
        paramViews["phase_bias"] = ParamViews(
            minusButton = findViewById(R.id.phaseBiasMinusButton),
            plusButton = findViewById(R.id.phaseBiasPlusButton),
            valueView = findViewById(R.id.phaseBiasValueView)
        )
        paramViews["phase_bias_at_0p6"] = ParamViews(
            minusButton = findViewById(R.id.phaseBias0p6MinusButton),
            plusButton = findViewById(R.id.phaseBias0p6PlusButton),
            valueView = findViewById(R.id.phaseBias0p6ValueView)
        )
        paramViews["phase_bias_slope"] = ParamViews(
            minusButton = findViewById(R.id.phaseBiasSlopeMinusButton),
            plusButton = findViewById(R.id.phaseBiasSlopePlusButton),
            valueView = findViewById(R.id.phaseBiasSlopeValueView)
        )
        paramViews["event_prob_threshold"] = ParamViews(
            minusButton = findViewById(R.id.rTMinusButton),
            plusButton = findViewById(R.id.rTPlusButton),
            valueView = findViewById(R.id.rTValueView)
        )
        paramViews["swing_threshold"] = ParamViews(
            minusButton = findViewById(R.id.swingThresholdMinusButton),
            plusButton = findViewById(R.id.swingThresholdPlusButton),
            valueView = findViewById(R.id.swingThresholdValueView)
        )

        for ((key, views) in paramViews) {
            configureAdjustButton(views.minusButton, "-")
            configureAdjustButton(views.plusButton, "+")
            views.minusButton.setOnClickListener { changeParamValue(key, negative = true) }
            views.plusButton.setOnClickListener { changeParamValue(key, negative = false) }
            views.valueView.setOnClickListener { showNumericInputDialog(key) }
        }
    }

    private fun setupModeDropdown() {
        val modeNames = GaitMotionModes.all.map { it.name }
        val adapter = ArrayAdapter(this, R.layout.item_dropdown_black_text, modeNames)
        modeDropdown.setAdapter(adapter)
        modeDropdown.setDropDownBackgroundResource(R.drawable.bg_dropdown_white)
        modeDropdown.setOnClickListener {
            modeDropdown.showDropDown()
        }
        modeDropdown.setOnItemClickListener { _, _, position, _ ->
            val selectedMode = GaitMotionModes.all[position]
            sendModeToDevice(selectedMode.key)
        }
    }

    private fun sendModeToDevice(modeKey: String) {
        val modeName = GaitMotionModes.byKey[modeKey]?.name ?: modeKey
        if (!gaitBluetoothConnector.isConnected()) {
            pendingModeKey = null
            runOnUiThread {
                applyLocalModePreview(modeKey)
            }
            return
        }
        val modeCode = GaitProtocol.modeCodeFromKey(modeKey)
        if (modeCode == null) {
            showStatus("未知模式，无法发送")
            return
        }
        val payload = mapOf(
            "t" to GaitProtocol.TYPE_SET_MODE,
            "m" to modeCode
        )
        if (!gaitBluetoothConnector.sendLine(gson.toJson(payload))) {
            showStatus("步态蓝牙未连接，模式发送失败")
            runOnUiThread {
                val currentModeName = GaitMotionModes.byKey[currentModeKey]?.name ?: currentModeKey
                modeDropdown.setText(currentModeName, false)
            }
            pendingModeKey = null
            return
        }
        pendingModeKey = modeKey
        runOnUiThread {
            applyRequestedModePreview(modeKey, statusText = "请求切换模式: $modeName")
        }
    }

    private fun sendParamsToDevice(): Boolean {
        val modeCode = GaitProtocol.modeCodeFromKey(currentModeKey)
        if (modeCode == null) {
            showStatus("未知模式，参数发送失败")
            return false
        }
        val updates = paramValues.mapNotNull { (key, value) ->
            val code = GaitProtocol.paramCodeFromKey(key) ?: return@mapNotNull null
            listOf(code, value)
        }
        val payload = linkedMapOf<String, Any>(
            "t" to GaitProtocol.TYPE_SET_PARAMS,
            "m" to modeCode,
            "u" to updates
        )
        val sent = gaitBluetoothConnector.sendLine(gson.toJson(payload))
        if (!sent) {
            showStatus("步态蓝牙未连接，参数发送失败")
        }
        return sent
    }

    private fun sendEmergencyStop() {
        val payload = mapOf(
            "t" to GaitProtocol.TYPE_COMMAND,
            "c" to 2
        )
        if (!gaitBluetoothConnector.sendLine(gson.toJson(payload))) {
            showStatus("步态蓝牙未连接，紧急停止发送失败")
        }
    }

    private fun sendStartAssist() {
        val payload = mapOf(
            "t" to GaitProtocol.TYPE_COMMAND,
            "c" to 1
        )
        if (!gaitBluetoothConnector.sendLine(gson.toJson(payload))) {
            showStatus("步态蓝牙未连接，开始命令发送失败")
            return
        }
        val modeName = GaitMotionModes.byKey[currentModeKey]?.name ?: currentModeKey
        val hint = if (currentModeKey in IMU_PHASE_MODE_KEYS) {
            "请求开始传输并启动系统（先检查有线IMU 50Hz数据；通过后重新使能电机但不启用电机主动上报，执行标零；助力由手动启停触发）"
        } else {
            "请求开始传输并启动系统（同时重新使能电机、设置50Hz主动上报并执行标零；助力由手动启停或IMU判定触发）"
        }
        showStatus("$modeName $hint")
    }

    private fun sendStairsDownAssistToggle() {
        if (!isManualToggleMode(currentModeKey)) {
            showStatus("请先切换到下楼梯、骑车测试模式、步行测试模式或IMU相位模式")
            return
        }
        val targetEnabled = !stairsDownAssistEnabled
        val modeLabel = manualToggleModeLabel(currentModeKey)
        val payload = mapOf(
            "t" to GaitProtocol.TYPE_COMMAND,
            "c" to 3,
            "v" to targetEnabled
        )
        if (!gaitBluetoothConnector.sendLine(gson.toJson(payload))) {
            showStatus("步态蓝牙未连接，$modeLabel 手动启停命令发送失败")
            return
        }
        showStatus(
            if (targetEnabled) {
                "$modeLabel 手动启停命令已发送: 请求开启助力"
            } else {
                "$modeLabel 手动启停命令已发送: 请求关闭助力"
            }
        )
    }

    private fun setStairsDownAssistEnabled(enabled: Boolean) {
        stairsDownAssistEnabled = enabled
        if (::stairsDownToggleButton.isInitialized) {
            stairsDownToggleButton.text = if (enabled) {
                getString(R.string.ui_text_097)
            } else {
                getString(R.string.ui_text_096)
            }
        }
    }

    private fun updateModeUiState(modeKey: String, manualAssistEnabled: Boolean) {
        currentModeKey = modeKey
        val interpolationEnabled = modeKey == "walking" || modeKey == "cycling"
        val eventThresholdEnabled = modeKey == "cycling" || modeKey == "uphill"
        val swingThresholdVisible = modeKey in IMU_PHASE_MODE_KEYS
        val stairsDownManualToggleVisible = isManualToggleMode(modeKey)
        setRowEnabled(findViewById(R.id.phaseBias0p6Row), interpolationEnabled)
        setRowEnabled(findViewById(R.id.phaseBiasSlopeRow), interpolationEnabled)
        setRowEnabled(findViewById(R.id.eventProbThresholdRow), eventThresholdEnabled)
        findViewById<View>(R.id.swingThresholdRow).visibility =
            if (swingThresholdVisible) View.VISIBLE else View.GONE
        stairsDownToggleCard.visibility = if (stairsDownManualToggleVisible) View.VISIBLE else View.GONE
        setStairsDownAssistEnabled(if (stairsDownManualToggleVisible) manualAssistEnabled else false)
    }

    private fun applyModeDefaults(modeKey: String) {
        val mode = GaitMotionModes.byKey[modeKey] ?: return
        val defaultParams = defaultParamsForMode(mode)
        modeParamDrafts[mode.key] = defaultParams.toMutableMap()
        applyModeSnapshot(
            modeKey = mode.key,
            modeName = mode.name,
            modeDescription = mode.description,
            values = defaultParams,
            manualAssistEnabled = false,
            statusText = "已切换模式: ${mode.name}"
        )
    }

    private fun requestDeviceState(silentWhenDisconnected: Boolean = false) {
        if (!gaitBluetoothConnector.isConnected()) {
            if (!silentWhenDisconnected) {
                showStatus("步态蓝牙未连接，状态同步失败")
            }
            return
        }
        val payload = mapOf("t" to GaitProtocol.TYPE_GET_STATE)
        val sent = gaitBluetoothConnector.sendLine(gson.toJson(payload))
        if (!sent && !silentWhenDisconnected) {
            showStatus("步态蓝牙未连接，状态同步失败")
        }
    }

    private fun setParamValue(key: String, rawValue: Double, updatePreview: Boolean = true) {
        val spec = paramSpecs[key] ?: return
        val normalized = normalize(rawValue, spec)
        paramValues[key] = normalized
        modeParamDrafts.getOrPut(currentModeKey) { mutableMapOf() }[key] = normalized
        renderParamValue(key)
        if (updatePreview) {
            updateAssistCurvePreview()
        }
    }

    private fun changeParamValue(key: String, negative: Boolean) {
        val spec = paramSpecs[key] ?: return
        val current = paramValues[key] ?: spec.min
        val delta = if (negative) -spec.step else spec.step
        setParamValue(key, current + delta, updatePreview = true)
    }

    private fun applyLocalModePreview(modeKey: String) {
        val mode = GaitMotionModes.byKey[modeKey] ?: return
        val values = modeParamDrafts.getOrPut(mode.key) { defaultParamsForMode(mode) }
        applyModeSnapshot(
            modeKey = mode.key,
            modeName = mode.name,
            modeDescription = mode.description,
            values = values,
            manualAssistEnabled = false,
            statusText = "步态蓝牙未连接，已切换到本地预览: ${mode.name}"
        )
    }

    private fun applyRequestedModePreview(modeKey: String, statusText: String) {
        val mode = GaitMotionModes.byKey[modeKey] ?: return
        val values = modeParamDrafts.getOrPut(mode.key) { defaultParamsForMode(mode) }
        applyModeSnapshot(
            modeKey = mode.key,
            modeName = mode.name,
            modeDescription = mode.description,
            values = values,
            manualAssistEnabled = false,
            statusText = statusText
        )
    }

    private fun applyModeSnapshot(
        modeKey: String,
        modeName: String,
        modeDescription: String,
        values: Map<String, Double>,
        manualAssistEnabled: Boolean,
        statusText: String? = null
    ) {
        val restoredScrollY = if (::paramsScrollView.isInitialized) paramsScrollView.scrollY else 0
        currentModeKey = modeKey
        if (modeDropdown.text?.toString() != modeName) {
            modeDropdown.setText(modeName, false)
        }
        modeDescriptionView.text = modeDescription

        setParamValue("ext_t0", values["ext_t0"] ?: 0.0, updatePreview = false)
        setParamValue("ext_tf", values["ext_tf"] ?: 0.0, updatePreview = false)
        setParamValue("ext_p", values["ext_p"] ?: 0.0, updatePreview = false)
        setParamValue("ext_Tmax", values["ext_Tmax"] ?: 0.0, updatePreview = false)

        setParamValue("flex_t0", values["flex_t0"] ?: 0.0, updatePreview = false)
        setParamValue("flex_tf", values["flex_tf"] ?: 0.0, updatePreview = false)
        setParamValue("flex_p", values["flex_p"] ?: 0.0, updatePreview = false)
        setParamValue("flex_Tmax", values["flex_Tmax"] ?: 0.0, updatePreview = false)

        setParamValue("phase_bias", values["phase_bias"] ?: 0.0, updatePreview = false)
        setParamValue("phase_bias_at_0p6", values["phase_bias_at_0p6"] ?: 0.0, updatePreview = false)
        setParamValue("phase_bias_slope", values["phase_bias_slope"] ?: 0.0, updatePreview = false)
        setParamValue("event_prob_threshold", values["event_prob_threshold"] ?: 0.0, updatePreview = false)
        setParamValue("swing_threshold", values["swing_threshold"] ?: 25.0, updatePreview = false)

        updateModeUiState(modeKey, manualAssistEnabled)
        updateAssistCurvePreview()
        statusText?.let(::showStatus)
        if (::paramsScrollView.isInitialized) {
            paramsScrollView.post {
                paramsScrollView.scrollTo(0, restoredScrollY)
            }
        }
    }

    private fun defaultParamsForMode(mode: GaitMotionMode): MutableMap<String, Double> {
        return mutableMapOf(
            "ext_t0" to mode.extT0,
            "ext_tf" to mode.extTf,
            "ext_p" to mode.extP,
            "ext_Tmax" to mode.extTmax,
            "flex_t0" to mode.flexT0,
            "flex_tf" to mode.flexTf,
            "flex_p" to mode.flexP,
            "flex_Tmax" to mode.flexTmax,
            "phase_bias" to mode.phaseBias,
            "phase_bias_at_0p6" to mode.phaseBiasAt0p6,
            "phase_bias_slope" to mode.phaseBiasSlope,
            "event_prob_threshold" to mode.eventProbThreshold,
            "swing_threshold" to mode.swingThreshold
        )
    }

    private fun configureAdjustButton(button: Button, symbol: String) {
        button.text = ""
        button.setTextColor(ContextCompat.getColor(this, R.color.header_title))
        button.backgroundTintList = null
        button.setBackgroundResource(
            if (symbol == "+") R.drawable.bg_adjust_plus else R.drawable.bg_adjust_minus
        )
        button.isAllCaps = false
    }

    private fun showNumericInputDialog(key: String) {
        val spec = paramSpecs[key] ?: return
        val current = paramValues[key] ?: spec.min
        val inputType = if (spec.min < 0) {
            InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
        } else {
            InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL
        }
        val editText = EditText(this).apply {
            this.inputType = inputType
            setText(String.format(Locale.US, "%.${spec.decimals}f", current))
            setSelection(text.length)
        }

        val title = "输入参数: $key"
        val hint = "范围 ${spec.min} ~ ${spec.max}"
        val dialog = AlertDialog.Builder(this)
            .setTitle(title)
            .setMessage(hint)
            .setView(editText)
            .setPositiveButton("确定", null)
            .setNegativeButton("取消", null)
            .create()

        dialog.setOnShowListener {
            editText.requestFocus()
            val imm = getSystemService(Context.INPUT_METHOD_SERVICE) as? InputMethodManager
            editText.post {
                imm?.showSoftInput(editText, InputMethodManager.SHOW_IMPLICIT)
            }
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener {
                val parsed = editText.text.toString().trim().toDoubleOrNull()
                if (parsed == null) {
                    editText.error = "请输入数字"
                    return@setOnClickListener
                }
                setParamValue(key, parsed)
                dialog.dismiss()
            }
        }
        dialog.show()
    }

    private fun renderParamValue(key: String) {
        val value = paramValues[key] ?: return
        val spec = paramSpecs[key] ?: return
        val view = paramViews[key]?.valueView ?: return
        view.text = String.format(Locale.US, "%.${spec.decimals}f", value)
    }

    private fun normalize(value: Double, spec: ParamSpec): Double {
        val clamped = value.coerceIn(spec.min, spec.max)
        val steps = ((clamped - spec.min) / spec.step).roundToInt()
        val snapped = spec.min + steps * spec.step
        return BigDecimal(snapped).setScale(spec.decimals, RoundingMode.HALF_UP).toDouble()
    }

    private fun setRowEnabled(view: View, enabled: Boolean) {
        view.alpha = if (enabled) 1.0f else 0.45f
        setEnabledRecursively(view, enabled)
    }

    private fun setEnabledRecursively(view: View, enabled: Boolean) {
        view.isEnabled = enabled
        if (view is ViewGroup) {
            for (index in 0 until view.childCount) {
                setEnabledRecursively(view.getChildAt(index), enabled)
            }
        }
    }

    private fun setupAssistCurveEditor() {
        assistCurveEditorView = findViewById(R.id.assistCurveEditorView)
        assistCurveSummaryView = findViewById(R.id.assistCurveSummaryView)
        assistCurveEditorView.onParamsChanged = { updated ->
            applyAssistCurveEditorParams(updated)
        }
    }

    private fun currentAssistPreviewParams(): AssistCurveParams? {
        val extT0 = paramValues["ext_t0"] ?: return null
        val extTf = paramValues["ext_tf"] ?: return null
        val extP = paramValues["ext_p"] ?: return null
        val extTmax = paramValues["ext_Tmax"] ?: return null
        val flexT0 = paramValues["flex_t0"] ?: return null
        val flexTf = paramValues["flex_tf"] ?: return null
        val flexP = paramValues["flex_p"] ?: return null
        val flexTmax = paramValues["flex_Tmax"] ?: return null
        val phaseBias = paramValues["phase_bias"] ?: return null
        return AssistCurveParams(
            extT0 = extT0,
            extTf = extTf,
            extP = extP,
            extTmax = extTmax,
            flexT0 = flexT0,
            flexTf = flexTf,
            flexP = flexP,
            flexTmax = flexTmax,
            phaseBias = phaseBias
        )
    }

    private fun applyAssistCurveEditorParams(updated: AssistCurveParams) {
        setParamValue("ext_t0", updated.extT0, updatePreview = false)
        setParamValue("ext_tf", updated.extTf, updatePreview = false)
        setParamValue("ext_p", updated.extP, updatePreview = false)
        setParamValue("ext_Tmax", updated.extTmax, updatePreview = false)
        setParamValue("flex_t0", updated.flexT0, updatePreview = false)
        setParamValue("flex_tf", updated.flexTf, updatePreview = false)
        setParamValue("flex_p", updated.flexP, updatePreview = false)
        setParamValue("flex_Tmax", updated.flexTmax, updatePreview = false)
        setParamValue("phase_bias", updated.phaseBias, updatePreview = false)
        updateAssistCurvePreview()
    }

    private fun updateAssistCurvePreview() {
        if (!::assistCurveEditorView.isInitialized) {
            return
        }
        val params = currentAssistPreviewParams() ?: return
        assistCurveEditorView.setParams(params)
        val summary = AssistCurveMath.sample(params)

        assistCurveSummaryView.text = if (
            summary.maxExtensionTorque <= AssistCurveMath.previewEpsilon &&
            summary.maxFlexionTorque <= AssistCurveMath.previewEpsilon
        ) {
            getString(R.string.ui_text_091)
        } else {
            getString(
                R.string.ui_text_092,
                formatNumber(summary.maxTotalTorque, 1),
                formatNumber(summary.maxExtensionTorque, 1),
                formatNumber(summary.maxFlexionTorque, 1),
                formatNumber(params.phaseBias, 3)
            )
        }
    }

    private fun formatNumber(value: Double, decimals: Int): String {
        return String.format(Locale.US, "%.${decimals}f", value)
    }

    private fun fetchCurrentInfo(silentWhenDisconnected: Boolean = false) {
        runWhenBluetoothConnected(silentWhenDisconnected) {
            rokidBluetoothConnector.getGlassesInfo { status, info ->
                if (status == ValueUtil.CxrStatus.RESPONSE_SUCCEED && info != null) {
                    updateBatteryIndicator(info.batteryLevel)
                }
            }
        }
    }

    private fun runWhenBluetoothConnected(
        silentWhenDisconnected: Boolean = false,
        action: () -> Unit
    ) {
        if (!rokidBluetoothConnector.isBluetoothConnected()) {
            if (!silentWhenDisconnected) {
                Toast.makeText(this, "请先在首页完成 Rokid 会话连接", Toast.LENGTH_SHORT).show()
                showStatus("眼镜会话未连接（仅影响电量读取）")
            }
            updateBatteryIndicator(null)
            return
        }
        action()
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

    private fun updateBatteryIndicator(level: Int?) {
        val normalized = level?.coerceIn(0, 100)
        runOnUiThread {
            batteryPercentView.text = normalized?.let { "$it%" } ?: "--%"
        }
    }

    private fun JsonObject.doubleOrNull(key: String): Double? {
        val element = get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asDouble }.getOrNull()
    }

    private fun JsonObject.stringOrNull(key: String): String? {
        val element = get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asString }.getOrNull()
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

    private fun JsonObject.jsonObjectOrNull(key: String): JsonObject? {
        val element = get(key) ?: return null
        if (element.isJsonNull || !element.isJsonObject) return null
        return runCatching { element.asJsonObject }.getOrNull()
    }

    private fun JsonObject.jsonArrayOrNull(key: String): com.google.gson.JsonArray? {
        val element = get(key) ?: return null
        if (element.isJsonNull || !element.isJsonArray) return null
        return runCatching { element.asJsonArray }.getOrNull()
    }
}
