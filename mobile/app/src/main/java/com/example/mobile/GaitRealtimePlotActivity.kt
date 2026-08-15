package com.example.mobile

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.widget.ImageButton
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet

class GaitRealtimePlotActivity : AppCompatActivity() {
    companion object {
        private const val ACTIVE_REDRAW_INTERVAL_MS = 100L
        private const val IDLE_REDRAW_INTERVAL_MS = 400L
        private const val STALE_PLOT_THRESHOLD_MS = 600L
        private const val PLOT_STALL_DIAG_THRESHOLD_MS = 1200L
        private const val STATE_FRESH_THRESHOLD_MS = 1500L
        private const val STARTUP_WAIT_PLOT_MS = 4000L
        private const val PLOT_WINDOW_SECONDS = 10
        private val IMU_PHASE_MODE_KEYS = setOf("imu_phase", "imu_left_phase")
    }

    private lateinit var statusView: TextView
    private lateinit var roadConditionView: TextView
    private lateinit var plotTitleView: TextView
    private lateinit var plotSubtitleView: TextView
    private lateinit var jointLabelView: TextView
    private lateinit var angleLabelView: TextView
    private lateinit var phaseLabelView: TextView
    private lateinit var assistLabelView: TextView
    private lateinit var jointChart: LineChart
    private lateinit var angleChart: LineChart
    private lateinit var phaseChart: LineChart
    private lateinit var assistChart: LineChart

    private lateinit var leftJointDataSet: LineDataSet
    private lateinit var rightJointDataSet: LineDataSet
    private lateinit var angleDataSet: LineDataSet
    private lateinit var leftVelocityDataSet: LineDataSet
    private lateinit var rightVelocityDataSet: LineDataSet
    private lateinit var phaseDataSet: LineDataSet
    private lateinit var assistDataSet: LineDataSet
    private lateinit var rightAssistDataSet: LineDataSet

    private val mainHandler = Handler(Looper.getMainLooper())
    private var sampleIndex = 0f
    private val maxPoints = (PLOT_WINDOW_SECONDS * 1000) / ACTIVE_REDRAW_INTERVAL_MS.toInt()
    private var lastRenderedPlotReceivedElapsedMs = 0L
    private var lastPlotUpdateElapsedMs = 0L
    private var lastStateUpdateElapsedMs = 0L
    private var pageVisibleSinceElapsedMs = 0L
    private var lastBridgeStatusElapsedMs = 0L
    private var lastBridgeStatusMessage: String? = null
    private var latestRenderedFrame: GaitBluetoothBridge.PlotFrame? = null
    private var latestStateSnapshot: GaitBluetoothBridge.StateSnapshot? = null
    private var lastStatusText: String? = null
    private var lastRoadConditionText: String? = null
    private var currentPlotPresentationIsImuPhase: Boolean? = null

    private val redrawRunnable = object : Runnable {
        override fun run() {
            redrawLatestPlotFrame()
            updateStatusText()
            val hasRecentPlot =
                lastPlotUpdateElapsedMs > 0L &&
                    (SystemClock.elapsedRealtime() - lastPlotUpdateElapsedMs) <= STALE_PLOT_THRESHOLD_MS
            val nextDelay = if (hasRecentPlot) ACTIVE_REDRAW_INTERVAL_MS else IDLE_REDRAW_INTERVAL_MS
            mainHandler.postDelayed(this, nextDelay)
        }
    }

    private val stateListener: (GaitBluetoothBridge.StateSnapshot) -> Unit = { snapshot ->
        latestStateSnapshot = snapshot
        lastStateUpdateElapsedMs = SystemClock.elapsedRealtime()
        runOnUiThread {
            updatePlotModePresentation()
            updateStatusText()
        }
    }

    private val bridgeStatusListener: (String) -> Unit = { message ->
        lastBridgeStatusMessage = message
        lastBridgeStatusElapsedMs = SystemClock.elapsedRealtime()
        runOnUiThread {
            updateStatusText()
        }
    }

    private val roadConditionListener: (String) -> Unit = { condition ->
        runOnUiThread {
            setRoadConditionTextIfChanged(condition)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_gait_realtime_plot)
        ExoBottomNav.setup(this, ExoDestination.PLOT)

        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            ExoBottomNav.openHome(this)
        }

        statusView = findViewById(R.id.plotStatusView)
        roadConditionView = findViewById(R.id.roadConditionView)
        plotTitleView = findViewById(R.id.plotTitleView)
        plotSubtitleView = findViewById(R.id.plotSubtitleView)
        jointLabelView = findViewById(R.id.jointLabelView)
        angleLabelView = findViewById(R.id.angleLabelView)
        phaseLabelView = findViewById(R.id.phaseLabelView)
        assistLabelView = findViewById(R.id.assistLabelView)
        jointChart = findViewById(R.id.jointChart)
        angleChart = findViewById(R.id.angleChart)
        phaseChart = findViewById(R.id.phaseChart)
        assistChart = findViewById(R.id.assistChart)

        leftJointDataSet = createDataSet("LeftAngle", colorOf(R.color.chart_series_primary))
        rightJointDataSet = createDataSet("RightAngle", colorOf(R.color.chart_series_secondary))
        angleDataSet = createDataSet("AngleDiff", colorOf(R.color.chart_series_tertiary))
        leftVelocityDataSet = createDataSet("LeftVelocity", colorOf(R.color.chart_series_primary))
        rightVelocityDataSet = createDataSet("RightVelocity", colorOf(R.color.chart_series_secondary))
        phaseDataSet = createDataSet("Phase", colorOf(R.color.chart_series_quaternary))
        assistDataSet = createDataSet("Assist", colorOf(R.color.chart_series_quinary))
        rightAssistDataSet = createDataSet("RightAssist", colorOf(R.color.chart_series_senary))

        setupChart(jointChart, leftJointDataSet, rightJointDataSet)
        setupChart(angleChart, angleDataSet, leftVelocityDataSet, rightVelocityDataSet)
        setupChart(phaseChart, phaseDataSet)
        setupChart(assistChart, assistDataSet, rightAssistDataSet)
        updatePlotModePresentation()
    }

    override fun onStart() {
        super.onStart()
        pageVisibleSinceElapsedMs = SystemClock.elapsedRealtime()
        latestStateSnapshot = GaitBluetoothBridge.snapshotLatestState()
        if (latestStateSnapshot != null) {
            lastStateUpdateElapsedMs = pageVisibleSinceElapsedMs
        }
        GaitBluetoothBridge.addStateListener(stateListener)
        GaitBluetoothBridge.addStatusListener(bridgeStatusListener)
        GaitBluetoothBridge.addRoadConditionListener(roadConditionListener)
        mainHandler.removeCallbacks(redrawRunnable)
        mainHandler.post(redrawRunnable)
        updateStatusText()
    }

    override fun onStop() {
        mainHandler.removeCallbacks(redrawRunnable)
        GaitBluetoothBridge.removeStateListener(stateListener)
        GaitBluetoothBridge.removeStatusListener(bridgeStatusListener)
        GaitBluetoothBridge.removeRoadConditionListener(roadConditionListener)
        super.onStop()
    }

    private fun createDataSet(label: String, color: Int): LineDataSet {
        val set = LineDataSet(mutableListOf(), label)
        set.color = color
        set.setDrawCircles(false)
        set.setDrawValues(false)
        set.lineWidth = 2f
        return set
    }

    private fun setupChart(chart: LineChart, dataSet: LineDataSet) {
        chart.data = LineData(dataSet)
        configureChartBase(chart)
    }

    private fun setupChart(chart: LineChart, vararg dataSets: LineDataSet) {
        chart.data = LineData(*dataSets)
        configureChartBase(chart)
    }

    private fun configureChartBase(chart: LineChart) {
        val axisTextColor = colorOf(R.color.chart_axis_text)
        val axisGridColor = colorOf(R.color.chart_axis_grid)

        chart.description.isEnabled = false
        chart.axisRight.isEnabled = false
        chart.legend.isEnabled = false
        chart.setTouchEnabled(true)
        chart.setDragEnabled(true)
        chart.setScaleEnabled(false)
        chart.setDrawGridBackground(false)
        chart.setNoDataTextColor(axisTextColor)
        chart.setBackgroundColor(colorOf(R.color.card_bg))
        chart.xAxis.setDrawLabels(false)
        chart.xAxis.setDrawGridLines(false)
        chart.xAxis.textColor = axisTextColor
        chart.xAxis.setDrawAxisLine(false)
        chart.axisLeft.setDrawGridLines(true)
        chart.axisLeft.textColor = axisTextColor
        chart.axisLeft.gridColor = axisGridColor
        chart.axisLeft.setDrawAxisLine(false)
    }

    private fun colorOf(colorResId: Int): Int {
        return ContextCompat.getColor(this, colorResId)
    }

    private fun redrawLatestPlotFrame() {
        val frame = GaitBluetoothBridge.snapshotLatestPlotFrame()
        if (frame == null || frame.receivedElapsedMs <= lastRenderedPlotReceivedElapsedMs) {
            return
        }

        lastRenderedPlotReceivedElapsedMs = frame.receivedElapsedMs
        lastPlotUpdateElapsedMs = frame.receivedElapsedMs
        latestRenderedFrame = frame
        val isImuPhaseMode = isImuPhasePresentation(frame, latestStateSnapshot)
        updatePlotModePresentation(isImuPhaseMode)
        val x = sampleIndex
        sampleIndex += 1f

        addEntry(leftJointDataSet, x, frame.leftAngle)
        addEntry(rightJointDataSet, x, frame.rightAngle)
        addEntry(angleDataSet, x, if (isImuPhaseMode) 0f else frame.angleDiff)
        addEntry(leftVelocityDataSet, x, if (isImuPhaseMode) frame.leftAngularVelocity ?: 0f else 0f)
        addEntry(rightVelocityDataSet, x, if (isImuPhaseMode) frame.rightAngularVelocity ?: 0f else 0f)
        addEntry(phaseDataSet, x, frame.phase)
        addEntry(assistDataSet, x, frame.assist)
        addEntry(rightAssistDataSet, x, frame.rightAssist ?: 0f)

        refreshChart(jointChart)
        refreshChart(angleChart)
        refreshChart(phaseChart)
        refreshChart(assistChart)
        updateStatusText()
    }

    private fun addEntry(dataSet: LineDataSet, x: Float, value: Float) {
        dataSet.addEntry(Entry(x, value))
        while (dataSet.entryCount > maxPoints) {
            val first = dataSet.getEntryForIndex(0)
            if (first != null) {
                dataSet.removeEntry(first)
            } else {
                break
            }
        }
    }

    private fun refreshChart(chart: LineChart) {
        chart.data?.notifyDataChanged()
        chart.notifyDataSetChanged()
        chart.setVisibleXRangeMaximum(maxPoints.toFloat())
        chart.moveViewToX(sampleIndex)
        chart.invalidate()
    }

    private fun updateStatusText() {
        val nowElapsedMs = SystemClock.elapsedRealtime()
        val frame = latestRenderedFrame
        val state = latestStateSnapshot
        val plotDiagText = buildPlotDiagnosis(nowElapsedMs)
        if (frame == null && state == null) {
            setStatusTextIfChanged("状态: 等待步态数据\n绘图诊断: $plotDiagText")
            return
        }

        val motionMode = state?.motionMode ?: "-"
        val hasImuVelocityFrame = frame?.let {
            it.leftAngularVelocity != null && it.rightAngularVelocity != null
        } == true
        val isImuPhaseMode = isImuPhasePresentation(frame, state, hasImuVelocityFrame)
        val isTestMode = motionMode == "test" || motionMode == "walking_test"
        val isManualAssistMode = isTestMode ||
            motionMode == "stairs_down" ||
            motionMode in IMU_PHASE_MODE_KEYS
        val testAnyPhaseValid = state?.testLeftPhaseValid == true || state?.testRightPhaseValid == true
        val testAnyAssistReady = state?.testLeftAssistReady == true || state?.testRightAssistReady == true
        val testAwaitFirstPeak = isTestMode &&
            state?.manualAssistEnabled == true &&
            !testAnyPhaseValid
        val testSampling = isTestMode &&
            state?.manualAssistEnabled == true &&
            testAnyPhaseValid &&
            !testAnyAssistReady

        val assistTag = when {
            state?.assistOutputActive == true -> "助力输出中"
            isManualAssistMode && state?.manualAssistEnabled == false -> "待手动开启"
            isImuPhaseMode && state?.assistArmed == true && state.imuPhaseMotionActive == false ->
                "静止不助力"
            testAwaitFirstPeak -> "等待首个峰值"
            testSampling -> "首周期采样中"
            isTestMode && !testAnyAssistReady -> "等待峰值建立"
            state?.assistArmed == true -> "助力已就绪"
            else -> "助力停"
        }
        val imuStartStopTag = when (state?.gaitState) {
            1 -> "运动"
            0 -> "停止"
            else -> "未知"
        }
        val imuConnTag = when (state?.imuConnected) {
            true -> "已连接"
            false -> "未连接"
            null -> "未知"
        }
        val imuReadyTag = when (state?.imuReady) {
            true -> "已就绪"
            false -> "未就绪"
            null -> "未知"
        }
        val imuStaleTag = when (state?.imuStale) {
            true -> "超时"
            false -> "新鲜"
            null -> "未知"
        }
        val scoreText = state?.detectionScore?.let { formatValue(it) } ?: "-"
        val phaseText = frame?.phase?.let { formatValue(it.toDouble()) } ?: "-"
        val assistText = if (isImuPhaseMode && frame?.rightAssist != null) {
            "L=${formatValue(frame.assist.toDouble())}/R=${formatValue(frame.rightAssist.toDouble())}"
        } else {
            frame?.assist?.let { formatValue(it.toDouble()) } ?: "-"
        }
        val angleMetricText = if (isImuPhaseMode) {
            val lv = frame?.leftAngularVelocity?.let { formatValue(it.toDouble()) } ?: "-"
            val rv = frame?.rightAngularVelocity?.let { formatValue(it.toDouble()) } ?: "-"
            "L/R大腿矢状面角速度=$lv/$rv"
        } else {
            val diff = frame?.angleDiff?.let { formatValue(it.toDouble()) } ?: "-"
            "角度差=$diff"
        }
        val gateFlags = mutableListOf<String>()
        state?.let { snapshot ->
            gateFlags += "系统=${if (snapshot.mechanicalZeroReady) "就绪" else "未就绪"}"
            gateFlags += "运动确认=${if (snapshot.motionConfirmed) "通过" else "未通过"}"
            gateFlags += "助力=${if (snapshot.assistArmed) "开" else "关"}"
            gateFlags += "输出=${if (snapshot.assistOutputActive) "有" else "无"}"
            if (isManualAssistMode) {
                gateFlags += "手动=${if (snapshot.manualAssistEnabled) "开" else "关"}"
            }
            if (isTestMode) {
                gateFlags += "test相位=L${if (snapshot.testLeftPhaseValid) 1 else 0}/R${if (snapshot.testRightPhaseValid) 1 else 0}"
                gateFlags += "峰值=L${if (snapshot.testLeftAssistReady) 1 else 0}/R${if (snapshot.testRightAssistReady) 1 else 0}"
            }
            gateFlags += "相位=${if (snapshot.phaseActive) "有效" else "无效"}"
            if (isImuPhaseMode) {
                gateFlags += "IMU运动=${if (snapshot.imuPhaseMotionActive) "运动" else "静止"}"
            }
            gateFlags += if (snapshot.assistWaitNextZero) "门控=等零点" else "门控=开放"
        }
        val gateText = if (gateFlags.isEmpty()) "门控=未知" else gateFlags.joinToString(" ")
        val imuLabel = state?.imuLabel?.takeIf { it.isNotBlank() } ?: "-"
        val imuErrorText = state?.imuLastError?.takeIf { it.isNotBlank() } ?: "-"
        val testHint = when {
            !isTestMode -> ""
            state?.manualAssistEnabled == false -> "\n测试模式提示: 先到参数页点“开始手动助力”"
            testAwaitFirstPeak -> "\n测试模式提示: 正在等待左右腿出现首个峰值以建立相位"
            testSampling -> "\n测试模式提示: 正在采第一周期峰值，第二周期才开始助力"
            testAnyPhaseValid && !testAnyAssistReady ->
                "\n测试模式提示: 已有相位，但至少一侧还没采满第二个峰值周期"
            state?.testLeftPhaseValid == false && (state?.testLeftAssistReady == true || state?.testRightAssistReady == true) ->
                "\n测试模式提示: 当前左腿相位短暂失效，图上仍只显示左腿"
            else -> ""
        }

        val statusText =
            "状态: 模式=$motionMode $assistTag $angleMetricText 相位=$phaseText 助力=$assistText\n" +
                "IMU启停判定: $imuStartStopTag (连接=$imuConnTag, 就绪=$imuReadyTag, 数据=$imuStaleTag, 标签=$imuLabel, 概率=$scoreText)\n" +
                "助力门控: $gateText\n" +
                "IMU诊断: $imuErrorText\n" +
                "绘图诊断: $plotDiagText" +
                testHint
        setStatusTextIfChanged(statusText)
    }

    private fun updatePlotModePresentation(isImuPhaseModeOverride: Boolean? = null) {
        if (!::angleDataSet.isInitialized) {
            return
        }
        val isImuPhaseMode = isImuPhaseModeOverride ?: isImuPhasePresentation(
            latestRenderedFrame,
            latestStateSnapshot,
        )
        val presentationChanged =
            currentPlotPresentationIsImuPhase != null &&
                currentPlotPresentationIsImuPhase != isImuPhaseMode
        currentPlotPresentationIsImuPhase = isImuPhaseMode
        if (presentationChanged) {
            clearPlotData()
        }
        if (isImuPhaseMode) {
            val mode = latestStateSnapshot?.motionMode
            val isLeftOnlyMode = mode == "imu_left_phase"
            plotTitleView.text = if (isLeftOnlyMode) {
                "左IMU相位实时数据"
            } else {
                "IMU相位实时数据"
            }
            plotSubtitleView.text = if (isLeftOnlyMode) {
                "左大腿矢状面角度/角速度、左相位推导右相位、左右助力实时曲线"
            } else {
                "左右大腿矢状面角度/角速度/相位/左右助力实时曲线"
            }
            jointLabelView.text = "左右大腿矢状面角度 (deg)"
            angleLabelView.text = "左右大腿矢状面角速度 (deg/s)"
            phaseLabelView.text = "IMU相位 (rad)"
            assistLabelView.text = "左右助力 (Nm)"
            leftJointDataSet.label = "LeftThighSagittalAngle"
            rightJointDataSet.label = if (isLeftOnlyMode) {
                "RightThighSagittalAngleDerived"
            } else {
                "RightThighSagittalAngle"
            }
            assistDataSet.label = "LeftAssist"
            angleDataSet.setVisible(false)
            leftVelocityDataSet.setVisible(true)
            rightVelocityDataSet.setVisible(true)
            rightAssistDataSet.setVisible(true)
        } else {
            plotTitleView.text = getString(R.string.ui_text_076)
            plotSubtitleView.text = "左右关节角度/角度差/相位/助力实时曲线"
            jointLabelView.text = getString(R.string.ui_text_081)
            angleLabelView.text = getString(R.string.ui_text_077)
            phaseLabelView.text = getString(R.string.ui_text_078)
            assistLabelView.text = getString(R.string.ui_text_079)
            leftJointDataSet.label = "LeftAngle"
            rightJointDataSet.label = "RightAngle"
            assistDataSet.label = "Assist"
            angleDataSet.setVisible(true)
            leftVelocityDataSet.setVisible(false)
            rightVelocityDataSet.setVisible(false)
            rightAssistDataSet.setVisible(false)
        }
    }

    private fun clearPlotData() {
        listOf(
            leftJointDataSet,
            rightJointDataSet,
            angleDataSet,
            leftVelocityDataSet,
            rightVelocityDataSet,
            phaseDataSet,
            assistDataSet,
            rightAssistDataSet,
        ).forEach { it.clear() }
        sampleIndex = 0f
        listOf(jointChart, angleChart, phaseChart, assistChart).forEach { chart ->
            chart.data?.notifyDataChanged()
            chart.notifyDataSetChanged()
            chart.invalidate()
        }
    }

    private fun isImuPhasePresentation(
        frame: GaitBluetoothBridge.PlotFrame?,
        state: GaitBluetoothBridge.StateSnapshot?,
        hasImuVelocityFrame: Boolean = frame?.let {
            it.leftAngularVelocity != null && it.rightAngularVelocity != null
        } == true,
    ): Boolean {
        val mode = state?.motionMode?.takeIf { it.isNotBlank() && it != "-" }
        return if (mode != null) {
            mode in IMU_PHASE_MODE_KEYS
        } else {
            hasImuVelocityFrame
        }
    }

    private fun buildPlotDiagnosis(nowElapsedMs: Long): String {
        val connector = GaitBluetoothBridge.getConnector(this)
        if (!connector.isConnected()) {
            return if (connector.isConnecting()) {
                "蓝牙连接中，尚未进入稳定收数状态"
            } else {
                "蓝牙未连接或已断开，未接收绘图数据"
            }
        }

        val plotAgeMs = if (lastPlotUpdateElapsedMs > 0L) {
            nowElapsedMs - lastPlotUpdateElapsedMs
        } else {
            Long.MAX_VALUE
        }
        val stateAgeMs = if (lastStateUpdateElapsedMs > 0L) {
            nowElapsedMs - lastStateUpdateElapsedMs
        } else {
            Long.MAX_VALUE
        }

        if (lastPlotUpdateElapsedMs <= 0L) {
            val waitingMs = nowElapsedMs - pageVisibleSinceElapsedMs
            return if (waitingMs <= STARTUP_WAIT_PLOT_MS) {
                "已连接，正在等待首包plot数据"
            } else if (stateAgeMs <= STATE_FRESH_THRESHOLD_MS) {
                "状态通道有更新，但未收到plot帧，可能是流配置未生效"
            } else {
                "已连接但长期没有plot数据，请检查gait端是否在发送实时绘图流"
            }
        }

        if (plotAgeMs > PLOT_STALL_DIAG_THRESHOLD_MS) {
            val bridgeHint = recentBridgeStatusHint(nowElapsedMs)
            return if (stateAgeMs <= STATE_FRESH_THRESHOLD_MS) {
                "plot已${formatAgeSeconds(plotAgeMs)}未更新，状态仍在更新，可能仅绘图流中断$bridgeHint"
            } else {
                "plot与状态都已长时间无更新，可能链路卡住或服务端未推送$bridgeHint"
            }
        }
        return "正常（最近plot更新时间 ${formatAgeSeconds(plotAgeMs)}）"
    }

    private fun recentBridgeStatusHint(nowElapsedMs: Long): String {
        val status = lastBridgeStatusMessage?.trim().orEmpty()
        if (status.isBlank()) {
            return ""
        }
        val ageMs = if (lastBridgeStatusElapsedMs > 0L) nowElapsedMs - lastBridgeStatusElapsedMs else Long.MAX_VALUE
        if (ageMs > 10_000L) {
            return ""
        }
        return "（最近蓝牙状态: $status）"
    }

    private fun formatAgeSeconds(ageMs: Long): String {
        val seconds = ageMs.coerceAtLeast(0L) / 1000.0
        return String.format("%.1fs", seconds)
    }

    private fun formatValue(value: Double): String {
        return String.format("%.3f", value)
    }

    private fun setStatusTextIfChanged(text: String) {
        if (text == lastStatusText) {
            return
        }
        lastStatusText = text
        statusView.text = text
    }

    private fun setRoadConditionTextIfChanged(text: String) {
        if (text == lastRoadConditionText) {
            return
        }
        lastRoadConditionText = text
        roadConditionView.text = text
    }
}
