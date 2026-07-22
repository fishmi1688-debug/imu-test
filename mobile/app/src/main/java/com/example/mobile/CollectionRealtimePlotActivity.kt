package com.example.mobile

import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.ImageButton
import android.widget.LinearLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.core.graphics.ColorUtils
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.google.android.material.button.MaterialButton

class CollectionRealtimePlotActivity : AppCompatActivity(), CollectionRealtimeBridge.Listener {

    private data class PlotBuffer(
        var legends: List<String> = emptyList(),
        val samples: ArrayDeque<FloatArray> = ArrayDeque(),
    )

    private val mainHandler = Handler(Looper.getMainLooper())
    private val redrawRunnable = Runnable {
        redrawPending = false
        redrawChartNow()
    }
    private val durationTickRunnable = object : Runnable {
        override fun run() {
            updateCollectionDurationView()
            if (collectionState.active) {
                mainHandler.postDelayed(this, 1000L)
            }
        }
    }

    private lateinit var deviceTabContainer: LinearLayout
    private lateinit var collectionDurationView: TextView
    private lateinit var selectedDeviceView: TextView
    private lateinit var chartStatusView: TextView
    private lateinit var plotRoot: LinearLayout
    private lateinit var labelControlCard: LinearLayout
    private lateinit var currentLabelValueView: TextView
    private lateinit var labelButtonContainer: LinearLayout
    private lateinit var chart: LineChart
    private lateinit var insoleMatrixCard: LinearLayout
    private lateinit var insoleMatrixRowsContainer: LinearLayout
    private val insoleMatrixCells = mutableListOf<List<TextView>>()

    private val connectedDevices = LinkedHashMap<String, CollectionRealtimeBridge.DeviceMeta>()
    private val tabButtons = LinkedHashMap<String, MaterialButton>()
    private val plotBuffers = LinkedHashMap<String, PlotBuffer>()
    private val insoleMatrixRowCounts = intArrayOf(4, 4, 3, 3, 2, 2)
    private var insoleCellSizePx = 0

    private var selectedAddress: String? = null
    private var redrawPending = false
    private var labelState = CollectionRealtimeBridge.LabelState(false, emptyList(), null)
    private var collectionState = CollectionRealtimeBridge.CollectionState(false, null)

    private val maxPoints = 180
    private val colorPalette by lazy {
        intArrayOf(
            colorOf(R.color.chart_series_primary),
            colorOf(R.color.chart_series_secondary),
            colorOf(R.color.chart_series_tertiary),
            colorOf(R.color.chart_series_quaternary),
            colorOf(R.color.chart_series_quinary),
            colorOf(R.color.chart_series_senary),
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_collection_realtime_plot)

        findViewById<ImageButton>(R.id.backToCollectionButton).setOnClickListener {
            finish()
        }
        findViewById<ImageButton>(R.id.backToCollectionButtonSecondary).setOnClickListener {
            finish()
        }

        plotRoot = findViewById(R.id.collectionRealtimePlotRoot)
        deviceTabContainer = findViewById(R.id.deviceTabContainer)
        collectionDurationView = findViewById(R.id.collectionDurationView)
        selectedDeviceView = findViewById(R.id.selectedPlotDeviceView)
        chartStatusView = findViewById(R.id.collectionPlotStatusView)
        labelControlCard = findViewById(R.id.labelControlCard)
        currentLabelValueView = findViewById(R.id.currentLabelValueView)
        labelButtonContainer = findViewById(R.id.labelButtonContainer)
        chart = findViewById(R.id.collectionRealtimeChart)
        insoleMatrixCard = findViewById(R.id.insoleMatrixCard)
        insoleMatrixRowsContainer = findViewById(R.id.insoleMatrixRowsContainer)

        setupChart()
        setupInsoleMatrixView()
        plotRoot.addOnLayoutChangeListener { _, _, _, _, _, _, _, _, _ ->
            if (insoleMatrixCard.visibility == View.VISIBLE) {
                adjustInsoleMatrixCellSize()
            }
        }
    }

    override fun onStart() {
        super.onStart()
        CollectionRealtimeBridge.addListener(this)
    }

    override fun onStop() {
        CollectionRealtimeBridge.removeListener(this)
        stopDurationTicker()
        super.onStop()
    }

    override fun onDestroy() {
        mainHandler.removeCallbacks(redrawRunnable)
        stopDurationTicker()
        super.onDestroy()
    }

    override fun onDevicesChanged(devices: List<CollectionRealtimeBridge.DeviceMeta>) {
        connectedDevices.clear()
        devices.forEach { device ->
            connectedDevices[device.address] = device
        }

        val validAddresses = connectedDevices.keys.toSet()
        plotBuffers.keys.toList().forEach { address ->
            if (address !in validAddresses) {
                plotBuffers.remove(address)
            }
        }

        if (selectedAddress == null || selectedAddress !in connectedDevices) {
            selectedAddress = connectedDevices.keys.firstOrNull()
        }

        rebuildDeviceTabs()
        updateSelectedDeviceText()
        scheduleRedraw()
    }

    override fun onSample(sample: CollectionRealtimeBridge.Sample) {
        val previousMeta = connectedDevices[sample.address]
        if (previousMeta == null || previousMeta.name != sample.name || previousMeta.type != sample.type || previousMeta.side != sample.side) {
            connectedDevices[sample.address] = CollectionRealtimeBridge.DeviceMeta(
                address = sample.address,
                name = sample.name,
                type = sample.type,
                side = sample.side,
            )
            rebuildDeviceTabs()
        }

        val buffer = plotBuffers.getOrPut(sample.address) { PlotBuffer() }
        if (buffer.legends != sample.legends) {
            buffer.legends = sample.legends
            buffer.samples.clear()
        }
        buffer.samples.addLast(sample.values)
        while (buffer.samples.size > maxPoints) {
            buffer.samples.removeFirstOrNull()
        }

        if (selectedAddress == null) {
            selectedAddress = sample.address
            updateSelectedDeviceText()
            updateTabSelectionState()
        }

        if (selectedAddress == sample.address) {
            scheduleRedraw()
        }
    }

    override fun onLabelStateChanged(state: CollectionRealtimeBridge.LabelState) {
        labelState = state
        rebuildLabelButtons()
        updateCurrentLabelView()
    }

    override fun onCollectionStateChanged(state: CollectionRealtimeBridge.CollectionState) {
        collectionState = state
        updateCollectionDurationView()
        if (state.active) {
            startDurationTicker()
        } else {
            stopDurationTicker()
        }
    }

    private fun setupChart() {
        val axisTextColor = colorOf(R.color.chart_axis_text)
        val axisGridColor = colorOf(R.color.chart_axis_grid)

        chart.description.isEnabled = false
        chart.axisRight.isEnabled = false
        chart.legend.isEnabled = true
        chart.setTouchEnabled(true)
        chart.setDragEnabled(true)
        chart.setScaleEnabled(false)
        chart.setDrawGridBackground(false)
        chart.setBackgroundColor(colorOf(R.color.card_bg))
        chart.setNoDataTextColor(axisTextColor)
        chart.xAxis.setDrawLabels(false)
        chart.xAxis.setDrawGridLines(false)
        chart.xAxis.textColor = axisTextColor
        chart.xAxis.setDrawAxisLine(false)
        chart.axisLeft.setDrawGridLines(true)
        chart.axisLeft.textColor = axisTextColor
        chart.axisLeft.gridColor = axisGridColor
        chart.axisLeft.setDrawAxisLine(false)
        chart.legend.textColor = axisTextColor
    }

    private fun rebuildLabelButtons() {
        if (!labelState.enabled || labelState.labels.isEmpty()) {
            labelControlCard.visibility = View.GONE
            labelButtonContainer.removeAllViews()
            return
        }
        labelControlCard.visibility = View.VISIBLE
        labelButtonContainer.removeAllViews()
        labelState.labels.forEach { label ->
            val button = MaterialButton(this).apply {
                text = label
                isAllCaps = false
                minHeight = dpToPx(34)
                setPadding(dpToPx(12), dpToPx(6), dpToPx(12), dpToPx(6))
                val params = LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.WRAP_CONTENT,
                    ViewGroup.LayoutParams.WRAP_CONTENT,
                ).apply {
                    marginEnd = dpToPx(8)
                }
                layoutParams = params
                if (label == labelState.currentLabel) {
                    setBackgroundResource(R.drawable.bg_device_tab_selected)
                    setTextColor(ContextCompat.getColor(this@CollectionRealtimePlotActivity, R.color.white))
                } else {
                    setBackgroundResource(R.drawable.bg_device_tab_normal)
                    setTextColor(ContextCompat.getColor(this@CollectionRealtimePlotActivity, R.color.header_title))
                }
                setOnClickListener {
                    CollectionRealtimeBridge.setCurrentLabel(label)
                }
            }
            labelButtonContainer.addView(button)
        }
    }

    private fun updateCurrentLabelView() {
        currentLabelValueView.text = if (!labelState.enabled || labelState.currentLabel.isNullOrBlank()) {
            getString(R.string.collection_plot_label_none)
        } else {
            getString(R.string.collection_plot_label_current, labelState.currentLabel)
        }
    }

    private fun rebuildDeviceTabs() {
        deviceTabContainer.removeAllViews()
        tabButtons.clear()

        connectedDevices.values.forEach { device ->
            val tabButton = MaterialButton(this).apply {
                text = device.name
                isAllCaps = false
                minHeight = dpToPx(36)
                setPadding(dpToPx(14), dpToPx(8), dpToPx(14), dpToPx(8))
                val params = LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.WRAP_CONTENT,
                    ViewGroup.LayoutParams.WRAP_CONTENT,
                ).apply {
                    marginEnd = dpToPx(8)
                }
                layoutParams = params
                setOnClickListener {
                    selectedAddress = device.address
                    updateSelectedDeviceText()
                    updateTabSelectionState()
                    scheduleRedraw()
                }
            }
            tabButtons[device.address] = tabButton
            deviceTabContainer.addView(tabButton)
        }

        updateTabSelectionState()
    }

    private fun updateTabSelectionState() {
        tabButtons.forEach { (address, button) ->
            if (address == selectedAddress) {
                button.setBackgroundResource(R.drawable.bg_device_tab_selected)
                button.setTextColor(ContextCompat.getColor(this, R.color.white))
            } else {
                button.setBackgroundResource(R.drawable.bg_device_tab_normal)
                button.setTextColor(ContextCompat.getColor(this, R.color.header_title))
            }
        }
    }

    private fun updateSelectedDeviceText() {
        val selected = selectedAddress?.let { connectedDevices[it] }
        selectedDeviceView.text = if (selected == null) {
            getString(R.string.collection_plot_no_device)
        } else {
            getString(R.string.collection_plot_selected_device, selected.name, selected.type)
        }
    }

    private fun updateCollectionDurationView() {
        val startedAt = collectionState.startedAtMillis
        collectionDurationView.text = if (!collectionState.active || startedAt == null) {
            getString(R.string.collection_plot_duration_idle)
        } else {
            val elapsedMs = (System.currentTimeMillis() - startedAt).coerceAtLeast(0L)
            getString(R.string.collection_plot_duration, formatElapsedDuration(elapsedMs))
        }
    }

    private fun startDurationTicker() {
        mainHandler.removeCallbacks(durationTickRunnable)
        mainHandler.post(durationTickRunnable)
    }

    private fun stopDurationTicker() {
        mainHandler.removeCallbacks(durationTickRunnable)
    }

    private fun formatElapsedDuration(elapsedMs: Long): String {
        val totalSeconds = elapsedMs / 1000L
        val hours = totalSeconds / 3600L
        val minutes = (totalSeconds % 3600L) / 60L
        val seconds = totalSeconds % 60L
        return if (hours > 0L) {
            String.format("%02d:%02d:%02d", hours, minutes, seconds)
        } else {
            String.format("%02d:%02d", minutes, seconds)
        }
    }

    private fun scheduleRedraw() {
        if (redrawPending) {
            return
        }
        redrawPending = true
        mainHandler.postDelayed(redrawRunnable, 50L)
    }

    private fun redrawChartNow() {
        val address = selectedAddress
        if (address == null) {
            showChartPanel()
            chart.clear()
            chart.invalidate()
            clearInsoleMatrix()
            chartStatusView.text = getString(R.string.collection_plot_status_waiting_device)
            return
        }

        val buffer = plotBuffers[address]
        val samples = buffer?.samples?.toList().orEmpty()
        if (samples.isEmpty()) {
            showChartPanel()
            chart.clear()
            chart.invalidate()
            clearInsoleMatrix()
            chartStatusView.text = getString(R.string.collection_plot_status_waiting_data)
            return
        }

        val selectedDevice = connectedDevices[address]
        if (selectedDevice?.type == "insole_v2") {
            val latest = samples.lastOrNull()
            if (latest != null && latest.size >= 18) {
                showInsoleMatrixPanel()
                val side = selectedDevice.side ?: inferSideFromName(selectedDevice.name) ?: "Left"
                renderInsoleMatrix(latest, side)
                chartStatusView.text = getString(
                    R.string.collection_plot_status_insole_matrix,
                    selectedDevice.name,
                )
                return
            }
            showInsoleMatrixPanel()
            clearInsoleMatrix()
            chartStatusView.text = getString(R.string.collection_plot_status_waiting_data)
            return
        }

        showChartPanel()
        val dimensions = samples.maxOfOrNull { it.size } ?: 0
        if (dimensions <= 0) {
            chart.clear()
            chart.invalidate()
            chartStatusView.text = getString(R.string.collection_plot_status_waiting_data)
            return
        }

        val legends = buffer?.legends.orEmpty()
        val sets = mutableListOf<LineDataSet>()
        for (index in 0 until dimensions) {
            val label = legends.getOrNull(index) ?: "Value${index + 1}"
            val dataSet = LineDataSet(mutableListOf(), label).apply {
                color = colorPalette[index % colorPalette.size]
                lineWidth = 2f
                setDrawCircles(false)
                setDrawValues(false)
            }
            samples.forEachIndexed { pointIndex, values ->
                dataSet.addEntry(Entry(pointIndex.toFloat(), values.getOrElse(index) { 0f }))
            }
            sets.add(dataSet)
        }

        chart.data = LineData(*sets.toTypedArray())
        chart.data?.notifyDataChanged()
        chart.notifyDataSetChanged()
        chart.setVisibleXRangeMaximum(maxPoints.toFloat())
        chart.moveViewToX(samples.lastIndex.toFloat())
        chart.invalidate()

        val currentDeviceName = connectedDevices[address]?.name ?: address
        chartStatusView.text = getString(
            R.string.collection_plot_status_points,
            currentDeviceName,
            samples.size,
        )
    }

    private fun setupInsoleMatrixView() {
        insoleMatrixRowsContainer.removeAllViews()
        insoleMatrixCells.clear()
        insoleCellSizePx = dpToPx(66)
        insoleMatrixRowCounts.forEachIndexed { rowIndex, columnCount ->
            val rowLayout = LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER
                if (rowIndex > 0) {
                    val rowParams = LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.WRAP_CONTENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT,
                    ).apply {
                        topMargin = dpToPx(6)
                    }
                    layoutParams = rowParams
                }
            }
            val rowCells = mutableListOf<TextView>()
            repeat(columnCount) {
                val cell = TextView(this).apply {
                    gravity = Gravity.CENTER
                    setTypeface(typeface, Typeface.BOLD)
                    setTextColor(ContextCompat.getColor(this@CollectionRealtimePlotActivity, R.color.header_title))
                    background = buildPressureCellBackground(0f, isPlaceholder = true)
                    setTextSize(TypedValue.COMPLEX_UNIT_SP, 15f)
                }
                val params = LinearLayout.LayoutParams(insoleCellSizePx, insoleCellSizePx).apply {
                    marginStart = dpToPx(4)
                    marginEnd = dpToPx(4)
                }
                rowLayout.addView(cell, params)
                rowCells.add(cell)
            }
            insoleMatrixRowsContainer.addView(rowLayout)
            insoleMatrixCells.add(rowCells)
        }
        clearInsoleMatrix()
        insoleMatrixRowsContainer.post { adjustInsoleMatrixCellSize() }
    }

    private fun showInsoleMatrixPanel() {
        insoleMatrixCard.visibility = View.VISIBLE
        chart.visibility = View.GONE
        adjustInsoleMatrixCellSize()
    }

    private fun showChartPanel() {
        insoleMatrixCard.visibility = View.GONE
        chart.visibility = View.VISIBLE
    }

    private fun renderInsoleMatrix(values: FloatArray, side: String) {
        val rows = buildInsoleRows(values, side)
        val maxValue = rows.flatten().maxOrNull() ?: 0f
        rows.forEachIndexed { rowIndex, row ->
            val cells = insoleMatrixCells.getOrNull(rowIndex).orEmpty()
            cells.forEachIndexed { colIndex, cell ->
                val value = row.getOrNull(colIndex) ?: return@forEachIndexed
                val safeValue = value.coerceAtLeast(0f)
                val ratio = if (maxValue <= 0f) 0f else (safeValue / maxValue).coerceIn(0f, 1f)
                cell.text = safeValue.toInt().toString()
                cell.background = buildPressureCellBackground(ratio, isPlaceholder = false)
                val textColor = if (ratio >= 0.55f) R.color.white else R.color.header_title
                cell.setTextColor(ContextCompat.getColor(this, textColor))
            }
        }
    }

    private fun clearInsoleMatrix() {
        insoleMatrixCells.flatten().forEach { cell ->
            cell.text = "--"
            cell.setTextColor(ContextCompat.getColor(this, R.color.header_title))
            cell.background = buildPressureCellBackground(0f, isPlaceholder = true)
        }
    }

    private fun adjustInsoleMatrixCellSize() {
        if (insoleMatrixCells.isEmpty()) {
            return
        }
        val containerWidth = insoleMatrixRowsContainer.width - insoleMatrixRowsContainer.paddingLeft - insoleMatrixRowsContainer.paddingRight
        val availableHeight = plotRoot.height - insoleMatrixCard.top - insoleMatrixCard.paddingTop - insoleMatrixCard.paddingBottom
        if (containerWidth <= 0 || availableHeight <= 0) {
            return
        }

        val maxColumns = insoleMatrixRowCounts.maxOrNull() ?: return
        val horizontalMargin = dpToPx(4) * 2
        val verticalGap = dpToPx(6)
        val widthBased = (containerWidth - maxColumns * horizontalMargin) / maxColumns
        val heightBased = (availableHeight - (insoleMatrixRowCounts.size - 1) * verticalGap) / insoleMatrixRowCounts.size
        val targetSize = minOf(widthBased, heightBased).coerceIn(dpToPx(20), dpToPx(72))
        if (targetSize == insoleCellSizePx) {
            return
        }
        insoleCellSizePx = targetSize
        val textSizePx = (targetSize * 0.34f).coerceIn(spToPx(9f), spToPx(15f))
        insoleMatrixCells.flatten().forEach { cell ->
            val lp = (cell.layoutParams as? LinearLayout.LayoutParams) ?: return@forEach
            lp.width = targetSize
            lp.height = targetSize
            cell.layoutParams = lp
            cell.setTextSize(TypedValue.COMPLEX_UNIT_PX, textSizePx)
        }
        insoleMatrixRowsContainer.requestLayout()
    }

    private fun buildInsoleRows(values: FloatArray, side: String): List<List<Float>> {
        if (values.size < 18) {
            return listOf(
                listOf(0f, 0f, 0f, 0f),
                listOf(0f, 0f, 0f, 0f),
                listOf(0f, 0f, 0f),
                listOf(0f, 0f, 0f),
                listOf(0f, 0f),
                listOf(0f, 0f),
            )
        }
        val ordered = values.take(18)
        val isRight = side.equals("Right", ignoreCase = true)

        // Channel map provided by user:
        // Left : 18 14 8 2; 17 13 7 1; 16 12 6; 15 11 5; 10 4; 9 3
        // Right: 2 8 14 18; 1 7 13 17; 6 12 16; 5 11 15; 4 10; 3 9
        return if (isRight) {
            listOf(
                listOf(ordered[1], ordered[7], ordered[13], ordered[17]),
                listOf(ordered[0], ordered[6], ordered[12], ordered[16]),
                listOf(ordered[5], ordered[11], ordered[15]),
                listOf(ordered[4], ordered[10], ordered[14]),
                listOf(ordered[3], ordered[9]),
                listOf(ordered[2], ordered[8]),
            )
        } else {
            listOf(
                listOf(ordered[17], ordered[13], ordered[7], ordered[1]),
                listOf(ordered[16], ordered[12], ordered[6], ordered[0]),
                listOf(ordered[15], ordered[11], ordered[5]),
                listOf(ordered[14], ordered[10], ordered[4]),
                listOf(ordered[9], ordered[3]),
                listOf(ordered[8], ordered[2]),
            )
        }
    }

    private fun pressureColor(ratio: Float): Int {
        val clamped = ratio.coerceIn(0f, 1f)
        val cold = colorOf(R.color.pressure_cold)
        val hot = colorOf(R.color.pressure_hot)
        return ColorUtils.blendARGB(cold, hot, clamped)
    }

    private fun buildPressureCellBackground(ratio: Float, isPlaceholder: Boolean): GradientDrawable {
        val fill = if (isPlaceholder) {
            colorOf(R.color.pressure_placeholder_fill)
        } else {
            pressureColor(ratio)
        }
        val stroke = if (isPlaceholder) {
            colorOf(R.color.pressure_placeholder_stroke)
        } else {
            ColorUtils.blendARGB(colorOf(R.color.pressure_active_stroke), fill, 0.4f)
        }
        return GradientDrawable().apply {
            shape = GradientDrawable.RECTANGLE
            cornerRadius = dpToPx(14).toFloat()
            setColor(fill)
            setStroke(dpToPx(1), stroke)
        }
    }

    private fun colorOf(colorResId: Int): Int {
        return ContextCompat.getColor(this, colorResId)
    }

    private fun inferSideFromName(name: String?): String? {
        val text = name.orEmpty()
        return when {
            text.contains("右脚") -> "Right"
            text.contains("左脚") -> "Left"
            else -> null
        }
    }

    private fun dpToPx(dp: Int): Int {
        val density = resources.displayMetrics.density
        return (dp * density).toInt()
    }

    private fun spToPx(sp: Float): Float {
        return TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_SP, sp, resources.displayMetrics)
    }
}
