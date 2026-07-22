package com.example.mobile

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.util.AttributeSet
import android.util.TypedValue
import android.view.MotionEvent
import android.view.View
import androidx.core.content.ContextCompat
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

class AssistCurveEditorView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    companion object {
        private const val phaseMin = -1.0
        private const val phaseMax = 1.0
        private const val phaseStep = 0.01
        private const val torqueMax = 17.0
        private const val torqueStep = 0.1
        private const val phasePointStep = 0.01
        private const val minPhaseGap = 0.02
    }

    private enum class DragTarget {
        NONE,
        EXT_START,
        EXT_PEAK,
        EXT_END,
        FLEX_START,
        FLEX_PEAK,
        FLEX_END,
        EXT_TMAX,
        FLEX_TMAX,
        PHASE_BIAS
    }

    private data class Geometry(
        val graphRect: RectF,
        val zeroY: Float,
        val torqueSliderX: Float,
        val phaseSliderY: Float
    )

    private data class Handle(
        val target: DragTarget,
        val x: Float,
        val y: Float,
        val color: Int
    )

    var onParamsChanged: ((AssistCurveParams) -> Unit)? = null

    private var params = AssistCurveParams(
        extT0 = 0.0,
        extTf = 0.0,
        extP = 0.5,
        extTmax = 0.0,
        flexT0 = 0.0,
        flexTf = 0.0,
        flexP = 0.5,
        flexTmax = 0.0,
        phaseBias = 0.0
    )

    private var activeTarget = DragTarget.NONE
    private val gridPathEffect = DashPathEffect(floatArrayOf(dp(4f), dp(4f)), 0f)
    private val helperPath = Path()

    private val totalCurveColor = ContextCompat.getColor(context, R.color.chart_series_primary)
    private val extensionCurveColor = ContextCompat.getColor(context, R.color.chart_series_tertiary)
    private val flexionCurveColor = ContextCompat.getColor(context, R.color.chart_series_secondary)
    private val axisTextColor = ContextCompat.getColor(context, R.color.chart_axis_text)
    private val axisGridColor = ContextCompat.getColor(context, R.color.chart_axis_grid)
    private val zeroLineColor = ContextCompat.getColor(context, R.color.chart_zero_line)
    private val surfaceColor = ContextCompat.getColor(context, R.color.surface_soft)
    private val surfaceStrokeColor = ContextCompat.getColor(context, R.color.card_stroke)

    private val panelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = surfaceColor
        style = Paint.Style.FILL
    }
    private val panelStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = surfaceStrokeColor
        style = Paint.Style.STROKE
        strokeWidth = dp(1f)
    }
    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = axisGridColor
        style = Paint.Style.STROKE
        strokeWidth = dp(1f)
        pathEffect = gridPathEffect
    }
    private val zeroPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = zeroLineColor
        style = Paint.Style.STROKE
        strokeWidth = dp(1.5f)
    }
    private val axisTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = axisTextColor
        textSize = sp(11f)
        textAlign = Paint.Align.CENTER
    }
    private val smallLabelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = axisTextColor
        textSize = sp(10f)
        textAlign = Paint.Align.LEFT
    }
    private val totalCurvePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = totalCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(2.8f)
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
    }
    private val extensionCurvePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = extensionCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(2f)
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        pathEffect = DashPathEffect(floatArrayOf(dp(8f), dp(6f)), 0f)
    }
    private val flexionCurvePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = flexionCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(2f)
        strokeCap = Paint.Cap.ROUND
        strokeJoin = Paint.Join.ROUND
        pathEffect = DashPathEffect(floatArrayOf(dp(8f), dp(6f)), 0f)
    }
    private val helperPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = axisGridColor
        style = Paint.Style.STROKE
        strokeWidth = dp(1.2f)
    }
    private val sliderTrackPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = axisGridColor
        style = Paint.Style.STROKE
        strokeWidth = dp(4f)
        strokeCap = Paint.Cap.ROUND
    }
    private val phaseSliderActivePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = totalCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(4f)
        strokeCap = Paint.Cap.ROUND
    }
    private val extensionSliderActivePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = extensionCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(4f)
        strokeCap = Paint.Cap.ROUND
    }
    private val flexionSliderActivePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = flexionCurveColor
        style = Paint.Style.STROKE
        strokeWidth = dp(4f)
        strokeCap = Paint.Cap.ROUND
    }
    private val handleFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.FILL
    }
    private val handleStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = dp(2.2f)
    }
    private val handleGlowPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        alpha = 48
    }

    init {
        isClickable = true
        isFocusable = true
        minimumHeight = dp(340f).roundToInt()
    }

    fun setParams(newParams: AssistCurveParams) {
        if (params == newParams) {
            invalidate()
            return
        }
        params = newParams
        invalidate()
    }

    fun getParams(): AssistCurveParams = params

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val desiredHeight = dp(340f).roundToInt() + paddingTop + paddingBottom
        val resolvedHeight = resolveSize(desiredHeight, heightMeasureSpec)
        super.onMeasure(widthMeasureSpec, MeasureSpec.makeMeasureSpec(resolvedHeight, MeasureSpec.EXACTLY))
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val geometry = geometry()
        val summary = AssistCurveMath.sample(params)
        val axisHalfRange = max(1.0, summary.maxAbsTorque * 1.15)

        canvas.drawRoundRect(
            RectF(
                paddingLeft.toFloat(),
                paddingTop.toFloat(),
                width - paddingRight.toFloat(),
                height - paddingBottom.toFloat()
            ),
            dp(18f),
            dp(18f),
            panelPaint
        )
        canvas.drawRoundRect(
            RectF(
                paddingLeft.toFloat(),
                paddingTop.toFloat(),
                width - paddingRight.toFloat(),
                height - paddingBottom.toFloat()
            ),
            dp(18f),
            dp(18f),
            panelStrokePaint
        )

        drawGrid(canvas, geometry)
        drawCurves(canvas, geometry, summary, axisHalfRange)
        drawPhaseAxisLabels(canvas, geometry, axisHalfRange)
        drawTorqueSliders(canvas, geometry)
        drawPhaseBiasSlider(canvas, geometry)
        drawHandles(canvas, geometry, axisHalfRange)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        val geometry = geometry()
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                val target = findTarget(event.x, event.y, geometry)
                if (target == DragTarget.NONE) {
                    return super.onTouchEvent(event)
                }
                parent?.requestDisallowInterceptTouchEvent(true)
                activeTarget = target
                updateTarget(event.x, event.y, geometry)
                invalidate()
                return true
            }

            MotionEvent.ACTION_MOVE -> {
                if (activeTarget == DragTarget.NONE) {
                    return super.onTouchEvent(event)
                }
                parent?.requestDisallowInterceptTouchEvent(true)
                updateTarget(event.x, event.y, geometry)
                invalidate()
                return true
            }

            MotionEvent.ACTION_UP -> {
                if (activeTarget == DragTarget.NONE) {
                    return super.onTouchEvent(event)
                }
                updateTarget(event.x, event.y, geometry)
                activeTarget = DragTarget.NONE
                parent?.requestDisallowInterceptTouchEvent(false)
                invalidate()
                performClick()
                return true
            }

            MotionEvent.ACTION_CANCEL -> {
                if (activeTarget != DragTarget.NONE) {
                    activeTarget = DragTarget.NONE
                    parent?.requestDisallowInterceptTouchEvent(false)
                    invalidate()
                    return true
                }
            }
        }
        return super.onTouchEvent(event)
    }

    override fun performClick(): Boolean {
        return super.performClick()
    }

    private fun geometry(): Geometry {
        val outer = RectF(
            paddingLeft + dp(10f),
            paddingTop + dp(12f),
            width - paddingRight - dp(10f),
            height - paddingBottom - dp(12f)
        )
        val graphLeft = outer.left + dp(64f)
        val graphTop = outer.top + dp(16f)
        val graphRight = outer.right - dp(12f)
        val graphBottom = outer.bottom - dp(78f)
        val graphRect = RectF(graphLeft, graphTop, graphRight, graphBottom)
        return Geometry(
            graphRect = graphRect,
            zeroY = (graphTop + graphBottom) * 0.5f,
            torqueSliderX = graphLeft - dp(28f),
            phaseSliderY = graphBottom + dp(46f)
        )
    }

    private fun drawGrid(canvas: Canvas, geometry: Geometry) {
        val graph = geometry.graphRect
        val zeroY = geometry.zeroY
        val horizontalTicks = listOf(
            graph.top,
            graph.top + graph.height() * 0.25f,
            zeroY,
            graph.bottom - graph.height() * 0.25f,
            graph.bottom
        )
        for (y in horizontalTicks) {
            if (abs(y - zeroY) <= 0.5f) {
                canvas.drawLine(graph.left, y, graph.right, y, zeroPaint)
            } else {
                canvas.drawLine(graph.left, y, graph.right, y, gridPaint)
            }
        }
        for (tick in 0..4) {
            val x = graph.left + graph.width() * tick / 4f
            canvas.drawLine(x, graph.top, x, graph.bottom, gridPaint)
        }
    }

    private fun drawCurves(
        canvas: Canvas,
        geometry: Geometry,
        summary: AssistCurveSummary,
        axisHalfRange: Double
    ) {
        val totalPath = Path()
        val extensionPath = Path()
        val flexionPath = Path()
        summary.points.forEachIndexed { index, point ->
            // 曲线采样点按 0% -> 100% 顺序绘制，末点必须落在右边界，不能再 wrap 回 0%。
            val x = geometry.graphRect.left +
                geometry.graphRect.width() * (point.phasePercent / 100.0).toFloat()
            val totalY = torqueToY(point.totalTorque, geometry.graphRect, axisHalfRange)
            val extensionY = torqueToY(point.extensionTorque, geometry.graphRect, axisHalfRange)
            val flexionY = torqueToY(point.flexionTorque, geometry.graphRect, axisHalfRange)
            if (index == 0) {
                totalPath.moveTo(x, totalY)
                extensionPath.moveTo(x, extensionY)
                flexionPath.moveTo(x, flexionY)
            } else {
                totalPath.lineTo(x, totalY)
                extensionPath.lineTo(x, extensionY)
                flexionPath.lineTo(x, flexionY)
            }
        }
        canvas.drawPath(extensionPath, extensionCurvePaint)
        canvas.drawPath(flexionPath, flexionCurvePaint)
        canvas.drawPath(totalPath, totalCurvePaint)
    }

    private fun drawPhaseAxisLabels(canvas: Canvas, geometry: Geometry, axisHalfRange: Double) {
        val graph = geometry.graphRect
        val labelY = graph.bottom + dp(18f)
        for (tick in 0..4) {
            val phasePercent = tick * 25
            val x = graph.left + graph.width() * tick / 4f
            canvas.drawText(phasePercent.toString(), x, labelY, axisTextPaint)
        }
        canvas.drawText("0", graph.left - dp(16f), geometry.zeroY + dp(4f), axisTextPaint)
        smallLabelPaint.textAlign = Paint.Align.RIGHT
        canvas.drawText(
            "+${formatNumber(axisHalfRange, 1)}",
            graph.right - dp(2f),
            graph.top - dp(4f),
            smallLabelPaint
        )
        canvas.drawText(
            "-${formatNumber(axisHalfRange, 1)}",
            graph.right - dp(2f),
            graph.bottom + dp(12f),
            smallLabelPaint
        )
        smallLabelPaint.textAlign = Paint.Align.LEFT
    }

    private fun drawTorqueSliders(canvas: Canvas, geometry: Geometry) {
        val x = geometry.torqueSliderX
        val zeroY = geometry.zeroY
        val graph = geometry.graphRect
        canvas.drawLine(x, graph.top, x, zeroY, sliderTrackPaint)
        canvas.drawLine(x, zeroY, x, graph.bottom, sliderTrackPaint)

        val extHandleY = extensionTorqueHandleY(geometry)
        val flexHandleY = flexionTorqueHandleY(geometry)
        canvas.drawLine(x, zeroY, x, extHandleY, extensionSliderActivePaint)
        canvas.drawLine(x, zeroY, x, flexHandleY, flexionSliderActivePaint)

        drawSliderHandle(
            canvas = canvas,
            x = x,
            y = extHandleY,
            color = extensionCurveColor,
            active = activeTarget == DragTarget.EXT_TMAX
        )
        drawSliderHandle(
            canvas = canvas,
            x = x,
            y = flexHandleY,
            color = flexionCurveColor,
            active = activeTarget == DragTarget.FLEX_TMAX
        )

        smallLabelPaint.textAlign = Paint.Align.CENTER
        canvas.drawText("伸", x, graph.top - dp(6f), smallLabelPaint)
        canvas.drawText("屈", x, graph.bottom + dp(26f), smallLabelPaint)
        canvas.drawText(formatNumber(params.extTmax, 1), x - dp(20f), extHandleY + dp(4f), smallLabelPaint)
        canvas.drawText(formatNumber(params.flexTmax, 1), x - dp(20f), flexHandleY + dp(4f), smallLabelPaint)
        smallLabelPaint.textAlign = Paint.Align.LEFT
    }

    private fun drawPhaseBiasSlider(canvas: Canvas, geometry: Geometry) {
        val startX = geometry.graphRect.left
        val endX = geometry.graphRect.right
        val y = geometry.phaseSliderY
        val handleX = phaseBiasHandleX(geometry)
        canvas.drawLine(startX, y, endX, y, sliderTrackPaint)
        canvas.drawLine(startX, y, handleX, y, phaseSliderActivePaint)
        drawSliderHandle(
            canvas = canvas,
            x = handleX,
            y = y,
            color = totalCurveColor,
            active = activeTarget == DragTarget.PHASE_BIAS
        )

        axisTextPaint.textAlign = Paint.Align.LEFT
        canvas.drawText(
            "相位偏置 ${formatNumber(params.phaseBias, 3)}",
            startX,
            y - dp(14f),
            axisTextPaint
        )
        smallLabelPaint.textAlign = Paint.Align.LEFT
        canvas.drawText("-1", startX, y + dp(20f), smallLabelPaint)
        smallLabelPaint.textAlign = Paint.Align.CENTER
        canvas.drawText("0", (startX + endX) * 0.5f, y + dp(20f), smallLabelPaint)
        smallLabelPaint.textAlign = Paint.Align.RIGHT
        canvas.drawText("1", endX, y + dp(20f), smallLabelPaint)
        axisTextPaint.textAlign = Paint.Align.CENTER
        smallLabelPaint.textAlign = Paint.Align.LEFT
    }

    private fun drawHandles(canvas: Canvas, geometry: Geometry, axisHalfRange: Double) {
        val handles = handles(geometry, axisHalfRange)
        val zeroY = geometry.zeroY
        for (handle in handles) {
            if (handle.target == DragTarget.EXT_PEAK || handle.target == DragTarget.FLEX_PEAK) {
                helperPath.reset()
                helperPath.moveTo(handle.x, zeroY)
                helperPath.lineTo(handle.x, handle.y)
                canvas.drawPath(helperPath, helperPaint)
            }
            drawSliderHandle(
                canvas = canvas,
                x = handle.x,
                y = handle.y,
                color = handle.color,
                active = activeTarget == handle.target
            )
        }
    }

    private fun handles(geometry: Geometry, axisHalfRange: Double): List<Handle> {
        val extPoints = AssistCurveMath.extensionStagePoints(params)
        val flexPoints = AssistCurveMath.flexionStagePoints(params)
        val graph = geometry.graphRect
        return listOf(
            Handle(
                target = DragTarget.EXT_START,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(extPoints.startPhase, params.phaseBias),
                    graph
                ),
                y = geometry.zeroY,
                color = extensionCurveColor
            ),
            Handle(
                target = DragTarget.EXT_PEAK,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(extPoints.peakPhase, params.phaseBias),
                    graph
                ),
                y = torqueToY(params.extTmax, graph, axisHalfRange),
                color = extensionCurveColor
            ),
            Handle(
                target = DragTarget.EXT_END,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(extPoints.endPhase, params.phaseBias),
                    graph
                ),
                y = geometry.zeroY,
                color = extensionCurveColor
            ),
            Handle(
                target = DragTarget.FLEX_START,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(flexPoints.startPhase, params.phaseBias),
                    graph
                ),
                y = geometry.zeroY,
                color = flexionCurveColor
            ),
            Handle(
                target = DragTarget.FLEX_PEAK,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(flexPoints.peakPhase, params.phaseBias),
                    graph
                ),
                y = torqueToY(-params.flexTmax, graph, axisHalfRange),
                color = flexionCurveColor
            ),
            Handle(
                target = DragTarget.FLEX_END,
                x = phaseToX(
                    AssistCurveMath.phaseToDisplay(flexPoints.endPhase, params.phaseBias),
                    graph
                ),
                y = geometry.zeroY,
                color = flexionCurveColor
            )
        )
    }

    private fun findTarget(x: Float, y: Float, geometry: Geometry): DragTarget {
        val handles = handles(
            geometry = geometry,
            axisHalfRange = max(1.0, AssistCurveMath.sample(params).maxAbsTorque * 1.15)
        )
        val handleTouchRadius = dp(16f)
        val nearestHandle = handles.minByOrNull { handle ->
            val dx = handle.x - x
            val dy = handle.y - y
            dx * dx + dy * dy
        }
        if (nearestHandle != null) {
            val dx = nearestHandle.x - x
            val dy = nearestHandle.y - y
            if (dx * dx + dy * dy <= handleTouchRadius * handleTouchRadius) {
                return nearestHandle.target
            }
        }

        val torqueHitHalfWidth = dp(20f)
        if (abs(x - geometry.torqueSliderX) <= torqueHitHalfWidth) {
            if (y in geometry.graphRect.top - torqueHitHalfWidth..geometry.zeroY + torqueHitHalfWidth) {
                return DragTarget.EXT_TMAX
            }
            if (y in geometry.zeroY - torqueHitHalfWidth..geometry.graphRect.bottom + torqueHitHalfWidth) {
                return DragTarget.FLEX_TMAX
            }
        }

        val phaseHitHalfHeight = dp(20f)
        if (abs(y - geometry.phaseSliderY) <= phaseHitHalfHeight &&
            x in geometry.graphRect.left - dp(12f)..geometry.graphRect.right + dp(12f)
        ) {
            return DragTarget.PHASE_BIAS
        }

        return DragTarget.NONE
    }

    private fun updateTarget(x: Float, y: Float, geometry: Geometry) {
        when (activeTarget) {
            DragTarget.NONE -> return
            DragTarget.EXT_START -> updateExtensionStage { start, end, peak ->
                updateStartHandle(start, end, peak, x, geometry)
            }
            DragTarget.EXT_PEAK -> updateExtensionStage { start, end, peak ->
                updatePeakHandle(start, end, peak, x, geometry)
            }
            DragTarget.EXT_END -> updateExtensionStage { start, end, peak ->
                updateEndHandle(start, end, peak, x, geometry)
            }
            DragTarget.FLEX_START -> updateFlexionStage { start, end, peak ->
                updateStartHandle(start, end, peak, x, geometry)
            }
            DragTarget.FLEX_PEAK -> updateFlexionStage { start, end, peak ->
                updatePeakHandle(start, end, peak, x, geometry)
            }
            DragTarget.FLEX_END -> updateFlexionStage { start, end, peak ->
                updateEndHandle(start, end, peak, x, geometry)
            }
            DragTarget.EXT_TMAX -> updateParams {
                it.copy(extTmax = snapTorque(extensionTorqueFromY(y, geometry)))
            }
            DragTarget.FLEX_TMAX -> updateParams {
                it.copy(flexTmax = snapTorque(flexionTorqueFromY(y, geometry)))
            }
            DragTarget.PHASE_BIAS -> updateParams {
                it.copy(phaseBias = snapPhaseBias(phaseBiasFromX(x, geometry)))
            }
        }
    }

    private fun updateExtensionStage(
        transform: (Double, Double, Double) -> Triple<Double, Double, Double>
    ) {
        val (start, end, peakRatio) = transform(params.extT0, params.extTf, params.extP)
        updateParams {
            it.copy(extT0 = start, extTf = end, extP = peakRatio)
        }
    }

    private fun updateFlexionStage(
        transform: (Double, Double, Double) -> Triple<Double, Double, Double>
    ) {
        val (start, end, peakRatio) = transform(params.flexT0, params.flexTf, params.flexP)
        updateParams {
            it.copy(flexT0 = start, flexTf = end, flexP = peakRatio)
        }
    }

    private fun updateStartHandle(
        start: Double,
        end: Double,
        peakRatio: Double,
        x: Float,
        geometry: Geometry
    ): Triple<Double, Double, Double> {
        val stageStart = AssistCurveMath.wrapPhase(start)
        val stageDuration = AssistCurveMath.forwardDistance(stageStart, end).coerceAtLeast(minPhaseGap * 2)
        val stagePeak = stageStart + stageDuration * peakRatio.coerceIn(0.01, 0.99)
        val stageEnd = stageStart + stageDuration
        val draggedPhase = displayToBasePhase(x, geometry)
        var newStart = AssistCurveMath.unwrapNear(draggedPhase, stageStart)
        val lowerBound = stageEnd - (1.0 - minPhaseGap)
        val upperBound = stagePeak - minPhaseGap
        newStart = newStart.coerceIn(lowerBound, upperBound)
        val newDuration = (stageEnd - newStart).coerceAtLeast(minPhaseGap * 2)
        val newPeakRatio = ((stagePeak - newStart) / newDuration).coerceIn(0.01, 0.99)
        return snapStageUpdate(newStart, stageEnd, newPeakRatio)
    }

    private fun updateEndHandle(
        start: Double,
        end: Double,
        peakRatio: Double,
        x: Float,
        geometry: Geometry
    ): Triple<Double, Double, Double> {
        val stageStart = AssistCurveMath.wrapPhase(start)
        val stageDuration = AssistCurveMath.forwardDistance(stageStart, end).coerceAtLeast(minPhaseGap * 2)
        val stagePeak = stageStart + stageDuration * peakRatio.coerceIn(0.01, 0.99)
        val stageEnd = stageStart + stageDuration
        val draggedPhase = displayToBasePhase(x, geometry)
        var newEnd = AssistCurveMath.unwrapNear(draggedPhase, stageEnd)
        val lowerBound = stagePeak + minPhaseGap
        val upperBound = stageStart + (1.0 - minPhaseGap)
        newEnd = newEnd.coerceIn(lowerBound, upperBound)
        val newDuration = (newEnd - stageStart).coerceAtLeast(minPhaseGap * 2)
        val newPeakRatio = ((stagePeak - stageStart) / newDuration).coerceIn(0.01, 0.99)
        return snapStageUpdate(stageStart, newEnd, newPeakRatio)
    }

    private fun updatePeakHandle(
        start: Double,
        end: Double,
        peakRatio: Double,
        x: Float,
        geometry: Geometry
    ): Triple<Double, Double, Double> {
        val stageStart = AssistCurveMath.wrapPhase(start)
        val stageDuration = AssistCurveMath.forwardDistance(stageStart, end).coerceAtLeast(minPhaseGap * 2)
        val stagePeak = stageStart + stageDuration * peakRatio.coerceIn(0.01, 0.99)
        val stageEnd = stageStart + stageDuration
        val draggedPhase = displayToBasePhase(x, geometry)
        var newPeak = AssistCurveMath.unwrapNear(draggedPhase, stagePeak)
        newPeak = newPeak.coerceIn(stageStart + minPhaseGap, stageEnd - minPhaseGap)
        val newPeakRatio = ((newPeak - stageStart) / stageDuration).coerceIn(0.01, 0.99)
        return snapStageUpdate(stageStart, stageEnd, newPeakRatio)
    }

    private fun snapStageUpdate(
        rawStart: Double,
        rawEnd: Double,
        rawPeakRatio: Double
    ): Triple<Double, Double, Double> {
        var start = snapPhasePoint(AssistCurveMath.wrapPhase(rawStart))
        var end = snapPhasePoint(AssistCurveMath.wrapPhase(rawEnd))
        var duration = AssistCurveMath.forwardDistance(start, end)
        if (duration < minPhaseGap * 2) {
            end = snapPhasePoint(AssistCurveMath.wrapPhase(start + minPhaseGap * 2))
            duration = AssistCurveMath.forwardDistance(start, end).coerceAtLeast(minPhaseGap * 2)
        }
        val minimumRatio = min(0.49, minPhaseGap / duration)
        val maximumRatio = max(0.51, 1.0 - minimumRatio)
        val peakRatio = snapPeakRatio(rawPeakRatio.coerceIn(minimumRatio, maximumRatio))
        start = snapPhasePoint(start)
        end = snapPhasePoint(end)
        return Triple(start, end, peakRatio)
    }

    private fun updateParams(transform: (AssistCurveParams) -> AssistCurveParams) {
        val updated = transform(params)
        if (updated == params) {
            return
        }
        params = updated
        onParamsChanged?.invoke(updated)
        invalidate()
    }

    private fun extensionTorqueHandleY(geometry: Geometry): Float {
        val ratio = (params.extTmax / torqueMax).coerceIn(0.0, 1.0).toFloat()
        return geometry.zeroY - (geometry.zeroY - geometry.graphRect.top) * ratio
    }

    private fun flexionTorqueHandleY(geometry: Geometry): Float {
        val ratio = (params.flexTmax / torqueMax).coerceIn(0.0, 1.0).toFloat()
        return geometry.zeroY + (geometry.graphRect.bottom - geometry.zeroY) * ratio
    }

    private fun phaseBiasHandleX(geometry: Geometry): Float {
        val ratio = ((params.phaseBias - phaseMin) / (phaseMax - phaseMin)).coerceIn(0.0, 1.0)
        return geometry.graphRect.left + geometry.graphRect.width() * ratio.toFloat()
    }

    private fun extensionTorqueFromY(y: Float, geometry: Geometry): Double {
        val ratio = ((geometry.zeroY - y) / (geometry.zeroY - geometry.graphRect.top))
            .coerceIn(0f, 1f)
        return torqueMax * ratio
    }

    private fun flexionTorqueFromY(y: Float, geometry: Geometry): Double {
        val ratio = ((y - geometry.zeroY) / (geometry.graphRect.bottom - geometry.zeroY))
            .coerceIn(0f, 1f)
        return torqueMax * ratio
    }

    private fun phaseBiasFromX(x: Float, geometry: Geometry): Double {
        val ratio = ((x - geometry.graphRect.left) / geometry.graphRect.width()).coerceIn(0f, 1f)
        return phaseMin + (phaseMax - phaseMin) * ratio
    }

    private fun displayToBasePhase(x: Float, geometry: Geometry): Double {
        val displayPhase = xToPhase(x, geometry.graphRect)
        return AssistCurveMath.displayToPhase(displayPhase, params.phaseBias)
    }

    private fun phaseToX(phase: Double, graphRect: RectF): Float {
        return graphRect.left + graphRect.width() * AssistCurveMath.wrapPhase(phase).toFloat()
    }

    private fun xToPhase(x: Float, graphRect: RectF): Double {
        val ratio = ((x - graphRect.left) / graphRect.width()).coerceIn(0f, 1f)
        return ratio.toDouble()
    }

    private fun torqueToY(value: Double, graphRect: RectF, axisHalfRange: Double): Float {
        val normalized = ((value / axisHalfRange) + 1.0) * 0.5
        val clamped = normalized.coerceIn(0.0, 1.0)
        return graphRect.bottom - graphRect.height() * clamped.toFloat()
    }

    private fun drawSliderHandle(
        canvas: Canvas,
        x: Float,
        y: Float,
        color: Int,
        active: Boolean
    ) {
        val radius = if (active) dp(9f) else dp(7f)
        if (active) {
            handleGlowPaint.color = color
            canvas.drawCircle(x, y, radius + dp(6f), handleGlowPaint)
        }
        handleStrokePaint.color = color
        canvas.drawCircle(x, y, radius, handleFillPaint)
        canvas.drawCircle(x, y, radius, handleStrokePaint)
    }

    private fun snapPhasePoint(value: Double): Double {
        return snap(value, 0.0, 1.0, phasePointStep)
    }

    private fun snapPeakRatio(value: Double): Double {
        return snap(value, 0.01, 0.99, phasePointStep)
    }

    private fun snapPhaseBias(value: Double): Double {
        return snap(value, phaseMin, phaseMax, phaseStep)
    }

    private fun snapTorque(value: Double): Double {
        return snap(value, 0.0, torqueMax, torqueStep)
    }

    private fun snap(value: Double, min: Double, max: Double, step: Double): Double {
        val clamped = value.coerceIn(min, max)
        val steps = ((clamped - min) / step).roundToInt()
        val snapped = min + steps * step
        return snapped.coerceIn(min, max)
    }

    private fun formatNumber(value: Double, decimals: Int): String {
        return "%.${decimals}f".format(value)
    }

    private fun dp(value: Float): Float {
        return value * resources.displayMetrics.density
    }

    private fun sp(value: Float): Float {
        return TypedValue.applyDimension(
            TypedValue.COMPLEX_UNIT_SP,
            value,
            resources.displayMetrics
        )
    }
}
