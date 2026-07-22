package com.example.mobile

import kotlin.math.abs
import kotlin.math.max

data class AssistCurveParams(
    val extT0: Double,
    val extTf: Double,
    val extP: Double,
    val extTmax: Double,
    val flexT0: Double,
    val flexTf: Double,
    val flexP: Double,
    val flexTmax: Double,
    val phaseBias: Double
)

data class AssistCurvePoint(
    val phasePercent: Double,
    val extensionTorque: Double,
    val flexionTorque: Double,
    val totalTorque: Double
)

data class AssistStagePoints(
    val startPhase: Double,
    val peakPhase: Double,
    val endPhase: Double
)

data class AssistCurveSummary(
    val points: List<AssistCurvePoint>,
    val maxAbsTorque: Double,
    val maxExtensionTorque: Double,
    val maxFlexionTorque: Double,
    val maxTotalTorque: Double
)

object AssistCurveMath {
    const val previewEpsilon = 1e-6
    const val defaultPointCount = 201

    fun sample(params: AssistCurveParams, pointCount: Int = defaultPointCount): AssistCurveSummary {
        val points = ArrayList<AssistCurvePoint>(pointCount)
        var maxAbsTorque = 0.0
        var maxExtensionTorque = 0.0
        var maxFlexionTorque = 0.0
        var maxTotalTorque = 0.0

        for (index in 0 until pointCount) {
            val phasePercent = index.toDouble() * 100.0 / (pointCount - 1)
            val phaseNormalized = phasePercent / 100.0
            val shiftedPhase = wrapPhase(phaseNormalized + params.phaseBias)
            val extensionTorque = params.extTmax * quinticWindow(
                shiftedPhase,
                params.extT0,
                params.extTf,
                params.extP
            )
            val flexionTorque = -params.flexTmax * quinticWindow(
                shiftedPhase,
                params.flexT0,
                params.flexTf,
                params.flexP
            )
            val totalTorque = extensionTorque + flexionTorque

            points += AssistCurvePoint(
                phasePercent = phasePercent,
                extensionTorque = extensionTorque,
                flexionTorque = flexionTorque,
                totalTorque = totalTorque
            )

            maxExtensionTorque = max(maxExtensionTorque, extensionTorque)
            maxFlexionTorque = max(maxFlexionTorque, abs(flexionTorque))
            maxTotalTorque = max(maxTotalTorque, abs(totalTorque))
            maxAbsTorque = max(
                maxAbsTorque,
                max(abs(totalTorque), max(abs(extensionTorque), abs(flexionTorque)))
            )
        }

        return AssistCurveSummary(
            points = points,
            maxAbsTorque = maxAbsTorque,
            maxExtensionTorque = maxExtensionTorque,
            maxFlexionTorque = maxFlexionTorque,
            maxTotalTorque = maxTotalTorque
        )
    }

    fun extensionStagePoints(params: AssistCurveParams): AssistStagePoints {
        return stagePoints(params.extT0, params.extTf, params.extP)
    }

    fun flexionStagePoints(params: AssistCurveParams): AssistStagePoints {
        return stagePoints(params.flexT0, params.flexTf, params.flexP)
    }

    fun stagePoints(start: Double, end: Double, peakRatio: Double): AssistStagePoints {
        return AssistStagePoints(
            startPhase = wrapPhase(start),
            peakPhase = peakPhase(start, end, peakRatio),
            endPhase = wrapPhase(end)
        )
    }

    fun peakPhase(start: Double, end: Double, peakRatio: Double): Double {
        val duration = forwardDistance(start, end)
        val ratio = peakRatio.coerceIn(previewEpsilon, 1.0 - previewEpsilon)
        return wrapPhase(start + duration * ratio)
    }

    fun phaseToDisplay(basePhase: Double, phaseBias: Double): Double {
        return wrapPhase(basePhase - phaseBias)
    }

    fun displayToPhase(displayPhase: Double, phaseBias: Double): Double {
        return wrapPhase(displayPhase + phaseBias)
    }

    fun forwardDistance(start: Double, end: Double): Double {
        val normalizedStart = wrapPhase(start)
        val normalizedEnd = wrapPhase(end)
        return if (normalizedEnd >= normalizedStart) {
            normalizedEnd - normalizedStart
        } else {
            (1.0 - normalizedStart) + normalizedEnd
        }
    }

    fun unwrapNear(value: Double, reference: Double): Double {
        var candidate = value
        while (candidate - reference > 0.5) {
            candidate -= 1.0
        }
        while (reference - candidate > 0.5) {
            candidate += 1.0
        }
        return candidate
    }

    fun quinticWindow(phase: Double, t0: Double, tf: Double, peakRatio: Double): Double {
        val start = wrapPhase(t0)
        val end = wrapPhase(tf)
        val peak = peakRatio.coerceIn(previewEpsilon, 1.0 - previewEpsilon)
        val normalizedPhase = wrapPhase(phase)

        if (abs(start - end) <= previewEpsilon) {
            return 0.0
        }

        return if (end >= start) {
            val duration = end - start
            if (duration <= previewEpsilon) {
                return 0.0
            }
            val peakPhase = start + peak * duration
            when {
                normalizedPhase < start || normalizedPhase > end -> 0.0
                normalizedPhase <= peakPhase -> {
                    val denominator = peakPhase - start
                    if (denominator <= previewEpsilon) {
                        0.0
                    } else {
                        quinticMinJerk((normalizedPhase - start) / denominator)
                    }
                }
                else -> {
                    val denominator = end - peakPhase
                    if (denominator <= previewEpsilon) {
                        0.0
                    } else {
                        1.0 - quinticMinJerk((normalizedPhase - peakPhase) / denominator)
                    }
                }
            }
        } else {
            val duration = (1.0 - start) + end
            if (duration <= previewEpsilon) {
                return 0.0
            }
            val distance = if (normalizedPhase >= start) {
                normalizedPhase - start
            } else {
                (1.0 - start) + normalizedPhase
            }
            val peakDistance = peak * duration
            when {
                distance > duration -> 0.0
                distance <= peakDistance -> {
                    if (peakDistance <= previewEpsilon) {
                        0.0
                    } else {
                        quinticMinJerk(distance / peakDistance)
                    }
                }
                else -> {
                    val denominator = duration - peakDistance
                    if (denominator <= previewEpsilon) {
                        0.0
                    } else {
                        1.0 - quinticMinJerk((distance - peakDistance) / denominator)
                    }
                }
            }
        }
    }

    private fun quinticMinJerk(progress: Double): Double {
        val s = progress.coerceIn(0.0, 1.0)
        return 10.0 * s * s * s - 15.0 * s * s * s * s + 6.0 * s * s * s * s * s
    }

    fun wrapPhase(phase: Double): Double {
        val wrapped = phase % 1.0
        return if (wrapped < 0.0) wrapped + 1.0 else wrapped
    }
}
