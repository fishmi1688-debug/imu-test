package com.example.mobile

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Locale
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.PI
import kotlin.math.abs

object GaitBluetoothBridge {
    private const val ENABLE_TELEMETRY_CSV_RECORDING = false
    private const val PLOT_FORMAT_COMPACT_V1 = "c1"
    private const val PLOT_BINARY_HEADER_SIZE = 8
    private const val PLOT_BINARY_RECORD_SIZE = 18
    private const val PLOT_BINARY_EXT_RECORD_SIZE = 24
    private const val PLOT_BINARY_KIND_SINGLE = 1
    private const val PLOT_BINARY_KIND_BATCH = 2
    private val PLOT_BINARY_MAGIC = byteArrayOf(0x47, 0x42, 0x46, 0x31) // "GBF1"
    private const val STREAM_PLOT_BATCH_SIZE = 10
    private const val STATE_FORMAT_COMPACT_V1 = "c1"
    private const val FLAG_PHASE_ACTIVE = 0
    private const val FLAG_ASSIST_WAIT_NEXT_ZERO = 1
    private const val FLAG_STAIRS_DOWN_MANUAL_ASSIST = 2
    private const val FLAG_ASSIST_ENABLED = 3
    private const val FLAG_ASSIST_ARMED = 4
    private const val FLAG_ASSIST_OUTPUT_ACTIVE = 5
    private const val FLAG_MECHANICAL_ZERO_READY = 6
    private const val FLAG_MOTION_CONFIRMED = 7
    private const val FLAG_TEST_LEFT_PHASE_VALID = 9
    private const val FLAG_TEST_RIGHT_PHASE_VALID = 10
    private const val FLAG_TEST_LEFT_ASSIST_READY = 11
    private const val FLAG_TEST_RIGHT_ASSIST_READY = 12
    private const val FLAG_IMU_CONNECTED = 13
    private const val FLAG_IMU_READY = 14
    private const val FLAG_IMU_STALE = 15
    private const val FLAG_IMU_PHASE_MOTION_ACTIVE = 24
    private const val MAX_PLOT_ANGLE_ABS = 360f
    private const val MAX_PLOT_IMU_ANGLE_ABS = 170f
    private const val MAX_PLOT_IMU_ANGLE_JUMP = 45f
    private const val MAX_PLOT_VELOCITY_ABS = 2000f
    private const val MAX_PLOT_IMU_VELOCITY_JUMP = 900f
    private const val MAX_PLOT_ASSIST_ABS = 17f
    private val TWO_PI = (2.0 * PI).toFloat()

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
        "model_phase",
        "imu_left_ao_phase",
        "imu_ao_phase",
        "walking_diff_test",
    )
    private val IMU_PHASE_MODE_KEYS = setOf(
        "imu_phase",
        "imu_left_phase",
        "model_phase",
        "imu_left_ao_phase",
        "imu_ao_phase",
    )
    private val LEFT_ONLY_IMU_PHASE_MODE_KEYS = setOf(
        "imu_left_phase",
        "imu_left_ao_phase",
    )

    data class PlotFrame(
        val ts: Double,
        val leftAngle: Float,
        val rightAngle: Float,
        val angleDiff: Float,
        val phase: Float,
        val assist: Float,
        val leftAngularVelocity: Float? = null,
        val rightAngularVelocity: Float? = null,
        val rightAssist: Float? = null,
        val receivedElapsedMs: Long = SystemClock.elapsedRealtime(),
    )

    data class StateSnapshot(
        val ts: Double,
        val motionMode: String,
        val gaitState: Int,
        val phaseActive: Boolean,
        val imuPhaseMotionActive: Boolean,
        val assistWaitNextZero: Boolean,
        val manualAssistEnabled: Boolean,
        val assistEnabled: Boolean,
        val assistArmed: Boolean,
        val assistOutputActive: Boolean,
        val mechanicalZeroReady: Boolean,
        val motionConfirmed: Boolean,
        val testLeftPhaseValid: Boolean,
        val testRightPhaseValid: Boolean,
        val testLeftAssistReady: Boolean,
        val testRightAssistReady: Boolean,
        val detectionScore: Double,
        val imuConnected: Boolean?,
        val imuReady: Boolean?,
        val imuStale: Boolean?,
        val imuLastError: String?,
        val imuLabel: String?,
    )

    private const val TAG = "GaitBluetoothBridge"
    private val mainHandler = Handler(Looper.getMainLooper())
    private val lock = Any()
    private val statusListeners = CopyOnWriteArrayList<(String) -> Unit>()
    private val lineListeners = CopyOnWriteArrayList<(String) -> Unit>()
    private val stateListeners = CopyOnWriteArrayList<(StateSnapshot) -> Unit>()
    private val roadConditionListeners = CopyOnWriteArrayList<(String) -> Unit>()
    @Volatile private var telemetryRecorder: GaitTelemetryCsvRecorder? = null
    @Volatile private var connector: GaitBluetoothConnector? = null
    @Volatile private var streamProfileConfiguredForSession = false
    private var latestPlotFrame: PlotFrame? = null
    private var latestStateSnapshot: StateSnapshot? = null

    fun getConnector(context: Context): GaitBluetoothConnector {
        val recorder = if (ENABLE_TELEMETRY_CSV_RECORDING) {
            telemetryRecorder ?: GaitTelemetryCsvRecorder(context.applicationContext) { message ->
                for (listener in statusListeners) {
                    runCatching { listener(message) }
                        .onFailure { Log.w(TAG, "status listener failed", it) }
                }
            }.also { telemetryRecorder = it }
        } else {
            null
        }

        val existing = connector
        if (existing != null) {
            return existing
        }
        val created = GaitBluetoothConnector(context.applicationContext) { message ->
            if (message.contains("蓝牙连接已断开")) {
                streamProfileConfiguredForSession = false
            } else if (message.startsWith("BLE已连接") || message.startsWith("已连接:")) {
                streamProfileConfiguredForSession = false
            }
            for (listener in statusListeners) {
                runCatching { listener(message) }
                    .onFailure { Log.w(TAG, "status listener failed", it) }
            }
        }
        created.setLineCallback { line ->
            runCatching { handleIncomingLine(line, recorder) }
                .onFailure { Log.w(TAG, "incoming line dispatch failed", it) }
        }
        created.setBinaryFrameCallback { frame ->
            runCatching { handleIncomingBinaryFrame(frame, recorder) }
                .onFailure { Log.w(TAG, "incoming binary frame dispatch failed", it) }
        }
        connector = created
        return created
    }

    fun addStatusListener(listener: (String) -> Unit) {
        statusListeners.add(listener)
    }

    fun removeStatusListener(listener: (String) -> Unit) {
        statusListeners.remove(listener)
    }

    fun addLineListener(listener: (String) -> Unit) {
        lineListeners.add(listener)
    }

    fun removeLineListener(listener: (String) -> Unit) {
        lineListeners.remove(listener)
    }

    fun addStateListener(listener: (StateSnapshot) -> Unit) {
        stateListeners.add(listener)
        snapshotLatestState()?.let { snapshot ->
            postOnMain {
                runCatching { listener(snapshot) }
                    .onFailure { Log.w(TAG, "initial state listener failed", it) }
            }
        }
    }

    fun removeStateListener(listener: (StateSnapshot) -> Unit) {
        stateListeners.remove(listener)
    }

    fun addRoadConditionListener(listener: (String) -> Unit) {
        roadConditionListeners.add(listener)
    }

    fun removeRoadConditionListener(listener: (String) -> Unit) {
        roadConditionListeners.remove(listener)
    }

    fun snapshotLatestPlotFrame(): PlotFrame? {
        return synchronized(lock) { latestPlotFrame }
    }

    fun snapshotLatestState(): StateSnapshot? {
        return synchronized(lock) { latestStateSnapshot }
    }

    fun notifyRoadCondition(condition: String) {
        for (listener in roadConditionListeners) {
            listener(condition)
        }
    }

    fun onInsoleV2Sample(
        address: String,
        deviceName: String,
        insoleTimestamp: Long,
        footId: Int,
        values: List<Int>
    ) {
        telemetryRecorder?.onInsoleSample(
            address = address,
            deviceName = deviceName,
            insoleTimestamp = insoleTimestamp,
            footId = footId,
            values = values
        )
    }

    private fun handleIncomingLine(line: String, recorder: GaitTelemetryCsvRecorder?) {
        val obj = runCatching { JsonParser.parseString(line).asJsonObject }.getOrNull()
        if (obj == null) {
            dispatchRawLine(line)
            return
        }

        when (GaitProtocol.resolveMessageType(obj)?.lowercase(Locale.US)) {
            "plot" -> {
                val frame = obj.toPlotFrame() ?: return
                acceptPlotFrame(frame, recorder)
            }
            "plot_batch" -> {
                val frames = obj.toPlotFrames()
                if (frames.isEmpty()) {
                    return
                }
                frames.forEach { frame ->
                    acceptPlotFrame(frame, recorder)
                }
            }
            "state" -> {
                val snapshot = obj.toStateSnapshot() ?: return
                synchronized(lock) {
                    if (latestStateSnapshot?.motionMode != snapshot.motionMode) {
                        latestPlotFrame = null
                    }
                    latestStateSnapshot = snapshot
                }
                recorder?.onStateSnapshot(snapshot)
                dispatchState(snapshot)
                dispatchRawLine(line)
            }
            "pong" -> {
                ensurePreferredStreamProfile()
                dispatchRawLine(line)
            }
            "ack" -> {
                val ok = obj.booleanOrNull("ok") ?: obj.booleanOrNull("o") ?: false
                val message = (
                    obj.stringOrNull("message")
                        ?: when (obj.intOrNull("a")) {
                            1 -> "start_assist"
                            2 -> "stairs_down_assist"
                            3 -> "set_stream"
                            4 -> "mode"
                            5 -> "params"
                            else -> ""
                        }
                    ).lowercase(Locale.US)
                if (ok && (message == "mechanical_zero" || message == "system_already_ready")) {
                    recorder?.onMechanicalZeroAck()
                }
                if (ok && message == "set_stream") {
                    streamProfileConfiguredForSession = true
                }
                dispatchRawLine(line)
            }
            else -> {
                dispatchRawLine(line)
            }
        }
    }

    private fun ensurePreferredStreamProfile() {
        if (streamProfileConfiguredForSession) {
            return
        }
        val activeConnector = connector ?: return
        if (!activeConnector.isConnected()) {
            return
        }
        val payload = JsonObject().apply {
            addProperty("t", GaitProtocol.TYPE_SET_STREAM)
            addProperty("pf", GaitProtocol.STREAM_PLOT_FORMAT_BINARY)
            addProperty("pm", GaitProtocol.STREAM_PLOT_MODE_BATCH)
            addProperty("pn", STREAM_PLOT_BATCH_SIZE)
        }
        if (activeConnector.sendLine(payload.toString())) {
            streamProfileConfiguredForSession = true
        }
    }

    private fun handleIncomingBinaryFrame(frame: ByteArray, recorder: GaitTelemetryCsvRecorder?) {
        val parsedFrames = decodeBinaryPlotFrames(frame)
        if (parsedFrames.isEmpty()) {
            return
        }
        parsedFrames.forEach { plotFrame ->
            acceptPlotFrame(plotFrame, recorder)
        }
    }

    private fun acceptPlotFrame(frame: PlotFrame, recorder: GaitTelemetryCsvRecorder?) {
        val safeFrame = sanitizePlotFrame(frame)
        synchronized(lock) {
            latestPlotFrame = safeFrame
        }
        recorder?.onPlotFrame(safeFrame)
    }

    private fun sanitizePlotFrame(frame: PlotFrame): PlotFrame {
        val (previous, stateSnapshot) = synchronized(lock) {
            latestPlotFrame to latestStateSnapshot
        }
        val isImuPhaseMode = stateSnapshot?.motionMode in IMU_PHASE_MODE_KEYS ||
            frame.leftAngularVelocity != null ||
            frame.rightAngularVelocity != null ||
            frame.rightAssist != null
        val angleMaxAbs = if (isImuPhaseMode) MAX_PLOT_IMU_ANGLE_ABS else MAX_PLOT_ANGLE_ABS
        val angleJump = if (isImuPhaseMode) MAX_PLOT_IMU_ANGLE_JUMP else 0f
        val velocityJump = if (isImuPhaseMode) MAX_PLOT_IMU_VELOCITY_JUMP else 0f
        return frame.copy(
            leftAngle = finiteOrFallback(
                frame.leftAngle,
                previous?.leftAngle ?: 0f,
                angleMaxAbs,
                maxJump = angleJump,
                rejectJump = previous != null,
            ),
            rightAngle = finiteOrFallback(
                frame.rightAngle,
                previous?.rightAngle ?: 0f,
                angleMaxAbs,
                maxJump = angleJump,
                rejectJump = previous != null,
            ),
            angleDiff = finiteOrFallback(
                frame.angleDiff,
                previous?.angleDiff ?: 0f,
                angleMaxAbs * 2f,
                maxJump = angleJump * 2f,
                rejectJump = previous != null,
            ),
            phase = normalizePhaseRad(frame.phase, previous?.phase ?: 0f),
            assist = finiteOrFallback(
                frame.assist,
                previous?.assist ?: 0f,
                MAX_PLOT_ASSIST_ABS,
            ),
            leftAngularVelocity = frame.leftAngularVelocity?.let {
                finiteOrFallback(
                    it,
                    previous?.leftAngularVelocity ?: 0f,
                    MAX_PLOT_VELOCITY_ABS,
                    maxJump = velocityJump,
                    rejectJump = previous?.leftAngularVelocity != null,
                )
            },
            rightAngularVelocity = frame.rightAngularVelocity?.let {
                finiteOrFallback(
                    it,
                    previous?.rightAngularVelocity ?: 0f,
                    MAX_PLOT_VELOCITY_ABS,
                    maxJump = velocityJump,
                    rejectJump = previous?.rightAngularVelocity != null,
                )
            },
            rightAssist = frame.rightAssist?.let {
                finiteOrFallback(
                    it,
                    previous?.rightAssist ?: 0f,
                    MAX_PLOT_ASSIST_ABS,
                )
            },
        )
    }

    private fun finiteOrFallback(
        value: Float,
        fallback: Float,
        maxAbs: Float,
        maxJump: Float = 0f,
        rejectJump: Boolean = false,
    ): Float {
        if (!value.isFinite()) {
            return fallback
        }
        if (maxAbs > 0f && abs(value) > maxAbs) {
            return fallback
        }
        if (rejectJump && maxJump > 0f && abs(value - fallback) > maxJump) {
            return fallback
        }
        return value
    }

    private fun normalizePhaseRad(value: Float, fallback: Float): Float {
        if (!value.isFinite()) {
            return fallback
        }
        var wrapped = value % TWO_PI
        if (wrapped < 0f) {
            wrapped += TWO_PI
        }
        return wrapped
    }

    private fun dispatchRawLine(line: String) {
        if (lineListeners.isEmpty()) {
            return
        }
        postOnMain {
            for (listener in lineListeners) {
                runCatching { listener(line) }
                    .onFailure { Log.w(TAG, "line listener failed", it) }
            }
        }
    }

    private fun dispatchState(snapshot: StateSnapshot) {
        if (stateListeners.isEmpty()) {
            return
        }
        postOnMain {
            for (listener in stateListeners) {
                runCatching { listener(snapshot) }
                    .onFailure { Log.w(TAG, "state listener failed", it) }
            }
        }
    }

    private fun postOnMain(action: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            action()
        } else {
            mainHandler.post(action)
        }
    }

    private fun JsonObject.toPlotFrame(): PlotFrame? {
        val compact = (
            stringOrNull("fmt")?.lowercase(Locale.US) == PLOT_FORMAT_COMPACT_V1
                || (has("l") && has("r") && has("d") && has("p") && has("a"))
            )
        val ts = if (compact) doubleOrNull("x") ?: doubleOrNull("t") else doubleOrNull("ts") ?: doubleOrNull("x") ?: doubleOrNull("t")
        val leftAngle = if (compact) doubleOrNull("l") else doubleOrNull("left_angle") ?: doubleOrNull("l")
        val rightAngle = if (compact) doubleOrNull("r") else doubleOrNull("right_angle") ?: doubleOrNull("r")
        val angleDiff = if (compact) doubleOrNull("d") else doubleOrNull("angle_diff") ?: doubleOrNull("d")
        val phase = if (compact) doubleOrNull("p") else doubleOrNull("phase") ?: doubleOrNull("p")
        val assist = if (compact) doubleOrNull("a") else doubleOrNull("assist") ?: doubleOrNull("a")
        val leftAngularVelocity = doubleOrNull("lv") ?: doubleOrNull("left_velocity")
        val rightAngularVelocity = doubleOrNull("rv") ?: doubleOrNull("right_velocity")
        val rightAssist = doubleOrNull("ra") ?: doubleOrNull("assist_right")
        if (ts == null || leftAngle == null || rightAngle == null || angleDiff == null || phase == null || assist == null) {
            return null
        }
        return PlotFrame(
            ts = ts,
            leftAngle = leftAngle.toFloat(),
            rightAngle = rightAngle.toFloat(),
            angleDiff = angleDiff.toFloat(),
            phase = phase.toFloat(),
            assist = assist.toFloat(),
            leftAngularVelocity = leftAngularVelocity?.toFloat(),
            rightAngularVelocity = rightAngularVelocity?.toFloat(),
            rightAssist = rightAssist?.toFloat(),
        )
    }

    private fun JsonObject.toPlotFrames(): List<PlotFrame> {
        val compact = (
            stringOrNull("fmt")?.lowercase(Locale.US) == PLOT_FORMAT_COMPACT_V1
                || has("f")
            )
        if (compact) {
            val compactArray = getAsJsonArray("f") ?: return emptyList()
            return compactArray.mapNotNull { element ->
                runCatching {
                    val row = element.asJsonArray
                    if (row.size() < 6) return@runCatching null
                    val ts = row[0].asDouble
                    val left = row[1].asDouble
                    val right = row[2].asDouble
                    val diff = row[3].asDouble
                    val phase = row[4].asDouble
                    val assist = row[5].asDouble
                    val leftVelocity = if (row.size() >= 8) row[6].asDouble else null
                    val rightVelocity = if (row.size() >= 8) row[7].asDouble else null
                    val rightAssist = if (row.size() >= 9) row[8].asDouble else null
                    PlotFrame(
                        ts = ts,
                        leftAngle = left.toFloat(),
                        rightAngle = right.toFloat(),
                        angleDiff = diff.toFloat(),
                        phase = phase.toFloat(),
                        assist = assist.toFloat(),
                        leftAngularVelocity = leftVelocity?.toFloat(),
                        rightAngularVelocity = rightVelocity?.toFloat(),
                        rightAssist = rightAssist?.toFloat(),
                    )
                }.getOrNull()
            }
        }
        val array = getAsJsonArray("frames") ?: return emptyList()
        return array.mapNotNull { element ->
            runCatching { element.asJsonObject.toPlotFrame() }.getOrNull()
        }
    }

    private fun decodeBinaryPlotFrames(frame: ByteArray): List<PlotFrame> {
        if (frame.size < PLOT_BINARY_HEADER_SIZE) {
            return emptyList()
        }
        if (!frame.hasBinaryMagic()) {
            return emptyList()
        }
        val kind = frame[4].toInt() and 0xFF
        val count = frame[5].toInt() and 0xFF
        val payloadLength = (frame[6].toInt() and 0xFF) or ((frame[7].toInt() and 0xFF) shl 8)
        if (payloadLength <= 0 || frame.size != PLOT_BINARY_HEADER_SIZE + payloadLength) {
            return emptyList()
        }
        if (kind != PLOT_BINARY_KIND_SINGLE && kind != PLOT_BINARY_KIND_BATCH) {
            return emptyList()
        }
        val recordSize = when (payloadLength) {
            count * PLOT_BINARY_RECORD_SIZE -> PLOT_BINARY_RECORD_SIZE
            count * PLOT_BINARY_EXT_RECORD_SIZE -> PLOT_BINARY_EXT_RECORD_SIZE
            else -> return emptyList()
        }
        if (count <= 0) {
            return emptyList()
        }
        val payload = ByteBuffer.wrap(frame, PLOT_BINARY_HEADER_SIZE, payloadLength)
            .order(ByteOrder.LITTLE_ENDIAN)
        val result = ArrayList<PlotFrame>(count)
        repeat(count) {
            val tsMs = payload.getLong()
            val left = payload.getShort().toInt() / 100.0
            val right = payload.getShort().toInt() / 100.0
            val diff = payload.getShort().toInt() / 100.0
            val phase = payload.getShort().toInt() / 1000.0
            val assist = payload.getShort().toInt() / 100.0
            val leftVelocity: Double?
            val rightVelocity: Double?
            val rightAssist: Double?
            if (recordSize == PLOT_BINARY_EXT_RECORD_SIZE) {
                leftVelocity = payload.getShort().toInt() / 100.0
                rightVelocity = payload.getShort().toInt() / 100.0
                rightAssist = payload.getShort().toInt() / 100.0
            } else {
                leftVelocity = null
                rightVelocity = null
                rightAssist = null
            }
            result.add(
                PlotFrame(
                    ts = tsMs / 1000.0,
                    leftAngle = left.toFloat(),
                    rightAngle = right.toFloat(),
                    angleDiff = diff.toFloat(),
                    phase = phase.toFloat(),
                    assist = assist.toFloat(),
                    leftAngularVelocity = leftVelocity?.toFloat(),
                    rightAngularVelocity = rightVelocity?.toFloat(),
                    rightAssist = rightAssist?.toFloat(),
                )
            )
        }
        return result
    }

    private fun ByteArray.hasBinaryMagic(): Boolean {
        if (size < PLOT_BINARY_MAGIC.size) {
            return false
        }
        for (idx in PLOT_BINARY_MAGIC.indices) {
            if (this[idx] != PLOT_BINARY_MAGIC[idx]) {
                return false
            }
        }
        return true
    }

    private fun JsonObject.toStateSnapshot(): StateSnapshot? {
        val compactFlags = intOrNull("f")
        val compactFormat = stringOrNull("fmt")?.lowercase(Locale.US)
        val compactMode = GaitProtocol.resolveModeKey(this)
            ?: intOrNull("mi")?.let { COMPACT_MODE_KEYS.getOrNull(it) }
        val motionMode = stringOrNull("motion_mode")
            ?: if (
                compactFormat == STATE_FORMAT_COMPACT_V1
                || has("mi")
                || has("gs")
                || has("f")
            ) compactMode else null
        if (motionMode == null) {
            return null
        }

        val assistArmed = booleanOrNull("assist_armed")
            ?: compactFlags?.hasStateFlag(FLAG_ASSIST_ARMED)
            ?: false
        val assistEnabled = booleanOrNull("assist_enabled")
            ?: compactFlags?.hasStateFlag(FLAG_ASSIST_ENABLED)
            ?: assistArmed
        val phaseLeftConnected = booleanOrNull("imu_phase_left_connected")
            ?: booleanOrNull("iplc")
        val phaseRightConnected = booleanOrNull("imu_phase_right_connected")
            ?: booleanOrNull("iprc")
        val phaseLeftReady = booleanOrNull("imu_phase_left_ready")
            ?: booleanOrNull("ipld")
        val phaseRightReady = booleanOrNull("imu_phase_right_ready")
            ?: booleanOrNull("iprd")
        val phaseLeftStale = booleanOrNull("imu_phase_left_stale")
            ?: booleanOrNull("iplz")
        val phaseRightStale = booleanOrNull("imu_phase_right_stale")
            ?: booleanOrNull("iprz")
        val phaseLeftError = stringOrNull("imu_phase_left_last_error")
            ?: stringOrNull("iple")
        val phaseRightError = stringOrNull("imu_phase_right_last_error")
            ?: stringOrNull("ipre")
        val isImuPhaseMode = motionMode in IMU_PHASE_MODE_KEYS
        val isLeftOnlyImuPhaseMode = motionMode in LEFT_ONLY_IMU_PHASE_MODE_KEYS
        val imuConnectedForMode = if (isImuPhaseMode) {
            when {
                phaseLeftConnected != null || phaseRightConnected != null ->
                    phaseLeftConnected == true &&
                        (isLeftOnlyImuPhaseMode || phaseRightConnected == true)
                else -> compactFlags?.hasStateFlag(FLAG_IMU_CONNECTED)
            }
        } else {
            booleanOrNull("imu_connected")
                ?: compactFlags?.hasStateFlag(FLAG_IMU_CONNECTED)
        }
        val imuReadyForMode = if (isImuPhaseMode) {
            when {
                phaseLeftReady != null || phaseRightReady != null ->
                    phaseLeftReady == true &&
                        (isLeftOnlyImuPhaseMode || phaseRightReady == true)
                else -> compactFlags?.hasStateFlag(FLAG_IMU_READY)
            }
        } else {
            booleanOrNull("imu_ready")
                ?: compactFlags?.hasStateFlag(FLAG_IMU_READY)
        }
        val imuStaleForMode = if (isImuPhaseMode) {
            when {
                phaseLeftStale != null || phaseRightStale != null ->
                    phaseLeftStale == true ||
                        (!isLeftOnlyImuPhaseMode && phaseRightStale == true)
                else -> compactFlags?.hasStateFlag(FLAG_IMU_STALE)
            }
        } else {
            booleanOrNull("imu_stale")
                ?: compactFlags?.hasStateFlag(FLAG_IMU_STALE)
        }
        val imuErrorForMode = if (isImuPhaseMode) {
            if (isLeftOnlyImuPhaseMode) {
                phaseLeftError?.takeIf { it.isNotBlank() }?.let { "L:$it" }
            } else {
                listOfNotNull(
                    phaseLeftError?.takeIf { it.isNotBlank() }?.let { "L:$it" },
                    phaseRightError?.takeIf { it.isNotBlank() }?.let { "R:$it" },
                ).joinToString(" ").ifBlank { null }
            }
        } else {
            stringOrNull("imu_last_error")
        }
        val imuLabelForMode = if (isImuPhaseMode) {
            if (isLeftOnlyImuPhaseMode) {
                "L${if (phaseLeftConnected == true) 1 else 0}/R=+pi"
            } else {
                "L${if (phaseLeftConnected == true) 1 else 0}/R${if (phaseRightConnected == true) 1 else 0}"
            }
        } else {
            stringOrNull("imu_label")
        }

        return StateSnapshot(
            ts = doubleOrNull("ts") ?: 0.0,
            motionMode = motionMode,
            gaitState = intOrNull("gait_state") ?: intOrNull("gs") ?: 0,
            phaseActive = booleanOrNull("phase_active")
                ?: compactFlags?.hasStateFlag(FLAG_PHASE_ACTIVE)
                ?: false,
            imuPhaseMotionActive = booleanOrNull("imu_phase_motion_active")
                ?: booleanOrNull("ipm")
                ?: compactFlags?.hasStateFlag(FLAG_IMU_PHASE_MOTION_ACTIVE)
                ?: false,
            assistWaitNextZero = booleanOrNull("assist_wait_next_zero")
                ?: compactFlags?.hasStateFlag(FLAG_ASSIST_WAIT_NEXT_ZERO)
                ?: false,
            manualAssistEnabled = booleanOrNull("stairs_down_manual_assist")
                ?: compactFlags?.hasStateFlag(FLAG_STAIRS_DOWN_MANUAL_ASSIST)
                ?: false,
            assistEnabled = assistEnabled,
            assistArmed = assistArmed,
            assistOutputActive = booleanOrNull("assist_output_active")
                ?: compactFlags?.hasStateFlag(FLAG_ASSIST_OUTPUT_ACTIVE)
                ?: false,
            mechanicalZeroReady = booleanOrNull("mechanical_zero_ready")
                ?: compactFlags?.hasStateFlag(FLAG_MECHANICAL_ZERO_READY)
                ?: false,
            motionConfirmed = booleanOrNull("motion_confirmed")
                ?: compactFlags?.hasStateFlag(FLAG_MOTION_CONFIRMED)
                ?: false,
            testLeftPhaseValid = booleanOrNull("test_left_phase_valid")
                ?: compactFlags?.hasStateFlag(FLAG_TEST_LEFT_PHASE_VALID)
                ?: false,
            testRightPhaseValid = booleanOrNull("test_right_phase_valid")
                ?: compactFlags?.hasStateFlag(FLAG_TEST_RIGHT_PHASE_VALID)
                ?: false,
            testLeftAssistReady = booleanOrNull("test_left_assist_ready")
                ?: compactFlags?.hasStateFlag(FLAG_TEST_LEFT_ASSIST_READY)
                ?: false,
            testRightAssistReady = booleanOrNull("test_right_assist_ready")
                ?: compactFlags?.hasStateFlag(FLAG_TEST_RIGHT_ASSIST_READY)
                ?: false,
            detectionScore = doubleOrNull("detection_score") ?: doubleOrNull("ds") ?: 0.0,
            imuConnected = imuConnectedForMode,
            imuReady = imuReadyForMode,
            imuStale = imuStaleForMode,
            imuLastError = imuErrorForMode,
            imuLabel = imuLabelForMode,
        )
    }

    private fun Int.hasStateFlag(bit: Int): Boolean {
        return (this and (1 shl bit)) != 0
    }

    private fun JsonObject.stringOrNull(key: String): String? {
        val element = get(key) ?: return null
        return runCatching { element.asString }.getOrNull()
    }

    private fun JsonObject.doubleOrNull(key: String): Double? {
        val element = get(key) ?: return null
        return runCatching { element.asDouble }.getOrNull()
    }

    private fun JsonObject.intOrNull(key: String): Int? {
        val element = get(key) ?: return null
        return runCatching { element.asInt }.getOrNull()
    }

    private fun JsonObject.booleanOrNull(key: String): Boolean? {
        return GaitProtocol.boolOrNull(this, key)
    }
}
