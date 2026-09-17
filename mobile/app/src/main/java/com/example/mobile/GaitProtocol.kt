package com.example.mobile

import com.google.gson.JsonElement
import com.google.gson.JsonObject
import java.util.Locale

object GaitProtocol {
    const val TYPE_PING = 1
    const val TYPE_GET_STATE = 2
    const val TYPE_SET_MODE = 3
    const val TYPE_SET_PARAMS = 4
    const val TYPE_COMMAND = 5
    const val TYPE_SET_STREAM = 6
    const val TYPE_IMU_MANAGE = 7
    const val TYPE_STATE = 8
    const val TYPE_PLOT = 9
    const val TYPE_PLOT_BATCH = 10
    const val TYPE_ACK = 11
    const val TYPE_ERROR = 12
    const val TYPE_PONG = 13
    const val TYPE_IMU_MANAGE_ACK = 15

    const val ACTION_START_ASSIST = 1
    const val ACTION_STAIRS_DOWN_ASSIST = 2
    const val ACTION_SET_STREAM = 3
    const val ACTION_SET_MODE = 4
    const val ACTION_SET_PARAMS = 5

    const val COMMAND_START_ASSIST = 1
    const val COMMAND_EMERGENCY_STOP = 2
    const val COMMAND_STAIRS_DOWN_TOGGLE = 3
    const val COMMAND_MECHANICAL_ZERO = 4
    const val COMMAND_MOTOR_ENABLE = 5

    const val STREAM_PLOT_FORMAT_LEGACY = 0
    const val STREAM_PLOT_FORMAT_COMPACT = 1
    const val STREAM_PLOT_FORMAT_BINARY = 2
    const val STREAM_PLOT_MODE_BATCH = 0
    const val STREAM_PLOT_MODE_SAMPLE = 1

    private val modeKeys = listOf(
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

    private val messageTypeCodeToText = mapOf(
        TYPE_PING to "ping",
        TYPE_GET_STATE to "get_state",
        TYPE_SET_MODE to "set_mode",
        TYPE_SET_PARAMS to "set_params",
        TYPE_COMMAND to "command",
        TYPE_SET_STREAM to "set_stream",
        TYPE_IMU_MANAGE to "imu_manage",
        TYPE_STATE to "state",
        TYPE_PLOT to "plot",
        TYPE_PLOT_BATCH to "plot_batch",
        TYPE_ACK to "ack",
        TYPE_ERROR to "error",
        TYPE_PONG to "pong",
        14 to "telemetry",
        TYPE_IMU_MANAGE_ACK to "imu_manage_ack",
        16 to "imu_manage_status",
    )

    private val commandCodeToText = mapOf(
        COMMAND_START_ASSIST to "start_assist",
        COMMAND_EMERGENCY_STOP to "emergency_stop",
        COMMAND_STAIRS_DOWN_TOGGLE to "stairs_down_toggle",
        COMMAND_MECHANICAL_ZERO to "mechanical_zero",
        COMMAND_MOTOR_ENABLE to "motor_enable",
    )

    private val imuSlotCodeToText = mapOf(
        0 to "walking",
        1 to "cycling",
        2 to "imu_phase_left",
        3 to "imu_phase_right",
    )

    private val reasonCodeToText = mapOf(
        0 to "ok",
        1 to "mechanical_zero_failed",
        2 to "motor_enable_failed",
        3 to "manual_prepare_failed",
        4 to "manual_mode_required",
        5 to "unknown_command",
        6 to "unknown_mode",
        7 to "invalid_value",
        8 to "no_params",
        9 to "invalid_plot_format",
        10 to "invalid_plot_mode",
        11 to "invalid_plot_batch_size",
        12 to "invalid_plot_every_n_frames",
        13 to "mode_required",
        14 to "invalid_json",
        15 to "unknown_type",
        16 to "imu_control_failed",
        17 to "invalid_slot",
        18 to "motor_feedback_timeout",
        19 to "imu_rate_check_failed",
    )

    private val paramKeys = listOf(
        "ext_t0",
        "ext_tf",
        "ext_p",
        "ext_Tmax",
        "flex_t0",
        "flex_tf",
        "flex_p",
        "flex_Tmax",
        "phase_bias",
        "phase_bias_at_0p6",
        "phase_bias_slope",
        "event_prob_threshold",
        "swing_threshold",
    )

    fun resolveMessageType(obj: JsonObject): String? {
        stringOrNull(obj, "type")?.let {
            val trimmed = it.trim()
            if (trimmed.isNotEmpty()) {
                val numeric = trimmed.toIntOrNull()
                if (numeric != null) {
                    return messageTypeCodeToText[numeric]
                }
                return trimmed
            }
        }
        val typeCode = intOrNull(obj, "t") ?: return null
        return messageTypeCodeToText[typeCode]
    }

    fun resolveModeKey(obj: JsonObject): String? {
        stringOrNull(obj, "mode")?.let {
            val trimmed = it.trim()
            if (trimmed.isNotEmpty()) {
                return trimmed
            }
        }
        stringOrNull(obj, "m")?.let {
            val trimmed = it.trim()
            if (trimmed.isNotEmpty()) {
                val numeric = trimmed.toIntOrNull()
                if (numeric != null) {
                    return modeKeyFromCode(numeric)
                }
                return trimmed
            }
        }
        val modeCode = intOrNull(obj, "m") ?: intOrNull(obj, "mi") ?: return null
        return modeKeyFromCode(modeCode)
    }

    fun modeCodeFromKey(modeKey: String): Int? {
        return modeKeys.indexOf(modeKey).takeIf { it >= 0 }
    }

    fun modeKeyFromCode(modeCode: Int): String? {
        return modeKeys.getOrNull(modeCode)
    }

    fun commandNameFromCode(commandCode: Int): String? {
        return commandCodeToText[commandCode]
    }

    fun commandCodeFromName(commandName: String): Int? {
        val normalized = commandName.trim().lowercase()
        return commandCodeToText.entries.firstOrNull { it.value == normalized }?.key
    }

    fun resolveAckActionCode(obj: JsonObject): Int? {
        return intOrNull(obj, "a")
    }

    fun reasonTextFromCode(code: Int?): String? {
        return code?.let(reasonCodeToText::get)
    }

    fun formatReasonCode(code: Int?): String {
        if (code == null) {
            return "unknown"
        }
        val text = reasonTextFromCode(code)
        return if (text.isNullOrBlank()) code.toString() else "$code($text)"
    }

    fun imuSlotTextFromCode(slotCode: Int): String? {
        return imuSlotCodeToText[slotCode]
    }

    fun imuSlotCodeFromText(slotKey: String): Int? {
        val normalized = slotKey.trim().lowercase()
        return imuSlotCodeToText.entries.firstOrNull { it.value == normalized }?.key
    }

    fun paramCodeFromKey(key: String): Int? {
        val idx = paramKeys.indexOf(key)
        return if (idx >= 0) idx + 1 else null
    }

    fun paramKeyFromCode(code: Int): String? {
        if (code <= 0) return null
        return paramKeys.getOrNull(code - 1)
    }

    fun boolOrNull(obj: JsonObject, key: String): Boolean? {
        return boolElementOrNull(obj.get(key))
    }

    fun boolElementOrNull(element: JsonElement?): Boolean? {
        val target = element ?: return null
        if (target.isJsonNull) return null
        val primitive = runCatching { target.asJsonPrimitive }.getOrNull() ?: return null
        if (primitive.isBoolean) {
            return runCatching { primitive.asBoolean }.getOrNull()
        }
        if (primitive.isNumber) {
            return runCatching { primitive.asInt != 0 }.getOrNull()
        }
        if (!primitive.isString) {
            return null
        }
        val normalized = runCatching { primitive.asString.trim().lowercase(Locale.US) }
            .getOrNull()
            ?: return null
        return when (normalized) {
            "1", "true", "on", "yes" -> true
            "0", "false", "off", "no" -> false
            else -> normalized.toIntOrNull()?.let { it != 0 }
        }
    }

    fun intOrNull(obj: JsonObject, key: String): Int? {
        val element = obj.get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asInt }.getOrNull()
    }

    fun stringOrNull(obj: JsonObject, key: String): String? {
        val element = obj.get(key) ?: return null
        if (element.isJsonNull) return null
        return runCatching { element.asString }.getOrNull()
    }
}
