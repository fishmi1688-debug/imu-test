package com.example.mobile

import android.content.Context

class CollectionSettingsStore(context: Context) {
    companion object {
        private const val PREFS_NAME = "collection_settings"
        private const val KEY_OUTPUT_RATE = "output_rate"
        private const val KEY_FILTER_PROFILE = "filter_profile"
        private const val KEY_LOGGER_FLAG = "logger_flag"
    }

    private val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    fun getOutputRate(): Int = prefs.getInt(KEY_OUTPUT_RATE, 60)

    fun setOutputRate(rate: Int) {
        prefs.edit().putInt(KEY_OUTPUT_RATE, rate).apply()
    }

    fun getFilterProfile(): Int = prefs.getInt(KEY_FILTER_PROFILE, 0)

    fun setFilterProfile(profile: Int) {
        prefs.edit().putInt(KEY_FILTER_PROFILE, profile).apply()
    }

    fun getLoggerFlag(): Int = prefs.getInt(KEY_LOGGER_FLAG, 0)

    fun setLoggerFlag(flag: Int) {
        prefs.edit().putInt(KEY_LOGGER_FLAG, flag).apply()
    }

    fun getSensorName(address: String, fallback: String): String {
        return prefs.getString("sensor.$address.name", fallback) ?: fallback
    }

    fun setSensorName(address: String, name: String) {
        prefs.edit().putString("sensor.$address.name", name).apply()
    }

    fun getSensorSide(address: String): String? {
        return prefs.getString("sensor.$address.side", null)
    }

    fun setSensorSide(address: String, side: String?) {
        prefs.edit().apply {
            if (side.isNullOrBlank()) {
                remove("sensor.$address.side")
            } else {
                putString("sensor.$address.side", side)
            }
        }.apply()
    }

    fun getSensorBodyPart(address: String): String? {
        return prefs.getString("sensor.$address.bodyPart", null)
    }

    fun setSensorBodyPart(address: String, bodyPart: String?) {
        prefs.edit().apply {
            if (bodyPart.isNullOrBlank()) {
                remove("sensor.$address.bodyPart")
            } else {
                putString("sensor.$address.bodyPart", bodyPart)
            }
        }.apply()
    }
}
