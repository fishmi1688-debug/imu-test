package com.imu.motionrecorder.util

import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 时间格式化工具
 */
object DateUtil {
    // 格式化时间戳为：yyyy-MM-dd HH:mm:ss
    fun formatTimestamp(timestamp: Long): String {
        val sdf = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault())
        return sdf.format(Date(timestamp))
    }

    // 格式化时长（ms）为：mm:ss
    fun formatDuration(durationMs: Long): String {
        val seconds = (durationMs / 1000) % 60
        val minutes = (durationMs / (1000 * 60)) % 60
        return String.format(Locale.getDefault(), "%02d:%02d", minutes, seconds)
    }
}