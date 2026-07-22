package com.imu.motionrecorder.database

import androidx.room.Entity
import androidx.room.PrimaryKey
import java.util.Date

/**
 * 运动记录实体类（存储统计数据）
 */
@Entity(tableName = "motion_records")
data class MotionRecord(
    @PrimaryKey(autoGenerate = true) val id: Long = 0,
    val startTime: Long, // 开始时间戳（ms）
    val duration: Long, // 运动时长（ms）
    val totalDistance: Float, // 总距离（m）
    val maxSpeed: Float, // 最大速度（m/s）
    val avgSpeed: Float, // 平均速度（m/s）
    val maxAcceleration: Float, // 最大线性加速度（m/s²）
    val maxPitch: Float, // 最大俯仰角（°）
    val maxRoll: Float, // 最大横滚角（°）
    val maxHeight: Float, // 最高高度（m）
    val endTime: Long = Date().time // 结束时间戳（ms）
)