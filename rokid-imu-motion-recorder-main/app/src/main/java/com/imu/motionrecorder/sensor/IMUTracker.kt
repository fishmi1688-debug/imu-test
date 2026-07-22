package com.imu.motionrecorder.sensor

import kotlin.math.*

/**
 * IMU数据融合：计算速度、距离、倾角、高度等
 */
class IMUTracker {
    // 物理常量
    private val GRAVITY = 9.80665f // 重力加速度（m/s²）

    // 姿态角（弧度，内部计算统一使用弧度）
    var pitch = 0f // 俯仰角 (绕X轴)
    var roll = 0f  // 横滚角 (绕Y轴)
    var yaw = 0f   // 偏航角 (绕Z轴)

    // 运动参数（设备坐标系）
    var linearAccelX = 0f
    var linearAccelY = 0f
    var linearAccelZ = 0f
    var velocityX = 0f
    var velocityY = 0f
    var velocityZ = 0f

    // 统计最大值
    var maxSpeed = 0f
    var maxLinearAccel = 0f
    var maxPitch = 0f // 存储弧度
    var maxRoll = 0f  // 存储弧度

    // 算法参数
    private val alpha = 0.98f // 互补滤波：陀螺仪权重
    private val beta = 1 - alpha  // 互补滤波：加速度计权重

    // 为高通滤波器添加状态变量，用于追踪并消除加速度的漂移
    private var accelBiasX = 0f
    private var accelBiasY = 0f
    private var accelBiasZ = 0f

    // 校准相关属性
    private var gyroBiasX = 0f
    private var gyroBiasY = 0f
    private var gyroBiasZ = 0f
    var isCalibrated = false
        private set

    private val calibrationSamples = mutableListOf<FloatArray>()
    private val CALIBRATION_SAMPLE_COUNT = 200

    // ZUPT (零速更新) 相关属性
    private var stationaryCounter = 0
    private val STATIONARY_FRAMES_THRESHOLD = 25

    fun getCalibrationProgress(): Int = calibrationSamples.size

    fun startCalibration() {
        reset()
        isCalibrated = false
        calibrationSamples.clear()
    }

    fun processCalibrationData(imuData: IMUSensorManager.IMUData): Boolean {
        if (isCalibrated) return true
        calibrationSamples.add(floatArrayOf(imuData.gyroX, imuData.gyroY, imuData.gyroZ))
        if (calibrationSamples.size >= CALIBRATION_SAMPLE_COUNT) {
            finishCalibration()
            return true
        }
        return false
    }

    private fun finishCalibration() {
        if (calibrationSamples.isEmpty()) return
        val sum = floatArrayOf(0f, 0f, 0f)
        calibrationSamples.forEach {
            sum[0] += it[0]
            sum[1] += it[1]
            sum[2] += it[2]
        }
        gyroBiasX = sum[0] / calibrationSamples.size
        gyroBiasY = sum[1] / calibrationSamples.size
        gyroBiasZ = sum[2] / calibrationSamples.size
        isCalibrated = true
        calibrationSamples.clear()
    }

    fun update(imuData: IMUSensorManager.IMUData) {
        if (!isCalibrated) return
        val dt = imuData.deltaTime
        if (dt <= 0 || dt > 0.1f) return

        // 1. 校准陀螺仪数据
        val calibratedGyroX = imuData.gyroX - gyroBiasX
        val calibratedGyroY = imuData.gyroY - gyroBiasY
        val calibratedGyroZ = imuData.gyroZ - gyroBiasZ

        // 2. 提前进行零速更新(ZUPT)判断
        val accelMagnitude = sqrt(imuData.accelX.pow(2) + imuData.accelY.pow(2) + imuData.accelZ.pow(2))
        val gyroMagnitude = sqrt(calibratedGyroX.pow(2) + calibratedGyroY.pow(2) + calibratedGyroZ.pow(2))
        val ZUPT_ACCEL_STRICT_THRESHOLD = 0.25f
        val ZUPT_GYRO_STRICT_THRESHOLD = 0.06f

        if (abs(accelMagnitude - GRAVITY) < ZUPT_ACCEL_STRICT_THRESHOLD && gyroMagnitude < ZUPT_GYRO_STRICT_THRESHOLD) {
            stationaryCounter++
        } else {
            stationaryCounter = 0
        }

        if (stationaryCounter > STATIONARY_FRAMES_THRESHOLD) {
            linearAccelX = 0f; linearAccelY = 0f; linearAccelZ = 0f
            velocityX = 0f; velocityY = 0f; velocityZ = 0f
            // yaw = 0f // 在这个简化模型中，yaw的重置意义不大，可以保留或去掉
            return
        }

        // --- 只有在设备运动时，才执行以下计算 ---

        // 3. 姿态计算 (互补滤波器)
        // 回归到最标准、最直观的姿态角定义
        // Pitch (绕X轴): 手机上下点头
        val accelPitch = atan2(imuData.accelY, imuData.accelZ)
        // Roll (绕Y轴): 手机左右翻滚
        val accelRoll = atan2(-imuData.accelX, sqrt(imuData.accelY.pow(2) + imuData.accelZ.pow(2)))

        val ATTITUDE_CORRECTION_ACCEL_THRESHOLD = 0.3f
        if (abs(accelMagnitude - GRAVITY) < ATTITUDE_CORRECTION_ACCEL_THRESHOLD) {
            pitch = alpha * (pitch + calibratedGyroX * dt) + beta * accelPitch // pitch 绕X轴，由 gyroX 驱动
            roll = alpha * (roll + calibratedGyroY * dt) + beta * accelRoll   // roll 绕Y轴，由 gyroY 驱动
        } else {
            pitch += calibratedGyroX * dt
            roll += calibratedGyroY * dt
        }
        // Yaw (绕Z轴)，由 gyroZ 驱动
        yaw -= calibratedGyroZ * dt // 修正方向

        // 【【【核心简化】】】
        // 4. 在设备坐标系下移除重力，得到设备坐标系下的线性加速度
        // 这是一个近似计算，但在旋转不剧烈时效果尚可
        val cp = cos(pitch)
        val sp = sin(pitch)
        val cr = cos(roll)
        val sr = sin(roll)

        // 计算重力在当前姿态下，在设备各轴上的投影分量
        val gravityX_sensor = -GRAVITY * sr
        val gravityY_sensor = GRAVITY * sp * cr
        val gravityZ_sensor = GRAVITY * cp * cr

        // 从加速度计读数中减去重力分量
        val rawLinearAccelX = imuData.accelX - gravityX_sensor
        val rawLinearAccelY = imuData.accelY - gravityY_sensor
        val rawLinearAccelZ = imuData.accelZ - gravityZ_sensor

        // --- 第1道防线：高通滤波器 (HPF) ---
        // 职责：滤除直流偏置（漂移），让加速度的基线回归到0。
        val highPassFilterFactor = 0.86f // 可调参数：建议值在 0.8 到 0.95 之间

        // 1. 更新漂移估算值 (这是一个低通滤波器，用于追踪漂移)
        accelBiasX = accelBiasX * highPassFilterFactor + rawLinearAccelX * (1 - highPassFilterFactor)
        accelBiasY = accelBiasY * highPassFilterFactor + rawLinearAccelY * (1 - highPassFilterFactor)
        accelBiasZ = accelBiasZ * highPassFilterFactor + rawLinearAccelZ * (1 - highPassFilterFactor)

        // 2. 从原始线性加速度中减去漂移估算值，得到去除了漂移的信号
        val highPassedAccelX = rawLinearAccelX - accelBiasX
        val highPassedAccelY = rawLinearAccelY - accelBiasY
        val highPassedAccelZ = rawLinearAccelZ - accelBiasZ

        // --- 第2道防线：低通滤波器 (LPF) ---
        // 职责：对已经去除了漂移的信号进行平滑处理，消除高频噪声和抖动。
        val lowPassFilterFactor = 1f // 可调参数：值越小，曲线越平滑，但响应越慢。建议值在 0.1 到 0.25 之间。

        // 3. 将经过高通滤波的信号再进行低通滤波，得到最终的平滑加速度
        linearAccelX = linearAccelX * (1 - lowPassFilterFactor) + highPassedAccelX * lowPassFilterFactor
        linearAccelY = linearAccelY * (1 - lowPassFilterFactor) + highPassedAccelY * lowPassFilterFactor
        linearAccelZ = linearAccelZ * (1 - lowPassFilterFactor) + highPassedAccelZ * lowPassFilterFactor

        linearAccelX = -linearAccelX
        linearAccelY = -linearAccelY

        // 5. 【【【核心简化】】】直接对设备坐标系下的线性加速度进行积分
        velocityX += linearAccelX * dt
        velocityY += linearAccelY * dt
        velocityZ += linearAccelZ * dt

        // 6. 更新最大值
        updateMaxValues()
    }

    private fun updateMaxValues() {
        val currentSpeed = getCurrentSpeed()
        val currentLinearAccel = sqrt(linearAccelX.pow(2) + linearAccelY.pow(2) + linearAccelZ.pow(2))

        if (currentSpeed > maxSpeed) maxSpeed = currentSpeed
        if (currentLinearAccel > maxLinearAccel) maxLinearAccel = currentLinearAccel
        if (abs(pitch) > maxPitch) maxPitch = abs(pitch)
        if (abs(roll) > maxRoll) maxRoll = abs(roll)
    }

    fun getCurrentSpeed(): Float = sqrt(velocityX.pow(2) + velocityY.pow(2) + velocityZ.pow(2))

    fun reset() {
        pitch = 0f; roll = 0f; yaw = 0f
        linearAccelX = 0f; linearAccelY = 0f; linearAccelZ = 0f
        velocityX = 0f; velocityY = 0f; velocityZ = 0f
        maxSpeed = 0f; maxLinearAccel = 0f
        maxPitch = 0f; maxRoll = 0f
        stationaryCounter = 0

        // 重置高通滤波器的状态
        accelBiasX = 0f
        accelBiasY = 0f
        accelBiasZ = 0f
    }
}
