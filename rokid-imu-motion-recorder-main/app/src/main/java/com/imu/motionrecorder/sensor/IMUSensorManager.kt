package com.imu.motionrecorder.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread

/**
 * 传感器管理：统一获取加速度计、陀螺仪、气压计数据
 */
class IMUSensorManager(context: Context) {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    // 目标传感器（API32+兼容）
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val barometer = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE)

    // 传感器可用性检查
    val isAccelerometerAvailable: Boolean = accelerometer != null
    val isGyroscopeAvailable: Boolean = gyroscope != null
    val isBarometerAvailable: Boolean = barometer != null

    // 数据回调
    private var dataCallback: ((IMUData) -> Unit)? = null

    // 采样控制（100Hz，10ms间隔）
    private val samplingInterval = 10L
    private var lastTimestamp = 0L

    // 专用后台线程处理传感器数据，避免主线程卡顿和协程频繁创建导致的崩溃
    private val sensorThread = HandlerThread("SensorThread")
    private val sensorHandler: Handler

    init {
        sensorThread.start()
        sensorHandler = Handler(sensorThread.looper)
    }

    private var currentIMUData = IMUData(0f, 0f, 0f, 0f, 0f, 0f, null, 0f)

    private var calibrationListener: ((IMUData) -> Unit)? = null
    private var calibrationSensorListener: SensorEventListener? = null

    // IMU数据模型
    data class IMUData(
        val accelX: Float, // 加速度X（m/s²，含重力）
        val accelY: Float, // 加速度Y
        val accelZ: Float, // 加速度Z
        val gyroX: Float,  // 角速度X（rad/s）
        val gyroY: Float,  // 角速度Y
        val gyroZ: Float,  // 角速度Z
        val pressure: Float?, // 气压（hPa，可选）
//        val timestamp: Long // 时间戳（ms）
        val deltaTime: Float // 使用Float类型的秒作为单位
    )

    // 传感器监听器
    private val sensorListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            if (lastTimestamp == 0L) {
                lastTimestamp = event.timestamp
                return
            }
            val dt = (event.timestamp - lastTimestamp) * 1.0e-9f // 纳秒转秒 (Float)
            lastTimestamp = event.timestamp
            if (dt <= 0) return

            // 直接在 HandlerThread 中处理，无需启动协程
            val newData = when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> currentIMUData.copy(
                    accelX = event.values[0],
                    accelY = event.values[1],
                    accelZ = event.values[2],
                    deltaTime = dt
                )
                Sensor.TYPE_GYROSCOPE -> currentIMUData.copy(
                    gyroX = event.values[0],
                    gyroY = event.values[1],
                    gyroZ = event.values[2],
                    deltaTime = dt
                )
                Sensor.TYPE_PRESSURE -> currentIMUData.copy(
                    pressure = event.values[0],
                    deltaTime = dt
                )
                else -> return
            }
            currentIMUData = newData
            dataCallback?.invoke(newData)
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
    }

    // 注册传感器监听（API32+采样率配置）
    fun startListening(callback: (IMUData) -> Unit) {
        dataCallback = callback
        accelerometer?.let {
            sensorManager.registerListener(
                sensorListener,
                it,
                SensorManager.SENSOR_DELAY_GAME, // 20ms延迟
                sensorHandler // 指定在后台线程运行
            )
        }
        gyroscope?.let {
            sensorManager.registerListener(
                sensorListener,
                it,
                SensorManager.SENSOR_DELAY_GAME,
                sensorHandler
            )
        }
        barometer?.let {
            sensorManager.registerListener(
                sensorListener,
                it,
                SensorManager.SENSOR_DELAY_NORMAL,
                sensorHandler
            )
        }
    }

    // 取消监听，释放资源
    fun stopListening() {
        sensorManager.unregisterListener(sensorListener)
        dataCallback = null
        lastTimestamp = 0
    }

    // --- 新增：用于校准的临时监听 ---
    fun startCalibrationListening(listener: (IMUData) -> Unit) {
        if (!isGyroscopeAvailable) return
        this.calibrationListener = listener

        calibrationSensorListener = object : SensorEventListener {
            private var lastTimestamp = 0L

            override fun onSensorChanged(event: SensorEvent) {
                if (event.sensor.type != Sensor.TYPE_GYROSCOPE) return

                if (lastTimestamp == 0L) {
                    lastTimestamp = event.timestamp
                    return
                }

                val dt = (event.timestamp - lastTimestamp) * 1.0e-9f // 纳秒转秒
                lastTimestamp = event.timestamp
                if (dt <= 0) return

                // 校准只需要陀螺仪数据
                val imuData = IMUData(
                    gyroX = event.values[0],
                    gyroY = event.values[1],
                    gyroZ = event.values[2],
                    // 加速度计数据在校准期间可以为0
                    accelX = 0f, accelY = 0f, accelZ = 0f,
                    pressure = null,
//                    timestamp = currentTime
                    deltaTime = dt
                )
                calibrationListener?.invoke(imuData)
            }

            override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
        }
        // 使用尽可能高的频率进行校准，以快速收集样本
        sensorManager.registerListener(calibrationSensorListener, gyroscope, SensorManager.SENSOR_DELAY_GAME)
    }

    // --- 新增：停止校准监听 ---
    fun stopCalibrationListening() {
        calibrationSensorListener?.let {
            sensorManager.unregisterListener(it)
        }
        calibrationListener = null
    }
}