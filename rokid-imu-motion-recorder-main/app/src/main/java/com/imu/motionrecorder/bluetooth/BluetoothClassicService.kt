package com.imu.motionrecorder.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothServerSocket
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import android.util.Base64
import androidx.core.app.ActivityCompat
import java.io.IOException
import java.util.UUID
import java.util.concurrent.atomic.AtomicBoolean

/**
 * 经典蓝牙服务 (RFCOMM)，用于提供稳定的连接和固定的MAC地址
 */
class BluetoothClassicService(private val context: Context) {

    companion object {
        private const val TAG = "BluetoothClassicService"
        private const val NAME = "IMU_Motion_Recorder"
        // 使用自定义 UUID 避免与系统 SPP 服务冲突
        val SPP_UUID: UUID = UUID.fromString("96f66030-50c9-11ee-be56-0242ac120002")
        private const val IMU_PREFIX = "IMU,"
        private const val AUDIO_PREFIX = "AUD,"
        private const val AUDIO_CONFIG_PREFIX = "AUDCFG,"
        private const val IMAGE_PREFIX = "IMG,"
    }

    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager.adapter

    private var acceptThread: AcceptThread? = null
    private var connectedThread: ConnectedThread? = null
    
    private val isRunning = AtomicBoolean(false)
    private val isTransmitting = AtomicBoolean(false)
    private val isConnected = AtomicBoolean(false)
    
    var onConnectionStateChanged: ((Boolean) -> Unit)? = null
    var onMessageReceived: ((String) -> Unit)? = null

    fun isBluetoothAvailable(): Boolean = bluetoothAdapter != null

    fun isBluetoothEnabled(): Boolean = bluetoothAdapter?.isEnabled == true
    
    fun enableBluetooth(): Boolean {
        if (bluetoothAdapter?.isEnabled == true) return true
        if (!hasPermission()) {
            Log.e(TAG, "Missing Bluetooth permissions")
            return false
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            // Android 13+ 禁止静默开启蓝牙，只能提示用户手动开启
            Log.w(TAG, "Cannot enable Bluetooth silently on Android 13+, please enable manually")
            return false
        }
        @Suppress("MissingPermission", "Deprecation")
        return bluetoothAdapter?.enable() == true
    }

    fun startServer() {
        if (isRunning.get()) return
        
        if (!hasPermission()) {
            Log.e(TAG, "Missing Bluetooth permissions")
            return
        }
        
        if (!isBluetoothEnabled()) {
            Log.e(TAG, "Bluetooth is disabled, cannot start server")
            return
        }

        // 停止之前的线程，但不触发断开回调，避免 UI 误以为异常断开
        stopServer(silent = true)
        
        isRunning.set(true)
        acceptThread = AcceptThread()
        acceptThread?.start()
        Log.d(TAG, "Classic Bluetooth server started")
    }

    fun stopServer(silent: Boolean = false) {
        isRunning.set(false)
        isTransmitting.set(false)
        
        try {
            acceptThread?.cancel()
            acceptThread = null
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping accept thread", e)
        }
        
        try {
            connectedThread?.cancel(silent = silent)
            connectedThread = null
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping connected thread", e)
        }
        
        if (!silent && isConnected.getAndSet(false)) {
            onConnectionStateChanged?.invoke(false)
        }
        Log.d(TAG, "Classic Bluetooth server stopped")
    }

    fun startTransmission() {
        isTransmitting.set(true)
    }

    fun stopTransmission() {
        isTransmitting.set(false)
    }

    fun sendIMUData(
        accelX: Float, accelY: Float, accelZ: Float,
        gyroX: Float, gyroY: Float, gyroZ: Float
    ) {
        if (!isTransmitting.get()) return

        val data = String.format(
            "$IMU_PREFIX%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            accelX, accelY, accelZ,
            gyroX, gyroY, gyroZ
        )
        sendData(data)
    }

    fun sendAudioConfig(sampleRate: Int, channels: Int, sampleWidthBytes: Int) {
        if (!isTransmitting.get()) return
        val data = "$AUDIO_CONFIG_PREFIX$sampleRate,$channels,$sampleWidthBytes\n"
        sendData(data)
    }

    fun sendAudioData(pcmData: ByteArray, length: Int) {
        if (!isTransmitting.get() || length <= 0) return
        val base64 = Base64.encodeToString(pcmData, 0, length, Base64.NO_WRAP)
        val data = "$AUDIO_PREFIX$base64\n"
        sendData(data)
    }

    fun sendImageData(jpegData: ByteArray) {
        if (!isTransmitting.get() || jpegData.isEmpty()) return
        val base64 = Base64.encodeToString(jpegData, Base64.NO_WRAP)
        val data = "$IMAGE_PREFIX$base64\n"
        sendData(data)
    }

    fun sendData(data: String) {
        sendData(data.toByteArray(Charsets.UTF_8))
    }

    fun sendData(data: ByteArray) {
        connectedThread?.write(data)
    }

    private fun hasPermission(): Boolean {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED
        }
        return true
    }

    private inner class AcceptThread : Thread() {
        private var serverSocket: BluetoothServerSocket? = null

        override fun run() {
            while (isRunning.get()) {
                try {
                    if (!hasPermission()) {
                        Log.e(TAG, "No BLUETOOTH_CONNECT permission, stop listening")
                        break
                    }
                    serverSocket = openServerSocket()
                    if (serverSocket == null) {
                        Log.e(TAG, "Failed to open RFCOMM server socket")
                        break
                    }
                    Log.d(TAG, "RFCOMM server listening with UUID: $SPP_UUID")

                    // 持续阻塞等待 PC 连接，断开后继续 accept，直到 stopServer 被调用
                    while (isRunning.get()) {
                        val socket: BluetoothSocket? = try {
                            serverSocket?.accept()
                        } catch (e: IOException) {
                            if (isRunning.get()) {
                                Log.e(TAG, "accept() failed, retrying...", e)
                            }
                            null
                        }
                        if (socket != null) {
                            manageMyConnectedSocket(socket)
                        } else {
                            if (!isRunning.get()) break
                            // 短暂休眠后重新创建 serverSocket
                            Thread.sleep(300)
                            break
                        }
                    }
                } finally {
                    try {
                        serverSocket?.close()
                    } catch (e: IOException) {
                        Log.e(TAG, "Could not close server socket", e)
                    }
                    serverSocket = null
                }
            }
            // 如果 accept 循环退出（异常或关闭），标记为未运行，方便外部重新启动
            isRunning.set(false)
        }

        fun cancel() {
            try {
                serverSocket?.close()
            } catch (e: IOException) {
                Log.e(TAG, "Could not close the connect socket", e)
            }
        }

        private fun openServerSocket(): BluetoothServerSocket? {
            return try {
                // 优先使用不安全（无需配对）的服务记录方式，系统分配端口，PC 端按 UUID 查找
                bluetoothAdapter?.listenUsingInsecureRfcommWithServiceRecord(NAME, SPP_UUID)
            } catch (e: Exception) {
                Log.w(TAG, "Insecure listen failed, fallback to secure", e)
                try {
                    bluetoothAdapter?.listenUsingRfcommWithServiceRecord(NAME, SPP_UUID)
                } catch (fallback: IOException) {
                    Log.e(TAG, "Secure listen also failed", fallback)
                    null
                }
            }
        }
    }

    private fun manageMyConnectedSocket(socket: BluetoothSocket) {
        Log.d(TAG, "Device connected: ${socket.remoteDevice.name}")
        
        // 如果已有连接，先断开
        try {
            connectedThread?.cancel(silent = true)
        } catch (e: Exception) {
            Log.e(TAG, "Error closing existing connection", e)
        }

        connectedThread = ConnectedThread(socket)
        connectedThread?.start()
        isConnected.set(true)
        onConnectionStateChanged?.invoke(true)
    }

    private inner class ConnectedThread(private val mmSocket: BluetoothSocket) : Thread() {
        private val mmInStream = mmSocket.inputStream
        private val mmOutStream = mmSocket.outputStream

        override fun run() {
            val buffer = ByteArray(1024)
            val textBuffer = StringBuilder()
            // 保持连接活跃
            while (isRunning.get()) {
                try {
                    // 读取输入流。这是一个阻塞调用。
                    // 如果连接断开，read() 会抛出 IOException 或返回 -1
                    val bytes = mmInStream.read(buffer)
                    if (bytes == -1) {
                        Log.e(TAG, "Input stream closed (EOF)")
                        throw IOException("Input stream closed")
                    }
                    if (bytes > 0) {
                        val incoming = String(buffer, 0, bytes, Charsets.UTF_8)
                        textBuffer.append(incoming)
                        var newlineIndex = textBuffer.indexOf("\n")
                        while (newlineIndex >= 0) {
                            val line = textBuffer.substring(0, newlineIndex).trim()
                            textBuffer.delete(0, newlineIndex + 1)
                            if (line.isNotEmpty()) {
                                onMessageReceived?.invoke(line)
                            }
                            newlineIndex = textBuffer.indexOf("\n")
                        }
                    }
                } catch (e: IOException) {
                    Log.e(TAG, "Connection lost", e)
                    cancel()
                    break
                }
            }
        }

        fun write(bytes: ByteArray) {
            synchronized(this) {
                try {
                    mmOutStream.write(bytes)
                } catch (e: IOException) {
                    Log.e(TAG, "Error occurred when sending data", e)
                    cancel()
                }
            }
        }

        fun cancel(silent: Boolean = false) {
            try {
                mmSocket.close()
            } catch (e: IOException) {
                Log.e(TAG, "Could not close the connect socket", e)
            }
            val wasConnected = isConnected.getAndSet(false)
            if (!silent && wasConnected) {
                onConnectionStateChanged?.invoke(false)
            }
        }
    }
}
