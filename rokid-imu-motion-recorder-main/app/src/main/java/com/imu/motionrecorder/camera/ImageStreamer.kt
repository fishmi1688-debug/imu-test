package com.imu.motionrecorder.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import androidx.core.content.ContextCompat
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executor
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.abs
import kotlin.math.max

class ImageStreamer(
    private val context: Context,
    private val targetWidth: Int = 640,
    private val targetHeight: Int = 480,
    private val fps: Int = 1
) {
    companion object {
        private const val TAG = "ImageStreamer"
        private const val JPEG_QUALITY = 70
    }

    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private val isRunning = AtomicBoolean(false)

    private var handlerThread: HandlerThread? = null
    private var handler: Handler? = null
    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null
    private var captureRequest: CaptureRequest? = null
    private var actualSize: Size? = null
    private var onFrame: ((ByteArray) -> Unit)? = null
    private var captureRunnable: Runnable? = null

    @Synchronized
    fun start(onFrame: (ByteArray) -> Unit): Boolean {
        if (isRunning.get()) {
            return true
        }
        if (!hasCameraPermission()) {
            Log.e(TAG, "Missing CAMERA permission")
            return false
        }
        val cameraId = chooseCameraId() ?: run {
            Log.e(TAG, "No camera available")
            return false
        }

        handlerThread = HandlerThread("ImageStreamer").apply { start() }
        handler = Handler(handlerThread!!.looper)
        isRunning.set(true)
        this.onFrame = onFrame

        try {
            cameraManager.openCamera(cameraId, cameraStateCallback, handler)
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Open camera failed", e)
            stop()
            return false
        } catch (e: SecurityException) {
            Log.e(TAG, "Open camera denied", e)
            stop()
            return false
        }
        return true
    }

    @Synchronized
    fun stop() {
        if (!isRunning.getAndSet(false)) {
            return
        }
        captureRunnable?.let { handler?.removeCallbacks(it) }
        captureRunnable = null

        try {
            captureSession?.close()
        } catch (e: Exception) {
            Log.w(TAG, "Close capture session failed", e)
        }
        captureSession = null

        try {
            cameraDevice?.close()
        } catch (e: Exception) {
            Log.w(TAG, "Close camera failed", e)
        }
        cameraDevice = null

        try {
            imageReader?.close()
        } catch (e: Exception) {
            Log.w(TAG, "Close image reader failed", e)
        }
        imageReader = null

        handlerThread?.quitSafely()
        handlerThread = null
        handler = null
        captureRequest = null
        onFrame = null
        actualSize = null
    }

    private fun hasCameraPermission(): Boolean {
        return ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
    }

    private fun chooseCameraId(): String? {
        val cameraIds = try {
            cameraManager.cameraIdList
        } catch (e: CameraAccessException) {
            Log.e(TAG, "Read camera list failed", e)
            return null
        }
        if (cameraIds.isEmpty()) {
            return null
        }
        val preferred = cameraIds.firstOrNull { id ->
            try {
                val characteristics = cameraManager.getCameraCharacteristics(id)
                val facing = characteristics.get(CameraCharacteristics.LENS_FACING)
                facing == CameraCharacteristics.LENS_FACING_BACK
            } catch (e: CameraAccessException) {
                false
            }
        }
        val candidates = listOfNotNull(preferred) + cameraIds.filter { it != preferred }
        for (cameraId in candidates) {
            val characteristics = try {
                cameraManager.getCameraCharacteristics(cameraId)
            } catch (e: CameraAccessException) {
                Log.w(TAG, "Read camera characteristics failed for $cameraId", e)
                continue
            }
            val size = chooseOutputSize(characteristics) ?: continue
            actualSize = size
            return cameraId
        }
        return null
    }

    private fun chooseOutputSize(characteristics: CameraCharacteristics): Size? {
        val map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return null
        val sizes = map.getOutputSizes(ImageFormat.JPEG) ?: return null
        if (sizes.isEmpty()) {
            return null
        }
        val exact = sizes.firstOrNull { it.width == targetWidth && it.height == targetHeight }
        if (exact != null) {
            return exact
        }
        val targetRatio = targetWidth.toDouble() / targetHeight.toDouble()
        return sizes.minWithOrNull(
            compareBy<Size> { abs(it.width.toDouble() / it.height.toDouble() - targetRatio) }
                .thenBy { abs(it.width * it.height - targetWidth * targetHeight) }
        )
    }

    private val cameraStateCallback = object : CameraDevice.StateCallback() {
        override fun onOpened(camera: CameraDevice) {
            cameraDevice = camera
            createCaptureSession()
        }

        override fun onDisconnected(camera: CameraDevice) {
            camera.close()
            stop()
        }

        override fun onError(camera: CameraDevice, error: Int) {
            Log.e(TAG, "Camera error: $error")
            camera.close()
            stop()
        }
    }

    private fun createCaptureSession() {
        val size = actualSize ?: Size(targetWidth, targetHeight)
        val reader = ImageReader.newInstance(size.width, size.height, ImageFormat.JPEG, 2)
        imageReader = reader
        reader.setOnImageAvailableListener({ r ->
            handleImage(r)
        }, handler)
        val device = cameraDevice ?: return
        val sessionHandler = requireNotNull(handler) { "Handler not ready" }
        val executor = Executor { command -> sessionHandler.post(command) }
        val outputConfigs = listOf(OutputConfiguration(reader.surface))
        val stateCallback = object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(session: CameraCaptureSession) {
                captureSession = session
                val requestBuilder = device.createCaptureRequest(CameraDevice.TEMPLATE_STILL_CAPTURE)
                requestBuilder.addTarget(reader.surface)
                requestBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                requestBuilder.set(CaptureRequest.JPEG_QUALITY, JPEG_QUALITY.toByte())
                captureRequest = requestBuilder.build()
                startCaptureLoop()
            }

            override fun onConfigureFailed(session: CameraCaptureSession) {
                Log.e(TAG, "Capture session config failed")
                stop()
            }
        }
        val sessionConfig = SessionConfiguration(
            SessionConfiguration.SESSION_REGULAR,
            outputConfigs,
            executor,
            stateCallback
        )
        device.createCaptureSession(sessionConfig)
    }

    private fun startCaptureLoop() {
        val intervalMs = max(1L, 1000L / max(1, fps))
        val runnable = object : Runnable {
            override fun run() {
                if (!isRunning.get()) {
                    return
                }
                val session = captureSession
                val request = captureRequest
                if (session != null && request != null) {
                    try {
                        session.capture(request, null, handler)
                    } catch (e: CameraAccessException) {
                        Log.e(TAG, "Capture failed", e)
                    } catch (e: IllegalStateException) {
                        Log.e(TAG, "Capture session invalid", e)
                    }
                }
                handler?.postDelayed(this, intervalMs)
            }
        }
        captureRunnable = runnable
        handler?.post(runnable)
    }

    private fun handleImage(reader: ImageReader) {
        val image = try {
            reader.acquireLatestImage()
        } catch (e: Exception) {
            null
        } ?: return
        try {
            val buffer = image.planes[0].buffer
            val data = ByteArray(buffer.remaining())
            buffer.get(data)
            val scaled = scaleIfNeeded(data)
            onFrame?.invoke(scaled)
        } catch (e: Exception) {
            Log.e(TAG, "Handle image failed", e)
        } finally {
            image.close()
        }
    }

    private fun scaleIfNeeded(data: ByteArray): ByteArray {
        val size = actualSize
        if (size != null && size.width == targetWidth && size.height == targetHeight) {
            return data
        }
        val bitmap = BitmapFactory.decodeByteArray(data, 0, data.size) ?: return data
        val scaled = Bitmap.createScaledBitmap(bitmap, targetWidth, targetHeight, true)
        if (scaled !== bitmap) {
            bitmap.recycle()
        }
        val output = ByteArrayOutputStream()
        scaled.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, output)
        scaled.recycle()
        return output.toByteArray()
    }
}
