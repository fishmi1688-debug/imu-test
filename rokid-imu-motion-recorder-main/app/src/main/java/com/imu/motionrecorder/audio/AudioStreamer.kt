package com.imu.motionrecorder.audio

import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import android.util.Log
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max

class AudioStreamer(
    val sampleRate: Int = 8000,
    private val channelConfig: Int = AudioFormat.CHANNEL_IN_MONO,
    private val audioEncoding: Int = AudioFormat.ENCODING_PCM_16BIT
) {
    companion object {
        private const val TAG = "AudioStreamer"
        private const val CHUNK_MS = 20
    }

    val channelCount: Int
        get() = if (channelConfig == AudioFormat.CHANNEL_IN_MONO) 1 else 2

    val sampleWidthBytes: Int
        get() = when (audioEncoding) {
            AudioFormat.ENCODING_PCM_16BIT -> 2
            AudioFormat.ENCODING_PCM_8BIT -> 1
            else -> 2
        }

    private val isRunning = AtomicBoolean(false)
    private var audioRecord: AudioRecord? = null
    private var workerThread: Thread? = null

    @Synchronized
    fun start(onAudioData: (ByteArray, Int) -> Unit): Boolean {
        if (isRunning.get()) {
            return true
        }

        val minBuffer = AudioRecord.getMinBufferSize(sampleRate, channelConfig, audioEncoding)
        if (minBuffer <= 0) {
            Log.e(TAG, "Invalid min buffer size: $minBuffer")
            return false
        }

        val frameSize = channelCount * sampleWidthBytes
        val chunkSize = max(frameSize, (sampleRate * CHUNK_MS / 1000) * frameSize)
        val bufferSize = max(minBuffer, chunkSize * 2)

        val record = AudioRecord.Builder()
            .setAudioSource(MediaRecorder.AudioSource.MIC)
            .setAudioFormat(
                AudioFormat.Builder()
                    .setEncoding(audioEncoding)
                    .setSampleRate(sampleRate)
                    .setChannelMask(channelConfig)
                    .build()
            )
            .setBufferSizeInBytes(bufferSize)
            .build()

        if (record.state != AudioRecord.STATE_INITIALIZED) {
            Log.e(TAG, "AudioRecord init failed")
            record.release()
            return false
        }

        audioRecord = record
        record.startRecording()
        isRunning.set(true)

        workerThread = Thread {
            val buffer = ByteArray(chunkSize)
            try {
                while (isRunning.get()) {
                    val read = record.read(buffer, 0, buffer.size, AudioRecord.READ_BLOCKING)
                    if (read > 0) {
                        onAudioData(buffer, read)
                    } else if (read < 0) {
                        Log.w(TAG, "Audio read error: $read")
                        break
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Audio capture failed", e)
            } finally {
                stop()
            }
        }.apply {
            name = "AudioStreamer"
            start()
        }

        return true
    }

    @Synchronized
    fun stop() {
        if (!isRunning.getAndSet(false)) {
            return
        }
        val record = audioRecord
        audioRecord = null
        workerThread = null
        if (record != null) {
            try {
                record.stop()
            } catch (e: Exception) {
                Log.w(TAG, "AudioRecord stop failed", e)
            }
            try {
                record.release()
            } catch (e: Exception) {
                Log.w(TAG, "AudioRecord release failed", e)
            }
        }
    }
}
