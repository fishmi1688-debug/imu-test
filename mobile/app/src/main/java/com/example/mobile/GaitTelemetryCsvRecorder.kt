package com.example.mobile

import android.content.ContentValues
import android.content.Context
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.provider.MediaStore
import java.io.File
import java.io.FileOutputStream
import java.text.SimpleDateFormat
import java.util.Date
import java.util.LinkedHashMap
import java.util.Locale
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.math.roundToLong

/**
 * Save gait plot telemetry into Download/gait_data as soon as gait frames arrive.
 * Disk writes are buffered and flushed in batches to avoid blocking the UI/BT pipeline.
 */
class GaitTelemetryCsvRecorder(
    context: Context,
    private val statusCallback: (String) -> Unit
) {
    companion object {
        private const val ROOT_DIR_NAME = "gait_data"
        private const val CSV_FILE_NAME = "gait_telemetry.csv"
        private const val DOWNLOAD_ROOT_DISPLAY_PATH = "Download/$ROOT_DIR_NAME"
        private const val CSV_HEADER =
            "local_time,plot_ts,left_angle,right_angle,angle_diff,phase,assist,assist_enabled\n"
        private const val INSOLE_HEADER_PREFIX = "local_time,insole_ts,foot_id"
        private const val FLUSH_INTERVAL_MS = 750L
        private const val FLUSH_ROW_THRESHOLD = 25
    }

    private val appContext = context.applicationContext
    private val folderTimeFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)
    private val rowTimeFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US)
    private val resolver = appContext.contentResolver
    private val ioExecutor = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "gait-csv-recorder").apply { isDaemon = true }
    }

    private sealed interface CsvTarget {
        val displayPath: String
    }

    private data class FileCsvTarget(
        val file: File,
        override val displayPath: String
    ) : CsvTarget

    private data class MediaStoreCsvTarget(
        val uri: Uri,
        override val displayPath: String
    ) : CsvTarget

    private var currentCsvTarget: CsvTarget? = null
    private var currentSessionDirName: String? = null
    private var currentSessionDisplayPath: String? = null
    private var currentSessionTag: String? = null
    private var latestAssistEnabled = false
    private val insoleCsvMap = LinkedHashMap<String, CsvTarget>()
    private val pendingBuffers = LinkedHashMap<String, StringBuilder>()
    private val pendingTargets = LinkedHashMap<String, CsvTarget>()
    private var pendingRowCount = 0

    init {
        ioExecutor.scheduleWithFixedDelay(
            { flushPendingBuffers() },
            FLUSH_INTERVAL_MS,
            FLUSH_INTERVAL_MS,
            TimeUnit.MILLISECONDS,
        )
    }

    fun onMechanicalZeroAck() {
        ioExecutor.execute {
            flushPendingBuffers()
            currentCsvTarget = null
            currentSessionDirName = null
            currentSessionDisplayPath = null
            currentSessionTag = null
            latestAssistEnabled = false
            insoleCsvMap.clear()
            statusCallback("已重置步态记录会话，等待新的步态数据流")
        }
    }

    fun onStateSnapshot(snapshot: GaitBluetoothBridge.StateSnapshot) {
        ioExecutor.execute {
            latestAssistEnabled = snapshot.assistEnabled
        }
    }

    fun onPlotFrame(frame: GaitBluetoothBridge.PlotFrame) {
        ioExecutor.execute {
            if (currentCsvTarget == null) {
                startNewSession(frame.ts)
            }
            val csvTarget = currentCsvTarget ?: return@execute
            val row = buildString {
                append(rowTimeFormat.format(Date()))
                append(',')
                append(formatDouble(frame.ts))
                append(',')
                append(formatFloat(frame.leftAngle))
                append(',')
                append(formatFloat(frame.rightAngle))
                append(',')
                append(formatFloat(frame.angleDiff))
                append(',')
                append(formatFloat(frame.phase))
                append(',')
                append(formatFloat(frame.assist))
                append(',')
                append(if (latestAssistEnabled) "1" else "0")
                append('\n')
            }
            queueText(csvTarget, row, countAsRow = true)
        }
    }

    fun onInsoleSample(
        address: String,
        deviceName: String,
        insoleTimestamp: Long,
        footId: Int,
        values: List<Int>
    ) {
        if (values.isEmpty()) {
            return
        }
        ioExecutor.execute {
            if (currentCsvTarget == null) {
                return@execute
            }
            val insoleCsvTarget = ensureInsoleCsvFile(address, deviceName, values.size) ?: return@execute
            val row = buildString {
                append(rowTimeFormat.format(Date()))
                append(',')
                append(insoleTimestamp)
                append(',')
                append(footId)
                values.forEach { value ->
                    append(',')
                    append(value)
                }
                append('\n')
            }
            queueText(insoleCsvTarget, row, countAsRow = true)
        }
    }

    private fun startNewSession(plotTs: Double?) {
        val completionMillis = if (plotTs != null) {
            (plotTs * 1000.0).roundToLong()
        } else {
            System.currentTimeMillis()
        }
        val folderBase = folderTimeFormat.format(Date(completionMillis))
        val sessionDirName = buildUniqueSessionDirName(folderBase) ?: return
        val sessionDisplayPath = "$DOWNLOAD_ROOT_DISPLAY_PATH/$sessionDirName"
        val csvTarget = createCsvTarget(sessionDirName, CSV_FILE_NAME, CSV_HEADER) ?: run {
            return
        }

        currentCsvTarget = csvTarget
        currentSessionDirName = sessionDirName
        currentSessionDisplayPath = sessionDisplayPath
        currentSessionTag = folderBase
        insoleCsvMap.clear()
        statusCallback("开始记录步态数据: $sessionDisplayPath")
    }

    private fun ensureInsoleCsvFile(address: String, deviceName: String, valueCount: Int): CsvTarget? {
        val existing = insoleCsvMap[address]
        if (existing != null) {
            return existing
        }
        val sessionDirName = currentSessionDirName ?: return null
        val sessionTag = currentSessionTag ?: folderTimeFormat.format(Date())
        val safeName = sanitizeFilePart(deviceName)
        val fileName = "${safeName}+${sessionTag}.csv"
        val header = buildString {
            append(INSOLE_HEADER_PREFIX)
            for (index in 1..valueCount) {
                append(",p")
                append(index)
            }
            append('\n')
        }
        val target = createCsvTarget(sessionDirName, fileName, header) ?: return null
        insoleCsvMap[address] = target
        return target
    }

    private fun queueText(target: CsvTarget, text: String, countAsRow: Boolean) {
        val key = target.displayPath
        pendingTargets[key] = target
        pendingBuffers.getOrPut(key) { StringBuilder() }.append(text)
        if (countAsRow) {
            pendingRowCount += 1
        }
        if (pendingRowCount >= FLUSH_ROW_THRESHOLD) {
            flushPendingBuffers()
        }
    }

    private fun flushPendingBuffers() {
        if (pendingBuffers.isEmpty()) {
            return
        }
        val iterator = pendingBuffers.entries.iterator()
        while (iterator.hasNext()) {
            val entry = iterator.next()
            val target = pendingTargets[entry.key] ?: continue
            val text = entry.value.toString()
            if (text.isNotEmpty()) {
                writeTextNow(target, text)
            }
            iterator.remove()
        }
        pendingTargets.clear()
        pendingRowCount = 0
    }

    private fun buildUniqueSessionDirName(baseName: String): String? {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            buildUniqueMediaStoreSessionDirName(baseName)
        } else {
            buildUniqueLegacySessionDirName(baseName)
        }
    }

    @Suppress("DEPRECATION")
    private fun buildUniqueLegacySessionDirName(baseName: String): String? {
        val root = File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), ROOT_DIR_NAME)
        if (!root.exists() && !root.mkdirs()) {
            statusCallback("创建步态日志根目录失败: ${root.absolutePath}")
            return null
        }
        var candidate = baseName
        var index = 1
        while (true) {
            if (!File(root, candidate).exists()) {
                return candidate
            }
            candidate = "${baseName}_$index"
            index += 1
        }
    }

    private fun buildUniqueMediaStoreSessionDirName(baseName: String): String {
        var candidate = baseName
        var index = 1
        while (sessionDirExistsInDownloads(candidate)) {
            candidate = "${baseName}_$index"
            index += 1
        }
        return candidate
    }

    private fun sessionDirExistsInDownloads(sessionDirName: String): Boolean {
        val projection = arrayOf(MediaStore.Downloads._ID)
        val selection = "${MediaStore.MediaColumns.RELATIVE_PATH}=?"
        val selectionArgs = arrayOf(mediaStoreRelativePath(sessionDirName, trailingSlash = true))
        return runCatching {
            resolver.query(
                MediaStore.Downloads.EXTERNAL_CONTENT_URI,
                projection,
                selection,
                selectionArgs,
                null
            )?.use { cursor ->
                cursor.moveToFirst()
            } ?: false
        }.getOrDefault(false)
    }

    private fun createCsvTarget(sessionDirName: String, fileName: String, header: String): CsvTarget? {
        val target = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            createMediaStoreCsvTarget(sessionDirName, fileName)
        } else {
            createLegacyCsvTarget(sessionDirName, fileName)
        } ?: return null

        if (!writeTextNow(target, header)) {
            if (target is MediaStoreCsvTarget) {
                resolver.delete(target.uri, null, null)
            }
            return null
        }
        return target
    }

    private fun createMediaStoreCsvTarget(sessionDirName: String, fileName: String): CsvTarget? {
        val relativePath = mediaStoreRelativePath(sessionDirName, trailingSlash = false)
        val displayPath = "$DOWNLOAD_ROOT_DISPLAY_PATH/$sessionDirName/$fileName"
        val values = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, fileName)
            put(MediaStore.MediaColumns.MIME_TYPE, "text/csv")
            put(MediaStore.MediaColumns.RELATIVE_PATH, relativePath)
        }
        val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
        if (uri == null) {
            statusCallback("创建步态CSV失败: $displayPath")
            return null
        }
        return MediaStoreCsvTarget(uri, displayPath)
    }

    @Suppress("DEPRECATION")
    private fun createLegacyCsvTarget(sessionDirName: String, fileName: String): CsvTarget? {
        val downloadsDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS)
        val sessionDir = File(downloadsDir, "$ROOT_DIR_NAME/$sessionDirName")
        if (!sessionDir.exists() && !sessionDir.mkdirs()) {
            statusCallback("创建步态日志目录失败: ${sessionDir.absolutePath}")
            return null
        }
        val file = File(sessionDir, fileName)
        return FileCsvTarget(file, "$DOWNLOAD_ROOT_DISPLAY_PATH/$sessionDirName/$fileName")
    }

    private fun mediaStoreRelativePath(sessionDirName: String, trailingSlash: Boolean): String {
        val base = "${Environment.DIRECTORY_DOWNLOADS}/$ROOT_DIR_NAME/$sessionDirName"
        return if (trailingSlash) "$base/" else base
    }

    private fun writeTextNow(target: CsvTarget, text: String): Boolean {
        return try {
            when (target) {
                is FileCsvTarget -> {
                    FileOutputStream(target.file, true).use { out ->
                        out.write(text.toByteArray(Charsets.UTF_8))
                    }
                }
                is MediaStoreCsvTarget -> {
                    resolver.openOutputStream(target.uri, "wa")?.use { out ->
                        out.write(text.toByteArray(Charsets.UTF_8))
                    } ?: error("打开 Download 输出流失败")
                }
            }
            true
        } catch (e: Exception) {
            statusCallback("写入步态CSV失败: ${target.displayPath} (${e.message})")
            false
        }
    }

    private fun formatDouble(value: Double): String {
        return String.format(Locale.US, "%.6f", value)
    }

    private fun formatFloat(value: Float): String {
        return String.format(Locale.US, "%.6f", value)
    }

    private fun sanitizeFilePart(raw: String): String {
        return raw.trim()
            .replace("\\s+".toRegex(), "_")
            .replace("[\\\\/:*?\"<>|]".toRegex(), "_")
            .trim('_')
            .ifBlank { "insole_v2" }
    }
}
