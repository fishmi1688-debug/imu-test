package com.example.mobile

import java.io.File
import java.io.FileOutputStream

class CsvBufferedLogger(
    private val file: File,
    private val flushThreshold: Int = 500
) {
    private val buffer = StringBuilder()
    private var lineCount = 0

    @Synchronized
    fun append(line: String) {
        buffer.append(line)
        lineCount += 1
        if (lineCount >= flushThreshold) {
            flush()
        }
    }

    @Synchronized
    fun flush() {
        if (buffer.isEmpty()) {
            return
        }
        try {
            file.parentFile?.mkdirs()
            FileOutputStream(file, true).use { out ->
                out.write(buffer.toString().toByteArray(Charsets.UTF_8))
            }
            buffer.setLength(0)
            lineCount = 0
        } catch (_: Exception) {
            // Keep buffered data for next flush attempt.
        }
    }
}
