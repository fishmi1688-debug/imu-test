package com.example.mobile

import java.util.Locale

class InsoleV2Parser {
    data class Result(
        val timestamp: Long,
        val count: Int,
        val values: List<Int>
    )

    private data class Item(
        var cache: MutableList<Int> = mutableListOf()
    )

    private val map: MutableMap<String, Item> = mutableMapOf()

    fun clear(address: String) {
        map.remove(address.uppercase(Locale.US))
    }

    fun clearAll() {
        map.clear()
    }

    fun parse(address: String, packet: ByteArray): Result? {
        if (packet.isEmpty()) {
            return null
        }
        val key = address.uppercase(Locale.US)
        val item = map.getOrPut(key) { Item() }
        val bytes = packet.map { it.toInt() and 0xFF }

        val headIndex = bytes.indexOf(0xAA)
        when {
            bytes.first() == 0xAA -> {
                item.cache.clear()
                item.cache.addAll(bytes)
            }

            headIndex >= 0 -> {
                item.cache.clear()
                item.cache.addAll(bytes.subList(headIndex, bytes.size))
            }

            item.cache.isNotEmpty() -> {
                item.cache.addAll(bytes)
            }

            else -> {
                return null
            }
        }

        if (item.cache.isEmpty() || item.cache.first() != 0xAA) {
            item.cache.clear()
            return null
        }

        val frameLength = if (item.cache.size >= 39) 39 else return null

        val frame = item.cache.take(frameLength)
        val remains = item.cache.drop(frameLength)
        item.cache = remains.toMutableList()
        if (item.cache.isNotEmpty() && item.cache.first() != 0xAA) {
            val nextHeadIndex = item.cache.indexOf(0xAA)
            item.cache = if (nextHeadIndex >= 0) {
                item.cache.drop(nextHeadIndex).toMutableList()
            } else {
                mutableListOf()
            }
        }
        map[key] = item

        return parseFrame(frame)
    }

    private fun parseFrame(bytes: List<Int>): Result? {
        if (bytes.isEmpty() || bytes[0] != 0xAA) {
            return null
        }

        val rawSensorCount = ((bytes.size - 3) / 2).coerceAtLeast(0)
        if (rawSensorCount < 18) return null

        val footId = bytes[1]
        val rawValues = ArrayList<Int>(rawSensorCount)
        for (i in 0 until rawSensorCount) {
            val high = bytes[2 + i * 2]
            val low = bytes[3 + i * 2]
            rawValues.add(high * 256 + low)
        }
        val values18 = rawValues.take(18)
        if (values18.size < 18) {
            return null
        }

        return Result(
            timestamp = System.currentTimeMillis(),
            count = footId,
            values = values18
        )
    }
}
