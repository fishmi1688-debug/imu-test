package com.example.mobile

class InsoleV1Parser {
    data class Result(
        val timestamp: Long,
        val count: Int,
        val values: List<List<Int>>
    )

    private data class Item(
        var count: Int = 0,
        var cache: MutableList<Int> = mutableListOf(),
        var timestamp: Long = 0L,
        var isReady: Boolean = false
    )

    private val map: MutableMap<String, Item> = mutableMapOf()

    fun clear(address: String) {
        map.remove(address.uppercase())
    }

    fun clearAll() {
        map.clear()
    }

    fun parse(address: String, packet: ByteArray): Result? {
        if (packet.isEmpty()) {
            return null
        }

        val key = address.uppercase()
        val item = map.getOrPut(key) { Item() }
        val bytes = packet.map { it.toInt() and 0xFF }

        var result: Result? = null

        if (bytes.size >= 2 && bytes[0] == 0x55 && bytes[1] == 0xAA) {
            if (item.isReady) {
                result = handle(item)
            }
            item.count = bytes.getOrElse(3) { 0 }
            item.cache = bytes.toMutableList()
            item.timestamp = System.currentTimeMillis()
            item.isReady = true
            map[key] = item
            return result
        }

        if (item.isReady && item.cache.size < 244) {
            item.cache.addAll(bytes)
        }

        if (item.cache.size >= 244) {
            result = handle(item)
            item.cache = mutableListOf()
            item.isReady = false
        }

        map[key] = item
        return result
    }

    private fun handle(item: Item): Result? {
        val payload = item.cache.drop(4)
        if (payload.size < 16) {
            return null
        }

        val values = mutableListOf<List<Int>>()
        var index = 0
        while (index + 16 <= payload.size) {
            values.add(payload.subList(index, index + 16).toList())
            index += 16
        }

        return Result(
            timestamp = item.timestamp,
            count = item.count,
            values = values
        )
    }
}
