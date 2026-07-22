package com.example.mobile

import android.os.Handler
import android.os.Looper
import java.util.LinkedHashMap
import java.util.LinkedHashSet

object CollectionRealtimeBridge {

    data class DeviceMeta(
        val address: String,
        val name: String,
        val type: String,
        val side: String? = null,
    )

    data class Sample(
        val address: String,
        val name: String,
        val type: String,
        val side: String?,
        val values: FloatArray,
        val legends: List<String>,
        val timestamp: Long,
    )

    data class LabelState(
        val enabled: Boolean,
        val labels: List<String>,
        val currentLabel: String?,
    )

    data class CollectionState(
        val active: Boolean,
        val startedAtMillis: Long?,
    )

    interface Listener {
        fun onDevicesChanged(devices: List<DeviceMeta>)
        fun onSample(sample: Sample)
        fun onLabelStateChanged(state: LabelState) {}
        fun onCollectionStateChanged(state: CollectionState) {}
    }

    private val mainHandler = Handler(Looper.getMainLooper())
    private val lock = Any()
    private val devices = LinkedHashMap<String, DeviceMeta>()
    private val listeners = LinkedHashSet<Listener>()
    private var labelState = LabelState(
        enabled = false,
        labels = emptyList(),
        currentLabel = null,
    )
    private var collectionState = CollectionState(
        active = false,
        startedAtMillis = null,
    )

    fun setConnectedDevices(items: List<DeviceMeta>) {
        synchronized(lock) {
            devices.clear()
            items.forEach { item ->
                devices[item.address] = item
            }
        }
        dispatchDevicesChanged()
    }

    fun upsertDevice(address: String, name: String, type: String, side: String? = null) {
        var changed = false
        synchronized(lock) {
            val previous = devices[address]
            if (previous == null || previous.name != name || previous.type != type || previous.side != side) {
                devices[address] = DeviceMeta(address = address, name = name, type = type, side = side)
                changed = true
            }
        }
        if (changed) {
            dispatchDevicesChanged()
        }
    }

    fun removeDevice(address: String) {
        val changed = synchronized(lock) {
            devices.remove(address) != null
        }
        if (changed) {
            dispatchDevicesChanged()
        }
    }

    fun clear() {
        synchronized(lock) {
            devices.clear()
        }
        dispatchDevicesChanged()
    }

    fun addListener(listener: Listener) {
        synchronized(lock) {
            listeners.add(listener)
        }
        dispatchDevicesChanged(target = listener)
        dispatchLabelState(target = listener)
        dispatchCollectionState(target = listener)
    }

    fun removeListener(listener: Listener) {
        synchronized(lock) {
            listeners.remove(listener)
        }
    }

    fun publishSample(
        address: String,
        name: String,
        type: String,
        side: String?,
        values: FloatArray,
        legends: List<String>,
    ) {
        if (values.isEmpty()) {
            return
        }
        upsertDevice(address, name, type, side)
        val sample = Sample(
            address = address,
            name = name,
            type = type,
            side = side,
            values = values.copyOf(),
            legends = legends.toList(),
            timestamp = System.currentTimeMillis(),
        )
        dispatchSample(sample)
    }

    fun configureLabelMode(enabled: Boolean, labels: List<String>, currentLabel: String?) {
        synchronized(lock) {
            labelState = LabelState(
                enabled = enabled,
                labels = labels.toList(),
                currentLabel = currentLabel,
            )
        }
        dispatchLabelState()
    }

    fun setCurrentLabel(label: String): Boolean {
        val changed = synchronized(lock) {
            if (!labelState.enabled) {
                false
            } else {
                val exists = labelState.labels.any { it == label }
                if (!exists || labelState.currentLabel == label) {
                    false
                } else {
                    labelState = labelState.copy(currentLabel = label)
                    true
                }
            }
        }
        if (changed) {
            dispatchLabelState()
        }
        return changed
    }

    fun getCurrentLabel(): String? {
        return synchronized(lock) {
            labelState.currentLabel
        }
    }

    fun configureCollectionState(active: Boolean, startedAtMillis: Long?) {
        synchronized(lock) {
            collectionState = CollectionState(
                active = active,
                startedAtMillis = if (active) startedAtMillis else null,
            )
        }
        dispatchCollectionState()
    }

    private fun dispatchDevicesChanged(target: Listener? = null) {
        val snapshotDevices = synchronized(lock) {
            devices.values.toList()
        }
        if (target != null) {
            postOnMain {
                target.onDevicesChanged(snapshotDevices)
            }
            return
        }

        val snapshotListeners = synchronized(lock) {
            listeners.toList()
        }
        postOnMain {
            snapshotListeners.forEach { listener ->
                listener.onDevicesChanged(snapshotDevices)
            }
        }
    }

    private fun dispatchSample(sample: Sample) {
        val snapshotListeners = synchronized(lock) {
            listeners.toList()
        }
        postOnMain {
            snapshotListeners.forEach { listener ->
                listener.onSample(sample)
            }
        }
    }

    private fun dispatchLabelState(target: Listener? = null) {
        val snapshot = synchronized(lock) { labelState }
        if (target != null) {
            postOnMain {
                target.onLabelStateChanged(snapshot)
            }
            return
        }
        val snapshotListeners = synchronized(lock) { listeners.toList() }
        postOnMain {
            snapshotListeners.forEach { listener ->
                listener.onLabelStateChanged(snapshot)
            }
        }
    }

    private fun dispatchCollectionState(target: Listener? = null) {
        val snapshot = synchronized(lock) { collectionState }
        if (target != null) {
            postOnMain {
                target.onCollectionStateChanged(snapshot)
            }
            return
        }
        val snapshotListeners = synchronized(lock) { listeners.toList() }
        postOnMain {
            snapshotListeners.forEach { listener ->
                listener.onCollectionStateChanged(snapshot)
            }
        }
    }

    private fun postOnMain(action: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            action()
        } else {
            mainHandler.post(action)
        }
    }
}
