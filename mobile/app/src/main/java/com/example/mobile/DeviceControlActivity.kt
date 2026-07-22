package com.example.mobile

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.ArrayAdapter
import android.widget.AutoCompleteTextView
import android.widget.Button
import android.widget.ImageButton
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.rokid.cxr.client.utils.ValueUtil

class DeviceControlActivity : AppCompatActivity() {

    companion object {
        private const val GLASS_VALUE_MIN = 0
        private const val GLASS_VALUE_MAX = 15
        private const val BATTERY_REFRESH_INTERVAL_MS = 15000L
    }

    private data class SoundEffectOption(val label: String, val mode: String)

    private val soundEffectOptions = listOf(
        SoundEffectOption("浑厚 (AdiMode0)", RokidBluetoothConnector.SOUND_EFFECT_LOUD),
        SoundEffectOption("韵律 (AdiMode1)", RokidBluetoothConnector.SOUND_EFFECT_RHYTHM),
        SoundEffectOption("播客 (AdiMode2)", RokidBluetoothConnector.SOUND_EFFECT_PODCAST)
    )
    private var currentSoundEffectMode: String = RokidBluetoothConnector.SOUND_EFFECT_RHYTHM

    private lateinit var statusView: TextView
    private lateinit var batteryPercentView: TextView
    private lateinit var volumeValueView: TextView
    private lateinit var brightnessValueView: TextView
    private lateinit var volumeSeekBar: SeekBar
    private lateinit var brightnessSeekBar: SeekBar
    private lateinit var soundEffectDropdown: AutoCompleteTextView
    private lateinit var refreshInfoButton: Button
    private lateinit var applyVolumeButton: Button
    private lateinit var applyBrightnessButton: Button
    private lateinit var rebootGlassButton: Button
    private lateinit var shutdownGlassButton: Button
    private lateinit var checkBluetoothStatusButton: Button
    private lateinit var reconnectBluetoothButton: Button

    private lateinit var rokidBluetoothConnector: RokidBluetoothConnector
    private val batteryRefreshHandler = Handler(Looper.getMainLooper())
    private val batteryRefreshRunnable = object : Runnable {
        override fun run() {
            fetchCurrentInfo(silentWhenDisconnected = true)
            batteryRefreshHandler.postDelayed(this, BATTERY_REFRESH_INTERVAL_MS)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_device_control)
        ExoBottomNav.setup(this, ExoDestination.GLASSES_CONTROL)

        statusView = findViewById(R.id.controlStatusView)
        batteryPercentView = findViewById(R.id.controlBatteryPercentView)
        volumeValueView = findViewById(R.id.volumeValueView)
        brightnessValueView = findViewById(R.id.brightnessValueView)
        volumeSeekBar = findViewById(R.id.volumeSeekBar)
        brightnessSeekBar = findViewById(R.id.brightnessSeekBar)
        soundEffectDropdown = findViewById(R.id.soundEffectDropdown)
        refreshInfoButton = findViewById(R.id.refreshInfoButton)
        applyVolumeButton = findViewById(R.id.applyVolumeButton)
        applyBrightnessButton = findViewById(R.id.applyBrightnessButton)
        rebootGlassButton = findViewById(R.id.rebootGlassButton)
        shutdownGlassButton = findViewById(R.id.shutdownGlassButton)
        checkBluetoothStatusButton = findViewById(R.id.controlStatusButton)
        reconnectBluetoothButton = findViewById(R.id.controlReconnectButton)
        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            ExoBottomNav.openHome(this)
        }

        rokidBluetoothConnector = RokidBluetoothConnector(this) { message ->
            showStatus(message)
        }

        volumeSeekBar.max = GLASS_VALUE_MAX
        brightnessSeekBar.max = GLASS_VALUE_MAX
        volumeSeekBar.progress = 8
        brightnessSeekBar.progress = 8
        updateVolumeText(volumeSeekBar.progress)
        updateBrightnessText(brightnessSeekBar.progress)
        updateBatteryIndicator(null)
        setupSoundEffectDropdown()
        disableUnsupportedControls()

        volumeSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                updateVolumeText(progress)
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) = Unit

            override fun onStopTrackingTouch(seekBar: SeekBar?) = Unit
        })

        brightnessSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar?, progress: Int, fromUser: Boolean) {
                updateBrightnessText(progress)
            }

            override fun onStartTrackingTouch(seekBar: SeekBar?) = Unit

            override fun onStopTrackingTouch(seekBar: SeekBar?) = Unit
        })

        refreshInfoButton.setOnClickListener {
            fetchCurrentInfo()
        }

        applyVolumeButton.setOnClickListener {
            runWhenBluetoothConnected {
                rokidBluetoothConnector.setVolume(volumeSeekBar.progress)
            }
        }

        applyBrightnessButton.setOnClickListener {
            runWhenBluetoothConnected {
                rokidBluetoothConnector.setBrightness(brightnessSeekBar.progress)
            }
        }

        rebootGlassButton.setOnClickListener {
            runWhenBluetoothConnected {
                showConfirmDialog(
                    title = "确认重启眼镜",
                    message = "确定要重启眼镜吗？"
                ) {
                    rokidBluetoothConnector.rebootGlasses()
                }
            }
        }

        shutdownGlassButton.setOnClickListener {
            runWhenBluetoothConnected {
                showConfirmDialog(
                    title = "确认关机眼镜",
                    message = "确定要让眼镜关机吗？"
                ) {
                    rokidBluetoothConnector.shutdownGlasses()
                }
            }
        }

        checkBluetoothStatusButton.setOnClickListener {
            val connected = rokidBluetoothConnector.isBluetoothConnected()
            showStatus("蓝牙通信状态: ${if (connected) "已连接" else "未连接"}")
            if (!connected) {
                updateBatteryIndicator(null)
            }
        }

        reconnectBluetoothButton.setOnClickListener {
            rokidBluetoothConnector.reconnect()
        }

        fetchCurrentInfo(silentWhenDisconnected = true)
        startBatteryRefreshLoop()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopBatteryRefreshLoop()
        rokidBluetoothConnector.release()
    }

    private fun setupSoundEffectDropdown() {
        val labels = soundEffectOptions.map { it.label }
        val adapter = ArrayAdapter(this, R.layout.item_dropdown_black_text, labels)
        soundEffectDropdown.setAdapter(adapter)
        soundEffectDropdown.setDropDownBackgroundResource(R.drawable.bg_dropdown_white)
        setSoundEffectDropdownSelection(currentSoundEffectMode)
        soundEffectDropdown.setOnClickListener {
            soundEffectDropdown.showDropDown()
        }
        soundEffectDropdown.setOnItemClickListener { _, _, position, _ ->
            val selected = soundEffectOptions[position]
            runWhenBluetoothConnected {
                val status = rokidBluetoothConnector.setSoundEffect(selected.mode)
                if (status == ValueUtil.CxrStatus.REQUEST_SUCCEED) {
                    currentSoundEffectMode = selected.mode
                    setSoundEffectDropdownSelection(currentSoundEffectMode)
                }
            }
        }
    }

    private fun setSoundEffectDropdownSelection(mode: String) {
        val label = soundEffectOptions.firstOrNull { it.mode == mode }?.label
            ?: soundEffectOptions.first().label
        soundEffectDropdown.setText(label, false)
    }

    private fun fetchCurrentInfo(silentWhenDisconnected: Boolean = false) {
        runWhenBluetoothConnected(silentWhenDisconnected) {
            rokidBluetoothConnector.getGlassesInfo { status, info ->
                if (status == ValueUtil.CxrStatus.RESPONSE_SUCCEED && info != null) {
                    runOnUiThread {
                        val volume = info.volume.coerceIn(GLASS_VALUE_MIN, GLASS_VALUE_MAX)
                        val brightness = info.brightness.coerceIn(GLASS_VALUE_MIN, GLASS_VALUE_MAX)
                        volumeSeekBar.progress = volume
                        brightnessSeekBar.progress = brightness
                        updateVolumeText(volume)
                        updateBrightnessText(brightness)
                        updateBatteryIndicator(info.batteryLevel)
                    }
                }
            }
        }
    }

    private fun updateVolumeText(value: Int) {
        volumeValueView.text = "当前值: $value"
    }

    private fun updateBrightnessText(value: Int) {
        brightnessValueView.text = "当前值: $value"
    }

    private fun runWhenBluetoothConnected(
        silentWhenDisconnected: Boolean = false,
        action: () -> Unit
    ) {
        if (!rokidBluetoothConnector.isBluetoothConnected()) {
            if (!silentWhenDisconnected) {
                Toast.makeText(this, "请先在首页完成 Rokid 会话连接", Toast.LENGTH_SHORT).show()
                showStatus("眼镜会话未连接")
            }
            updateBatteryIndicator(null)
            return
        }
        action()
    }

    private fun showStatus(text: String) {
        runOnUiThread {
            statusView.text = "状态: $text"
        }
    }

    private fun updateBatteryIndicator(level: Int?) {
        val normalized = level?.coerceIn(0, 100)
        runOnUiThread {
            batteryPercentView.text = normalized?.let { "$it%" } ?: "--%"
        }
    }

    private fun startBatteryRefreshLoop() {
        batteryRefreshHandler.removeCallbacks(batteryRefreshRunnable)
        batteryRefreshHandler.post(batteryRefreshRunnable)
    }

    private fun stopBatteryRefreshLoop() {
        batteryRefreshHandler.removeCallbacks(batteryRefreshRunnable)
    }

    private fun disableUnsupportedControls() {
        soundEffectDropdown.isEnabled = false
        soundEffectDropdown.alpha = 0.5f
        rebootGlassButton.isEnabled = false
        rebootGlassButton.alpha = 0.5f
        shutdownGlassButton.isEnabled = false
        shutdownGlassButton.alpha = 0.5f
    }

    private fun showConfirmDialog(title: String, message: String, onConfirm: () -> Unit) {
        AlertDialog.Builder(this)
            .setTitle(title)
            .setMessage(message)
            .setPositiveButton("确定") { _, _ -> onConfirm() }
            .setNegativeButton("取消", null)
            .show()
    }
}
