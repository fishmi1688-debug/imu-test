package com.imu.motionrecorder.activity

import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.google.android.material.button.MaterialButton
import com.imu.motionrecorder.R
import com.rokid.cxr.CXRServiceBridge
import com.rokid.cxr.Caps

class MainActivity : AppCompatActivity() {

    private lateinit var tvStatus: TextView
    private lateinit var tvCommand: TextView
    private lateinit var tvClass: TextView
    private lateinit var tvMode: TextView
    private lateinit var btnToggleRecord: MaterialButton
    private lateinit var btnExit: MaterialButton
    private lateinit var btnBluetooth: MaterialButton

    private val statusTag = "StatusListener"
    private val messageTag = "MessageSubscribe"
    private val sendTag = "MessageSend"
    private val controlMessageChannel = "imu_capture_control"
    private val resultMessageChannel = "imu_capture_result"

    private var isCxrSubscribed = false
    @Volatile
    private var isCxrConnected = false
    private val cxrBridge = CXRServiceBridge()

    private enum class RecordState { IDLE, RECORDING }
    private var currentState = RecordState.IDLE
    private val mainHandler = Handler(Looper.getMainLooper())
    private var commandToken = 0

    private val statusListener by lazy {
        object : CXRServiceBridge.StatusListener {
            override fun onConnected(name: String, type: Int) {
                val typeLabel = when (type) {
                    CXRServiceBridge.StatusListener.DEVICE_TYPE_ANDROID -> "Android"
                    CXRServiceBridge.StatusListener.DEVICE_TYPE_IOS -> "iPhone"
                    else -> "Unknown"
                }
                isCxrConnected = true
                Log.i(statusTag, "Connected to $name, type=$typeLabel($type)")
                subscribeMessages()
                runOnUiThread {
                    btnBluetooth.text = getString(R.string.bluetooth_connected)
                }
            }

            override fun onDisconnected() {
                isCxrConnected = false
                Log.i(statusTag, "Disconnected")
                runOnUiThread {
                    btnBluetooth.text = getString(R.string.bluetooth)
                }
            }

            override fun onARTCStatus(health: Float, reset: Boolean) {
                Log.d(statusTag, "ARTC Status: Health: ${(health * 100).toInt()}%, reset=$reset")
            }
        }
    }

    private val resultCallback = object : CXRServiceBridge.MsgCallback {
        override fun onReceive(name: String, args: Caps, value: ByteArray?) {
            val line = value?.toString(Charsets.UTF_8)?.trim().orEmpty()
            Log.i(messageTag, "Result message name=$name, argsSize=${args.size()}, value=$line")
            if (line.isNotEmpty()) {
                handleIncomingMessage(line)
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        initView()
        bindEvents()
        setStatusListener()
        subscribeMessages()
    }

    private fun initView() {
        tvStatus = findViewById(R.id.tv_status)
        tvCommand = findViewById(R.id.tv_command)
        tvClass = findViewById(R.id.tv_class)
        tvMode = findViewById(R.id.tv_mode)
        btnToggleRecord = findViewById(R.id.btn_toggle_record)
        btnExit = findViewById(R.id.btn_exit)
        btnBluetooth = findViewById(R.id.btn_bluetooth)
    }

    private fun setStatusListener() {
        cxrBridge.setStatusListener(statusListener)
    }

    private fun subscribeMessages() {
        if (isCxrSubscribed) return
        val result = cxrBridge.subscribe(resultMessageChannel, resultCallback)
        logSubscribeResult(resultMessageChannel, result)
        isCxrSubscribed = result == 0 || result == -2
    }

    private fun logSubscribeResult(name: String, result: Int) {
        val message = when (result) {
            0 -> "subscribe success"
            -1 -> "subscribe failed: invalid parameter"
            -2 -> "subscribe skipped: duplicate subscription"
            else -> "subscribe failed: code=$result"
        }
        if (result == 0 || result == -2) {
            Log.i(messageTag, "[$name] $message")
        } else {
            Log.e(messageTag, "[$name] $message")
        }
    }

    private fun sendControlMessage(command: String) {
        if (!isCxrConnected) {
            Log.w(sendTag, "Skip control send: CXR not connected, command=$command")
            return
        }
        val payload = command.toByteArray(Charsets.UTF_8)
        val args = Caps().apply {
            write("send_message")
            writeUInt32(payload.size)
        }
        val result = cxrBridge.sendMessage(controlMessageChannel, args, payload, 0, payload.size)
        logSendResult(controlMessageChannel, result, command)
    }

    private fun logSendResult(channel: String, result: Int, payload: String) {
        val message = when (result) {
            0 -> "send success"
            -1 -> "send failed: invalid parameter"
            -3 -> "send failed: internal error"
            else -> "send failed: code=$result"
        }
        if (result == 0) {
            Log.i(sendTag, "[$channel] $message payload=$payload")
        } else {
            Log.e(sendTag, "[$channel] $message payload=$payload")
        }
    }

    private fun bindEvents() {
        btnToggleRecord.setOnClickListener {
            if (currentState == RecordState.IDLE) {
                startRecordingFlow()
            } else {
                stopRecordingFlow()
            }
        }

        btnExit.setOnClickListener {
            finish()
        }

        btnBluetooth.setOnClickListener {
            val tip = if (isCxrConnected) {
                "已连接手机，可开始采集"
            } else {
                "请先在手机端完成 SDK 连接"
            }
            Toast.makeText(this, tip, Toast.LENGTH_SHORT).show()
        }
    }

    private fun startRecordingFlow() {
        currentState = RecordState.RECORDING
        tvStatus.text = getString(R.string.recording)
        tvStatus.setTextColor(ContextCompat.getColor(this, R.color.rgrcolor))
        btnToggleRecord.text = getString(R.string.stop)

        sendControlMessage("record_start")
    }

    private fun stopRecordingFlow() {
        currentState = RecordState.IDLE

        sendControlMessage("record_stop")
        Toast.makeText(this, "记录已停止", Toast.LENGTH_SHORT).show()

        tvStatus.text = getString(R.string.ready_to_record)
        tvStatus.setTextColor(ContextCompat.getColor(this, R.color.rgrcolor_normal))
        btnToggleRecord.text = getString(R.string.start)

        resetUI()
    }

    private fun resetUI() {
        tvCommand.text = getString(R.string.empty_placeholder)
        tvClass.text = getString(R.string.empty_placeholder)
        tvMode.text = getString(R.string.empty_placeholder)
    }

    private fun handleIncomingMessage(raw: String) {
        val line = raw.trim()
        if (line.isEmpty()) return

        if (line.startsWith("RESULT|", ignoreCase = true)) {
            showRelayResult(line)
            return
        }

        if (line.startsWith("CMD,", ignoreCase = true)) {
            val value = line.substringAfter("CMD,").trim()
            showCommand(value)
            return
        }
        if (line.startsWith("CLS,", ignoreCase = true)) {
            val label = line.substringAfter("CLS,").trim()
            if (label.isNotEmpty()) {
                showClass(label)
            }
            return
        }
        if (line.startsWith("MODE,", ignoreCase = true)) {
            val mode = line.substringAfter("MODE,").trim()
            if (mode.isNotEmpty()) {
                showMode(mode)
            }
            return
        }

        showClass(line)
    }

    private fun showRelayResult(line: String) {
        val kv = mutableMapOf<String, String>()
        line.split('|').drop(1).forEach { segment ->
            val key = segment.substringBefore('=', "").trim()
            val value = segment.substringAfter('=', "").trim()
            if (key.isNotEmpty()) {
                kv[key] = value
            }
        }

        val frame = kv["frame"] ?: "-"
        val scene = kv["scene"] ?: "-"
        val luma = kv["luma"] ?: "-"
        val size = kv["size"] ?: "-"

        runOnUiThread {
            tvCommand.text = if (frame == "-") "-" else "第${frame}帧"
            tvClass.text = "场景:$scene 亮度:$luma"
            tvMode.text = "$size @1Hz"
        }
    }

    private fun showCommand(value: String) {
        runOnUiThread {
            commandToken += 1
            val token = commandToken
            tvCommand.text = value
            mainHandler.postDelayed({
                if (token == commandToken) {
                    tvCommand.text = getString(R.string.empty_placeholder)
                }
            }, 1000)
        }
    }

    private fun showClass(label: String) {
        runOnUiThread {
            tvClass.text = label
        }
    }

    private fun showMode(mode: String) {
        runOnUiThread {
            tvMode.text = mode
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        cxrBridge.setStatusListener(null)
    }
}
