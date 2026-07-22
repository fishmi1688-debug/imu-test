package com.example.mobile

import android.app.Activity
import android.content.Context
import android.content.Intent
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.example.mobile.RokidBluetoothConnector.GlassesInfo
import com.example.mobile.RokidBluetoothConnector.ScannedDevice
import com.rokid.cxr.Caps
import com.rokid.cxr.client.utils.ValueUtil
import com.rokid.cxr.link.CXRLink
import com.rokid.cxr.link.callbacks.ICXRLinkCbk
import com.rokid.cxr.link.callbacks.ICustomCmdCbk
import com.rokid.cxr.link.callbacks.IGlassAppCbk
import com.rokid.cxr.link.callbacks.IImageStreamCbk
import com.rokid.cxr.link.utils.CxrDefs
import com.rokid.cxr.link.utils.GlassInfo
import com.rokid.sprite.aiapp.externalapp.auth.AuthResult
import com.rokid.sprite.aiapp.externalapp.auth.AuthorizationHelper
import com.rokid.sprite.aiapp.externalapp.auth.GlassPermission
import java.util.concurrent.CopyOnWriteArraySet

class RokidBluetoothConnector(
    context: Context,
    private val statusCallback: (String) -> Unit
) {
    companion object {
        private const val TAG = "RokidBluetooth"
        internal const val AUTH_REQUEST_CODE = 1001
        internal const val PSEUDO_DEVICE_ADDRESS = "cxrl-customapp"
        internal const val PSEUDO_DEVICE_NAME = "Rokid Glasses (CXR-L)"
        internal const val TARGET_PACKAGE_NAME = "com.imu.motionrecorder"
        internal const val TARGET_ENTRY_ACTIVITY = "com.imu.motionrecorder.activity.MainActivity"
        const val SOUND_EFFECT_LOUD = "AdiMode0"
        const val SOUND_EFFECT_RHYTHM = "AdiMode1"
        const val SOUND_EFFECT_PODCAST = "AdiMode2"
    }

    data class ScannedDevice(
        val address: String,
        val displayName: String,
        val bonded: Boolean
    )

    data class GlassesInfo(
        val deviceName: String,
        val batteryLevel: Int,
        val brightness: Int,
        val volume: Int,
        val systemVersion: String?,
        val wearingStatus: String?,
        val screenOn: Boolean
    )

    private val activity = context as? Activity
    private val app = context.applicationContext as CXRLApplication
    private val core = app.rokidSessionCore
    private var customMessageListener: ((String, ByteArray?) -> Unit)? = null

    private val listener = object : RokidSessionCore.Listener {
        override fun onStatus(message: String) {
            statusCallback(message)
        }

        override fun onCustomMessage(channel: String, payload: ByteArray?) {
            customMessageListener?.invoke(channel, payload)
        }
    }

    init {
        core.addListener(listener)
    }

    fun release() {
        core.removeListener(listener)
    }

    fun hasBluetoothAdapter(): Boolean = true

    fun isBluetoothEnabled(): Boolean = true

    fun isScanning(): Boolean = core.isScanning

    fun isConnecting(): Boolean = core.isConnecting

    fun currentDeviceAddress(): String? = core.currentDeviceAddress

    fun isBluetoothConnected(): Boolean = core.isSessionReady

    fun reconnect(): Boolean = core.reconnect()

    fun deinitBluetooth() {
        core.disconnect()
    }

    fun setCustomMessageListener(listener: ((String, ByteArray?) -> Unit)?) {
        customMessageListener = listener
    }

    fun handleActivityResult(requestCode: Int, resultCode: Int, data: Intent?): Boolean {
        val hostActivity = activity ?: return false
        return core.handleActivityResult(hostActivity, requestCode, resultCode, data)
    }

    fun startScanForSelection(onDevicesUpdated: (List<ScannedDevice>) -> Unit): Boolean {
        val hostActivity = activity ?: return false
        return core.startScanForSelection(hostActivity, onDevicesUpdated)
    }

    fun stopScan() {
        core.stopScan()
    }

    fun connectDeviceByAddress(address: String): Boolean {
        if (address != PSEUDO_DEVICE_ADDRESS) {
            statusCallback("不支持的眼镜连接目标: $address")
            return false
        }
        val hostActivity = activity ?: return false
        return core.connect(hostActivity)
    }

    fun getGlassesInfo(
        onResult: ((ValueUtil.CxrStatus?, GlassesInfo?) -> Unit)? = null
    ): ValueUtil.CxrStatus {
        return core.getGlassesInfo(onResult)
    }

    fun setBrightness(brightness: Int): ValueUtil.CxrStatus {
        return if (core.setBrightness(brightness)) {
            ValueUtil.CxrStatus.REQUEST_SUCCEED
        } else {
            ValueUtil.CxrStatus.REQUEST_FAILED
        }
    }

    fun setVolume(volume: Int): ValueUtil.CxrStatus {
        return if (core.setVolume(volume)) {
            ValueUtil.CxrStatus.REQUEST_SUCCEED
        } else {
            ValueUtil.CxrStatus.REQUEST_FAILED
        }
    }

    fun setSoundEffect(mode: String): ValueUtil.CxrStatus {
        statusCallback("CXR-L 1.0.4 不支持音效模式设置: $mode")
        return ValueUtil.CxrStatus.REQUEST_FAILED
    }

    fun rebootGlasses(): ValueUtil.CxrStatus {
        statusCallback("CXR-L 1.0.4 未提供眼镜重启接口")
        return ValueUtil.CxrStatus.REQUEST_FAILED
    }

    fun shutdownGlasses(): ValueUtil.CxrStatus {
        statusCallback("CXR-L 1.0.4 未提供眼镜关机接口")
        return ValueUtil.CxrStatus.REQUEST_FAILED
    }

    fun aiOpenCamera(width: Int, height: Int, quality: Int): ValueUtil.CxrStatus {
        return if (core.isSessionReady) {
            statusCallback("拍照链路已就绪: ${width}x$height, quality=$quality")
            ValueUtil.CxrStatus.REQUEST_SUCCEED
        } else {
            statusCallback("眼镜会话未就绪，无法开始拍照")
            ValueUtil.CxrStatus.REQUEST_FAILED
        }
    }

    fun takePhoto(
        width: Int,
        height: Int,
        quality: Int,
        onResult: ((ValueUtil.CxrStatus?, ByteArray?) -> Unit)? = null
    ): ValueUtil.CxrStatus {
        return core.takePhoto(width, height, quality, onResult)
    }

    fun sendCustomMessage(channel: String, payload: String): Boolean {
        return core.sendCustomMessage(channel, payload)
    }
}

internal class RokidSessionCore(
    private val app: CXRLApplication
) {
    interface Listener {
        fun onStatus(message: String)
        fun onCustomMessage(channel: String, payload: ByteArray?) = Unit
    }

    companion object {
        private const val TAG = "RokidSessionCore"
        private const val PREFS_NAME = "cxrl_session"
        private const val KEY_TOKEN = "auth_token"
    }

    private val appContext = app.applicationContext
    private val mainHandler = Handler(Looper.getMainLooper())
    private val prefs = appContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    private val listeners = CopyOnWriteArraySet<Listener>()
    private val photoCallbacks = mutableListOf<(ValueUtil.CxrStatus?, ByteArray?) -> Unit>()
    private val infoCallbacks = mutableListOf<(ValueUtil.CxrStatus?, GlassesInfo?) -> Unit>()
    private var token: String? = prefs.getString(KEY_TOKEN, null)
    private var lastGlassInfo: GlassInfo? = null
    private var customAppStarting = false
    private var connectRequested = false
    private var authRequested = false
    private var _isScanning = false
    private var currentDeviceAddressInternal: String? = null

    val isScanning: Boolean
        get() = _isScanning

    val isConnecting: Boolean
        get() = connectRequested || authRequested || customAppStarting

    val currentDeviceAddress: String?
        get() = currentDeviceAddressInternal

    val isLinkReady: Boolean
        get() = app.isCxrlConnected && app.isGlassBtConnected

    val isSessionReady: Boolean
        get() = isLinkReady && app.isCustomAppOpened

    private val linkCallback = object : ICXRLinkCbk {
        override fun onCXRLConnected(connected: Boolean) {
            app.isCxrlConnected = connected
            if (!connected) {
                app.isCustomAppOpened = false
                customAppStarting = false
                connectRequested = false
            }
            publishStatus(if (connected) "CXR 服务已连接" else "CXR 服务已断开")
            if (connected) {
                maybeStartCustomApp()
            }
        }

        override fun onGlassBtConnected(connected: Boolean) {
            app.isGlassBtConnected = connected
            if (!connected) {
                app.isCustomAppOpened = false
                customAppStarting = false
                connectRequested = false
            }
            publishStatus(if (connected) "眼镜蓝牙已连接" else "眼镜蓝牙已断开")
            if (connected) {
                maybeStartCustomApp()
            }
        }

        override fun onGlassDeviceInfo(info: GlassInfo) {
            lastGlassInfo = info
            val compatInfo = info.toCompat()
            val callbacks = infoCallbacks.toList()
            infoCallbacks.clear()
            callbacks.forEach { callback ->
                mainHandler.post {
                    callback(ValueUtil.CxrStatus.RESPONSE_SUCCEED, compatInfo)
                }
            }
        }

        override fun onGlassWearingStatus(wearing: Boolean) {
            Log.d(TAG, "onGlassWearingStatus: $wearing")
        }

        override fun onGlassAiAssistStart() {
            Log.d(TAG, "onGlassAiAssistStart")
        }

        override fun onGlassAiAssistStop() {
            Log.d(TAG, "onGlassAiAssistStop")
        }

        override fun onGlassAiInterrupt(interrupted: Boolean) {
            Log.d(TAG, "onGlassAiInterrupt: $interrupted")
        }
    }

    private val imageCallback = object : IImageStreamCbk {
        override fun onImageReceived(data: ByteArray?) {
            val callbacks = photoCallbacks.toList()
            photoCallbacks.clear()
            callbacks.forEach { callback ->
                mainHandler.post {
                    callback(ValueUtil.CxrStatus.RESPONSE_SUCCEED, data)
                }
            }
        }

        override fun onImageError(code: Int, msg: String?) {
            publishStatus("拍照失败: code=$code, msg=${msg ?: "-"}")
            val callbacks = photoCallbacks.toList()
            photoCallbacks.clear()
            callbacks.forEach { callback ->
                mainHandler.post {
                    callback(ValueUtil.CxrStatus.RESPONSE_FAILED, null)
                }
            }
        }
    }

    private val customCmdCallback = object : ICustomCmdCbk {
        override fun onCustomCmdResult(key: String, payload: ByteArray?) {
            listeners.forEach { listener ->
                mainHandler.post {
                    listener.onCustomMessage(key, payload)
                }
            }
        }
    }

    private val appCallback = object : IGlassAppCbk {
        override fun onInstallAppResult(success: Boolean) {
            publishStatus(if (success) "眼镜端应用安装成功" else "眼镜端应用安装失败")
        }

        override fun onUnInstallAppResult(success: Boolean) {
            publishStatus(if (success) "眼镜端应用已卸载" else "眼镜端应用卸载失败")
        }

        override fun onOpenAppResult(success: Boolean) {
            customAppStarting = false
            connectRequested = false
            app.isCustomAppOpened = success
            publishStatus(if (success) "眼镜端应用已启动，会话已就绪" else "眼镜端应用启动失败")
        }

        override fun onStopAppResult(success: Boolean) {
            if (success) {
                app.isCustomAppOpened = false
            }
            publishStatus(if (success) "眼镜端应用已停止" else "眼镜端应用停止失败")
        }

        override fun onGlassAppResume(resumed: Boolean) {
            app.isCustomAppOpened = resumed
            if (resumed) {
                customAppStarting = false
                connectRequested = false
            }
            publishStatus(if (resumed) "眼镜端应用已恢复到前台" else "眼镜端应用已离开前台")
        }

        override fun onQueryAppResult(installed: Boolean) {
            if (!installed) {
                customAppStarting = false
                connectRequested = false
                publishStatus("眼镜端应用未安装: com.imu.motionrecorder")
                return
            }
            publishStatus("眼镜端应用已安装，正在启动")
            app.sharedLink?.appStart(RokidBluetoothConnector.TARGET_ENTRY_ACTIVITY, this)
        }
    }

    fun addListener(listener: Listener) {
        listeners.add(listener)
    }

    fun removeListener(listener: Listener) {
        listeners.remove(listener)
    }

    fun startScanForSelection(
        activity: Activity,
        onDevicesUpdated: (List<ScannedDevice>) -> Unit
    ): Boolean {
        _isScanning = true
        mainHandler.post {
            val devices = if (hasRequiredCompanionApp(activity)) {
                listOf(
                    ScannedDevice(
                        address = RokidBluetoothConnector.PSEUDO_DEVICE_ADDRESS,
                        displayName = RokidBluetoothConnector.PSEUDO_DEVICE_NAME,
                        bonded = true
                    )
                )
            } else {
                publishStatus("请先安装 Rokid AI App 1.9.0+ 或 Hi Rokid")
                emptyList()
            }
            onDevicesUpdated(devices)
            _isScanning = false
        }
        return true
    }

    fun stopScan() {
        _isScanning = false
    }

    fun connect(activity: Activity): Boolean {
        if (isSessionReady) {
            publishStatus("眼镜会话已就绪")
            return true
        }
        if (!hasRequiredCompanionApp(activity)) {
            publishStatus("未检测到可用的 Rokid AI App / Hi Rokid")
            return false
        }
        currentDeviceAddressInternal = RokidBluetoothConnector.PSEUDO_DEVICE_ADDRESS
        val cachedToken = token
        return if (!cachedToken.isNullOrBlank()) {
            publishStatus("使用已授权 token 建立会话")
            if (connectWithToken(cachedToken)) {
                true
            } else {
                publishStatus("缓存 token 建链失败，重新发起授权")
                token = null
                prefs.edit().remove(KEY_TOKEN).apply()
                requestAuthorization(activity)
            }
        } else {
            requestAuthorization(activity)
        }
    }

    fun reconnect(): Boolean {
        val cachedToken = token
        if (cachedToken.isNullOrBlank()) {
            publishStatus("缺少授权 token，请返回首页重新授权")
            return false
        }
        publishStatus("正在重连眼镜会话")
        return connectWithToken(cachedToken)
    }

    fun disconnect() {
        photoCallbacks.clear()
        infoCallbacks.clear()
        customAppStarting = false
        connectRequested = false
        authRequested = false
        app.isCustomAppOpened = false
        app.isCxrlConnected = false
        app.isGlassBtConnected = false
        app.sharedLink?.let { link ->
            runCatching {
                if (isLinkReady) {
                    link.appStop(appCallback)
                }
            }.onFailure { throwable ->
                Log.w(TAG, "appStop failed before disconnect", throwable)
            }
            runCatching { link.disconnect() }
                .onFailure { throwable -> Log.w(TAG, "disconnect failed", throwable) }
        }
        app.sharedLink = null
        currentDeviceAddressInternal = null
        publishStatus("眼镜会话已断开")
    }

    fun resetSession(clearToken: Boolean) {
        disconnect()
        if (clearToken) {
            token = null
            prefs.edit().remove(KEY_TOKEN).apply()
        }
    }

    fun handleActivityResult(
        activity: Activity,
        requestCode: Int,
        resultCode: Int,
        data: Intent?
    ): Boolean {
        if (requestCode != RokidBluetoothConnector.AUTH_REQUEST_CODE) {
            return false
        }
        authRequested = false
        when (val result = AuthorizationHelper.parseAuthorizationResult(resultCode, data)) {
            is AuthResult.AuthSuccess -> {
                token = result.token
                prefs.edit().putString(KEY_TOKEN, result.token).apply()
                publishStatus("授权成功，正在连接眼镜")
                connectWithToken(result.token)
            }
            is AuthResult.AuthCancel -> {
                connectRequested = false
                publishStatus("授权已取消")
            }
            is AuthResult.AuthFail -> {
                connectRequested = false
                token = null
                prefs.edit().remove(KEY_TOKEN).apply()
                publishStatus("授权失败，请重试")
            }
        }
        return true
    }

    fun getGlassesInfo(
        onResult: ((ValueUtil.CxrStatus?, GlassesInfo?) -> Unit)? = null
    ): ValueUtil.CxrStatus {
        if (!isLinkReady) {
            publishStatus("链路未就绪，无法获取眼镜信息")
            onResult?.invoke(ValueUtil.CxrStatus.REQUEST_FAILED, null)
            return ValueUtil.CxrStatus.REQUEST_FAILED
        }
        onResult?.let(infoCallbacks::add)
        app.sharedLink?.getGlassDeviceInfo()
        return ValueUtil.CxrStatus.REQUEST_SUCCEED
    }

    fun setBrightness(brightness: Int): Boolean {
        if (!isLinkReady) {
            publishStatus("链路未就绪，无法设置亮度")
            return false
        }
        if (brightness !in 0..15) {
            publishStatus("亮度值非法: $brightness，合法范围[0,15]")
            return false
        }
        val success = app.sharedLink?.setGlassBrightness(brightness) == true
        if (!success) {
            publishStatus("亮度设置请求失败")
        }
        return success
    }

    fun setVolume(volume: Int): Boolean {
        if (!isLinkReady) {
            publishStatus("链路未就绪，无法设置音量")
            return false
        }
        if (volume !in 0..15) {
            publishStatus("音量值非法: $volume，合法范围[0,15]")
            return false
        }
        val success = app.sharedLink?.setGlassVolume(volume) == true
        if (!success) {
            publishStatus("音量设置请求失败")
        }
        return success
    }

    fun takePhoto(
        width: Int,
        height: Int,
        quality: Int,
        onResult: ((ValueUtil.CxrStatus?, ByteArray?) -> Unit)? = null
    ): ValueUtil.CxrStatus {
        if (!isSessionReady) {
            publishStatus("眼镜会话未就绪，无法拍照")
            onResult?.invoke(ValueUtil.CxrStatus.REQUEST_FAILED, null)
            return ValueUtil.CxrStatus.REQUEST_FAILED
        }
        onResult?.let(photoCallbacks::add)
        val started = app.sharedLink?.takePhoto(width, height, quality) == true
        if (!started) {
            photoCallbacks.clear()
            publishStatus("拍照请求发起失败")
            onResult?.invoke(ValueUtil.CxrStatus.REQUEST_FAILED, null)
            return ValueUtil.CxrStatus.REQUEST_FAILED
        }
        return ValueUtil.CxrStatus.REQUEST_SUCCEED
    }

    fun sendCustomMessage(channel: String, payload: String): Boolean {
        if (!isSessionReady) {
            publishStatus("眼镜会话未就绪，无法发送自定义指令")
            return false
        }
        val bytes = payload.toByteArray(Charsets.UTF_8)
        val args = Caps().apply {
            write("send_message")
            writeUInt32(bytes.size)
        }
        val result = app.sharedLink?.sendCustomCmd(channel, args, bytes)
        if (result != 0) {
            publishStatus("发送自定义指令失败: channel=$channel, code=$result")
            return false
        }
        return true
    }

    private fun requestAuthorization(activity: Activity): Boolean {
        connectRequested = true
        authRequested = true
        val permissions = arrayOf(
            GlassPermission.CAMERA,
            GlassPermission.MICROPHONE
        )
        val pair: android.util.Pair<Int, Intent>? = runCatching {
            AuthorizationHelper.requestAuthorization(
                activity,
                permissions,
                RokidBluetoothConnector.AUTH_REQUEST_CODE
            )
        }.onFailure { throwable ->
            authRequested = false
            connectRequested = false
            publishStatus("发起授权失败: ${throwable.message ?: throwable::class.java.simpleName}")
        }.getOrNull()
        if (pair == null) {
            publishStatus("请在 Rokid AI App / Hi Rokid 中完成授权")
            return true
        }
        return handleActivityResult(
            activity,
            RokidBluetoothConnector.AUTH_REQUEST_CODE,
            pair.first,
            pair.second
        )
    }

    private fun connectWithToken(token: String): Boolean {
        val link = ensureLink()
        connectRequested = true
        val started = runCatching { link.connect(token) }
            .onFailure { throwable ->
                connectRequested = false
                publishStatus("connect(token) 调用失败: ${throwable.message ?: throwable::class.java.simpleName}")
            }
            .getOrDefault(false)
        if (!started) {
            connectRequested = false
            publishStatus("connect(token) 未成功发起")
            return false
        }
        publishStatus("已发起 CXRLink 连接请求")
        return true
    }

    private fun ensureLink(): CXRLink {
        app.sharedLink?.let { return it }
        val link = CXRLink(appContext).apply {
            configCXRSession(
                CxrDefs.CXRSession(
                    CxrDefs.CXRSessionType.CUSTOMAPP,
                    RokidBluetoothConnector.TARGET_PACKAGE_NAME
                )
            )
            setCXRLinkCbk(linkCallback)
            setCXRImageCbk(imageCallback)
            setCXRCustomCmdCbk(customCmdCallback)
        }
        app.sharedLink = link
        return link
    }

    private fun maybeStartCustomApp() {
        if (!isLinkReady || app.isCustomAppOpened || customAppStarting) {
            return
        }
        customAppStarting = true
        publishStatus("链路就绪，正在检查并启动眼镜端应用")
        app.sharedLink?.appIsInstalled(appCallback)
    }

    private fun hasRequiredCompanionApp(activity: Activity): Boolean {
        return AuthorizationHelper.isRequiredRokidAppInstalled(activity) ||
            AuthorizationHelper.isRequiredHiRokidInstalled(activity)
    }

    private fun GlassInfo.toCompat(): GlassesInfo {
        return GlassesInfo(
            deviceName = deviceName ?: "Rokid Glasses",
            batteryLevel = batteryLevel,
            brightness = brightness,
            volume = sound,
            systemVersion = systemVersion,
            wearingStatus = wearingStatus,
            screenOn = screenOn
        )
    }

    private fun publishStatus(message: String) {
        listeners.forEach { listener ->
            mainHandler.post {
                listener.onStatus(message)
            }
        }
    }
}
