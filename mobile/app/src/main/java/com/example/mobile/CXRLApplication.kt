package com.example.mobile

import android.app.Application
import com.rokid.cxr.link.CXRLink

class CXRLApplication : Application() {
    var sharedLink: CXRLink? = null
    var isCxrlConnected: Boolean = false
    var isGlassBtConnected: Boolean = false
    var isCustomAppOpened: Boolean = false

    internal val rokidSessionCore: RokidSessionCore by lazy {
        RokidSessionCore(this)
    }

    fun resetSession() {
        rokidSessionCore.resetSession(clearToken = false)
    }
}
