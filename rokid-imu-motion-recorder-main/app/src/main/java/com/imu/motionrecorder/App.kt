package com.imu.motionrecorder

import android.app.Application
import com.imu.motionrecorder.database.MotionDatabase

class App : Application() {
    lateinit var motionDatabase: MotionDatabase
        private set

    override fun onCreate() {
        super.onCreate()
        // 初始化数据库
        motionDatabase = MotionDatabase.getInstance(this)
    }
}