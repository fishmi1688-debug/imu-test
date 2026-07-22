package com.imu.motionrecorder.database

import android.content.Context
import androidx.room.Database
import androidx.room.Room
import androidx.room.RoomDatabase

@Database(entities = [MotionRecord::class], version = 1, exportSchema = false)
abstract class MotionDatabase : RoomDatabase() {
    abstract fun motionRecordDao(): MotionRecordDao

    companion object {
        // 单例模式
        @Volatile
        private var INSTANCE: MotionDatabase? = null

        fun getInstance(context: Context): MotionDatabase {
            return INSTANCE ?: synchronized(this) {
                val instance = Room.databaseBuilder(
                    context.applicationContext,
                    MotionDatabase::class.java,
                    "motion_database"
                ).build()
                INSTANCE = instance
                instance
            }
        }
    }
}