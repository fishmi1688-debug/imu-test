package com.imu.motionrecorder.database

import androidx.room.Dao
import androidx.room.Insert
import androidx.room.Query
import kotlinx.coroutines.flow.Flow

@Dao
interface MotionRecordDao {
    // 插入一条记录
    @Insert
    suspend fun insertRecord(record: MotionRecord)

    // 查询所有记录（按开始时间倒序）
    @Query("SELECT * FROM motion_records ORDER BY startTime DESC")
    fun getAllRecordsFlow(): Flow<List<MotionRecord>> // Flow支持实时更新

    // 根据ID查询单条记录
    @Query("SELECT * FROM motion_records WHERE id = :recordId")
    suspend fun getRecordById(recordId: Long): MotionRecord?

    // 删除所有记录
    @Query("DELETE FROM motion_records")
    suspend fun deleteAllRecords()

    // 根据ID删除记录
    @Query("DELETE FROM motion_records WHERE id = :recordId")
    suspend fun deleteRecordById(recordId: Long)
}