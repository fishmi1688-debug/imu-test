package com.imu.motionrecorder.util

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat

/**
 * 权限申请工具（API32+适配）
 */
object PermissionUtil {
    // 所需权限
    private val REQUIRED_PERMISSIONS = mutableListOf(Manifest.permission.BODY_SENSORS).apply {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            add(Manifest.permission.BODY_SENSORS_BACKGROUND) // API33+ 后台传感器权限
        }
    }.toTypedArray()

    const val PERMISSION_REQUEST_CODE = 1001

    // 检查是否拥有所有必要权限
    fun hasAllPermissions(activity: Activity): Boolean {
        return REQUIRED_PERMISSIONS.all {
            ContextCompat.checkSelfPermission(activity, it) == PackageManager.PERMISSION_GRANTED
        }
    }

    // 申请权限
    fun requestPermissions(activity: Activity) {
        ActivityCompat.requestPermissions(
            activity,
            REQUIRED_PERMISSIONS,
            PERMISSION_REQUEST_CODE
        )
    }
}
