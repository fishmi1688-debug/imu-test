package com.example.mobile

private fun loggerFlagDataTypes(flag: Int): String? {
    return when (flag) {
        XsensDotManager.LOGGER_FLAG_DEFAULT -> "Quat+Euler+FreeAcc+ACC+Gyr+Mag"
        1 -> "Euler+FreeAcc+Gyr"
        2 -> "Euler+FreeAcc+Mag"
        3 -> "Quat+Gyr"
        4 -> "Quat+dQ+dV+Mag+Status"
        5 -> "Quat+ACC+Gyr"
        XsensDotManager.LOGGER_FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER -> "Quat+FreeAcc+ACC+Gyr+Mag"
        XsensDotManager.LOGGER_FLAG_ACC_GYR_ONLY -> "ACC+Gyr"
        else -> null
    }
}

fun collectionLoggerFlagLabel(flag: Int, includePrefix: Boolean = false): String {
    val label = when (flag) {
        XsensDotManager.LOGGER_FLAG_DEFAULT -> "${loggerFlagDataTypes(flag)}"
        1 -> "Flag=1 (${loggerFlagDataTypes(flag)})"
        2 -> "Flag=2 (${loggerFlagDataTypes(flag)})"
        3 -> "Flag=3 (${loggerFlagDataTypes(flag)})"
        4 -> "Flag=4 (${loggerFlagDataTypes(flag)})"
        5 -> "Flag=5 (${loggerFlagDataTypes(flag)})"
        XsensDotManager.LOGGER_FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER -> "${loggerFlagDataTypes(flag)}"
        XsensDotManager.LOGGER_FLAG_ACC_GYR_ONLY -> "${loggerFlagDataTypes(flag)}"
        else -> "Flag=$flag"
    }
    return if (includePrefix) "采集类型: $label" else label
}
