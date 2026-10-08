package com.example.mobile

data class GaitMotionMode(
    val key: String,
    val name: String,
    val description: String,
    val extT0: Double,
    val extTf: Double,
    val extP: Double,
    val extTmax: Double,
    val flexT0: Double,
    val flexTf: Double,
    val flexP: Double,
    val flexTmax: Double,
    val phaseBias: Double,
    val swingThreshold: Double
)

object GaitMotionModes {
    val all: List<GaitMotionMode> = listOf(
        GaitMotionMode(
            key = "imu_phase",
            name = "平地行走",
            description = "使用左右MI1有线CAN大腿IMU生成相位，助力曲线沿用平地行走默认参数，手动启停",
            extT0 = 0.0,
            extTf = 0.30,
            extP = 0.70,
            extTmax = 5.0,
            flexT0 = 0.50,
            flexTf = 0.80,
            flexP = 0.80,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "downhill",
            name = "下坡行走",
            description = "适用于下坡行走",
            extT0 = 0.0,
            extTf = 0.30,
            extP = 0.70,
            extTmax = 5.0,
            flexT0 = 0.50,
            flexTf = 0.80,
            flexP = 0.80,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "uphill",
            name = "上坡行走",
            description = "适用于上坡行走",
            extT0 = 0.0,
            extTf = 0.0,
            extP = 0.0,
            extTmax = 5.0,
            flexT0 = 0.4,
            flexTf = 0.9,
            flexP = 0.2,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "cycling",
            name = "骑自行车",
            description = "适用于骑自行车运动",
            extT0 = 0.0,
            extTf = 0.0,
            extP = 0.0,
            extTmax = 5.0,
            flexT0 = 0.40,
            flexTf = 0.90,
            flexP = 0.20,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "stairs_up",
            name = "上楼梯",
            description = "适用于上楼梯运动",
            extT0 = 0.0,
            extTf = 0.50,
            extP = 0.40,
            extTmax = 5.0,
            flexT0 = 0.55,
            flexTf = 0.85,
            flexP = 0.35,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "stairs_down",
            name = "下楼梯",
            description = "适用于下楼梯运动",
            extT0 = 0.0,
            extTf = 0.0,
            extP = 0.0,
            extTmax = 5.0,
            flexT0 = 0.4,
            flexTf = 0.90,
            flexP = 0.2,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        ),
        GaitMotionMode(
            key = "imu_left_phase",
            name = "test",
            description = "使用左MI1有线CAN大腿IMU生成左腿相位，右腿相位=左腿相位+pi，手动启停",
            extT0 = 0.0,
            extTf = 0.30,
            extP = 0.70,
            extTmax = 5.0,
            flexT0 = 0.50,
            flexTf = 0.80,
            flexP = 0.80,
            flexTmax = 5.0,
            phaseBias = 0.0,
            swingThreshold = 25.0
        )
    )

    val byKey: Map<String, GaitMotionMode> = all.associateBy { it.key }
}
