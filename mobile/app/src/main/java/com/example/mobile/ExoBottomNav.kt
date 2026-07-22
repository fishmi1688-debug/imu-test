package com.example.mobile

import android.app.Activity
import android.content.Intent
import android.widget.ImageView
import android.widget.TextView
import androidx.core.content.ContextCompat

enum class ExoDestination(val activityClass: Class<out Activity>) {
    BLUETOOTH(MainActivity::class.java),
    GAIT_PARAMS(GaitParameterActivity::class.java),
    PLOT(GaitRealtimePlotActivity::class.java),
    GLASSES_CONTROL(DeviceControlActivity::class.java),
    API(ApiConfigActivity::class.java),
}

object ExoBottomNav {

    private data class NavItem(
        val destination: ExoDestination,
        val containerId: Int,
        val iconId: Int,
        val labelId: Int,
    )

    private val items = listOf(
        NavItem(
            destination = ExoDestination.BLUETOOTH,
            containerId = R.id.exoNavBluetoothItem,
            iconId = R.id.exoNavBluetoothIcon,
            labelId = R.id.exoNavBluetoothText,
        ),
        NavItem(
            destination = ExoDestination.GAIT_PARAMS,
            containerId = R.id.exoNavGaitItem,
            iconId = R.id.exoNavGaitIcon,
            labelId = R.id.exoNavGaitText,
        ),
        NavItem(
            destination = ExoDestination.PLOT,
            containerId = R.id.exoNavPlotItem,
            iconId = R.id.exoNavPlotIcon,
            labelId = R.id.exoNavPlotText,
        ),
        NavItem(
            destination = ExoDestination.GLASSES_CONTROL,
            containerId = R.id.exoNavGlassItem,
            iconId = R.id.exoNavGlassIcon,
            labelId = R.id.exoNavGlassText,
        ),
        NavItem(
            destination = ExoDestination.API,
            containerId = R.id.exoNavApiItem,
            iconId = R.id.exoNavApiIcon,
            labelId = R.id.exoNavApiText,
        ),
    )

    fun setup(activity: Activity, current: ExoDestination) {
        items.forEach { item ->
            val container = activity.findViewById<android.view.View>(item.containerId)
            val icon = activity.findViewById<ImageView>(item.iconId)
            val label = activity.findViewById<TextView>(item.labelId)
            val selected = item.destination == current
            val tint = ContextCompat.getColor(
                activity,
                if (selected) R.color.accent_blue else R.color.text_secondary,
            )
            container.setBackgroundResource(
                if (selected) R.drawable.bg_exo_nav_item_selected else R.drawable.bg_exo_nav_item_normal
            )
            icon.setColorFilter(tint)
            label.setTextColor(tint)
            container.setOnClickListener {
                if (selected) {
                    return@setOnClickListener
                }
                activity.startActivity(
                    Intent(activity, item.destination.activityClass).apply {
                        addFlags(Intent.FLAG_ACTIVITY_REORDER_TO_FRONT)
                        addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP)
                    }
                )
            }
        }
    }

    fun openHome(activity: Activity) {
        activity.startActivity(
            Intent(activity, EntryActivity::class.java).apply {
                addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP)
                addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP)
            }
        )
    }
}
