package com.example.mobile

import android.app.Activity
import android.content.Intent
import android.os.Bundle
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ImageButton
import android.widget.Spinner
import androidx.appcompat.app.AppCompatActivity

class CollectionSensorConfigActivity : AppCompatActivity() {

    companion object {
        const val EXTRA_OUTPUT_RATE = "extra_output_rate"
        const val EXTRA_FILTER_PROFILE = "extra_filter_profile"
        const val EXTRA_LOGGER_FLAG = "extra_logger_flag"
    }

    private val outputRates = intArrayOf(1, 4, 10, 12, 15, 20, 30, 60, 120)
    private val filterProfiles = intArrayOf(
        XsensDotManager.FILTER_PROFILE_GENERAL,
        XsensDotManager.FILTER_PROFILE_DYNAMIC,
    )
    private val loggerFlags = intArrayOf(
        XsensDotManager.LOGGER_FLAG_DEFAULT,
        1, 2, 3, 4, 5,
        XsensDotManager.LOGGER_FLAG_DEFAULT_WITH_FREE_ACC_NO_EULER,
        XsensDotManager.LOGGER_FLAG_ACC_GYR_ONLY,
    )

    private lateinit var outputRateSpinner: Spinner
    private lateinit var filterProfileSpinner: Spinner
    private lateinit var loggerFlagSpinner: Spinner
    private lateinit var applyButton: Button

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_collection_sensor_config)

        findViewById<ImageButton>(R.id.backToCollectionFromConfigButton).setOnClickListener {
            finish()
        }
        findViewById<ImageButton>(R.id.backToCollectionFromConfigButtonSecondary).setOnClickListener {
            finish()
        }

        outputRateSpinner = findViewById(R.id.sensorConfigOutputRateSpinner)
        filterProfileSpinner = findViewById(R.id.sensorConfigFilterProfileSpinner)
        loggerFlagSpinner = findViewById(R.id.sensorConfigLoggerFlagSpinner)
        applyButton = findViewById(R.id.sensorConfigApplyButton)

        outputRateSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            outputRates.map { "${it}Hz" }
        )
        filterProfileSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            listOf("General", "Dynamic")
        )
        loggerFlagSpinner.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_dropdown_item,
            loggerFlags.map(::loggerFlagLabel)
        )

        val initOutputRate = intent.getIntExtra(EXTRA_OUTPUT_RATE, 60)
        val initFilterProfile = intent.getIntExtra(EXTRA_FILTER_PROFILE, XsensDotManager.FILTER_PROFILE_GENERAL)
        val initLoggerFlag = intent.getIntExtra(EXTRA_LOGGER_FLAG, XsensDotManager.LOGGER_FLAG_DEFAULT)

        outputRateSpinner.setSelection(outputRates.indexOf(initOutputRate).let { if (it < 0) outputRates.indexOf(60) else it })
        filterProfileSpinner.setSelection(filterProfiles.indexOf(initFilterProfile).coerceAtLeast(0))
        loggerFlagSpinner.setSelection(loggerFlags.indexOf(initLoggerFlag).let { if (it < 0) 0 else it })

        applyButton.setOnClickListener {
            val outputRate = outputRates[outputRateSpinner.selectedItemPosition.coerceIn(outputRates.indices)]
            val filterProfile = filterProfiles[filterProfileSpinner.selectedItemPosition.coerceIn(filterProfiles.indices)]
            val loggerFlag = loggerFlags[loggerFlagSpinner.selectedItemPosition.coerceIn(loggerFlags.indices)]

            val data = Intent().apply {
                putExtra(EXTRA_OUTPUT_RATE, outputRate)
                putExtra(EXTRA_FILTER_PROFILE, filterProfile)
                putExtra(EXTRA_LOGGER_FLAG, loggerFlag)
            }
            setResult(Activity.RESULT_OK, data)
            finish()
        }
    }

    private fun loggerFlagLabel(flag: Int): String {
        return collectionLoggerFlagLabel(flag)
    }
}
