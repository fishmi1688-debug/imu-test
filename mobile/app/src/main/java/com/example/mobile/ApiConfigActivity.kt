package com.example.mobile

import android.os.Bundle
import android.widget.Button
import android.widget.EditText
import android.widget.ImageButton
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity

class ApiConfigActivity : AppCompatActivity() {

    private lateinit var statusView: TextView
    private lateinit var baseUrlInput: EditText
    private lateinit var apiKeyInput: EditText
    private lateinit var apiModelInput: EditText
    private lateinit var saveButton: Button

    private lateinit var roadConditionAnalyzer: RoadConditionAnalyzer

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_api_config)

        statusView = findViewById(R.id.apiConfigStatusView)
        baseUrlInput = findViewById(R.id.baseUrlInput)
        apiKeyInput = findViewById(R.id.apiKeyInput)
        apiModelInput = findViewById(R.id.apiModelInput)
        saveButton = findViewById(R.id.saveApiConfigButton)

        findViewById<ImageButton>(R.id.backToMainButton).setOnClickListener {
            ExoBottomNav.openHome(this)
        }

        roadConditionAnalyzer = RoadConditionAnalyzer(applicationContext)
        ExoBottomNav.setup(this, ExoDestination.API)

        loadApiConfig()

        saveButton.setOnClickListener {
            val baseUrl = baseUrlInput.text?.toString()?.trim().orEmpty()
            val apiKey = apiKeyInput.text?.toString()?.trim().orEmpty()
            val model = apiModelInput.text?.toString()?.trim().orEmpty()
            if (model.isBlank()) {
                apiModelInput.error = getString(R.string.api_config_model_required)
                return@setOnClickListener
            }
            roadConditionAnalyzer.updateApiConfig(
                apiKey = apiKey,
                model = model,
                baseUrl = baseUrl,
            )
            val savedConfig = roadConditionAnalyzer.getApiConfig()
            updateStatus(
                baseUrl = savedConfig.baseUrl,
                model = savedConfig.model,
                apiKey = savedConfig.apiKey,
            )
            Toast.makeText(this, getString(R.string.api_config_saved), Toast.LENGTH_SHORT).show()
        }
    }

    private fun loadApiConfig() {
        val currentConfig = roadConditionAnalyzer.getApiConfig()
        baseUrlInput.setText(currentConfig.baseUrl)
        apiKeyInput.setText(currentConfig.apiKey)
        apiModelInput.setText(currentConfig.model)
        updateStatus(
            baseUrl = currentConfig.baseUrl,
            model = currentConfig.model,
            apiKey = currentConfig.apiKey,
        )
    }

    private fun updateStatus(baseUrl: String, model: String, apiKey: String) {
        statusView.text = getString(
            R.string.api_config_status_value,
            baseUrl,
            model,
            maskApiKey(apiKey),
        )
    }

    private fun maskApiKey(apiKey: String): String {
        if (apiKey.isBlank()) {
            return "(empty)"
        }
        return if (apiKey.length <= 8) {
            "****"
        } else {
            "${apiKey.take(4)}****${apiKey.takeLast(4)}"
        }
    }
}
