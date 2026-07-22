package com.example.mobile

import android.content.Intent
import android.os.Bundle
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat

class EntryActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_entry)
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.entryRoot)) { view, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            view.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }

        findViewById<com.google.android.material.button.MaterialButton>(R.id.openDataCollectionButton)
            .setOnClickListener {
                startActivity(Intent(this, DataCollectionActivity::class.java))
            }

        findViewById<com.google.android.material.button.MaterialButton>(R.id.openDataCollectionV2Button)
            .setOnClickListener {
                startActivity(
                    Intent(this, DataCollectionActivity::class.java).apply {
                        putExtra(DataCollectionActivity.EXTRA_ENABLE_LABEL_MODE, true)
                    }
                )
            }

        findViewById<com.google.android.material.button.MaterialButton>(R.id.openRokidConsoleButton)
            .setOnClickListener {
                startActivity(Intent(this, MainActivity::class.java))
            }
    }
}
