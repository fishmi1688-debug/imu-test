plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

// Keep build outputs on /tmp to avoid intermittent I/O errors on mounted workspace disks.
layout.buildDirectory.set(file("/tmp/pc-test-mobile-app-build"))

android {
    namespace = "com.example.mobile"
    compileSdk {
        version = release(36)
    }

    defaultConfig {
        applicationId = "com.example.mobile"
        minSdk = 31
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }
    packaging {
        resources {
            excludes += "META-INF/DEPENDENCIES"
        }
    }
}

dependencies {
    implementation(fileTree(mapOf("dir" to "libs", "include" to listOf("*.jar"))))
    implementation(files("libs/XsensDotSdkCore2.aar"))

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.material)
    implementation(libs.androidx.activity)
    implementation(libs.androidx.constraintlayout)
    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)

    implementation ("com.squareup.retrofit2:retrofit:2.9.0")
    implementation ("com.squareup.retrofit2:converter-gson:2.9.0")
    implementation ("com.squareup.okhttp3:okhttp:4.9.3")
    implementation ("org.jetbrains.kotlin:kotlin-stdlib:2.1.0")
    implementation ("com.squareup.okio:okio:2.8.0")
    implementation ("com.google.code.gson:gson:2.10.1")
    implementation ("com.squareup.okhttp3:logging-interceptor:4.9.1")
    implementation("com.github.PhilJay:MPAndroidChart:3.1.0")
    implementation("org.apache.commons:commons-numbers-quaternion:1.2")
    implementation("org.apache.commons:commons-math3:3.6.1")

    implementation("com.rokid.cxr:client-l:1.0.4")

    implementation("com.openai:openai-java:3.5.0")
}
