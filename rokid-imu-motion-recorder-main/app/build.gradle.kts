plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("kotlin-kapt") // 必须保留（Room 注解处理器依赖）
    id("androidx.room") version "2.7.0" // 插件版本与依赖一致
}

android {
    namespace = "com.imu.motionrecorder"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.imu.motionrecorder"
        minSdk = 32
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        // 关键：所有 Room 配置都用注解处理器参数（兼容所有版本）
        javaCompileOptions {
            annotationProcessorOptions {
//                // 1. 指定 Schema 输出路径（核心必配）
//                argument("room.schemaLocation", "${buildDir}/room/schemas")
//                // 2. 启用 Schema 导出（对应 @Database(exportSchema = true)）
//                argument("room.exportSchema", "true")
//                // 3. 启用增量编译（默认已启用，显式写更稳妥）
//                argument("room.incremental", "true")
//                // 开启 Kapt 详细日志（输出具体注解错误）
//                argument("kapt.verbose", "true")
            }
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
    viewBinding {
        enable = true
    }

    // 【精简 Room DSL】仅保留 schemaDirectory（若仍报错，直接删除整个 room 块）
    room {
        // 仅保留路径配置（用字符串写法，避免参数类型问题）
        schemaDirectory(layout.buildDirectory.dir("room/schemas").map { it.asFile.path })
    }
}

// 新增：Kapt 任务配置，输出详细日志
tasks.withType<org.jetbrains.kotlin.gradle.internal.KaptWithoutKotlincTask> {
//    logging.level = org.gradle.api.logging.LogLevel.INFO
    doFirst {
        println("=== Kapt 注解处理开始，详细日志如下 ===")
    }
}
dependencies {
    // 基础依赖（不变）
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")

    // Room 依赖（版本严格一致）
    implementation("androidx.room:room-runtime:2.7.0")
    implementation(libs.androidx.fragment)
    kapt("androidx.room:room-compiler:2.7.0") // Kotlin 项目必须用 kapt 引入注解处理器
    implementation("androidx.room:room-ktx:2.7.0")

    // 协程依赖（Dao 用 suspend/Flow 必需）
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3")

    // 图表库（不变）
    implementation("com.github.PhilJay:MPAndroidChart:v3.1.0")

    // 新增：AndroidX GridLayout 依赖（解决 ClassNotFoundException）
    implementation("androidx.gridlayout:gridlayout:1.0.0")
    // CXR-S SDK（眼镜端 service bridge）
    implementation("com.rokid.cxr:cxr-service-bridge:1.0-20250519.061355-45")

    // 测试依赖（不变）
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}
