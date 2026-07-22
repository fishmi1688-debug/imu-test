pluginManagement {
    repositories {
        gradlePluginPortal()
        google()
        mavenCentral()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        // 关键：添加 JitPack 仓库（MPAndroidChart 托管在此）
        maven { url = uri("https://jitpack.io") }
        maven { url = uri("https://maven.rokid.com/repository/maven-public/") }
    }
}
rootProject.name = "IMUMotionRecorder"
include(":app")