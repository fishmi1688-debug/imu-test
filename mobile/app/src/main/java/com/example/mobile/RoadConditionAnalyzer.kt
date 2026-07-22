package com.example.mobile

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Base64
import android.util.Log
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.IOException
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

class RoadConditionAnalyzer(context: Context) {
    data class ApiConfig(
        val apiKey: String,
        val model: String,
        val baseUrl: String
    )

    companion object {
        private const val TAG = "RoadConditionAnalyzer"
        private const val PREFS_NAME = "road_condition_api_config"
        private const val PREF_API_KEY = "api_key"
        private const val PREF_MODEL = "model"
        private const val PREF_BASE_URL = "base_url"
        private const val DEFAULT_API_KEY = "sk-d2dbb798479e4e4dbc07a7e8771761fd"
        private const val DEFAULT_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
        private const val DEFAULT_MODEL = "qwen3-vl-flash"
        private const val MAX_TOKENS = 32
        private const val MAX_IMAGE_WIDTH = 640
        private const val MAX_IMAGE_HEIGHT = 480
        private const val JPEG_QUALITY = 85
        private const val SYSTEM_PROMPT =
            "你是路况分类器。只能输出固定标签，不要解释，不要扩展。"
        private const val USER_PROMPT =
            "请识别图片中的路况。" +
                "地形标签只能选一个：平地、上楼梯、下楼梯、上坡、下坡。" +
                "如果前方存在明显障碍物，再额外输出“前方有障碍物”。" +
                "只输出标签，多个标签用中文逗号分隔；不要输出其他文字。"
        private const val TIMEOUT_SECONDS = 30L
    }

    private val client = OkHttpClient.Builder()
        .connectTimeout(TIMEOUT_SECONDS, java.util.concurrent.TimeUnit.SECONDS)
        .readTimeout(TIMEOUT_SECONDS, java.util.concurrent.TimeUnit.SECONDS)
        .writeTimeout(TIMEOUT_SECONDS, java.util.concurrent.TimeUnit.SECONDS)
        .build()
    private val appContext = context.applicationContext
    private val prefs = appContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    fun getApiConfig(): ApiConfig {
        val apiKey = prefs.getString(PREF_API_KEY, DEFAULT_API_KEY)
            ?.trim()
            .orEmpty()
            .ifBlank { DEFAULT_API_KEY }
        val model = prefs.getString(PREF_MODEL, DEFAULT_MODEL)?.trim().orEmpty().ifBlank { DEFAULT_MODEL }
        val baseUrl = prefs.getString(PREF_BASE_URL, DEFAULT_BASE_URL)
            ?.trim()
            .orEmpty()
            .ifBlank { DEFAULT_BASE_URL }
        return ApiConfig(apiKey = apiKey, model = model, baseUrl = baseUrl)
    }

    fun updateApiConfig(apiKey: String, model: String, baseUrl: String) {
        val normalizedApiKey = apiKey.trim()
        val normalizedModel = model.trim().ifBlank { DEFAULT_MODEL }
        val normalizedBaseUrl = baseUrl.trim().ifBlank { DEFAULT_BASE_URL }.trimEnd('/')
        prefs.edit()
            .putString(PREF_API_KEY, normalizedApiKey)
            .putString(PREF_MODEL, normalizedModel)
            .putString(PREF_BASE_URL, normalizedBaseUrl)
            .apply()
    }

    /**
     * 分析图片中的路况
     * @param imageBytes 图片字节数组
     * @param callback 分析结果回调，参数为路况描述字符串或null（失败时）
     */
    fun analyzeRoadCondition(imageBytes: ByteArray, callback: (String?) -> Unit) {
        Thread {
            try {
                val apiConfig = getApiConfig()
                if (apiConfig.apiKey.isBlank()) {
                    Log.e(TAG, "API key is empty, abort road condition analysis")
                    callback(null)
                    return@Thread
                }

                val preparedImage = prepareImageBytes(imageBytes)
                if (preparedImage == null) {
                    Log.e(TAG, "Failed to prepare image bytes")
                    callback(null)
                    return@Thread
                }

                // 将图片转换为Base64 Data URL
                val base64Image = Base64.encodeToString(preparedImage, Base64.NO_WRAP)
                val dataUrl = "data:image/jpeg;base64,$base64Image"

                val messages = JSONArray().apply {
                    put(
                        JSONObject().apply {
                            put("role", "system")
                            put("content", SYSTEM_PROMPT)
                        }
                    )

                    put(
                        JSONObject().apply {
                            put("role", "user")
                            put(
                                "content",
                                JSONArray().apply {
                                    put(
                                        JSONObject().apply {
                                            put("type", "image_url")
                                            put(
                                                "image_url",
                                                JSONObject().apply {
                                                    put("url", dataUrl)
                                                }
                                            )
                                        }
                                    )
                                    put(
                                        JSONObject().apply {
                                            put("type", "text")
                                            put("text", USER_PROMPT)
                                        }
                                    )
                                }
                            )
                        }
                    )
                }

                // 构建请求JSON
                val requestJson = JSONObject().apply {
                    put("model", apiConfig.model)
                    put("max_tokens", MAX_TOKENS)
                    put("messages", messages)
                }

                // 创建请求
                val requestBody = requestJson.toString()
                    .toRequestBody("application/json; charset=utf-8".toMediaType())

                val request = Request.Builder()
                    .url(buildChatCompletionsUrl(apiConfig.baseUrl))
                    .addHeader("Authorization", "Bearer ${apiConfig.apiKey}")
                    .addHeader("Content-Type", "application/json")
                    .post(requestBody)
                    .build()

                // 发送请求
                client.newCall(request).execute().use { response ->
                    if (!response.isSuccessful) {
                        Log.e(TAG, "API request failed: ${response.code} ${response.message}")
                        callback(null)
                        return@Thread
                    }

                    val responseBody = response.body?.string()
                    if (responseBody == null) {
                        Log.e(TAG, "Response body is null")
                        callback(null)
                        return@Thread
                    }

                    // 解析响应
                    val jsonResponse = JSONObject(responseBody)
                    val choices = jsonResponse.optJSONArray("choices")
                    if (choices == null || choices.length() == 0) {
                        Log.e(TAG, "No choices in response")
                        callback(null)
                        return@Thread
                    }

                    val message = choices.getJSONObject(0).optJSONObject("message")
                    val content = extractMessageContent(message)?.trim()
                    val normalizedContent = normalizeRoadCondition(content)
                    
                    if (normalizedContent == null) {
                        Log.e(TAG, "Unable to normalize response content: $content")
                        callback(null)
                    } else {
                        Log.i(TAG, "Road condition analysis success: raw=$content normalized=$normalizedContent")
                        callback(normalizedContent)
                    }
                }
            } catch (e: IOException) {
                Log.e(TAG, "Network error during analysis", e)
                callback(null)
            } catch (e: Exception) {
                Log.e(TAG, "Error analyzing road condition", e)
                callback(null)
            }
        }.start()
    }

    /**
     * 分析Bitmap图片中的路况
     * @param bitmap 位图对象
     * @param callback 分析结果回调，参数为路况描述字符串或null（失败时）
     */
    fun analyzeRoadCondition(bitmap: Bitmap, callback: (String?) -> Unit) {
        val outputStream = ByteArrayOutputStream()
        bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, outputStream)
        val imageBytes = outputStream.toByteArray()
        analyzeRoadCondition(imageBytes, callback)
    }

    private fun prepareImageBytes(imageBytes: ByteArray): ByteArray? {
        val sourceBitmap = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size) ?: return null
        val targetSize = constrainImageSize(sourceBitmap.width, sourceBitmap.height)
        val scaledBitmap = if (sourceBitmap.width == targetSize.first && sourceBitmap.height == targetSize.second) {
            sourceBitmap
        } else {
            Bitmap.createScaledBitmap(sourceBitmap, targetSize.first, targetSize.second, true)
        }
        val outputStream = ByteArrayOutputStream()
        scaledBitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, outputStream)
        if (scaledBitmap !== sourceBitmap) {
            sourceBitmap.recycle()
            scaledBitmap.recycle()
        } else {
            sourceBitmap.recycle()
        }
        return outputStream.toByteArray()
    }

    private fun extractMessageContent(message: JSONObject?): String? {
        if (message == null) return null
        val content = message.opt("content") ?: return null
        if (content is String) return content
        if (content is JSONArray) {
            val parts = mutableListOf<String>()
            for (i in 0 until content.length()) {
                val part = content.optJSONObject(i) ?: continue
                val text = part.optString("text")
                if (text.isNotBlank()) {
                    parts.add(text)
                }
            }
            if (parts.isNotEmpty()) return parts.joinToString("")
        }
        return null
    }

    private fun constrainImageSize(width: Int, height: Int): Pair<Int, Int> {
        val widthScale = MAX_IMAGE_WIDTH.toFloat() / width.toFloat()
        val heightScale = MAX_IMAGE_HEIGHT.toFloat() / height.toFloat()
        val scale = min(1f, min(widthScale, heightScale))
        val targetWidth = max(1, (width * scale).roundToInt())
        val targetHeight = max(1, (height * scale).roundToInt())
        return targetWidth to targetHeight
    }

    private fun buildChatCompletionsUrl(baseUrl: String): String {
        return "${baseUrl.trim().trimEnd('/')}/chat/completions"
    }

    private fun normalizeRoadCondition(raw: String?): String? {
        if (raw.isNullOrBlank()) {
            return null
        }

        val text = raw
            .replace("\n", "")
            .replace(" ", "")
            .replace("：", ":")

        val terrain = when {
            text.contains("上楼梯") || text.contains("上楼") || text.contains("上台阶") -> "上楼梯"
            text.contains("下楼梯") || text.contains("下楼") || text.contains("下台阶") -> "下楼梯"
            text.contains("上坡") || text.contains("上斜坡") -> "上坡"
            text.contains("下坡") || text.contains("下斜坡") -> "下坡"
            text.contains("平地") || text.contains("平路") || text.contains("路面平坦") || text.contains("地面平坦") -> "平地"
            else -> null
        }

        if (terrain == null) {
            return null
        }

        val hasObstacle = text.contains("障碍")
        return if (hasObstacle) {
            "$terrain，前方有障碍物"
        } else {
            terrain
        }
    }
}
