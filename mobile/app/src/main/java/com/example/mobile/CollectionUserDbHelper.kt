package com.example.mobile

import android.content.ContentValues
import android.content.Context
import android.database.sqlite.SQLiteDatabase
import android.database.sqlite.SQLiteOpenHelper
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

data class CollectionUser(
    val id: Long,
    val name: String,
    val gender: String,
    val birthYear: Int,
    val birthMonth: Int,
    val height: Int,
    val weight: Int,
    val createdAt: String,
    val updatedAt: String,
)

class CollectionUserDbHelper(context: Context) :
    SQLiteOpenHelper(context, DATABASE_NAME, null, DATABASE_VERSION) {

    companion object {
        private const val DATABASE_NAME = "collection_users.db"
        private const val DATABASE_VERSION = 1
        private const val TABLE_USERS = "users_table"
    }

    override fun onCreate(db: SQLiteDatabase) {
        db.execSQL(
            """
            CREATE TABLE IF NOT EXISTS $TABLE_USERS (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL UNIQUE,
                birthYear INTEGER NOT NULL,
                birthMonth INTEGER NOT NULL,
                gender TEXT NOT NULL,
                height INTEGER NOT NULL,
                weight INTEGER NOT NULL,
                createdAt TEXT NOT NULL DEFAULT (datetime('now','localtime')),
                updatedAt TEXT NOT NULL DEFAULT (datetime('now','localtime'))
            )
            """.trimIndent()
        )
    }

    override fun onUpgrade(db: SQLiteDatabase, oldVersion: Int, newVersion: Int) = Unit

    fun insertUser(
        name: String,
        gender: String,
        birthYear: Int,
        birthMonth: Int,
        height: Int,
        weight: Int,
    ): Long {
        val values = ContentValues().apply {
            put("name", name)
            put("gender", gender)
            put("birthYear", birthYear)
            put("birthMonth", birthMonth)
            put("height", height)
            put("weight", weight)
        }
        return writableDatabase.insertOrThrow(TABLE_USERS, null, values)
    }

    fun updateUser(user: CollectionUser): Int {
        val now = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()).format(Date())
        val values = ContentValues().apply {
            put("name", user.name)
            put("gender", user.gender)
            put("birthYear", user.birthYear)
            put("birthMonth", user.birthMonth)
            put("height", user.height)
            put("weight", user.weight)
            put("updatedAt", now)
        }
        return writableDatabase.update(TABLE_USERS, values, "id=?", arrayOf(user.id.toString()))
    }

    fun deleteUser(id: Long): Int {
        return writableDatabase.delete(TABLE_USERS, "id=?", arrayOf(id.toString()))
    }

    fun getUserById(id: Long): CollectionUser? {
        readableDatabase.query(
            TABLE_USERS,
            null,
            "id=?",
            arrayOf(id.toString()),
            null,
            null,
            null,
        ).use { cursor ->
            if (!cursor.moveToFirst()) {
                return null
            }
            return cursor.toUser()
        }
    }

    fun getAllUsers(): List<CollectionUser> {
        val users = mutableListOf<CollectionUser>()
        readableDatabase.query(
            TABLE_USERS,
            null,
            null,
            null,
            null,
            null,
            "updatedAt DESC",
        ).use { cursor ->
            while (cursor.moveToNext()) {
                users.add(cursor.toUser())
            }
        }
        return users
    }

    private fun android.database.Cursor.toUser(): CollectionUser {
        fun str(name: String) = getString(getColumnIndexOrThrow(name))
        fun i(name: String) = getInt(getColumnIndexOrThrow(name))
        fun l(name: String) = getLong(getColumnIndexOrThrow(name))

        return CollectionUser(
            id = l("id"),
            name = str("name"),
            gender = str("gender"),
            birthYear = i("birthYear"),
            birthMonth = i("birthMonth"),
            height = i("height"),
            weight = i("weight"),
            createdAt = str("createdAt"),
            updatedAt = str("updatedAt"),
        )
    }
}
