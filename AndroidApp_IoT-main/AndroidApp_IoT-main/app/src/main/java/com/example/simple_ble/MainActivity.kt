// MainActivity.kt
package com.example.simple_ble

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import com.example.simple_ble.ui.theme.Simple_bleTheme
import com.google.firebase.FirebaseApp
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase

class MainActivity : ComponentActivity() {

    private val bleViewModel: BleViewModel by viewModels()
    companion object {
        lateinit var firebaseDatabase: DatabaseReference
    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Initialize Firebase Database
        //firebaseDatabase.push().setValue("Test Data")
        Log.d("LogTest", "This is a test log message.")

        FirebaseApp.initializeApp(this)
        Log.d("FirebaseInitCheck", "Firebase manually initialized.")
        // Initialize Firebase
        firebaseDatabase = FirebaseDatabase.getInstance().getReference("decoded_floats")

        Log.w("Firebase", "Firebase Database Initialized: $firebaseDatabase")

        try {
            val testDatabase = FirebaseDatabase.getInstance().getReference("test_node")
            testDatabase.setValue("Hello, Firebase!")
                .addOnSuccessListener {
                    Log.d("FirebaseInitTest", "Firebase successfully initialized and data written.")
                }
                .addOnFailureListener { e ->
                    Log.e("FirebaseInitTest", "Failed to write data to Firebase.", e)
                }
        } catch (e: Exception) {
            Log.e("FirebaseInitTest", "Firebase initialization failed.", e)
        }
        // Test a simple write operation
        firebaseDatabase.push().setValue(mapOf("test" to "value"))
            .addOnSuccessListener {
                Log.w("FirebaseTest", "Simple write succeeded.")
            }
            .addOnFailureListener { e ->
                Log.w("FirebaseTest", "Simple write failed.", e)
            }



        ViewModelSingleton.bleViewModel = bleViewModel

        setContent {
            Simple_bleTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    BLEApp(bleViewModel)
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        bleViewModel.disconnect()
    }
}
