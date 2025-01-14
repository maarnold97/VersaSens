package com.example.simple_ble

import android.app.Application
import android.bluetooth.*
import android.bluetooth.le.*
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.lifecycle.AndroidViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import java.util.*

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch

private val mainScope = CoroutineScope(Dispatchers.Main)

/**
 * Data class to represent a scanned BLE device along with its RSSI.
 */
data class ScannedDevice(
    val device: BluetoothDevice,
    val rssi: Int
)

/**
 * ViewModel to manage BLE operations.
 */
class BleViewModel(application: Application) : AndroidViewModel(application) {
    private val TAG = "BLEViewModel"

    // UUID Constants
    private val CONSOLE_SERVICE_UUID: UUID = UUID.fromString("E11D2E00-04AB-4DA5-B66A-EECB738F90F3")
    private val READ_CHAR_UUID: UUID = UUID.fromString("E11D2E01-04AB-4DA5-B66A-EECB738F90F3")

    // BLE Components
    private var bluetoothGatt: BluetoothGatt? = null
    private val handler = Handler(Looper.getMainLooper())

    // State Management
    private val _receivedData = MutableStateFlow<String?>(null)
    val receivedData: StateFlow<String?> = _receivedData

    private val _scanResults = MutableStateFlow<List<ScannedDevice>>(emptyList())
    val scanResults: StateFlow<List<ScannedDevice>> = _scanResults

    private val _isScanning = MutableStateFlow(false)
    val isScanning: StateFlow<Boolean> = _isScanning

    private val _isConnected = MutableStateFlow(false)
    val isConnected: StateFlow<Boolean> = _isConnected

    // Bluetooth Adapter
    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val bluetoothManager = application.getSystemService(Application.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothManager.adapter
    }

    /**
     * Start scanning for BLE devices.
     */
    fun startScan() {
        if (_isScanning.value) return

        bluetoothAdapter?.bluetoothLeScanner?.let { scanner ->
            // Clear previous scan results
            _scanResults.value = emptyList()

            // Define scan settings
            val scanSettings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_POWER)
                .build()

            // Define scan callback
            val scanCallback = object : ScanCallback() {
                override fun onScanResult(callbackType: Int, result: ScanResult?) {
                    result?.let { scanResult ->
                        val device = scanResult.device
                        val rssi = scanResult.rssi
                        val scannedDevice = ScannedDevice(device, rssi)

                        // Avoid duplicates
                        if (_scanResults.value.none { it.device.address == device.address }) {
                            _scanResults.value = _scanResults.value + scannedDevice
                        }
                    }
                }

                override fun onBatchScanResults(results: List<ScanResult?>?) {
                    results?.forEach { scanResult ->
                        scanResult?.let {
                            val device = it.device
                            val rssi = it.rssi
                            val scannedDevice = ScannedDevice(device, rssi)

                            if (_scanResults.value.none { scanned -> scanned.device.address == device.address }) {
                                _scanResults.value = _scanResults.value + scannedDevice
                            }
                        }
                    }
                }

                override fun onScanFailed(errorCode: Int) {
                    Log.e(TAG, "Scan failed with error: $errorCode")
                }
            }

            // Start scanning
            scanner.startScan(null, scanSettings, scanCallback)
            _isScanning.value = true
            Log.i(TAG, "Started scanning for BLE devices.")

            // Stop scanning after 10 seconds
            handler.postDelayed({
                scanner.stopScan(scanCallback)
                _isScanning.value = false
                Log.i(TAG, "Stopped scanning for BLE devices.")
            }, 10000)
        }
    }

    /**
     * Connect to a BLE device.
     */
    fun connectToDevice(device: BluetoothDevice) {
        if (_isConnected.value) return

        bluetoothGatt = device.connectGatt(getApplication(), false, gattCallback)
        Log.i(TAG, "Connecting to device: ${device.address}")
    }

    /**
     * Disconnect and clean up GATT connection.
     */
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        _isConnected.value = false
        _receivedData.value = null
        Log.i(TAG, "Disconnected from device.")
    }

    /**
     * GATT Callback to handle Bluetooth GATT events.
     */
    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.i(TAG, "Connected to GATT server.")
                    _isConnected.value = true
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.i(TAG, "Disconnected from GATT server.")
                    _isConnected.value = false
                    _receivedData.value = null
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(CONSOLE_SERVICE_UUID)
                val characteristic = service?.getCharacteristic(READ_CHAR_UUID)
                if (characteristic != null) {
                    enableNotifications(gatt, characteristic)
                }
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            val packet = characteristic.value
            Log.d("BleViewModel", "Received packet: ${packet.joinToString("") { "%02x".format(it) }}")


            try {
                // Pass the application context to the DataDecoder
                val (nPeaks, decodedFloats) = DataDecoder.processPacket(getApplication(), packet)
                //decodedFloats?.let { floats ->
                //    mainScope.launch {
                        // Combine new floats with existing data for continuous updates
                //        val currentData = _receivedData.value.orEmpty()
                //        val newData = (currentData.split(", ").mapNotNull { it.toFloatOrNull() } + floats)
                //            .takeLast(200) // Keep only the last 200 points
                //            .joinToString(", ") { "%.2f".format(it) }
                //        _receivedData.value = newData
                //    }
                //}

                    // Update UI for loading state or number of coughs
                if (nPeaks == null && decodedFloats == null) {
                    mainScope.launch {
                        _receivedData.value = "Loading data from sensor"
                    }
                } else if (nPeaks != null) {
                    mainScope.launch {
                        _receivedData.value = "Number of Coughs: $nPeaks"
                    }
                }

                // Update with decoded float data if available
                decodedFloats?.let { floats ->
                    val displayData = floats.joinToString(", ") { "%.2f".format(it) }
                    mainScope.launch {
                        _receivedData.value = displayData
                    }
                }
            } catch (e: Exception) {
                Log.e("BleViewModel", "Error processing packet: ${e.message}")
                mainScope.launch {
                    _receivedData.value = "Error processing packet: ${e.message}"
                }
            }
        }







        override fun onCharacteristicRead(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val data = characteristic.value
                val dataStr = data?.let { String(it) } ?: "No Data"
                _receivedData.value = dataStr
            }
        }
    }

    /**
     * Enable notifications for a characteristic.
     */
    /**private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
        val cccdUuid = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        val descriptor = characteristic.getDescriptor(cccdUuid)
        if (descriptor != null) {
            gatt.setCharacteristicNotification(characteristic, true)
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(descriptor)
        }
    }**/
    private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
        val cccdUuid = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        val descriptor = characteristic.getDescriptor(cccdUuid)
        if (descriptor != null) {
            gatt.setCharacteristicNotification(characteristic, true)
            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(descriptor)
            Log.d(TAG, "Notifications enabled for ${characteristic.uuid}")
        } else {
            Log.w(TAG, "Descriptor not found for characteristic ${characteristic.uuid}")
        }
    }

}
