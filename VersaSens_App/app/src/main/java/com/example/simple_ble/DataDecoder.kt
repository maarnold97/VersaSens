package com.example.simple_ble

import android.util.Log
import android.widget.Toast
import android.content.Context
import java.nio.ByteBuffer
import java.nio.ByteOrder

object DataDecoder {
    private val fullDataBuffer = mutableListOf<Byte>() // To store all received bytes
    private var totalExpectedSize: Int = 0            // Total expected size in bytes
    private var isDataLoading = true // True until the first `AAAA` packet is received
    private var currentPeaks: Int? = null // Holds the current `n_peaks` value for the data window

    /**
     * Processes incoming BLE data, combining packets if necessary and decoding floats.
     *
     * @param context The application context needed for certain operations.
     * @param packet The received BLE packet as a byte array.
     * @return A pair containing the number of peaks and a list of decoded floats if complete; null otherwise.
     */
    fun processPacket(context: Context, packet: ByteArray): Pair<Int?, List<Float>?> {
        if (packet.size < 6) throw IllegalArgumentException("Packet too short to process.")

        // Check header
        val header = packet.slice(0..1).toByteArray()
        when {
            header.contentEquals(byteArrayOf(0xAA.toByte(), 0xAA.toByte())) -> {
                // First packet: Extract `n_peaks` and total size
                currentPeaks = packet[2].toInt()
                totalExpectedSize = ByteBuffer.wrap(packet.slice(3..4).toByteArray())
                    .order(ByteOrder.LITTLE_ENDIAN)
                    .short
                    .toInt()
                fullDataBuffer.clear() // Reset buffer for new data
                fullDataBuffer.addAll(packet.slice(5 until packet.size))
                isDataLoading = false // First packet received, no longer loading
                Log.d("DataDecoder", "First packet: n_peaks=$currentPeaks, totalExpectedSize=$totalExpectedSize")
                return currentPeaks to null
            }
            header.contentEquals(byteArrayOf(0xBB.toByte(), 0xBB.toByte())) -> {
                // Subsequent packet
                if (isDataLoading) {
                    Log.d("DataDecoder", "Still loading, ignoring packet")
                    return null to null // Ignore packets until the first `AAAA`
                }
                if (totalExpectedSize == 0) throw IllegalStateException("Received data out of sequence.")
                fullDataBuffer.addAll(packet.slice(4 until packet.size)) // Adjust for BB BB header
            }
            else -> {
                throw IllegalArgumentException("Invalid packet header: ${header.joinToString { "%02x".format(it) }}")
            }
        }

        // Check if we have received all data
        if (fullDataBuffer.size >= totalExpectedSize) {
            val fullData = fullDataBuffer.toByteArray()
            fullDataBuffer.clear() // Reset buffer after processing
            val decodedFloats = decodeFloats(fullData)

            // Reset for next window
            isDataLoading = true
            currentPeaks = null

            // Send to Firebase
            sendToFirebase(context, decodedFloats)

            return null to decodedFloats
        }

        return currentPeaks to null // Return `n_peaks` while still loading data
    }

    /**
     * Sends decoded data to Firebase and shows a Toast message for debugging.
     *
     * @param context The application context for Toast.
     * @param floats The list of decoded floats to send.
     */
    private fun sendToFirebase(context: Context, floats: List<Float>) {
        try {
            val currentThread = Thread.currentThread().name

            // Debug output
            Toast.makeText(context, "sendToFirebase Called", Toast.LENGTH_SHORT).show()
            Log.d("DataDecoder", "sendToFirebase: Preparing to send floats to Firebase: $floats on thread $currentThread")

            val dataMap = mapOf(
                "timestamp" to System.currentTimeMillis(),
                "data" to floats
            )

            MainActivity.firebaseDatabase.push().setValue(dataMap)
                .addOnSuccessListener {
                    Log.d("DFirebaseSuccess", "Data successfully sent to Firebase: $dataMap")
                }
                .addOnFailureListener { e ->
                    Log.e("DFirebaseError", "Failed to send data to Firebase. Error: ${e.message}", e)
                }
        } catch (e: Exception) {
            Log.e("DataDecoderException", "Exception in sendToFirebase: ${e.message}", e)
        }
    }

    /**
     * Decodes a byte array into a list of floats.
     *
     * @param dataBytes The byte array containing the float data.
     * @return A list of decoded float values.
     */
    private fun decodeFloats(dataBytes: ByteArray): List<Float> {
        if (dataBytes.size % 4 != 0) throw IllegalArgumentException("Data size is not a multiple of 4.")

        val floats = mutableListOf<Float>()
        val buffer = ByteBuffer.wrap(dataBytes).order(ByteOrder.LITTLE_ENDIAN)

        while (buffer.remaining() >= 4) {
            floats.add(buffer.float)
        }

        return floats
    }
}
