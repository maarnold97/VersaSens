package com.example.simple_ble

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.example.simple_ble.ui.theme.Simple_bleTheme
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.Description
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet

class DataDisplayActivity : ComponentActivity() {

    private val bleViewModel: BleViewModel
        get() = ViewModelSingleton.bleViewModel
            ?: throw IllegalStateException("BleViewModel is not initialized")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContent {
            Simple_bleTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    val receivedData by bleViewModel.receivedData.collectAsState(initial = "No data received")
                    var numberOfCoughs by remember { mutableStateOf(0) }
                    var floatData by remember { mutableStateOf(emptyList<Float>()) }

                    // Parse received data into floats and number of coughs
                    LaunchedEffect(receivedData) {
                        receivedData?.let {
                            when {
                                it.startsWith("Number of Coughs:") -> {
                                    numberOfCoughs = it.substringAfter("Number of Coughs:").trim().toIntOrNull() ?: 0
                                }
                                it != "No data received" -> {
                                    floatData = it.split(", ").mapNotNull { item -> item.toFloatOrNull() }
                                    Log.d("DataDisplayActivity", "floatData updated: $floatData")
                                }
                            }
                        }
                    }


                    Column(
                        modifier = Modifier
                            .padding(16.dp)
                            .fillMaxSize()
                    ) {
                        Text(
                            text = "Received Data",
                            style = MaterialTheme.typography.headlineMedium
                        )
                        Spacer(modifier = Modifier.height(16.dp))
                        Text(
                            text = "Number of Coughs: $numberOfCoughs",
                            style = MaterialTheme.typography.bodyLarge
                        )
                        Spacer(modifier = Modifier.height(16.dp))

                        // Plot the data if available
                        if (floatData.isNotEmpty()) {
                            LineChartView(floatData)
                            Spacer(modifier = Modifier.height(16.dp))
                            Text(
                                text = "Raw Data",
                                style = MaterialTheme.typography.headlineSmall
                            )
                            Spacer(modifier = Modifier.height(8.dp))
                            LazyColumn(
                                modifier = Modifier.fillMaxSize()
                            ) {
                                items(floatData) { value ->
                                    Text(
                                        text = value.toString(),
                                        style = MaterialTheme.typography.bodyMedium,
                                        modifier = Modifier.padding(vertical = 4.dp)
                                    )
                                }
                            }
                        } else {
                            Text(
                                text = "No plot available",
                                style = MaterialTheme.typography.bodyMedium
                            )
                        }
                    }
                }
            }
        }
    }

    @Composable
    fun LineChartViewOld(data: List<Float>) {
        // Remember a mutable state for animated data
        var animatedData by remember { mutableStateOf(emptyList<Float>()) }

        // Incrementally update the animatedData
        LaunchedEffect(data) {
            animatedData = emptyList() // Reset animation on new data
            data.forEachIndexed { index, value ->
                // Gradually add points with a delay to simulate real-time plotting
                animatedData = animatedData + value
                kotlinx.coroutines.delay(20L) // Adjust delay for smoother/faster animation
            }
        }

        AndroidView(factory = { context ->
            val chart = LineChart(context)

            // Customize static axes
            chart.xAxis.apply {
                axisMinimum = 0f
                axisMaximum = 0.8f
                setDrawLabels(true)
            }
            chart.axisLeft.apply {
                axisMinimum = -2000f
                axisMaximum = 2000f
                setDrawGridLines(true)
            }
            chart.axisRight.isEnabled = false
            chart.legend.isEnabled = false

            // Chart description
            chart.description = Description().apply { text = "Time (s)" }

            // Customize animation
            chart.animateX(800) // 800ms animation for the X-axis

            chart
        }, update = { chart ->
            // Update chart data dynamically
            val entries = animatedData.mapIndexed { index, value ->
                Entry(index * 0.8f / data.size, value) // Scale x-axis to 0.8 seconds
            }

            val dataSet = LineDataSet(entries, "Audio Signal")
            dataSet.color = android.graphics.Color.BLUE
            dataSet.setCircleColor(android.graphics.Color.RED)
            dataSet.setDrawCircles(false)
            dataSet.setDrawValues(false) // Disable point values on the chart

            chart.data = LineData(dataSet)
            chart.invalidate() // Refresh chart
        }, modifier = Modifier
            .fillMaxWidth()
            .height(300.dp))
    }

    @Composable
    fun LineChartViewMeh(data: List<Float>) {
        // Check if the data list size is 400
        //if (data.size != 400) {
        //    throw IllegalArgumentException("Data size must be exactly 400. Received size: ${data.size}")
        //}
        // Remember a mutable state for animated data
        var animatedData by remember { mutableStateOf(emptyList<Float>()) }

        // Incrementally update the animatedData
        LaunchedEffect(data) {
            animatedData = emptyList() // Reset animation on new data
            data.forEachIndexed { index, value ->
                // Gradually add points with a delay to simulate real-time plotting
                animatedData = animatedData + value
                kotlinx.coroutines.delay(4L) // Adjust delay to match 1.6s animation for 400 points
            }
        }

        AndroidView(factory = { context ->
            val chart = LineChart(context)

            // Customize static axes
            chart.xAxis.apply {
                axisMinimum = 0f
                axisMaximum = 1.6f // Fixed X-axis for 1.6 seconds
                setDrawLabels(true)
                granularity = 0.2f // Label intervals every 0.2 seconds
            }
            chart.axisLeft.apply {
                axisMinimum = -8f // Fixed Y-axis
                axisMaximum = 8f
                setDrawGridLines(true) // Optional: Show grid lines
            }
            chart.axisRight.isEnabled = false // Disable the right Y-axis
            chart.legend.isEnabled = false

            // Chart description
            chart.description = Description().apply { text = "Time (s)" }

            // Customize animation
            chart.animateX(1600) // Animate X-axis over 1.6 seconds

            chart
        }, update = { chart ->
            // Update chart data dynamically
            val entries = animatedData.mapIndexed { index, value ->
                Entry(index * 1.6f / 400, value) // Scale x-axis for 400 points over 1.6 seconds
            }

            val dataSet = LineDataSet(entries, "Audio Signal")
            dataSet.color = android.graphics.Color.BLUE
            dataSet.setCircleColor(android.graphics.Color.RED)
            dataSet.setDrawCircles(false)
            dataSet.setDrawValues(false) // Disable point values on the chart

            chart.data = LineData(dataSet)
            chart.invalidate() // Refresh chart
        }, modifier = Modifier
            .fillMaxWidth()
            .height(300.dp))
    }

    @Composable
    fun LineChartView(data: List<Float>) {
        // Ensure data size is exactly 400
        //if (data.size != 400) {
        //    throw IllegalArgumentException("Data size must be exactly 400. Received size: ${data.size}")
        //}

        // Remember a mutable state for animated data
        var animatedData by remember { mutableStateOf(emptyList<Float>()) }

        // Incrementally update the animatedData
        LaunchedEffect(data) {
            animatedData = emptyList() // Reset animation on new data
            data.forEachIndexed { index, value ->
                // Gradually add points with a delay to simulate real-time plotting
                animatedData = animatedData + value
                kotlinx.coroutines.delay(1L) // Adjust delay to match 1.6s animation for 400 points
                //kotlinx.coroutines.delay()
            }
        }

        AndroidView(factory = { context ->
            val chart = LineChart(context)

            // Customize static axes
            chart.xAxis.apply {
                axisMinimum = 0f
                axisMaximum = 1.6f // Fixed X-axis for 1.6 seconds
                setDrawLabels(true)
                granularity = 0.1f // Label intervals every 0.1 seconds
            }
            chart.axisLeft.apply {
                axisMinimum = -30f // Fixed Y-axis
                axisMaximum = 30f
                setDrawGridLines(true) // Optional: Show grid lines
            }
            chart.axisRight.isEnabled = false // Disable the right Y-axis
            chart.legend.isEnabled = false

            // Chart description
            chart.description = Description().apply { text = "Time (s)" }

            // Customize animation
            chart.animateX(1600) // Animate X-axis over 1.6 seconds

            chart
        }, update = { chart ->
            // Correctly map data to the X-axis
            val entries = animatedData.mapIndexed { index, value ->
                Entry(index * (1.6f / 400), value) // Scale x-axis for 400 points over 1.6 seconds
            }

            val dataSet = LineDataSet(entries, "Audio Signal")
            dataSet.color = android.graphics.Color.BLUE
            dataSet.setCircleColor(android.graphics.Color.RED)
            dataSet.setDrawCircles(false)
            dataSet.setDrawValues(false) // Disable point values on the chart

            chart.data = LineData(dataSet)
            chart.invalidate() // Refresh chart
        }, modifier = Modifier
            .fillMaxWidth()
            .height(300.dp))
    }


}
