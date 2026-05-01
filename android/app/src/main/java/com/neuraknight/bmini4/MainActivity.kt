package com.leonardo.bmini4

import android.annotation.SuppressLint
import android.content.pm.ActivityInfo
import android.graphics.Color
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.VibrationEffect
import android.os.Vibrator
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.CheckBox
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import com.leonardo.bmini4.databinding.ActivityMainBinding
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.PrintWriter
import java.net.Socket
import java.util.concurrent.Executors
import kotlin.math.abs

object Debug {
    private val LEVEL = DebugLevel.NONE
    enum class DebugLevel { NONE, INFO }
    
    fun info(tag: String, msg: String) {
        if (LEVEL.ordinal >= DebugLevel.INFO.ordinal) Log.i(tag, "[INFO] $msg")
    }
}

enum class PerformanceMode {
    NORMAL, SPORT, ECO
}

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var binding: ActivityMainBinding
    private lateinit var sensorManager: SensorManager
    private lateinit var vibrator: Vibrator
    private var accelSensor: Sensor? = null
    
    private var socket: Socket? = null
    private var output: PrintWriter? = null
    private var input: BufferedReader? = null
    private val executor = Executors.newSingleThreadExecutor()
    private val handler = Handler(Looper.getMainLooper())
    private lateinit var sendRunnable: Runnable
    
    private var currentServo = SERVO_CENTER
    private var lastSentServo = SERVO_CENTER
    private var currentGas = 0
    private var lastSentGas = 0
    private var lastTelemetryTime = 0L
    private var connectionLost = false
    private var batteryLevel = 0
    private var currentGear = 0
    private var currentDirection = "N"

    // RGB state
    private var currentRgbColor = 0xFFFFFF
    private var currentAnimation = "off"
    private var currentBrightness = 255
    private var currentSpeed = 50

    // Auto indicators
    private var autoIndicatorsEnabled = true
    private var currentAutoIndicator = "off"
    
    // Double-tap detection
    private var lastHeadlightTap = 0L
    
    // Sound & status
    private var soundEnabled = true
    private var statusIndicatorEnabled = true
    
    // Performance mode
    private var currentPerformanceMode = PerformanceMode.NORMAL

    companion object {
        private const val TAG = "BMini4"
        private const val SERVO_MIN = 45
        private const val SERVO_MAX = 135
        private const val SERVO_CENTER = (SERVO_MIN + SERVO_MAX) / 2
        private const val STEERING_THRESHOLD = 6
        private const val AUTO_IND_THRESHOLD = 15
        private const val THROTTLE_LEVELS = 5
        
        private const val SERVER_IP = "192.168.4.1"
        private const val SERVER_PORT = 80
        private const val CONNECTION_TIMEOUT_MS = 9000L
        private const val SEND_INTERVAL_MS = 67L
        private const val DOUBLE_TAP_MS = 400L
        
        private const val VIBRATE_SHORT = 30L
        private const val VIBRATE_LONG = 50L
    }

    private data class RgbMode(val name: String, val displayName: String)
    
    private val rgbModes = listOf(
        RgbMode("off", "Off"),
        RgbMode("solid", "Solid"),
        RgbMode("breathe", "Breathe"),
        RgbMode("blink", "Blink"),
        RgbMode("strobe", "Strobe"),
        RgbMode("pulse", "Pulse"),
        RgbMode("rainbow", "Rainbow"),
        RgbMode("fire", "Fire"),
        RgbMode("sparkle", "Sparkle"),
        RgbMode("comet", "Comet"),
        RgbMode("wave", "Wave"),
        RgbMode("police", "Police")
    )

    private data class ColorPreset(val name: String, val color: Int)
    
    private val colorPresets = listOf(
        ColorPreset("Red", 0xFF0000),
        ColorPreset("Green", 0x00FF00),
        ColorPreset("Blue", 0x0000FF),
        ColorPreset("Yellow", 0xFFFF00),
        ColorPreset("White", 0xFFFFFF)
    )

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE
        
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, binding.root).apply {
            hide(WindowInsetsCompat.Type.systemBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }

        @Suppress("DEPRECATION")
        vibrator = getSystemService(VIBRATOR_SERVICE) as Vibrator
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        accelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        
        setupControls()
        setupPeriodicSender()
        connectToServer()
    }

    override fun onResume() {
        super.onResume()
        sensorManager.registerListener(this, accelSensor, SensorManager.SENSOR_DELAY_GAME)
        handler.post(sendRunnable)
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
        handler.removeCallbacks(sendRunnable)
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnectFromServer()
        executor.shutdownNow()
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun setupControls() {
        // Emergency Stop Button
        binding.buttonEmergencyStop.setOnClickListener {
            vibrate(VIBRATE_LONG)
            send("brake on")
            send("gas 0")
            send("sound alarm")
            binding.textviewGearStatus.text = "E"
            binding.textviewGearStatus.setTextColor(Color.RED)
            handler.postDelayed({
                send("brake off")
                send("sound off")
                binding.textviewGearStatus.text = "N"
                binding.textviewGearStatus.setTextColor(getColor(R.color.primary))
            }, 2000)
        }

        // Performance Mode Buttons
        binding.buttonModeNormal.setOnClickListener {
            currentPerformanceMode = PerformanceMode.NORMAL
            updatePerformanceModeUI()
            send("settings accRate 20")
            send("settings decRate 30")
            vibrate(VIBRATE_SHORT)
        }
        
        binding.buttonModeSport.setOnClickListener {
            currentPerformanceMode = PerformanceMode.SPORT
            updatePerformanceModeUI()
            send("settings accRate 40")
            send("settings decRate 20")
            vibrate(VIBRATE_SHORT)
        }
        
        binding.buttonModeEco.setOnClickListener {
            currentPerformanceMode = PerformanceMode.ECO
            updatePerformanceModeUI()
            send("settings accRate 10")
            send("settings decRate 40")
            vibrate(VIBRATE_SHORT)
        }

        // Sound Controls
        binding.buttonSoundEngine.setOnClickListener {
            soundEnabled = true
            send("sound on")
            vibrate(VIBRATE_SHORT)
        }
        
        binding.buttonSoundPolice.setOnClickListener {
            soundEnabled = true
            send("sound police")
            vibrate(VIBRATE_SHORT)
        }
        
        binding.buttonSoundHorn.setOnClickListener {
            send("beep on")
            vibrate(VIBRATE_LONG)
            handler.postDelayed({ send("beep off") }, 500)
        }

        // Headlights
        binding.switchHeadlights.setOnCheckedChangeListener { _, isChecked ->
            send("head ${if (isChecked) "on" else "off"}")
        }
        
        binding.root.setOnClickListener {
            val now = System.currentTimeMillis()
            if (now - lastHeadlightTap < DOUBLE_TAP_MS) {
                binding.switchHeadlights.isChecked = !binding.switchHeadlights.isChecked
                vibrate(VIBRATE_SHORT)
            }
            lastHeadlightTap = now
        }

        // RGB Mode Spinner
        binding.spinnerRgbMode.adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_item,
            rgbModes.map { it.displayName }
        ).apply {
            setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        }
        
        binding.spinnerRgbMode.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>?, view: View?, pos: Int, id: Long) {
                currentAnimation = rgbModes[pos].name
                sendRgbCommand()
            }
            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }
        
        binding.colorPreview.setOnClickListener {
            vibrate(VIBRATE_SHORT)
            showColorPicker()
        }
        
        binding.settingsButton.setOnClickListener {
            vibrate(VIBRATE_SHORT)
            showBrightnessDialog()
        }
        
        setupColorPresets()

        // Turn Indicators
        var selectedIndicator: View? = null

        fun selectIndicator(button: View, cmd: String) {
            selectedIndicator?.isSelected = false
            button.isSelected = true
            selectedIndicator = button
            send("ind $cmd")
            autoIndicatorsEnabled = false
        }
        
        binding.buttonIndLeft.setOnClickListener { selectIndicator(it, "left") }
        binding.buttonIndOff.setOnClickListener { selectIndicator(it, "off") }
        binding.buttonIndRight.setOnClickListener { selectIndicator(it, "right") }
        binding.buttonIndOff.performClick()

        // Brake Pedal
        binding.buttonBrake.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    vibrate(VIBRATE_SHORT)
                    send("brake on")
                    binding.buttonGas.isEnabled = false
                    binding.buttonGas.isEnabled = true
                    currentGas = 0
                    updateGasVisual(0)
                    binding.brakeProgress.progress = 100
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    send("brake off")
                    binding.brakeProgress.progress = 0
                }
            }
            true
        }

        // Gas Pedal
        binding.buttonGas.setOnTouchListener { v, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                    val level = ((THROTTLE_LEVELS * (v.height - event.y)) / v.height).toInt().coerceIn(0, THROTTLE_LEVELS)
                    if (level != currentGas) {
                        currentGas = level
                        updateGasVisual(level)
                        send("gas $level")
                        binding.gasProgress.progress = (level * 100) / THROTTLE_LEVELS
                    }
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    currentGas = 0
                    updateGasVisual(0)
                    send("gas 0")
                    binding.gasProgress.progress = 0
                }
            }
            true
        }
    }

    private fun updatePerformanceModeUI() {
        binding.buttonModeNormal.isSelected = currentPerformanceMode == PerformanceMode.NORMAL
        binding.buttonModeSport.isSelected = currentPerformanceMode == PerformanceMode.SPORT
        binding.buttonModeEco.isSelected = currentPerformanceMode == PerformanceMode.ECO
    }

    private fun setupColorPresets() {
        val presetButtons = listOf(
            binding.colorPreset1 to colorPresets[0],
            binding.colorPreset2 to colorPresets[1],
            binding.colorPreset3 to colorPresets[2],
            binding.colorPreset4 to colorPresets[3],
            binding.colorPreset5 to colorPresets[4]
        )
        
        presetButtons.forEach { (button, preset) ->
            button.setBackgroundColor(Color.rgb(
                (preset.color shr 16) and 0xFF,
                (preset.color shr 8) and 0xFF,
                preset.color and 0xFF
            ))
            button.setOnClickListener {
                vibrate(VIBRATE_SHORT)
                currentRgbColor = preset.color
                updateColorPreview()
                sendRgbCommand()
            }
        }
    }
    
    private fun showColorPicker() {
        val dialogView = layoutInflater.inflate(R.layout.dialog_color_picker, null)
        
        val currentR = (currentRgbColor shr 16) and 0xFF
        val currentG = (currentRgbColor shr 8) and 0xFF
        val currentB = currentRgbColor and 0xFF
        
        val sliderR = dialogView.findViewById<SeekBar>(R.id.slider_red)
        val sliderG = dialogView.findViewById<SeekBar>(R.id.slider_green)
        val sliderB = dialogView.findViewById<SeekBar>(R.id.slider_blue)
        val valueR = dialogView.findViewById<TextView>(R.id.value_red)
        val valueG = dialogView.findViewById<TextView>(R.id.value_green)
        val valueB = dialogView.findViewById<TextView>(R.id.value_blue)
        val previewBox = dialogView.findViewById<View>(R.id.color_preview_large)
        val hexLabel = dialogView.findViewById<TextView>(R.id.hex_value)
        
        sliderR.progress = currentR
        sliderG.progress = currentG
        sliderB.progress = currentB
        
        val updatePreview = {
            val r = sliderR.progress
            val g = sliderG.progress
            val b = sliderB.progress
            val color = (r shl 16) or (g shl 8) or b
            
            previewBox.setBackgroundColor(Color.rgb(r, g, b))
            hexLabel.text = String.format("#%06X", color)
            valueR.text = r.toString()
            valueG.text = g.toString()
            valueB.text = b.toString()
        }
        
        val listener = object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar?, p: Int, f: Boolean) = updatePreview()
            override fun onStartTrackingTouch(s: SeekBar?) {}
            override fun onStopTrackingTouch(s: SeekBar?) {}
        }
        sliderR.setOnSeekBarChangeListener(listener)
        sliderG.setOnSeekBarChangeListener(listener)
        sliderB.setOnSeekBarChangeListener(listener)
        
        val quickColors = listOf(
            R.id.quick_red to 0xFF0000,
            R.id.quick_green to 0x00FF00,
            R.id.quick_blue to 0x0000FF,
            R.id.quick_yellow to 0xFFFF00,
            R.id.quick_cyan to 0x00FFFF,
            R.id.quick_magenta to 0xFF00FF,
            R.id.quick_white to 0xFFFFFF
        )
        
        quickColors.forEach { (id, color) ->
            dialogView.findViewById<View>(id)?.setOnClickListener {
                sliderR.progress = (color shr 16) and 0xFF
                sliderG.progress = (color shr 8) and 0xFF
                sliderB.progress = color and 0xFF
                updatePreview()
            }
        }
        
        updatePreview()
        
        AlertDialog.Builder(this)
            .setTitle(R.string.dialog_choose_rgb_color)
            .setView(dialogView)
            .setPositiveButton(R.string.dialog_apply) { _, _ ->
                currentRgbColor = (sliderR.progress shl 16) or (sliderG.progress shl 8) or sliderB.progress
                updateColorPreview()
                sendRgbCommand()
            }
            .setNegativeButton(R.string.dialog_cancel, null)
            .show()
    }
    
    private fun showBrightnessDialog() {
        val dialogView = layoutInflater.inflate(R.layout.dialog_settings, null)
        
        val sliderBrightness = dialogView.findViewById<SeekBar>(R.id.slider_brightness)
        val sliderSpeed = dialogView.findViewById<SeekBar>(R.id.slider_speed)
        val valueBrightness = dialogView.findViewById<TextView>(R.id.value_brightness)
        val valueSpeed = dialogView.findViewById<TextView>(R.id.value_speed)
        val previewBox = dialogView.findViewById<View>(R.id.brightness_preview)
        val checkAuto = dialogView.findViewById<CheckBox>(R.id.checkbox_auto_indicators)
        val checkSound = dialogView.findViewById<CheckBox>(R.id.checkbox_sound)
        val checkStatus = dialogView.findViewById<CheckBox>(R.id.checkbox_status_mode)
        
        sliderBrightness.progress = currentBrightness
        sliderSpeed.progress = currentSpeed
        checkAuto.isChecked = autoIndicatorsEnabled
        checkSound.isChecked = soundEnabled
        checkStatus.isChecked = statusIndicatorEnabled
        
        val updatePreview = {
            valueBrightness.text = sliderBrightness.progress.toString()
            valueSpeed.text = getString(R.string.label_speed_ms, sliderSpeed.progress)
            previewBox.setBackgroundColor(Color.argb(sliderBrightness.progress, 255, 255, 255))
        }
        
        val listener = object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar?, p: Int, f: Boolean) = updatePreview()
            override fun onStartTrackingTouch(s: SeekBar?) {}
            override fun onStopTrackingTouch(s: SeekBar?) {}
        }
        sliderBrightness.setOnSeekBarChangeListener(listener)
        sliderSpeed.setOnSeekBarChangeListener(listener)
        
        updatePreview()
        
        AlertDialog.Builder(this)
            .setTitle(R.string.dialog_rgb_settings)
            .setView(dialogView)
            .setPositiveButton(R.string.dialog_apply) { _, _ ->
                currentBrightness = sliderBrightness.progress
                currentSpeed = sliderSpeed.progress
                autoIndicatorsEnabled = checkAuto.isChecked
                soundEnabled = checkSound.isChecked
                statusIndicatorEnabled = checkStatus.isChecked
                
                send("brightness $currentBrightness")
                send("speed $currentSpeed")
                send("sound ${if (soundEnabled) "on" else "off"}")
                send("status ${if (statusIndicatorEnabled) "on" else "off"}")
                if (!autoIndicatorsEnabled) {
                    send("ind off")
                    currentAutoIndicator = "off"
                }
            }
            .setNegativeButton(R.string.dialog_cancel, null)
            .show()
    }
    
    private fun updateColorPreview() {
        val r = (currentRgbColor shr 16) and 0xFF
        val g = (currentRgbColor shr 8) and 0xFF
        val b = currentRgbColor and 0xFF
        binding.colorPreview.setBackgroundColor(Color.rgb(r, g, b))
    }
    
    private fun sendRgbCommand() {
        val hexColor = String.format("%06X", currentRgbColor)
        val command = if (currentAnimation == "off") "rgb off" else "rgb $currentAnimation $hexColor"
        send(command)
    }

    private fun updateGasVisual(level: Int) {
        binding.gasProgress.progress = level / THROTTLE_LEVELS * 100 
        binding.labelGasLevel.text = level.toString()
    }
    
    private fun updateSteeringVisual(angle: Int) {
        val offset = angle - SERVO_CENTER
        binding.labelSteeringAngle.text = getString(R.string.steering_angle, offset)
        
        if (autoIndicatorsEnabled) {
            val newIndicator = when {
                offset < -AUTO_IND_THRESHOLD -> "left"
                offset > AUTO_IND_THRESHOLD -> "right"
                else -> "off"
            }
            
            if (newIndicator != currentAutoIndicator) {
                currentAutoIndicator = newIndicator
                send("ind $newIndicator")
            }
        }
    }
    
    private fun updateGearDisplay(gear: Int, direction: String) {
        currentGear = gear
        currentDirection = direction
        val gearText = when {
            gear == 0 && direction == "N" -> "N"
            gear == -1 -> "R"
            gear > 0 -> gear.toString()
            else -> "N"
        }
        binding.textviewGearStatus.text = gearText
        binding.textviewGearDisplay.text = gearText
        
        // Color code based on gear
        when {
            gear == -1 -> {
                binding.textviewGearStatus.setTextColor(Color.RED)
                binding.textviewGearDisplay.setTextColor(Color.RED)
            }
            gear == 0 -> {
                binding.textviewGearStatus.setTextColor(getColor(R.color.primary))
                binding.textviewGearDisplay.setTextColor(getColor(R.color.primary))
            }
            else -> {
                binding.textviewGearStatus.setTextColor(Color.GREEN)
                binding.textviewGearDisplay.setTextColor(Color.GREEN)
            }
        }
    }
    
    private fun vibrate(duration: Long) {
        if (vibrator.hasVibrator()) {
            vibrator.vibrate(VibrationEffect.createOneShot(duration, VibrationEffect.DEFAULT_AMPLITUDE))
        }
    }

    private fun connectToServer() {
        updateConnectionStatus(ConnectionStatus.CONNECTING)
        
        executor.execute {
            try {
                socket = Socket(SERVER_IP, SERVER_PORT).apply { 
                    tcpNoDelay = true
                    soTimeout = 5000
                }
                output = PrintWriter(socket!!.getOutputStream(), true)
                input = BufferedReader(InputStreamReader(socket!!.getInputStream()))
                
                handler.postDelayed({
                    send("sound ${if (soundEnabled) "on" else "off"}")
                    send("status ${if (statusIndicatorEnabled) "on" else "off"}")
                }, 200)
                
                Thread {
                    try {
                        while (socket?.isConnected == true) {
                            val line = input?.readLine() ?: break
                            
                            if (line.startsWith("TELEM:")) {
                                lastTelemetryTime = System.currentTimeMillis()
                                val parts = line.substring(6).split(",")
                                if (parts.size >= 6) {
                                    batteryLevel = parts[0].toIntOrNull() ?: 0
                                    val gear = parts[1].toIntOrNull() ?: 0
                                    val throttle = parts[2].toIntOrNull() ?: 0
                                    val direction = parts[3]
                                    val brake = parts[4]
                                    val connected = parts[5]
                                    
                                    val color = when {
                                        batteryLevel > 70 -> getColor(R.color.battery_good)
                                        batteryLevel > 30 -> getColor(R.color.battery_medium)
                                        else -> getColor(R.color.battery_low)
                                    }
                                    
                                    runOnUiThread {
                                        binding.textviewBatteryStatus.text = getString(R.string.label_battery, batteryLevel)
                                        binding.textviewBatteryStatus.setTextColor(color)
                                        updateGearDisplay(gear, direction)
                                    }
                                    
                                    if (connectionLost) {
                                        connectionLost = false
                                        runOnUiThread { updateConnectionStatus(ConnectionStatus.CONNECTED) }
                                    }
                                }
                            }
                        }
                    } catch (e: Exception) {
                        Debug.info(TAG, "Telemetry ended: ${e.message}")
                    }
                }.start()
                
                lastTelemetryTime = System.currentTimeMillis()
                connectionLost = false
                runOnUiThread { updateConnectionStatus(ConnectionStatus.CONNECTED) }
            } catch (_: Exception) {
                runOnUiThread { updateConnectionStatus(ConnectionStatus.FAILED) }
            }
        }
    }

    private fun disconnectFromServer() {
        executor.execute {
            try {
                output?.close()
                input?.close()
                socket?.close()
            } catch (e: Exception) {
                Debug.info(TAG, "Cleanup: ${e.message}")
            } finally {
                output = null
                input = null
                socket = null
            }
            runOnUiThread { updateConnectionStatus(ConnectionStatus.DISCONNECTED) }
        }
    }
    
    private enum class ConnectionStatus(val stringRes: Int, val colorRes: Int) {
        DISCONNECTED(R.string.status_disconnected_dot, R.color.status_disconnected),
        CONNECTING(R.string.status_connecting_dot, R.color.status_connecting),
        CONNECTED(R.string.status_connected_dot, R.color.status_connected),
        FAILED(R.string.status_failed_dot, R.color.status_failed),
        LOST(R.string.status_lost_dot, R.color.status_lost)
    }
    
    private fun updateConnectionStatus(status: ConnectionStatus) {
        binding.textviewConnectionStatus.text = getString(status.stringRes)
        binding.textviewConnectionStatus.setTextColor(getColor(status.colorRes))
    }

    private fun handleConnectionLoss() {
        updateConnectionStatus(ConnectionStatus.LOST)
        vibrate(VIBRATE_SHORT)
        handler.postDelayed({ vibrate(VIBRATE_SHORT) }, 100)
    }

    private fun setupPeriodicSender() {
        sendRunnable = object : Runnable {
            override fun run() {
                if (socket?.isConnected == true) {
                    if (abs(currentServo - lastSentServo) >= STEERING_THRESHOLD) {
                        send("servo $currentServo")
                        lastSentServo = currentServo
                        runOnUiThread { updateSteeringVisual(currentServo) }
                    }

                    if (currentGas != lastSentGas) {
                        send("gas $currentGas")
                        lastSentGas = currentGas
                    }

                    if (System.currentTimeMillis() - lastTelemetryTime > CONNECTION_TIMEOUT_MS && !connectionLost) {
                        connectionLost = true
                        runOnUiThread { handleConnectionLoss() }
                    }
                }
                handler.postDelayed(this, SEND_INTERVAL_MS)
            }
        }
    }

    private fun send(cmd: String) {
        if (socket?.isConnected != true) return
        executor.execute {
            try {
                output?.println(cmd)
                output?.flush()
            } catch (_: Exception) {}
        }
    }
    
    override fun onSensorChanged(event: SensorEvent?) {
        if (event?.sensor?.type == Sensor.TYPE_ACCELEROMETER) {
            val angle = (SERVO_CENTER + (event.values[1] * 5)).toInt().coerceIn(SERVO_MIN, SERVO_MAX)
            currentServo = angle
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}