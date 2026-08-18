package com.avatarrobot.camswitcher

import android.os.Bundle
import android.widget.Button
import android.widget.EditText
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity

/**
 * AIM / fire-control window, ported from the standalone ARMSWITCH app.
 *
 * Reached from the HUD's FIRING→AIM button. Flow is identical to the old app:
 * enter the password, slide to arm, press FIRE (sends "FIRE <ms>" and waits for
 * "OK", holding for the arm-time). Two additions for the merged app:
 *   - BACK returns to the HUD WITHOUT changing state — the FIRING/AIM button
 *     stays on AIM (RESULT_CANCELED).
 *   - RESET disarms the rover ("MODE 0") and returns to the HUD, which resets the
 *     FIRING/AIM button back to FIRING (RESULT_OK).
 */
class ArmFireActivity : AppCompatActivity() {

    private lateinit var passwordDisplay: TextView
    private lateinit var statusDisplay: TextView
    private lateinit var ipAddressInput: EditText
    private lateinit var portInput: EditText
    private lateinit var armSlider: SeekBar
    private lateinit var armTimeSlider: SeekBar
    private lateinit var armTimeLabel: TextView
    private lateinit var btnFire: Button
    private lateinit var btnReset: Button
    private lateinit var btnBack: Button

    private val enteredPassword = StringBuilder()

    private var passwordVerified = false
    private var armed = false                 // slider pulled past threshold
    @Volatile private var isArming = false    // TCP exchange in flight
    private var lockedOut = false             // after a failed connection

    @Volatile private var armHoldMs = 500     // 10–5000 ms via the arm-time slider

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_arm)
        configureSystemBars()

        passwordDisplay = findViewById(R.id.passwordDisplay)
        statusDisplay   = findViewById(R.id.statusDisplay)
        ipAddressInput  = findViewById(R.id.ipAddressInput)
        portInput       = findViewById(R.id.portInput)
        armSlider       = findViewById(R.id.armSlider)
        armTimeSlider   = findViewById(R.id.armTimeSlider)
        armTimeLabel    = findViewById(R.id.armTimeLabel)
        btnFire         = findViewById(R.id.btnFire)
        btnReset        = findViewById(R.id.btnReset)
        btnBack         = findViewById(R.id.btnBack)

        val numIds = intArrayOf(R.id.btn0, R.id.btn1, R.id.btn2, R.id.btn3, R.id.btn4,
            R.id.btn5, R.id.btn6, R.id.btn7, R.id.btn8, R.id.btn9)
        for (i in 0..9) {
            findViewById<Button>(numIds[i]).setOnClickListener { onDigitPressed(i) }
        }
        findViewById<Button>(R.id.btnClear).setOnClickListener { clearPassword() }
        findViewById<Button>(R.id.btnEnter).setOnClickListener { verifyPassword() }

        btnFire.setOnClickListener { fire() }
        btnReset.setOnClickListener { resetAndExit() }
        btnBack.setOnClickListener { finish() }   // RESULT_CANCELED → HUD stays on AIM

        armSlider.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar, p: Int, fromUser: Boolean) {}
            override fun onStartTrackingTouch(bar: SeekBar) {}
            override fun onStopTrackingTouch(bar: SeekBar) {
                if (armed || !passwordVerified || isArming || lockedOut) return
                if (bar.progress >= SLIDE_THRESHOLD) {
                    bar.progress = 100
                    armed = true
                    statusDisplay.text = "ARMED — press FIRE"
                    updateUI()
                } else {
                    bar.progress = 0
                }
            }
        })

        armTimeSlider.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar, p: Int, fromUser: Boolean) {
                armHoldMs = 10 + p              // 0–4990 → 10–5000 ms
                armTimeLabel.text = "ARM TIME: $armHoldMs ms"
            }
            override fun onStartTrackingTouch(bar: SeekBar) {}
            override fun onStopTrackingTouch(bar: SeekBar) {}
        })
        armHoldMs = 10 + armTimeSlider.progress
        armTimeLabel.text = "ARM TIME: $armHoldMs ms"

        updateUI()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) configureSystemBars()
    }

    // ---- keypad / password ----

    private fun onDigitPressed(digit: Int) {
        if (passwordVerified || isArming || lockedOut) return
        if (enteredPassword.length < 12) {
            enteredPassword.append(digit)
            passwordDisplay.text = "*".repeat(enteredPassword.length)
        }
    }

    private fun clearPassword() {
        if (passwordVerified || isArming || lockedOut) return
        enteredPassword.setLength(0)
        passwordDisplay.text = ""
    }

    private fun verifyPassword() {
        if (passwordVerified || isArming || lockedOut) return
        if (enteredPassword.toString() == CORRECT_PASSWORD) {
            passwordVerified = true
            statusDisplay.text = "Password OK! READY FOR ARM"
            updateUI()
        } else {
            Toast.makeText(this, "Wrong password", Toast.LENGTH_SHORT).show()
            enteredPassword.setLength(0)
            passwordDisplay.text = ""
        }
    }

    // ---- fire / reset ----

    private fun fire() {
        if (!armed || isArming || lockedOut) return
        val ip = ipAddressInput.text.toString().trim()
        val portStr = portInput.text.toString().trim()
        if (ip.isEmpty() || portStr.isEmpty()) {
            Toast.makeText(this, "Set IP and port", Toast.LENGTH_SHORT).show(); return
        }
        val port = portStr.toIntOrNull()
        if (port == null) {
            Toast.makeText(this, "Bad port", Toast.LENGTH_SHORT).show(); return
        }
        isArming = true
        val holdMs = armHoldMs
        statusDisplay.text = "FIRING — holding ${holdMs}ms…"
        updateUI()
        ArmFireClient.send(ip, port, "FIRE $holdMs", holdMs = holdMs) { ok, message ->
            if (ok) {
                statusDisplay.text = "DONE"
                isArming = false
                reset()                         // clear in-window state, stay in window
            } else {
                statusDisplay.text = "FAIL — $message"
                isArming = false
                lockedOut = true
                updateUI()
            }
        }
    }

    /** RESET button: return to the HUD; the HUD drops fire mode (stops the
     *  heartbeat → /fire_mode 0) on this RESULT_OK. */
    private fun resetAndExit() {
        if (isArming) return
        reset()
        setResult(RESULT_OK)                    // HUD stops heartbeat + resets FIRING/AIM → FIRING
        finish()
    }

    /** Clear the in-window arming state (used on entry and after a successful fire). */
    private fun reset() {
        passwordVerified = false
        armed = false
        lockedOut = false
        enteredPassword.setLength(0)
        passwordDisplay.text = ""
        statusDisplay.text = "IDLE — enter password"
        armSlider.progress = 0
        updateUI()
    }

    private fun updateUI() {
        val canSlide = passwordVerified && !armed && !isArming && !lockedOut
        val canFire = armed && !isArming && !lockedOut
        armSlider.isEnabled = canSlide
        btnFire.isEnabled = canFire
        btnReset.isEnabled = !isArming
        btnBack.isEnabled = !isArming
    }

    companion object {
        private const val CORRECT_PASSWORD = "1234"
        private const val SLIDE_THRESHOLD = 95
    }
}
