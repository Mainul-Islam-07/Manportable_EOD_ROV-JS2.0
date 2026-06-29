package com.avatarrobot.camswitcher

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.res.ColorStateList
import android.graphics.Color
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.media.projection.MediaProjectionManager
import android.view.KeyEvent
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.View
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.Button
import android.widget.FrameLayout
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.alexvas.rtsp.widget.RtspStatusListener
import com.alexvas.rtsp.widget.RtspSurfaceView
import com.avatarrobot.camswitcher.twin.JointState
import com.avatarrobot.camswitcher.twin.OrbitGLSurfaceView
import com.avatarrobot.camswitcher.twin.RobotRenderer
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Single-Activity Avatar HUD: a 5-feed RTSP switcher with a transparent 3D
 * digital-twin overlay, two-way intercom, screen recorder, gimbal control, and a
 * FIRING→AIM fire-control flow. (Consolidates the former AvatarCamSwitcher,
 * JS2.0_Digital_Twin, and ARMSWITCH apps.)
 *
 * - Camera/zoom/sound are tap-to-expand dropdowns (HudDropdown); the active
 *   option is shown green.
 * - Battery and the 3D twin both feed off ONE UDP :9870 socket
 *   (UdpTelemetryReceiver), bound while foreground and released in onPause so
 *   AvatarDashboard can own the port when we background.
 * - Switching is serialized via onRtspStatusDisconnected so two decoders never
 *   share the surface; a black cover masks the transient.
 */
class MainActivity : AppCompatActivity() {

    enum class CameraFeed(val label: String, val url: String, val rotation: Int) {
        FRONT("FRONT", "rtsp://192.168.144.65:8554/main.264", 180),  // upside-down mount
        BACK ("BACK",  "rtsp://192.168.144.66:8554/main.264", 180),  // upside-down mount
        WRIST("WRIST", "rtsp://192.168.144.67:8554/main.264", 0),
        GRIP ("GRIP",  "rtsp://192.168.144.68:8554/main.264", 0),
        MAIN ("MAIN",  "rtsp://192.168.144.25:8554/main.264", 0)
    }

    /** Three-state intercom selector. */
    private enum class SoundState { OFF, LISTEN, TALK }

    /** FIRING/AIM button lifecycle. */
    private enum class FireUi { IDLE, COUNTING, AIM }

    private lateinit var rtspView: RtspSurfaceView
    private lateinit var cover: View
    private lateinit var btnRecord: Button
    private lateinit var txtBattery: TextView

    // Dropdowns
    private lateinit var btnCamera: Button
    private lateinit var btnZoom: Button
    private lateinit var btnSound: Button
    private lateinit var cameraDropdown: HudDropdown<CameraFeed>
    private lateinit var zoomDropdown: HudDropdown<Int>
    private lateinit var soundDropdown: HudDropdown<SoundState>

    // Fire control
    private lateinit var btnFire: Button
    private var fireUi = FireUi.IDLE

    // SBUS mode chip (right of FIRING), fed by sbus_mode_udp_bridge on :9871.
    private lateinit var txtMode: TextView
    private val sbusReceiver by lazy {
        SbusModeReceiver(onMode = { mode -> runOnUiThread { updateMode(mode) } })
    }
    private val modeStaleRunnable = Runnable { showModeNoLink() }

    // 3D twin
    private lateinit var twinContainer: FrameLayout
    private lateinit var btn3d: Button
    private val jointState = JointState()
    private lateinit var glView: OrbitGLSurfaceView
    private var twinVisible = true

    private lateinit var gimbalPanel: View

    // SIYI A8 mini pan/tilt over UDP (192.168.144.25:37260). MAIN-feed only.
    private val gimbal = SiyiGimbalController()

    // ONE telemetry socket on :9870 → battery chip + 3D joint state.
    private val telemetryReceiver by lazy {
        UdpTelemetryReceiver(
            onBattery = { pct -> runOnUiThread { updateBattery(pct) } },
            onJoints = { json -> jointState.update(json, System.currentTimeMillis()) },
        )
    }

    // Two-way intercom over UDP with the rover host (192.168.144.10:5555).
    private val audioLink by lazy { AudioLinkController(applicationContext) }
    private var soundState = SoundState.OFF

    // Digital zoom for the MAIN feed only (1x..4x), persisted while MAIN is active.
    private var mainZoom = 1

    private var videoW = 0
    private var videoH = 0

    // ---- State engine -------------------------------------------------------
    private var currentFeed: CameraFeed = CameraFeed.MAIN
    private var pendingFeed: CameraFeed? = CameraFeed.MAIN
    private var surfaceReady = false
    private var switching = false
    private var reconnecting = false

    private val mainHandler = Handler(Looper.getMainLooper())
    private val forceStartRunnable = Runnable { beginPendingFeed() }
    private val retryRunnable = Runnable { retryCurrentFeed() }
    private val countdownRunnable = Runnable { onFireCountdownDone() }

    // ---- Recording ----------------------------------------------------------
    private var recStartMs = 0L
    private var lastRecordName: String? = null
    private val recTimerRunnable = object : Runnable {
        override fun run() {
            val s = (SystemClock.elapsedRealtime() - recStartMs) / 1000
            btnRecord.text = String.format(Locale.US, "REC %02d:%02d", s / 60, s % 60)
            mainHandler.postDelayed(this, 1000)
        }
    }

    // System screen-capture consent dialog. On OK we start the recording service.
    private val projectionLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        val data = result.data
        if (result.resultCode == RESULT_OK && data != null) {
            startScreenRecording(result.resultCode, data)
        } else {
            Toast.makeText(this, "Screen recording permission denied", Toast.LENGTH_SHORT).show()
        }
    }

    private val notifPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { /* recording still works if denied */ }

    private val storagePermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { /* legacy DCIM save fails silently if denied */ }

    // Mic capture needs RECORD_AUDIO. Requested when TALK is first chosen; if
    // granted we switch to talking, otherwise we fall back to OFF.
    private val micPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (granted) applySound(SoundState.TALK)
        else {
            Toast.makeText(this, "Microphone permission denied", Toast.LENGTH_SHORT).show()
            applySound(SoundState.OFF)
        }
    }

    // AIM window result: RESULT_OK means RESET was pressed (rover disarmed) → reset
    // the FIRING/AIM button to FIRING. Anything else (BACK) keeps it on AIM.
    private val armLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode == RESULT_OK) resetFireButton()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)
        configureSystemBars()

        rtspView      = findViewById(R.id.rtsp_view)
        cover         = findViewById(R.id.transition_cover)
        btnRecord     = findViewById(R.id.btn_record)
        txtBattery    = findViewById(R.id.txt_battery)
        btnCamera     = findViewById(R.id.btn_camera)
        btnZoom       = findViewById(R.id.btn_zoom)
        btnSound      = findViewById(R.id.btn_sound)
        btnFire       = findViewById(R.id.btn_fire)
        txtMode       = findViewById(R.id.txt_mode)
        gimbalPanel   = findViewById(R.id.gimbal_panel)
        twinContainer = findViewById(R.id.twin_container)
        btn3d         = findViewById(R.id.btn_3d)

        if (Build.VERSION.SDK_INT >= 33 &&
            checkSelfPermission(android.Manifest.permission.POST_NOTIFICATIONS)
            != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            notifPermLauncher.launch(android.Manifest.permission.POST_NOTIFICATIONS)
        }
        if (Build.VERSION.SDK_INT < 29 &&
            checkSelfPermission(android.Manifest.permission.WRITE_EXTERNAL_STORAGE)
            != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            storagePermLauncher.launch(android.Manifest.permission.WRITE_EXTERNAL_STORAGE)
        }

        // ---- 3D twin overlay ----
        glView = OrbitGLSurfaceView(this, RobotRenderer(assets, jointState))
        twinContainer.addView(glView, FrameLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
        btn3d.setOnClickListener { toggleTwin() }

        // Low-latency: no smoothing buffer; SPS rewrite OFF (it caused artifacts).
        rtspView.videoFrameRateStabilization = false
        rtspView.experimentalUpdateSpsFrameWithLowLatencyParams = LOW_LATENCY_SPS_REWRITE

        rtspView.setStatusListener(object : RtspStatusListener {
            override fun onRtspFirstFrameRendered() = runOnUiThread {
                reconnecting = false
                mainHandler.removeCallbacks(retryRunnable)
                hideCover()
            }
            override fun onRtspStatusDisconnected() = runOnUiThread {
                if (switching || pendingFeed != null) beginPendingFeed()
            }
            override fun onRtspFrameSizeChanged(width: Int, height: Int) = runOnUiThread {
                fitSurfaceToVideo(width, height)
            }
            override fun onRtspStatusFailed(message: String?) = runOnUiThread {
                if (!surfaceReady) return@runOnUiThread
                if (!reconnecting) {
                    reconnecting = true
                    Toast.makeText(this@MainActivity, "Stream error — reconnecting…",
                        Toast.LENGTH_SHORT).show()
                }
                switching = false
                showCover()
                mainHandler.removeCallbacks(forceStartRunnable)
                mainHandler.removeCallbacks(retryRunnable)
                mainHandler.postDelayed(retryRunnable, RETRY_DELAY_MS)
            }
        })

        rtspView.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                surfaceReady = true
                showCover()
                beginPendingFeed()
            }
            override fun surfaceChanged(h: SurfaceHolder, fmt: Int, w: Int, ht: Int) { /* no-op */ }
            override fun surfaceDestroyed(holder: SurfaceHolder) {
                surfaceReady = false
                switching = false
                mainHandler.removeCallbacks(forceStartRunnable)
                mainHandler.removeCallbacks(retryRunnable)
                rtspView.stop()
            }
        })

        ScreenRecordService.onStateChange = { active -> runOnUiThread { onRecordingStateChanged(active) } }

        setupDropdowns()
        // If the mic can't open (device error), fall back to OFF.
        audioLink.onMicFailed = {
            applySound(SoundState.OFF)
            Toast.makeText(this, "Couldn't start microphone", Toast.LENGTH_SHORT).show()
        }
        wireGimbalButtons()
        btnRecord.setOnClickListener { toggleRecording() }
        btnFire.setOnClickListener { onFirePressed() }

        cameraDropdown.setCurrent(currentFeed)
        updateMainOnlyControls(currentFeed)
        updateBattery(-1)                  // grey "--" until the first packet
        // 3D figure is shown by default → 3D button starts green (grey when off).
        btn3d.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#2E7D32"))
        showModeNoLink()                   // grey "—" until the first SBUS packet
        showCover()
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) configureSystemBars()
    }

    // -- Dropdowns -------------------------------------------------------------
    private fun setupDropdowns() {
        // CAMERA + ZOOM sit at the bottom, so their lists drop UP.
        cameraDropdown = HudDropdown(
            btnCamera,
            listOf(
                CameraFeed.MAIN to "MAIN", CameraFeed.FRONT to "FRONT",
                CameraFeed.BACK to "BACK", CameraFeed.WRIST to "WRIST",
                CameraFeed.GRIP to "GRIP",
            ),
            dropUp = true,
        ) { feed -> switchTo(feed) }

        zoomDropdown = HudDropdown(
            btnZoom,
            listOf(4 to "4X", 3 to "3X", 2 to "2X", 1 to "1X"),
            dropUp = true,
        ) { factor -> applyZoom(factor) }
        zoomDropdown.setCurrent(mainZoom)

        // SOUND sits at the top, so it drops DOWN (default). At rest (OFF) the
        // collapsed chip reads "SOUND"; LISTEN/TALK show their own label.
        soundDropdown = HudDropdown(
            btnSound,
            listOf(SoundState.LISTEN to "LISTEN", SoundState.TALK to "TALK", SoundState.OFF to "OFF"),
            collapsedLabel = { state, label -> if (state == SoundState.OFF) "SOUND" else label },
        ) { state -> requestSound(state) }
        soundDropdown.setCurrent(soundState)
    }

    // Battery chip (grey pill; red under 15%; "--" when unknown).
    private fun updateBattery(pct: Int) {
        val known = pct in 0..100
        val low = known && pct < 15
        txtBattery.text = if (known) "$pct%" else "--"
        txtBattery.backgroundTintList = ColorStateList.valueOf(
            if (low) 0xFFD32F2F.toInt() else 0xFF9E9E9E.toInt())
    }

    // -- Sound (LISTEN / TALK / OFF) ------------------------------------------
    // TALK needs RECORD_AUDIO; if not yet granted, request it and apply on grant.
    private fun requestSound(state: SoundState) {
        if (state == SoundState.TALK &&
            checkSelfPermission(android.Manifest.permission.RECORD_AUDIO)
            != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            micPermLauncher.launch(android.Manifest.permission.RECORD_AUDIO)
            return
        }
        applySound(state)
    }

    private fun applySound(state: SoundState) {
        soundState = state
        soundDropdown.setCurrent(state)
        when (state) {
            SoundState.OFF -> {
                audioLink.setMicEnabled(false)
                audioLink.setSpeakerEnabled(false)
            }
            SoundState.LISTEN -> {
                audioLink.setSpeakerEnabled(true)
                audioLink.setMicEnabled(false)
            }
            SoundState.TALK -> {
                audioLink.setMicEnabled(true)
                audioLink.setSpeakerEnabled(false)
            }
        }
    }

    // -- 3D twin toggle --------------------------------------------------------
    // The GL view is a z-ordered media-overlay surface, so hiding only the
    // container does NOT remove it. Toggle the GLSurfaceView itself (and pause /
    // resume its render thread) so the surface is actually torn down / recreated.
    private fun toggleTwin() {
        twinVisible = !twinVisible
        if (twinVisible) {
            glView.visibility = View.VISIBLE
            glView.onResume()
        } else {
            glView.onPause()
            glView.visibility = View.GONE
        }
        // Leave the (empty, transparent) container laid out — it is the battery
        // chip's end anchor, so collapsing it would shift the battery. Only the
        // GL surface above is toggled.
        btn3d.backgroundTintList = ColorStateList.valueOf(
            if (twinVisible) Color.parseColor("#2E7D32") else Color.parseColor("#9E9E9E"))
    }

    // -- Fire control (FIRING → 60 s → AIM) -----------------------------------
    private fun onFirePressed() {
        when (fireUi) {
            FireUi.IDLE -> startFiring()
            FireUi.COUNTING -> { /* locked during the hidden countdown */ }
            FireUi.AIM -> armLauncher.launch(Intent(this, ArmFireActivity::class.java))
        }
    }

    private fun startFiring() {
        fireUi = FireUi.COUNTING
        btnFire.text = "FIRING"
        btnFire.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#D32F2F"))
        // Enter fire mode 1: hold the presence heartbeat to the Pi (port 5006), so
        // fire_server.py publishes /fire_mode = 1. Stays up until RESET / app close.
        HeartbeatClient.start()
        mainHandler.removeCallbacks(countdownRunnable)
        mainHandler.postDelayed(countdownRunnable, FIRE_COUNTDOWN_MS)
    }

    private fun onFireCountdownDone() {
        fireUi = FireUi.AIM
        btnFire.text = "AIM"
        btnFire.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#2E7D32"))
    }

    /** Back to the resting FIRING state (after AIM-window RESET). */
    private fun resetFireButton() {
        mainHandler.removeCallbacks(countdownRunnable)
        // Fire mode 0: drop the heartbeat so /fire_mode times out to 0 on the Pi.
        HeartbeatClient.stop()
        fireUi = FireUi.IDLE
        btnFire.text = "FIRING"
        btnFire.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#9E9E9E"))
    }

    // -- SBUS mode chip (right of FIRING) -------------------------------------
    // DISARM is red (safe/disarmed stands out); ARM/HOME/DRIVE are green (armed).
    // Each packet reschedules the stale fallback so a dropped feed shows "—".
    private fun updateMode(mode: String) {
        mainHandler.removeCallbacks(modeStaleRunnable)
        txtMode.text = mode
        val color = if (mode == "DISARM") "#D32F2F" else "#2E7D32"
        txtMode.backgroundTintList = ColorStateList.valueOf(Color.parseColor(color))
        mainHandler.postDelayed(modeStaleRunnable, MODE_STALE_MS)
    }

    private fun showModeNoLink() {
        txtMode.text = "—"
        txtMode.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#9E9E9E"))
    }

    // -- Gimbal D-pad ----------------------------------------------------------
    private fun wireGimbalButtons() {
        val s = SiyiGimbalController.MOVE_SPEED
        holdToMove(R.id.btn_pan_left,  -s, 0)
        holdToMove(R.id.btn_pan_right,  s, 0)
        holdToMove(R.id.btn_tilt_up,    0, s)
        holdToMove(R.id.btn_tilt_down,  0, -s)
        findViewById<Button>(R.id.btn_gimbal_center).setOnClickListener { gimbal.center() }
    }

    @SuppressLint("ClickableViewAccessibility")
    private fun holdToMove(buttonId: Int, yaw: Int, pitch: Int) {
        findViewById<Button>(buttonId).setOnTouchListener { v, event ->
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> { v.isPressed = true; gimbal.startMove(yaw, pitch); true }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> { v.isPressed = false; gimbal.stopMove(); true }
                else -> false
            }
        }
    }

    private fun fitSurfaceToVideo(w: Int, h: Int) {
        if (w <= 0 || h <= 0) return
        videoW = w; videoH = h
        val parent = rtspView.parent as? View ?: return
        val pw = parent.width; val ph = parent.height
        if (pw == 0 || ph == 0) { rtspView.post { fitSurfaceToVideo(w, h) }; return }
        // Letterbox to the video's true aspect ratio (no fill zoom): the surface
        // is sized to fit and centered, so black bars on the unused axis are
        // expected — the feed is shown at its original proportions.
        val fit = minOf(pw / w.toFloat(), ph / h.toFloat())
        val lp = rtspView.layoutParams
        lp.width = (w * fit).toInt()
        lp.height = (h * fit).toInt()
        rtspView.layoutParams = lp
        applyVideoScale()
    }

    // Surface scale = the user's digital zoom only (MAIN feed); other feeds 1x.
    // Center-pivoted; no aspect-fill zoom, so the video keeps its true shape.
    private fun applyVideoScale() {
        val s = if (currentFeed == CameraFeed.MAIN) mainZoom.toFloat() else 1f
        rtspView.scaleX = s
        rtspView.scaleY = s
    }

    private fun applyZoom(factor: Int) {
        mainZoom = factor
        applyVideoScale()
        zoomDropdown.setCurrent(factor)
    }

    // Zoom + gimbal are MAIN-only controls.
    private fun updateMainOnlyControls(feed: CameraFeed) {
        if (feed == CameraFeed.MAIN) {
            btnZoom.visibility = View.VISIBLE
            gimbalPanel.visibility = View.VISIBLE
            applyZoom(mainZoom)
        } else {
            btnZoom.visibility = View.GONE
            gimbalPanel.visibility = View.GONE
            applyVideoScale()       // still fill the bars (no digital zoom off-MAIN)
            gimbal.stopMove()
        }
    }

    // -- Camera switching (does NOT touch recording) --------------------------
    private fun switchTo(feed: CameraFeed) {
        if (feed == currentFeed && rtspView.isStarted() && !switching) return

        currentFeed = feed
        pendingFeed = feed
        reconnecting = false
        mainHandler.removeCallbacks(retryRunnable)
        cameraDropdown.setCurrent(feed)
        updateMainOnlyControls(feed)
        showCover()

        if (!surfaceReady) return
        if (switching) return

        if (rtspView.isStarted()) {
            switching = true
            rtspView.stop()
            mainHandler.removeCallbacks(forceStartRunnable)
            mainHandler.postDelayed(forceStartRunnable, START_TIMEOUT_MS)
        } else {
            beginPendingFeed()
        }
    }

    private fun beginPendingFeed() {
        if (!surfaceReady) return
        mainHandler.removeCallbacks(forceStartRunnable)
        switching = false
        val feed = pendingFeed ?: currentFeed
        pendingFeed = null

        rtspView.stop()
        rtspView.videoRotation = feed.rotation
        rtspView.init(Uri.parse(feed.url))
        rtspView.start(requestVideo = true, requestAudio = false)
    }

    private fun retryCurrentFeed() {
        if (!surfaceReady) return
        pendingFeed = currentFeed
        showCover()
        beginPendingFeed()
    }

    // -- Recording (whole screen via MediaProjection) -------------------------
    private fun toggleRecording() {
        if (ScreenRecordService.isRecording) {
            startService(Intent(this, ScreenRecordService::class.java).apply {
                action = ScreenRecordService.ACTION_STOP
            })
            onRecordingStateChanged(false)
            Toast.makeText(this, "Saved to DCIM/JS2.0: ${lastRecordName ?: "file"}", Toast.LENGTH_LONG).show()
        } else {
            val mpm = getSystemService(Context.MEDIA_PROJECTION_SERVICE) as MediaProjectionManager
            projectionLauncher.launch(mpm.createScreenCaptureIntent())
        }
    }

    private fun startScreenRecording(resultCode: Int, data: Intent) {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val name = "Live_Feed_$stamp.mp4"
        lastRecordName = name

        val metrics = resources.displayMetrics
        val w = metrics.widthPixels - (metrics.widthPixels % 2)
        val h = metrics.heightPixels - (metrics.heightPixels % 2)

        val svc = Intent(this, ScreenRecordService::class.java).apply {
            action = ScreenRecordService.ACTION_START
            putExtra(ScreenRecordService.EXTRA_RESULT_CODE, resultCode)
            putExtra(ScreenRecordService.EXTRA_DATA, data)
            putExtra(ScreenRecordService.EXTRA_NAME, name)
            putExtra(ScreenRecordService.EXTRA_WIDTH, w)
            putExtra(ScreenRecordService.EXTRA_HEIGHT, h)
            putExtra(ScreenRecordService.EXTRA_DPI, metrics.densityDpi)
        }
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(svc) else startService(svc)

        onRecordingStateChanged(true)
        Toast.makeText(this, "Recording → DCIM/JS2.0/$name", Toast.LENGTH_LONG).show()
    }

    // REC oval: red + live MM:SS while recording, grey "REC" when idle.
    private fun onRecordingStateChanged(active: Boolean) {
        btnRecord.backgroundTintList = ColorStateList.valueOf(
            if (active) Color.parseColor("#D32F2F") else Color.parseColor("#9E9E9E"))
        if (active) {
            recStartMs = SystemClock.elapsedRealtime()
            mainHandler.post(recTimerRunnable)
        } else {
            mainHandler.removeCallbacks(recTimerRunnable)
            btnRecord.text = "REC"
        }
    }

    // -- Physical buttons (number keys switch; key 0 toggles recording) -------
    override fun onKeyDown(keyCode: Int, event: KeyEvent?): Boolean {
        val feed = when (keyCode) {
            KeyEvent.KEYCODE_1, KeyEvent.KEYCODE_NUMPAD_1 -> CameraFeed.FRONT
            KeyEvent.KEYCODE_2, KeyEvent.KEYCODE_NUMPAD_2 -> CameraFeed.BACK
            KeyEvent.KEYCODE_3, KeyEvent.KEYCODE_NUMPAD_3 -> CameraFeed.WRIST
            KeyEvent.KEYCODE_4, KeyEvent.KEYCODE_NUMPAD_4 -> CameraFeed.GRIP
            KeyEvent.KEYCODE_5, KeyEvent.KEYCODE_NUMPAD_5 -> CameraFeed.MAIN
            else -> null
        }
        if (feed != null) { switchTo(feed); return true }
        if (keyCode == KeyEvent.KEYCODE_0 || keyCode == KeyEvent.KEYCODE_NUMPAD_0) {
            toggleRecording(); return true
        }
        return super.onKeyDown(keyCode, event)
    }

    private fun showCover() { cover.visibility = View.VISIBLE }
    private fun hideCover() { cover.visibility = View.GONE }

    // -- Lifecycle ------------------------------------------------------------
    override fun onResume() {
        super.onResume()
        onRecordingStateChanged(ScreenRecordService.isRecording)
        // Re-apply the chosen intercom state (default OFF on first launch).
        applySound(soundState)
        // ONE socket on :9870 for battery + twin joints; released in onPause.
        telemetryReceiver.start()
        // SBUS mode feed on :9871 (separate port); also released in onPause.
        sbusReceiver.start()
        if (twinVisible) glView.onResume()    // stay hidden if toggled off
        if (surfaceReady) {
            pendingFeed = currentFeed
            showCover()
            beginPendingFeed()
        }
    }

    override fun onPause() {
        super.onPause()
        switching = false
        reconnecting = false
        mainHandler.removeCallbacks(forceStartRunnable)
        mainHandler.removeCallbacks(retryRunnable)
        rtspView.stop()
        glView.onPause()
        // Release :9870 so the dashboard can own it while we are backgrounded.
        telemetryReceiver.stop()
        // Release the SBUS mode socket too; stop the stale fallback timer.
        sbusReceiver.stop()
        mainHandler.removeCallbacks(modeStaleRunnable)
        gimbal.stopMove()
        // Drop mic/speaker in the background; the chosen state is re-applied in onResume.
        audioLink.setMicEnabled(false)
        audioLink.setSpeakerEnabled(false)
        showCover()
    }

    override fun onDestroy() {
        super.onDestroy()
        ScreenRecordService.onStateChange = null
        mainHandler.removeCallbacks(countdownRunnable)
        // App closing: drop fire mode (the server also auto-zeroes after the
        // socket drops, but stop explicitly so it happens immediately).
        HeartbeatClient.stop()
        gimbal.close()
        audioLink.close()
    }

    companion object {
        private const val LOW_LATENCY_SPS_REWRITE = false
        private const val START_TIMEOUT_MS = 800L
        private const val RETRY_DELAY_MS = 1500L
        private const val FIRE_COUNTDOWN_MS = 15_000L
        // SBUS mode feed is ~5 Hz (200 ms); ~12 missed packets → show "—".
        private const val MODE_STALE_MS = 2500L
    }
}
