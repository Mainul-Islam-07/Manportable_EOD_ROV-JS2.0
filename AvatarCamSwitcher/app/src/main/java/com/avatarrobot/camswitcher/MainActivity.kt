package com.avatarrobot.camswitcher

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.res.ColorStateList
import android.graphics.Color
import android.net.TrafficStats
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
import com.avatarrobot.camswitcher.video.GStreamerVideoView
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

    /** FIRING/AIM button lifecycle. */
    private enum class FireUi { IDLE, COUNTING, AIM }

    private lateinit var rtspView: GStreamerVideoView
    private lateinit var cover: View
    private lateinit var txtStatus: TextView      // centred "RECONNECTING …" indicator
    private lateinit var btnRecord: Button
    private lateinit var txtBattery: TextView
    private lateinit var txtBandwidth: TextView   // app download/upload, Mbps (beside battery)

    // Dropdowns
    private lateinit var btnCamera: Button
    private lateinit var btnZoom: Button
    private lateinit var btnSound: Button       // master intercom on/off
    private lateinit var btnTalk: Button         // hold-to-talk (momentary)
    private lateinit var cameraDropdown: HudDropdown<CameraFeed>
    private lateinit var zoomDropdown: HudDropdown<Int>

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
    private var twinVisible = false        // 3D twin starts closed; tap 3D to show
    private var twinSuspended = false      // twin overlay parked during a feed switch

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

    // Two-way intercom over UDP with the rover host (192.168.144.100:5555).
    private val audioLink by lazy { AudioLinkController(applicationContext) }
    // Master intercom on/off, plus momentary hold-to-talk. Half-duplex: when
    // sound is on, releasing TALK = LISTEN (Pi mic -> here), holding = TALK
    // (mic -> Pi speaker).
    private var soundOn = false
    private var talking = false

    // Digital zoom for the MAIN feed only (1x..4x), persisted while MAIN is active.
    private var mainZoom = 1

    private var videoW = 0
    private var videoH = 0

    // ---- State engine -------------------------------------------------------
    // The GStreamer pipeline (re)build serializes on its own GMainContext thread,
    // so there is no two-decoder-on-one-surface hazard: a switch just points the
    // view at the new feed and plays. Hence no `switching`/forceStart dance.
    private var currentFeed: CameraFeed = CameraFeed.MAIN
    private var pendingFeed: CameraFeed? = CameraFeed.MAIN
    private var surfaceReady = false
    private var reconnecting = false
    private var weakSignal = false     // native stall reported; last frame held

    private val mainHandler = Handler(Looper.getMainLooper())
    private val retryRunnable = Runnable { retryCurrentFeed() }
    // Fires if play() connects but no FIRST frame renders in time (camera booting,
    // bad codec). Mid-stream stalls after the first frame are caught by the native
    // frame-arrival watchdog (onStall/onOutage), not this one.
    private val firstFrameWatchdog = Runnable { enterReconnecting() }
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

    // Mic capture needs RECORD_AUDIO. Requested when the master is switched on so
    // hold-to-talk works immediately; LISTEN works regardless. If denied, sound
    // stays on for LISTEN only and holding TALK simply won't capture.
    private val micPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (!granted) {
            Toast.makeText(this, "Microphone permission denied — TALK disabled",
                Toast.LENGTH_SHORT).show()
        }
        applyAudio()
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
        txtStatus     = findViewById(R.id.txt_status)
        btnRecord     = findViewById(R.id.btn_record)
        txtBattery    = findViewById(R.id.txt_battery)
        txtBandwidth  = findViewById(R.id.txt_bandwidth)
        btnCamera     = findViewById(R.id.btn_camera)
        btnZoom       = findViewById(R.id.btn_zoom)
        btnSound      = findViewById(R.id.btn_sound)
        btnTalk       = findViewById(R.id.btn_talk)
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
        // Twin starts closed: keep the GL surface torn down until the user taps 3D.
        glView.visibility = View.GONE
        btn3d.setOnClickListener { toggleTwin() }

        // GStreamer pipeline events. onStall/onOutage/onRecovered implement the
        // ride-through UX: a brief stall holds the last frame under "WEAK SIGNAL";
        // only a real outage (or hard error) falls back to the reconnect path.
        rtspView.listener = object : GStreamerVideoView.Listener {
            override fun onFirstFrame() = runOnUiThread {
                reconnecting = false
                weakSignal = false
                mainHandler.removeCallbacks(retryRunnable)
                mainHandler.removeCallbacks(firstFrameWatchdog)
                hideReconnecting()
                hideCover()
                resumeTwinOverlay()
            }
            override fun onSizeChanged(width: Int, height: Int) = runOnUiThread {
                fitSurfaceToVideo(width, height)
            }
            override fun onError(message: String) = runOnUiThread { enterReconnecting() }
            override fun onStall() = runOnUiThread { onWeakSignal() }
            override fun onOutage() = runOnUiThread { enterReconnecting() }
            override fun onRecovered() = runOnUiThread { onSignalRecovered() }
        }

        // MainActivity tracks surface readiness and drives play; the view's own
        // holder callback binds the native surface (nativeSurfaceInit/Finalize).
        rtspView.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                surfaceReady = true
                showCover()
                beginPendingFeed()
            }
            override fun surfaceChanged(h: SurfaceHolder, fmt: Int, w: Int, ht: Int) { /* no-op */ }
            override fun surfaceDestroyed(holder: SurfaceHolder) {
                surfaceReady = false
                mainHandler.removeCallbacks(retryRunnable)
                mainHandler.removeCallbacks(firstFrameWatchdog)
                rtspView.stop()
            }
        })

        ScreenRecordService.onStateChange = { active -> runOnUiThread { onRecordingStateChanged(active) } }

        setupDropdowns()
        // If the mic can't open (device error), drop back to LISTEN (keep sound on).
        audioLink.onMicFailed = {
            talking = false
            applyAudio()
            Toast.makeText(this, "Couldn't start microphone", Toast.LENGTH_SHORT).show()
        }
        btnSound.setOnClickListener { toggleSound() }
        wireTalkButton()
        wireGimbalButtons()
        btnRecord.setOnClickListener { toggleRecording() }
        btnFire.setOnClickListener { onFirePressed() }

        cameraDropdown.setCurrent(currentFeed)
        updateMainOnlyControls(currentFeed)
        updateBattery(-1)                  // grey "--" until the first packet
        // 3D figure is hidden by default → 3D button starts grey (green when on).
        btn3d.backgroundTintList = ColorStateList.valueOf(Color.parseColor("#9E9E9E"))
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
    }

    // Battery chip (grey pill; red under 15%; "--" when unknown).
    private fun updateBattery(pct: Int) {
        val known = pct in 0..100
        val low = known && pct < 15
        txtBattery.text = if (known) "$pct%" else "--"
        txtBattery.backgroundTintList = ColorStateList.valueOf(
            if (low) 0xFFD32F2F.toInt() else 0xFF9E9E9E.toInt())
    }

    // -- Bandwidth meter (app download + upload over the app's UID, in Mbps) ---
    // Samples cumulative byte counters once a second and shows download (rx) and
    // upload (tx) separately next to the battery chip. Covers ALL of the app's
    // traffic: RTSP video, UDP telemetry/mode/audio, TCP fire/heartbeat, etc.
    private var lastRxBytes = -1L
    private var lastTxBytes = -1L
    private var lastNetTimeNs = 0L

    private val bandwidthRunnable = object : Runnable {
        override fun run() {
            updateBandwidth()
            mainHandler.postDelayed(this, 1000L)     // ~1 Hz refresh
        }
    }

    // (rx, tx) cumulative bytes for this app's UID; falls back to device totals
    // if per-UID stats are unsupported. null if unavailable.
    private fun appRxTxBytes(): Pair<Long, Long>? {
        val uid = android.os.Process.myUid()
        val rx = TrafficStats.getUidRxBytes(uid)
        val tx = TrafficStats.getUidTxBytes(uid)
        if (rx >= 0 && tx >= 0) return rx to tx
        val trx = TrafficStats.getTotalRxBytes()
        val ttx = TrafficStats.getTotalTxBytes()
        return if (trx >= 0 && ttx >= 0) trx to ttx else null
    }

    private fun updateBandwidth() {
        val now = SystemClock.elapsedRealtimeNanos()
        val cur = appRxTxBytes()
        if (cur == null) { txtBandwidth.text = "↓-- ↑-- Mbps"; return }
        val (rx, tx) = cur
        if (lastRxBytes < 0) {
            txtBandwidth.text = "↓-- ↑-- Mbps"          // seeding first sample
        } else if (now > lastNetTimeNs) {
            val dt = (now - lastNetTimeNs) / 1_000_000_000.0      // seconds
            val down = (rx - lastRxBytes) * 8.0 / 1_000_000.0 / dt
            val up   = (tx - lastTxBytes) * 8.0 / 1_000_000.0 / dt
            txtBandwidth.text = String.format(Locale.US, "↓%.1f ↑%.1f Mbps",
                maxOf(0.0, down), maxOf(0.0, up))
        }
        lastRxBytes = rx
        lastTxBytes = tx
        lastNetTimeNs = now
    }

    // -- Sound: master on/off + momentary hold-to-talk (half-duplex) ----------

    // Master button: toggles the whole intercom. Turning on requests RECORD_AUDIO
    // up front (so holding TALK works at once); LISTEN works either way.
    private fun toggleSound() {
        soundOn = !soundOn
        if (soundOn &&
            checkSelfPermission(android.Manifest.permission.RECORD_AUDIO)
            != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            micPermLauncher.launch(android.Manifest.permission.RECORD_AUDIO)
            return                    // applyAudio() runs from the permission callback
        }
        applyAudio()
    }

    // Hold-to-talk: press and hold = TALK, release = back to LISTEN. Same
    // ACTION_DOWN/UP pattern as the gimbal D-pad (holdToMove).
    @SuppressLint("ClickableViewAccessibility")
    private fun wireTalkButton() {
        btnTalk.setOnTouchListener { v, event ->
            when (event.actionMasked) {
                MotionEvent.ACTION_DOWN -> {
                    if (soundOn) { v.isPressed = true; talking = true; applyAudio() }
                    true
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    v.isPressed = false; talking = false; applyAudio(); true
                }
                else -> false
            }
        }
    }

    // Single place that enforces half-duplex from (soundOn, talking):
    //   off        -> mic off + speaker off
    //   on + hold  -> mic on  + speaker off   (TALK: mic -> Pi speaker)
    //   on + !hold -> speaker on + mic off    (LISTEN: Pi mic -> here)
    private fun applyAudio() {
        btnTalk.visibility = if (soundOn) View.VISIBLE else View.GONE
        btnSound.text = if (soundOn) "SOUND ON" else "SOUND"
        btnSound.backgroundTintList = ColorStateList.valueOf(
            if (soundOn) Color.parseColor("#2E7D32") else Color.parseColor("#9E9E9E"))

        if (!soundOn) {
            talking = false
            audioLink.setMicEnabled(false)
            audioLink.setSpeakerEnabled(false)
            return
        }
        btnTalk.backgroundTintList = ColorStateList.valueOf(
            if (talking) Color.parseColor("#D32F2F") else Color.parseColor("#9E9E9E"))
        if (talking) {
            audioLink.setMicEnabled(true)
            audioLink.setSpeakerEnabled(false)
        } else {
            audioLink.setSpeakerEnabled(true)
            audioLink.setMicEnabled(false)
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
        if (feed == currentFeed && rtspView.isStarted() && !reconnecting) return

        currentFeed = feed
        pendingFeed = feed
        reconnecting = false
        weakSignal = false
        mainHandler.removeCallbacks(retryRunnable)
        mainHandler.removeCallbacks(firstFrameWatchdog)
        hideReconnecting()
        cameraDropdown.setCurrent(feed)
        updateMainOnlyControls(feed)
        showCover()                 // black transition until the new feed's first frame
        suspendTwinOverlay()

        if (!surfaceReady) return
        beginPendingFeed()
    }

    private fun beginPendingFeed() {
        if (!surfaceReady) return
        val feed = pendingFeed ?: currentFeed
        pendingFeed = null
        // setFeed just records url+rotation; play() posts a pipeline (re)build on
        // the GStreamer context thread, which tears down any old pipeline first.
        rtspView.setFeed(feed.url, feed.rotation)
        rtspView.play()
        // Initial-connect guard: if no FIRST frame renders in time, reconnect.
        mainHandler.removeCallbacks(firstFrameWatchdog)
        mainHandler.postDelayed(firstFrameWatchdog, FIRST_FRAME_TIMEOUT_MS)
    }

    private fun retryCurrentFeed() {
        if (!surfaceReady) return
        pendingFeed = currentFeed
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

    private fun showReconnecting() {
        txtStatus.text = "RECONNECTING ${currentFeed.label}…"
        txtStatus.visibility = View.VISIBLE
    }
    private fun hideReconnecting() { txtStatus.visibility = View.GONE }

    // Brief stall (native onStall): the pipeline is intact and glimagesink holds
    // the last frame — just flag "WEAK SIGNAL", do NOT black out or rebuild.
    private fun onWeakSignal() {
        if (reconnecting) return
        weakSignal = true
        txtStatus.text = "WEAK SIGNAL ${currentFeed.label}…"
        txtStatus.visibility = View.VISIBLE
    }
    private fun onSignalRecovered() {
        weakSignal = false
        if (!reconnecting) hideReconnecting()
    }

    // Real outage / hard error / connected-but-no-first-frame: rebuilding the
    // pipeline blanks the surface anyway, so raise the black cover, park the twin,
    // show the indicator, and schedule the rebuild retry.
    private fun enterReconnecting() {
        if (!surfaceReady) return
        reconnecting = true
        showCover()
        suspendTwinOverlay()
        showReconnecting()
        mainHandler.removeCallbacks(retryRunnable)
        mainHandler.removeCallbacks(firstFrameWatchdog)
        mainHandler.postDelayed(retryRunnable, RETRY_DELAY_MS)
    }

    // The 3D twin is a media-overlay GL surface, which the compositor can float
    // ABOVE the plain-View cover — so during a feed switch its transparent pixels
    // leak the old feed's last frame in the twin rectangle. Take it down for the
    // black transition and restore it when the new feed's first frame renders.
    // Both are no-ops when 3D is off; the flag keeps pause/resume balanced.
    private fun suspendTwinOverlay() {
        if (!twinVisible || twinSuspended) return
        twinSuspended = true
        glView.onPause()
        glView.visibility = View.GONE
    }

    private fun resumeTwinOverlay() {
        if (!twinVisible || !twinSuspended) return
        twinSuspended = false
        glView.visibility = View.VISIBLE
        glView.onResume()
    }

    // -- Lifecycle ------------------------------------------------------------
    // The RTSP stream is torn down/restarted in onStop/onStart (true background),
    // NOT onPause/onResume. Transient overlays — the screen-record consent dialog,
    // permission prompts — only fire onPause, so the live video survives them
    // without a black reflash. onPause/onResume handle only the audio/telemetry
    // background housekeeping and the GL twin.
    override fun onResume() {
        super.onResume()
        onRecordingStateChanged(ScreenRecordService.isRecording)
        // Re-apply the intercom (default off on first launch). Never resume held.
        talking = false
        applyAudio()
        // Bandwidth meter: reset baseline so the first tick just seeds, then poll.
        lastRxBytes = -1L
        lastTxBytes = -1L
        mainHandler.post(bandwidthRunnable)
        // ONE socket on :9870 for battery + twin joints; released in onPause.
        telemetryReceiver.start()
        // SBUS mode feed on :9871 (separate port); also released in onPause.
        sbusReceiver.start()
        // Bring the GL twin back (paused in onPause). A restart after real
        // background runs on a fresh surface, so there is no stale frame to guard.
        if (twinVisible) {
            twinSuspended = false
            glView.visibility = View.VISIBLE
            glView.onResume()
        }
        // If the stream survived a transient pause, reveal it (the cover was raised
        // in onPause to blank the recents thumbnail).
        if (surfaceReady && rtspView.isStarted()) hideCover()
    }

    override fun onPause() {
        super.onPause()
        glView.onPause()
        twinSuspended = false                 // GL surface is paused; state reset for resume
        // Release :9870 so the dashboard can own it while we are backgrounded.
        telemetryReceiver.stop()
        // Release the SBUS mode socket too; stop the stale fallback timer.
        sbusReceiver.stop()
        mainHandler.removeCallbacks(modeStaleRunnable)
        mainHandler.removeCallbacks(bandwidthRunnable)
        gimbal.stopMove()
        // Drop mic/speaker in the background; state is re-applied in onResume.
        talking = false
        audioLink.setMicEnabled(false)
        audioLink.setSpeakerEnabled(false)
        // Blank the recents/multitasking thumbnail without stopping the stream.
        showCover()
    }

    override fun onStart() {
        super.onStart()
        // Restart if we were fully stopped but the surface survived; the usual
        // background path recreates the surface, which restarts via surfaceCreated.
        if (surfaceReady && !rtspView.isStarted()) {
            pendingFeed = currentFeed
            showCover()
            beginPendingFeed()
        }
    }

    override fun onStop() {
        super.onStop()
        // True background (screen off / home / a fully-obscuring activity like AIM).
        reconnecting = false
        weakSignal = false
        mainHandler.removeCallbacks(retryRunnable)
        mainHandler.removeCallbacks(firstFrameWatchdog)
        hideReconnecting()
        rtspView.stop()
        showCover()
    }

    override fun onDestroy() {
        super.onDestroy()
        ScreenRecordService.onStateChange = null
        mainHandler.removeCallbacks(countdownRunnable)
        rtspView.release()                     // destroy the native GStreamer context
        // App closing: drop fire mode (the server also auto-zeroes after the
        // socket drops, but stop explicitly so it happens immediately).
        HeartbeatClient.stop()
        gimbal.close()
        audioLink.close()
    }

    companion object {
        // Retry cadence for the outage/reconnect loop (pipeline rebuild).
        private const val RETRY_DELAY_MS = 3000L
        // Max wait for the FIRST frame after play() before we treat the feed as
        // failed (camera booting / bad codec). Mid-stream stalls are handled by
        // the native frame-arrival watchdog instead.
        private const val FIRST_FRAME_TIMEOUT_MS = 6000L
        private const val FIRE_COUNTDOWN_MS = 15_000L
        // SBUS mode feed is ~5 Hz (200 ms); ~12 missed packets → show "—".
        private const val MODE_STALE_MS = 2500L
    }
}
