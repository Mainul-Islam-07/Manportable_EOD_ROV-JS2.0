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
import android.view.WindowManager
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import com.alexvas.rtsp.widget.RtspStatusListener
import com.alexvas.rtsp.widget.RtspSurfaceView
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Single-Activity, 5-feed RTSP switcher + screen recorder for the Avatar rover.
 *
 * - Switching is serialized via onRtspStatusDisconnected so two decoders never
 *   share the surface (no cross-feed flicker / mosaic); a black cover masks the
 *   transient and lifts on the first clean frame.
 * - FRONT/BACK are upside-down mounted → flipped 180 via per-feed videoRotation.
 * - Recording captures the WHOLE screen via MediaProjection, so switching cameras
 *   happens inside one continuous file and does NOT stop the recording.
 */
class MainActivity : AppCompatActivity() {

    enum class CameraFeed(val label: String, val url: String, val rotation: Int) {
        FRONT("FRONT", "rtsp://192.168.144.65:8554/main.264", 180),  // upside-down mount
        BACK ("BACK",  "rtsp://192.168.144.66:8554/main.264", 180),  // upside-down mount
        WRIST("WRIST", "rtsp://192.168.144.67:8554/main.264", 0),
        GRIP ("GRIP",  "rtsp://192.168.144.68:8554/main.264", 0),
        MAIN ("MAIN",  "rtsp://192.168.144.25:8554/main.264", 0)
    }

    private lateinit var rtspView: RtspSurfaceView
    private lateinit var cover: View
    private lateinit var btnRecord: Button
    private lateinit var txtRec: TextView
    private lateinit var txtBattery: TextView
    private lateinit var zoomBar: View
    private lateinit var gimbalPanel: View
    private val buttons = mutableMapOf<CameraFeed, Button>()
    private val zoomButtons = mutableMapOf<Int, Button>()

    // SIYI A8 mini pan/tilt over UDP (192.168.144.25:37260). MAIN-feed only.
    private val gimbal = SiyiGimbalController()

    // Main-pack battery % from the rover telemetry bridge (UDP :9870). Bound while
    // foreground, released in onPause so the dashboard app can own the port when we
    // background (never both foreground at once).
    private val batteryReceiver by lazy {
        BatteryTelemetryReceiver(onBattery = { pct -> runOnUiThread { updateBattery(pct) } })
    }

    // Two-way intercom over UDP with the rover host (192.168.144.10:5555).
    // Half-duplex push-to-talk: resting = listening (speaker on), pressed = talking
    // (mic on, speaker muted).
    private val audioLink by lazy { AudioLinkController(applicationContext) }
    private lateinit var btnTalk: Button
    private var talking = false

    // Digital zoom for the MAIN feed only: a center-pivoted view scale (1x..4x).
    // Persists while MAIN is selected; the surface is reset to 1x on other feeds.
    private var mainZoom = 1

    // Last decoded frame size, used to letterbox the surface to the video's aspect
    // ratio (see fitSurfaceToVideo). 0 until the first onRtspFrameSizeChanged.
    private var videoW = 0
    private var videoH = 0

    // ---- State engine -------------------------------------------------------
    private var currentFeed: CameraFeed = CameraFeed.MAIN
    private var pendingFeed: CameraFeed? = CameraFeed.MAIN
    private var surfaceReady = false
    private var switching = false

    // Auto-reconnect: while the activity is in the foreground, a failed feed is
    // re-initialized on a timer until a frame finally renders. Cleared the moment
    // we get a first frame, switch feeds, or leave the foreground.
    private var reconnecting = false

    private val mainHandler = Handler(Looper.getMainLooper())
    private val forceStartRunnable = Runnable { beginPendingFeed() }
    private val retryRunnable = Runnable { retryCurrentFeed() }

    // ---- Recording ----------------------------------------------------------
    private var recStartMs = 0L
    private var lastRecordName: String? = null
    private val recTimerRunnable = object : Runnable {
        override fun run() {
            val s = (SystemClock.elapsedRealtime() - recStartMs) / 1000
            txtRec.text = String.format(Locale.US, "\u25CF REC  %02d:%02d", s / 60, s % 60)
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

    // Optional: lets the recording notification show on Android 13+.
    private val notifPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { /* recording still works if denied; notification is just suppressed */ }

    // Android 9 and below need this to write recordings into public DCIM; API 29+
    // uses MediaStore and needs no permission.
    private val storagePermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { /* if denied on legacy, the save into DCIM will simply fail */ }

    // Mic capture needs RECORD_AUDIO. Requested the first time the talk button is
    // pressed; the user then presses again to talk (the grant dialog consumes the
    // initial press).
    private val micPermLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted ->
        if (!granted) Toast.makeText(this, "Microphone permission denied", Toast.LENGTH_SHORT).show()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)

        rtspView  = findViewById(R.id.rtsp_view)
        cover     = findViewById(R.id.transition_cover)
        btnRecord = findViewById(R.id.btn_record)
        txtRec    = findViewById(R.id.txt_rec)
        txtBattery = findViewById(R.id.txt_battery)
        zoomBar     = findViewById(R.id.zoom_bar)
        gimbalPanel = findViewById(R.id.gimbal_panel)
        btnTalk     = findViewById(R.id.btn_talk)

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
                // Toast only once per outage so a long retry loop doesn't spam.
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

        // The service is the source of truth for recording state: update the button
        // the moment recording actually starts/stops (start is async, so the button
        // can't be driven reliably from the click path alone).
        ScreenRecordService.onStateChange = { active -> runOnUiThread { onRecordingStateChanged(active) } }

        wireButtons()
        wireZoomButtons()
        wireGimbalButtons()
        wireTalkButton()
        btnRecord.setOnClickListener { toggleRecording() }
        highlightActive(currentFeed)
        updateZoomControls(currentFeed)
        updateBattery(-1)                  // grey "--" until the first packet
        showCover()
    }

    // Main-pack charge chip (top-right): white percentage on a grey pill, the pill
    // turning red only when the charge drops under 15%. Unknown (no fresh reading)
    // stays grey and shows "--".
    private fun updateBattery(pct: Int) {
        val known = pct in 0..100
        val low = known && pct < 15
        txtBattery.text = if (known) "$pct%" else "--"
        txtBattery.backgroundTintList = ColorStateList.valueOf(
            if (low) 0xFFD32F2F.toInt() else 0xFF9E9E9E.toInt())
    }

    private fun wireButtons() {
        buttons[CameraFeed.FRONT] = findViewById(R.id.btn_front)
        buttons[CameraFeed.BACK]  = findViewById(R.id.btn_back)
        buttons[CameraFeed.WRIST] = findViewById(R.id.btn_wrist)
        buttons[CameraFeed.GRIP]  = findViewById(R.id.btn_grip)
        buttons[CameraFeed.MAIN]  = findViewById(R.id.btn_main)
        buttons.forEach { (feed, button) -> button.setOnClickListener { switchTo(feed) } }
    }

    private fun wireZoomButtons() {
        zoomButtons[1] = findViewById(R.id.btn_zoom_1)
        zoomButtons[2] = findViewById(R.id.btn_zoom_2)
        zoomButtons[3] = findViewById(R.id.btn_zoom_3)
        zoomButtons[4] = findViewById(R.id.btn_zoom_4)
        zoomButtons.forEach { (factor, button) -> button.setOnClickListener { applyZoom(factor) } }
    }

    // Hold-to-move gimbal D-pad: press an arrow → move the A8 mini at MOVE_SPEED;
    // release (or the touch is cancelled) → stop. Center recenters the gimbal.
    // Sign mapping: right = +yaw, up = +pitch; flip a sign here if an axis is inverted.
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
                MotionEvent.ACTION_DOWN -> {
                    v.isPressed = true
                    gimbal.startMove(yaw, pitch)
                    true
                }
                MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                    v.isPressed = false
                    gimbal.stopMove()
                    true
                }
                else -> false
            }
        }
    }

    // -- Intercom audio (single talk toggle) ----------------------------------
    // Half-duplex latching toggle: tap to talk (mic on + incoming audio muted),
    // tap again to listen (mic off + incoming audio playing).
    private fun wireTalkButton() {
        btnTalk.setOnClickListener { if (talking) startListening() else startTalking() }
        // If the mic can't open (device error), revert to listening on the UI thread.
        audioLink.onMicFailed = {
            startListening()
            Toast.makeText(this, "Couldn't start microphone", Toast.LENGTH_SHORT).show()
        }
        updateTalkButton()
    }

    // Enter listening: speaker on, mic off. The default/resting intercom state.
    // Enable the speaker BEFORE disabling the mic so communication-mode audio
    // routing stays set across the switch (avoids a slow HAL teardown/re-setup).
    private fun startListening() {
        talking = false
        updateTalkButton()                 // optimistic: UI flips instantly
        audioLink.setSpeakerEnabled(true)
        audioLink.setMicEnabled(false)
    }

    // Press: switch to talking — open the mic, mute the speaker. Mic first (same
    // sticky-routing reason as above). Needs RECORD_AUDIO; if not yet granted,
    // request it and stay listening (user presses again).
    private fun startTalking() {
        if (checkSelfPermission(android.Manifest.permission.RECORD_AUDIO)
            != android.content.pm.PackageManager.PERMISSION_GRANTED) {
            micPermLauncher.launch(android.Manifest.permission.RECORD_AUDIO)
            return
        }
        talking = true
        updateTalkButton()                 // optimistic: UI flips instantly
        audioLink.setMicEnabled(true)
        audioLink.setSpeakerEnabled(false)
    }

    // Green "Talking" while pressed, grey "Listening" at rest (REC button look).
    private fun updateTalkButton() {
        btnTalk.text = if (talking) "🎤 Talking" else "🔈 Listening"
        btnTalk.backgroundTintList = ColorStateList.valueOf(
            if (talking) Color.parseColor("#2E7D32") else Color.parseColor("#9E9E9E"))
    }

    // Size the surface to the video's aspect ratio (Fit/letterbox), centered. The
    // RtspSurfaceView stretches its content to its bounds, so making the bounds match
    // the frame's proportions is what removes the apparent zoom/distortion at 1x.
    // Digital zoom (scaleX/scaleY) still acts on this resized view from its center.
    private fun fitSurfaceToVideo(w: Int, h: Int) {
        if (w <= 0 || h <= 0) return
        videoW = w; videoH = h
        val parent = rtspView.parent as? View ?: return
        val pw = parent.width; val ph = parent.height
        if (pw == 0 || ph == 0) {                 // not laid out yet — retry next frame
            rtspView.post { fitSurfaceToVideo(w, h) }
            return
        }
        val scale = minOf(pw / w.toFloat(), ph / h.toFloat())
        val lp = rtspView.layoutParams
        lp.width = (w * scale).toInt()
        lp.height = (h * scale).toInt()
        rtspView.layoutParams = lp                // all 4 constraints remain → centered
    }

    // Scales the video surface from its center. SurfaceView honors view-level
    // scaleX/scaleY, so 2x crops to the middle half of the frame, etc.
    private fun applyZoom(factor: Int) {
        mainZoom = factor
        rtspView.scaleX = factor.toFloat()
        rtspView.scaleY = factor.toFloat()
        zoomButtons.forEach { (f, button) -> button.isSelected = (f == factor) }
    }

    // Zoom + gimbal are MAIN-only controls: show the right panel and (re)apply the
    // saved zoom on MAIN; hide it, reset the surface to 1x, and halt any gimbal
    // motion on every other feed.
    private fun updateZoomControls(feed: CameraFeed) {
        if (feed == CameraFeed.MAIN) {
            zoomBar.visibility = View.VISIBLE
            gimbalPanel.visibility = View.VISIBLE
            applyZoom(mainZoom)
        } else {
            zoomBar.visibility = View.GONE
            gimbalPanel.visibility = View.GONE
            rtspView.scaleX = 1f
            rtspView.scaleY = 1f
            gimbal.stopMove()
        }
    }

    // -- Camera switching (does NOT touch recording) --------------------------
    private fun switchTo(feed: CameraFeed) {
        if (feed == currentFeed && rtspView.isStarted() && !switching) return

        currentFeed = feed
        pendingFeed = feed
        // Abandon any in-flight reconnect for the previous feed.
        reconnecting = false
        mainHandler.removeCallbacks(retryRunnable)
        highlightActive(feed)
        updateZoomControls(feed)
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
        rtspView.videoRotation = feed.rotation     // 180 for upside-down-mounted cams
        rtspView.init(Uri.parse(feed.url))
        rtspView.start(requestVideo = true, requestAudio = false)
    }

    // Re-arm the CURRENT feed after a failure. Keeps the cover up; the retry loop
    // ends naturally when onRtspFirstFrameRendered fires (or the user leaves/switches).
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
            // Pops the system "Start recording?" consent dialog. Result → launcher.
            val mpm = getSystemService(Context.MEDIA_PROJECTION_SERVICE) as MediaProjectionManager
            projectionLauncher.launch(mpm.createScreenCaptureIntent())
        }
    }

    private fun startScreenRecording(resultCode: Int, data: Intent) {
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val name = "Live_Feed_$stamp.mp4"
        lastRecordName = name

        val metrics = resources.displayMetrics
        val w = metrics.widthPixels - (metrics.widthPixels % 2)   // encoder needs even dims
        val h = metrics.heightPixels - (metrics.heightPixels % 2)

        // Recordings land in the public DCIM/JS2.0 album (see ScreenRecordService).
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

    private fun onRecordingStateChanged(active: Boolean) {
        // Red while recording, grey when idle.
        btnRecord.backgroundTintList = ColorStateList.valueOf(
            if (active) Color.parseColor("#D32F2F") else Color.parseColor("#9E9E9E"))
        if (active) {
            recStartMs = SystemClock.elapsedRealtime()
            btnRecord.text = "\u25CF Rec On"
            txtRec.visibility = View.VISIBLE
            mainHandler.post(recTimerRunnable)
        } else {
            btnRecord.text = "\u25CF Rec Off"
            txtRec.visibility = View.GONE
            mainHandler.removeCallbacks(recTimerRunnable)
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

    // -- UI helpers -----------------------------------------------------------
    private fun highlightActive(active: CameraFeed) {
        buttons.forEach { (feed, button) -> button.isSelected = (feed == active) }
    }

    private fun showCover() { cover.visibility = View.VISIBLE }
    private fun hideCover() { cover.visibility = View.GONE }

    // -- Lifecycle ------------------------------------------------------------
    override fun onResume() {
        super.onResume()
        // Reconcile to the true state (e.g. recording stopped from the system bar
        // while we were away). The start-race transient is corrected separately by
        // ScreenRecordService.onStateChange, so this never stomps a fresh start.
        onRecordingStateChanged(ScreenRecordService.isRecording)
        // Resting intercom state: audio on, mic off (until the user presses talk).
        startListening()
        // Listen for battery telemetry while foreground; released in onPause.
        batteryReceiver.start()
        if (surfaceReady) {
            pendingFeed = currentFeed
            showCover()
            beginPendingFeed()
        }
    }

    override fun onPause() {
        super.onPause()
        // Stream socket is released, but the screen recording keeps running.
        switching = false
        reconnecting = false
        mainHandler.removeCallbacks(forceStartRunnable)
        mainHandler.removeCallbacks(retryRunnable)
        rtspView.stop()
        // Release the telemetry socket so the dashboard can own :9870 when we leave.
        batteryReceiver.stop()
        // Halt the gimbal so it never keeps slewing while the app is backgrounded.
        gimbal.stopMove()
        // Release the mic/speaker so we don't hold the mic or play audio in the
        // background; listening resumes in onResume.
        talking = false
        audioLink.setMicEnabled(false)
        audioLink.setSpeakerEnabled(false)
        updateTalkButton()
        showCover()
    }

    override fun onDestroy() {
        super.onDestroy()
        ScreenRecordService.onStateChange = null
        gimbal.close()
        audioLink.close()
    }

    companion object {
        private const val LOW_LATENCY_SPS_REWRITE = false
        private const val START_TIMEOUT_MS = 800L
        // Wait between reconnect attempts after a stream failure.
        private const val RETRY_DELAY_MS = 1500L
    }
}
