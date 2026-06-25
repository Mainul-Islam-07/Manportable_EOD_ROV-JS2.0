package com.avatarrobot.camswitcher

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.hardware.display.DisplayManager
import android.hardware.display.VirtualDisplay
import android.media.MediaRecorder
import android.media.MediaScannerConnection
import android.media.projection.MediaProjection
import android.media.projection.MediaProjectionManager
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.ParcelFileDescriptor
import android.provider.MediaStore
import android.util.Log
import java.io.File
import java.io.IOException

/**
 * Records the whole app screen using MediaProjection -> VirtualDisplay -> MediaRecorder.
 *
 * Why a foreground service: from Android 10 a MediaProjection capture must run
 * inside a foreground service, and from Android 14 that service must declare
 * foregroundServiceType="mediaProjection", call startForeground() with that type
 * BEFORE getMediaProjection(), and register a MediaProjection.Callback before
 * creating the virtual display. All of that is handled here.
 *
 * Because it captures the composited display, camera switches happen *inside* one
 * continuous recording — switching no longer stops it.
 */
class ScreenRecordService : Service() {

    private var projection: MediaProjection? = null
    private var virtualDisplay: VirtualDisplay? = null
    private var recorder: MediaRecorder? = null

    // Output target. On API 29+ we record into a MediaStore "pending" entry under
    // DCIM/JS2.0 via a file descriptor; on older devices we write the public
    // DCIM/JS2.0 file directly and media-scan it afterwards.
    private var outputUri: Uri? = null
    private var outputPfd: ParcelFileDescriptor? = null
    private var outputPath: String? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> handleStart(intent)
            ACTION_STOP  -> handleStop()
        }
        return START_NOT_STICKY
    }

    private fun handleStart(intent: Intent) {
        val resultCode = intent.getIntExtra(EXTRA_RESULT_CODE, 0)
        val data: Intent? = if (Build.VERSION.SDK_INT >= 33)
            intent.getParcelableExtra(EXTRA_DATA, Intent::class.java)
        else @Suppress("DEPRECATION") intent.getParcelableExtra(EXTRA_DATA)
        val name = intent.getStringExtra(EXTRA_NAME)
        val width = intent.getIntExtra(EXTRA_WIDTH, 1280)
        val height = intent.getIntExtra(EXTRA_HEIGHT, 720)
        val dpi = intent.getIntExtra(EXTRA_DPI, 320)
        if (data == null || name == null) { stopSelf(); return }

        // MUST be foreground (with mediaProjection type) before getMediaProjection().
        startAsForeground()

        val mpm = getSystemService(Context.MEDIA_PROJECTION_SERVICE) as MediaProjectionManager
        projection = mpm.getMediaProjection(resultCode, data)
        if (projection == null) { handleStop(); return }

        // Required on API 34+: register a callback before creating the display.
        projection!!.registerCallback(object : MediaProjection.Callback() {
            override fun onStop() { handleStop() }
        }, null)

        try {
            recorder = buildRecorder(width, height).apply {
                openOutput(this, name)
                prepare()
            }
            virtualDisplay = projection!!.createVirtualDisplay(
                "AvatarCamRec", width, height, dpi,
                DisplayManager.VIRTUAL_DISPLAY_FLAG_AUTO_MIRROR,
                recorder!!.surface, null, null
            )
            recorder!!.start()
            isRecording = true
            notifyState(true)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start screen recording", e)
            handleStop()
        }
    }

    private fun buildRecorder(width: Int, height: Int): MediaRecorder {
        val r = if (Build.VERSION.SDK_INT >= 31) MediaRecorder(this)
                else @Suppress("DEPRECATION") MediaRecorder()
        r.setVideoSource(MediaRecorder.VideoSource.SURFACE)
        r.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4)
        r.setVideoEncoder(MediaRecorder.VideoEncoder.H264)
        r.setVideoSize(width, height)
        r.setVideoFrameRate(30)
        r.setVideoEncodingBitRate(8_000_000)
        return r
    }

    // Point the recorder at DCIM/JS2.0/<name>. API 29+ uses a MediaStore
    // pending entry (no storage permission); older devices write the public file.
    private fun openOutput(r: MediaRecorder, name: String) {
        if (Build.VERSION.SDK_INT >= 29) {
            val values = ContentValues().apply {
                put(MediaStore.MediaColumns.DISPLAY_NAME, name)
                put(MediaStore.MediaColumns.MIME_TYPE, "video/mp4")
                put(MediaStore.MediaColumns.RELATIVE_PATH, "${Environment.DIRECTORY_DCIM}/JS2.0")
                put(MediaStore.MediaColumns.IS_PENDING, 1)
            }
            val uri = contentResolver.insert(MediaStore.Video.Media.EXTERNAL_CONTENT_URI, values)
                ?: throw IOException("MediaStore insert failed")
            outputUri = uri
            val pfd = contentResolver.openFileDescriptor(uri, "w")
                ?: throw IOException("openFileDescriptor failed")
            outputPfd = pfd
            r.setOutputFile(pfd.fileDescriptor)
        } else {
            val dir = File(
                Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM),
                "JS2.0"
            ).apply { mkdirs() }
            val file = File(dir, name)
            outputPath = file.absolutePath
            @Suppress("DEPRECATION") r.setOutputFile(file.absolutePath)
        }
    }

    // Publish the finished file: mark the MediaStore entry no-longer-pending (so it
    // becomes visible in the gallery), or media-scan the legacy public file.
    private fun finalizeOutput() {
        if (Build.VERSION.SDK_INT >= 29) {
            try { outputPfd?.close() } catch (_: Exception) {}
            outputPfd = null
            outputUri?.let { uri ->
                try {
                    contentResolver.update(
                        uri,
                        ContentValues().apply { put(MediaStore.MediaColumns.IS_PENDING, 0) },
                        null, null
                    )
                } catch (_: Exception) {}
            }
            outputUri = null
        } else {
            outputPath?.let {
                MediaScannerConnection.scanFile(this, arrayOf(it), arrayOf("video/mp4"), null)
            }
            outputPath = null
        }
    }

    private fun handleStop() {
        try { recorder?.stop() } catch (_: Exception) {}
        try { recorder?.reset(); recorder?.release() } catch (_: Exception) {}
        recorder = null
        virtualDisplay?.release(); virtualDisplay = null
        try { projection?.stop() } catch (_: Exception) {}
        projection = null
        finalizeOutput()
        val wasRecording = isRecording
        isRecording = false
        if (wasRecording) notifyState(false)
        if (Build.VERSION.SDK_INT >= 24) stopForeground(STOP_FOREGROUND_REMOVE)
        else @Suppress("DEPRECATION") stopForeground(true)
        stopSelf()
    }

    private fun startAsForeground() {
        val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= 26) {
            nm.createNotificationChannel(
                NotificationChannel(CHANNEL, "Screen recording", NotificationManager.IMPORTANCE_LOW)
            )
        }
        val notif: Notification = (if (Build.VERSION.SDK_INT >= 26)
            Notification.Builder(this, CHANNEL)
        else @Suppress("DEPRECATION") Notification.Builder(this))
            .setContentTitle("JS2.0")
            .setContentText("Recording screen…")
            .setSmallIcon(R.drawable.ic_launcher)
            .setOngoing(true)
            .build()

        if (Build.VERSION.SDK_INT >= 29) {
            startForeground(NOTIF_ID, notif, ServiceInfo.FOREGROUND_SERVICE_TYPE_MEDIA_PROJECTION)
        } else {
            startForeground(NOTIF_ID, notif)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        handleStop()
    }

    // Push the real start/stop to listeners on the main thread (these are called
    // from the service/binder thread). Lets the UI reflect the true state instead
    // of polling the asynchronously-set isRecording flag.
    private fun notifyState(active: Boolean) {
        val cb = onStateChange ?: return
        Handler(Looper.getMainLooper()).post { cb(active) }
    }

    companion object {
        @Volatile var isRecording: Boolean = false

        // Set by the foreground UI to be notified when recording truly starts/stops.
        @Volatile var onStateChange: ((Boolean) -> Unit)? = null

        const val ACTION_START = "com.avatarrobot.camswitcher.START"
        const val ACTION_STOP = "com.avatarrobot.camswitcher.STOP"
        const val EXTRA_RESULT_CODE = "result_code"
        const val EXTRA_DATA = "data"
        const val EXTRA_NAME = "name"
        const val EXTRA_WIDTH = "width"
        const val EXTRA_HEIGHT = "height"
        const val EXTRA_DPI = "dpi"

        private const val CHANNEL = "screen_rec"
        private const val NOTIF_ID = 42
        private const val TAG = "ScreenRecordService"
    }
}
