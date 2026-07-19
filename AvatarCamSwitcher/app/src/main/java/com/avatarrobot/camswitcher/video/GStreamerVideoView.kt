package com.avatarrobot.camswitcher.video

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.AttributeSet
import android.view.SurfaceHolder
import android.view.SurfaceView
import org.freedesktop.gstreamer.GStreamer

/**
 * A [SurfaceView] backed by a native GStreamer pipeline (see jni/gst_backend.c).
 *
 * Replaces the old RtspSurfaceView. The pipeline pulls RTP over UDP with a small
 * jitter buffer and hardware decode, so the feed rides through packet loss with
 * artifacts instead of freezing. Native events are marshalled onto the UI thread
 * and delivered through [Listener].
 *
 * Usage: [setFeed] then [play]; [stop] tears the pipeline down (reusable);
 * [release] destroys the native context (call from Activity onDestroy).
 */
class GStreamerVideoView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : SurfaceView(context, attrs, defStyleAttr), SurfaceHolder.Callback {

    interface Listener {
        fun onFirstFrame()
        fun onSizeChanged(width: Int, height: Int)
        fun onError(message: String)
        fun onStall()       // frames stopped ~1.5 s — keep last frame, show WEAK SIGNAL
        fun onOutage()      // frames stopped ~5 s — reconnect
        fun onRecovered()   // frames resumed after a stall
    }

    var listener: Listener? = null

    // Written from JNI (native CustomData pointer). Do not touch from Kotlin.
    @Suppress("unused")
    private var nativeCustomData: Long = 0

    private val ui = Handler(Looper.getMainLooper())
    private var started = false

    init {
        try {
            GStreamer.init(context)
        } catch (e: Exception) {
            throw RuntimeException("GStreamer.init failed: ${e.message}", e)
        }
        nativeInit()
        holder.addCallback(this)
    }

    // -- Public API -----------------------------------------------------------
    /** Point the pipeline at a feed. [rotation] is 0 or 180 (upside-down mounts). */
    fun setFeed(url: String, rotation: Int) = nativeSetUri(url, rotation)

    fun play() { started = true; nativePlay() }
    fun stop() { started = false; nativeStop() }
    fun isStarted(): Boolean = started

    /** Destroy the native pipeline + main loop. Call from Activity onDestroy. */
    fun release() {
        started = false
        nativeFinalize()
    }

    // -- SurfaceHolder.Callback ----------------------------------------------
    override fun surfaceCreated(h: SurfaceHolder) { /* handled in surfaceChanged */ }

    override fun surfaceChanged(h: SurfaceHolder, format: Int, width: Int, height: Int) {
        nativeSurfaceInit(h.surface)
    }

    override fun surfaceDestroyed(h: SurfaceHolder) {
        nativeSurfaceFinalize()
    }

    // -- Native up-calls (invoked on GStreamer threads → hop to UI) ----------
    // Block bodies (return Unit → JNI "()V"); an expression body would return
    // Handler.post's Boolean, giving "()Z" and a NoSuchMethodError from GetMethodID.
    @Suppress("unused")
    private fun onFirstFrameFromNative() { ui.post { listener?.onFirstFrame() } }
    @Suppress("unused")
    private fun onSizeChangedFromNative(w: Int, h: Int) { ui.post { listener?.onSizeChanged(w, h) } }
    @Suppress("unused")
    private fun onErrorFromNative(message: String) { ui.post { listener?.onError(message) } }
    @Suppress("unused")
    private fun onStallFromNative() { ui.post { listener?.onStall() } }
    @Suppress("unused")
    private fun onOutageFromNative() { ui.post { listener?.onOutage() } }
    @Suppress("unused")
    private fun onRecoveredFromNative() { ui.post { listener?.onRecovered() } }

    // -- Native methods (implemented in jni/gst_backend.c) -------------------
    private external fun nativeInit()
    private external fun nativeFinalize()
    private external fun nativeSetUri(uri: String, rotation: Int)
    private external fun nativePlay()
    private external fun nativeStop()
    private external fun nativeSurfaceInit(surface: Any)
    private external fun nativeSurfaceFinalize()

    companion object {
        init {
            // JNI methods bind by name (Java_..._GStreamerVideoView_native*), and
            // the field/method IDs are cached in nativeInit — so no class-init hook.
            System.loadLibrary("gstreamer_android")
            System.loadLibrary("gst_backend")
        }
    }
}
