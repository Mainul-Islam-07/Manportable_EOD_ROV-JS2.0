package com.avatarrobot.camswitcher

import android.annotation.SuppressLint
import android.content.Context
import android.media.AudioAttributes
import android.media.AudioDeviceCallback
import android.media.AudioDeviceInfo
import android.media.AudioFormat
import android.media.AudioManager
import android.media.AudioRecord
import android.media.AudioTrack
import android.media.MediaRecorder
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import android.media.audiofx.AcousticEchoCanceler
import android.media.audiofx.NoiseSuppressor
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.SocketTimeoutException

/**
 * Full-duplex voice intercom over UDP with the rover host at 192.168.144.100:5555.
 *
 * This is the Android port of the operator-station Python script (sounddevice +
 * UDP). One [DatagramSocket] bound to :5555 is shared for both directions, just
 * like the Python single-socket design:
 *   - capture thread:  AudioRecord.read  -> socket.send(peer)
 *   - playback thread: socket.receive    -> AudioTrack.write
 *
 * Speaker (playback) and microphone (capture) are independent toggles; either
 * can run alone. Unlike [SiyiGimbalController] (periodic Handler posts), audio
 * uses dedicated blocking threads because the read/receive calls block on a
 * continuous stream. Mic uses VOICE_COMMUNICATION + AcousticEchoCanceler so the
 * platform cancels the speaker leaking back into the mic.
 *
 * Wire format matches the Python side exactly: 16 kHz, mono, signed 16-bit PCM,
 * little-endian (native on ARM), 256-sample (512-byte) frames.
 */
class AudioLinkController(context: Context) {

    private val audioManager =
        context.applicationContext.getSystemService(Context.AUDIO_SERVICE) as AudioManager

    private val peer = InetSocketAddress(InetAddress.getByName(HOST), PORT)

    // All start/stop/routing work runs here, off the UI thread, so toggling
    // talk/listen never stalls the UI (AudioRecord/AudioTrack create+release and
    // audio-mode routing are slow). Mirrors SiyiGimbalController's "siyi-net".
    private val ctlThread = HandlerThread("audio-ctl").apply { start() }
    private val ctlHandler = Handler(ctlThread.looper)
    private val mainHandler = Handler(Looper.getMainLooper())

    /** Invoked (on the main thread) if the mic fails to open after setMicEnabled(true). */
    var onMicFailed: (() -> Unit)? = null

    // Shared by both directions; created on first enable, closed in close().
    private var socket: DatagramSocket? = null

    // Read by the capture/playback loop threads, written on ctlThread → volatile.
    @Volatile private var micEnabled = false
    @Volatile private var speakerEnabled = false

    private var captureThread: Thread? = null
    private var playbackThread: Thread? = null

    private var audioRecord: AudioRecord? = null
    private var audioTrack: AudioTrack? = null
    private var aec: AcousticEchoCanceler? = null
    private var ns: NoiseSuppressor? = null

    /**
     * Invoked (on the main thread) when the external USB mic is plugged in or pulled
     * out, so the HUD can show whether a mic is available. Fires on registration too,
     * giving the initial state.
     */
    var onMicPresenceChanged: ((present: Boolean) -> Unit)? = null

    // Live USB-mic presence tracking. Android delivers add/remove on ctlThread (we pass
    // ctlHandler), so the restart it triggers runs on the same thread as every other
    // start/stop and needs no extra synchronisation.
    private val deviceCallback = object : AudioDeviceCallback() {
        override fun onAudioDevicesAdded(added: Array<out AudioDeviceInfo>?) {
            if (added?.any { it.isSource && isUsbAudio(it.type) } == true) onUsbMicPresenceChanged()
        }
        override fun onAudioDevicesRemoved(removed: Array<out AudioDeviceInfo>?) {
            if (removed?.any { it.isSource && isUsbAudio(it.type) } == true) onUsbMicPresenceChanged()
        }
    }

    init {
        audioManager.registerAudioDeviceCallback(deviceCallback, ctlHandler)
    }

    // -- Public toggles (return immediately; work is posted to ctlThread) ------

    /** Enable/disable playback of incoming audio (speaker). Idempotent. */
    fun setSpeakerEnabled(enabled: Boolean) {
        ctlHandler.post { applySpeaker(enabled) }
    }

    /**
     * Enable/disable the microphone (capture + send). Idempotent. The caller must
     * hold RECORD_AUDIO before passing true. If capture can't start on the device,
     * [onMicFailed] is invoked on the main thread.
     */
    fun setMicEnabled(enabled: Boolean) {
        ctlHandler.post { applyMic(enabled) }
    }

    /**
     * The USB mic was plugged in or pulled out. Capture is external-mic-only, so there is
     * nothing to fall back to: if it disappears mid-TALK we stop capturing and tell the
     * HUD, and if it appears while TALK is held we start capturing on it immediately.
     */
    private fun onUsbMicPresenceChanged() {
        val present = findUsbMic() != null
        Log.i(TAG, "USB mic ${if (present) "attached" else "detached"}")
        mainHandler.post { onMicPresenceChanged?.invoke(present) }

        if (!micEnabled) return
        stopCapture()
        if (present && startCapture()) return

        // Pulled out (or it won't open) while transmitting — there is no built-in
        // fallback by design, so end the capture and surface it.
        micEnabled = false
        mainHandler.post { onMicFailed?.invoke() }
    }

    /** Release everything: stop both directions, close the socket, join threads. */
    fun close() {
        try { audioManager.unregisterAudioDeviceCallback(deviceCallback) } catch (_: Exception) {}
        ctlHandler.post {
            applySpeaker(false)
            applyMic(false)
            socket?.close()
            socket = null
        }
        ctlThread.quitSafely()
    }

    // -- Apply on ctlThread ---------------------------------------------------

    private fun applySpeaker(enabled: Boolean) {
        if (enabled == speakerEnabled) return
        speakerEnabled = enabled
        if (enabled) {
            ensureSocket()
            ensureNormalMode()
            startPlayback()
        } else {
            stopPlayback()
        }
    }

    private fun applyMic(enabled: Boolean) {
        if (enabled == micEnabled) return
        micEnabled = enabled
        if (enabled) {
            ensureSocket()
            if (!startCapture()) {            // device/permission failure
                micEnabled = false
                mainHandler.post { onMicFailed?.invoke() }
            }
        } else {
            stopCapture()
        }
    }

    // -- Socket / audio routing ----------------------------------------------

    private fun ensureSocket() {
        if (socket != null) return
        socket = DatagramSocket(null).apply {
            reuseAddress = true
            soTimeout = SOCKET_TIMEOUT_MS    // lets the playback loop poll its flag
            bind(InetSocketAddress(PORT))
        }
    }

    /**
     * Keep the audio system in MODE_NORMAL for the whole session — never touch
     * `audioManager.mode` or the speakerphone route again.
     *
     * Earlier revisions flipped MODE_IN_COMMUNICATION ⇄ MODE_NORMAL (and toggled
     * setSpeakerphoneOn) on every TALK/LISTEN transition. On this Qualcomm HAL each
     * flip tears down and rebuilds the `compress-voip-call` path, which takes over a
     * second; opening an AudioRecord during that window killed the server-side track
     * ("dead IAudioRecord, creating a new one from start()") and eventually wedged the
     * HAL outright ("Compress voip output cannot be closed, error:-22"), silencing BOTH
     * microphones.
     *
     * MODE_NORMAL is all we need:
     *  - USB capture: setPreferredDevice() is honoured (MODE_IN_COMMUNICATION was what
     *    used to override it), which is what made the external mic work in the first place.
     *  - Built-in capture: AudioSource.MIC records fine.
     *  - Playback: USAGE_MEDIA routes to the loudspeaker by default, so no speakerphone
     *    forcing is required.
     *
     * The only thing given up is the platform AEC on the built-in mic, which costs
     * nothing here: MainActivity.applyAudio() is strictly half-duplex, so the speaker is
     * always off while the mic is live and there is no echo to cancel.
     */
    private fun ensureNormalMode() {
        try {
            if (audioManager.mode != AudioManager.MODE_NORMAL) {
                audioManager.mode = AudioManager.MODE_NORMAL
            }
        } catch (_: Exception) { /* best-effort routing */ }
    }

    // -- Capture (mic -> UDP) -------------------------------------------------

    /**
     * Dump every audio device Android reports, both directions. Diagnostic only — when an
     * external mic doesn't work this answers "does the platform see it at all?" from
     * `adb logcat -s AudioLinkController`, instead of inferring it from a routing failure.
     * A device missing here never reached Android, so the problem is USB enumeration
     * (power, or an audio class the ROM doesn't support) rather than this app's routing.
     */
    private fun logAudioDevices() {
        try {
            val ins = audioManager.getDevices(AudioManager.GET_DEVICES_INPUTS)
                .joinToString { "${it.productName}(type=${it.type}${if (isUsbAudio(it.type)) ",USB" else ""})" }
            val outs = audioManager.getDevices(AudioManager.GET_DEVICES_OUTPUTS)
                .joinToString { "${it.productName}(type=${it.type}${if (isUsbAudio(it.type)) ",USB" else ""})" }
            Log.i(TAG, "audio devices | IN: $ins | OUT: $outs")
        } catch (_: Exception) { /* diagnostics must never break audio */ }
    }

    /** The first external USB audio input, or null when none is plugged in. */
    private fun findUsbMic(): AudioDeviceInfo? = try {
        audioManager.getDevices(AudioManager.GET_DEVICES_INPUTS)
            .firstOrNull { isUsbAudio(it.type) }
    } catch (_: Exception) {
        null
    }

    // A USB headset reports TYPE_USB_HEADSET (API 26+); bare USB sound cards and mics
    // report TYPE_USB_DEVICE. Both directions use the same type constants.
    private fun isUsbAudio(type: Int): Boolean =
        type == AudioDeviceInfo.TYPE_USB_DEVICE ||
        type == AudioDeviceInfo.TYPE_USB_ACCESSORY ||
        (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O &&
            type == AudioDeviceInfo.TYPE_USB_HEADSET)

    @SuppressLint("MissingPermission")   // caller gates on RECORD_AUDIO
    private fun startCapture(): Boolean {
        val minBuf = AudioRecord.getMinBufferSize(RATE, IN_CHANNEL, ENCODING)
        if (minBuf <= 0) return false
        val bufSize = maxOf(minBuf, FRAME_BYTES * 4)

        logAudioDevices()

        // Stay in MODE_NORMAL (see ensureNormalMode) — no mode flip here, so the HAL is
        // never torn down underneath the AudioRecord we are about to open.
        ensureNormalMode()

        // External USB mic only — no built-in fallback by design. If it isn't there the
        // capture fails and the HUD says so, rather than quietly transmitting from the
        // MK32's internal mic.
        val record = openUsbCapture(bufSize) ?: return false

        // Built-in echo cancellation + noise suppression, where the device offers it.
        if (AcousticEchoCanceler.isAvailable()) {
            aec = try { AcousticEchoCanceler.create(record.audioSessionId)?.apply { enabled = true } }
                  catch (_: Exception) { null }
        }
        if (NoiseSuppressor.isAvailable()) {
            ns = try { NoiseSuppressor.create(record.audioSessionId)?.apply { enabled = true } }
                 catch (_: Exception) { null }
        }

        // Already recording — the open* helpers start it so routing can be verified.
        audioRecord = record

        captureThread = Thread({ captureLoop(record) }, "audio-capture").apply {
            isDaemon = true
            start()
        }
        return true
    }

    /**
     * Try to capture from an external USB mic, returning a *started* [AudioRecord] or
     * null to fall back to the built-in one.
     *
     * Uses AudioSource.MIC rather than VOICE_COMMUNICATION: the latter is the platform's
     * processed voice path, hard-bound to the built-in comm mic (it is the AEC reference),
     * and it ignores a USB preferred device. After starting we confirm [AudioRecord
     * .getRoutedDevice] actually landed on USB, since setPreferredDevice() is only a hint.
     */
    @SuppressLint("MissingPermission")   // caller gates on RECORD_AUDIO
    private fun openUsbCapture(bufSize: Int): AudioRecord? {
        val usb = findUsbMic() ?: run {
            Log.w(TAG, "no USB mic enumerated — capture aborted (built-in fallback disabled)")
            return null
        }

        val record = try {
            AudioRecord(MediaRecorder.AudioSource.MIC, RATE, IN_CHANNEL, ENCODING, bufSize)
        } catch (_: Exception) {
            return null
        }
        // A USB mic is natively 44.1/48 kHz; the wire format must stay 16 kHz mono to
        // match the Pi. AudioFlinger resamples for us, but if this device can't do it
        // the record won't initialise — fall back rather than stream garbage.
        if (record.state != AudioRecord.STATE_INITIALIZED) {
            record.release()
            return null
        }

        if (!record.setPreferredDevice(usb)) {
            record.release()
            return null
        }

        try { record.startRecording() } catch (_: Exception) {
            record.release()
            return null
        }

        // setPreferredDevice() is advisory, and getRoutedDevice() reports the built-in
        // mic for a few ms after startRecording() until routing settles — so poll rather
        // than judging on the first read.
        var routed = record.routedDevice
        var waited = 0
        while ((routed == null || !isUsbAudio(routed.type)) && waited < ROUTE_SETTLE_MS) {
            try { Thread.sleep(ROUTE_POLL_MS.toLong()) } catch (_: InterruptedException) { break }
            waited += ROUTE_POLL_MS
            routed = record.routedDevice
        }

        if (routed == null || !isUsbAudio(routed.type)) {
            try { record.stop() } catch (_: Exception) {}
            record.release()
            Log.w(TAG, "USB mic '${usb.productName}' present but routing stuck on type=${routed?.type}" +
                       " after ${waited}ms; NOT falling back to built-in")
            return null
        }

        Log.i(TAG, "capture: USB mic '${usb.productName}' (type=${routed.type}), source=MIC, settled in ${waited}ms")
        return record
    }

    private fun captureLoop(record: AudioRecord) {
        val buf = ByteArray(FRAME_BYTES)
        while (micEnabled) {
            val n = try {
                record.read(buf, 0, buf.size)
            } catch (_: Exception) {
                break
            }
            if (n <= 0) continue
            // record.read() already returns AEC + noise-suppressed audio (those
            // are platform pre-processing effects on the session). We gate, then
            // amplify, so the boost is applied AFTER noise cancellation and never
            // pumps up the residual background hiss between words.
            gateAndAmplify(buf, n)
            try {
                socket?.send(DatagramPacket(buf, n, peer))
            } catch (_: Exception) {
                // Best-effort: a dropped frame is inaudible. Keep streaming.
            }
        }
    }

    // In-place on 16-bit LE PCM: if the frame's peak is below MIC_NOISE_GATE it is
    // near-silent (background noise) and zeroed; otherwise it is voice and gets
    // boosted by MIC_GAIN, clamped to the int16 range so it clips rather than
    // wraps. Runs on the already noise-cancelled samples from record.read().
    private fun gateAndAmplify(buf: ByteArray, len: Int) {
        val gate = MIC_NOISE_GATE
        val gain = MIC_GAIN

        var peak = 0
        var i = 0
        while (i + 1 < len) {
            val s = (buf[i].toInt() and 0xFF) or (buf[i + 1].toInt() shl 8)  // signed LE
            val abs = if (s < 0) -s else s
            if (abs > peak) peak = abs
            i += 2
        }

        if (peak < gate) {
            for (j in 0 until len) buf[j] = 0          // silence the noise floor
            return
        }
        if (gain == 1f) return

        i = 0
        while (i + 1 < len) {
            val s = (buf[i].toInt() and 0xFF) or (buf[i + 1].toInt() shl 8)
            val scaled = (s * gain).toInt().coerceIn(-32768, 32767)
            buf[i] = (scaled and 0xFF).toByte()
            buf[i + 1] = ((scaled shr 8) and 0xFF).toByte()
            i += 2
        }
    }

    private fun stopCapture() {
        captureThread?.let { t ->
            t.interrupt()
            try { t.join(JOIN_MS) } catch (_: InterruptedException) {}
        }
        captureThread = null
        aec?.let { try { it.release() } catch (_: Exception) {} }
        ns?.let { try { it.release() } catch (_: Exception) {} }
        aec = null
        ns = null
        audioRecord?.let { r ->
            try { if (r.recordingState == AudioRecord.RECORDSTATE_RECORDING) r.stop() } catch (_: Exception) {}
            try { r.release() } catch (_: Exception) {}
        }
        audioRecord = null
    }

    // -- Playback (UDP -> speaker) --------------------------------------------

    private fun startPlayback() {
        val minBuf = AudioTrack.getMinBufferSize(RATE, OUT_CHANNEL, ENCODING)
        // Keep the track buffer as small as the device allows — a big buffer is
        // pure added latency. minBuf is the floor; a couple of frames is plenty.
        val bufSize = maxOf(minBuf, FRAME_BYTES * 2)

        val track = try {
            AudioTrack.Builder()
                .setAudioAttributes(
                    AudioAttributes.Builder()
                        // USAGE_MEDIA, not USAGE_VOICE_COMMUNICATION: we stay in
                        // MODE_NORMAL, where the voice-communication path expects a call
                        // and would route to the earpiece. Media goes to the built-in
                        // loudspeaker by default and follows the media volume keys.
                        .setUsage(AudioAttributes.USAGE_MEDIA)
                        .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
                        .build()
                )
                .setAudioFormat(
                    AudioFormat.Builder()
                        .setSampleRate(RATE)
                        .setEncoding(ENCODING)
                        .setChannelMask(OUT_CHANNEL)
                        .build()
                )
                .setBufferSizeInBytes(bufSize)
                .setTransferMode(AudioTrack.MODE_STREAM)
                .apply {
                    // Ask the framework for the lowest-latency output path (API 26+).
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                        setPerformanceMode(AudioTrack.PERFORMANCE_MODE_LOW_LATENCY)
                    }
                }
                .build()
        } catch (_: Exception) {
            speakerEnabled = false
            return
        }
        if (track.state != AudioTrack.STATE_INITIALIZED) {
            track.release()
            speakerEnabled = false
            return
        }

        audioTrack = track
        track.play()

        playbackThread = Thread({ playbackLoop(track) }, "audio-playback").apply {
            isDaemon = true
            start()
        }
    }

    // Plays incoming frames but never lets latency accumulate: each cycle we
    // grab the next packet, then immediately drain any backlog that piled up
    // (jitter burst, or we briefly fell behind) and play ONLY the most recent
    // frame — the same drop-oldest strategy as the Python peer's bounded queue.
    private fun playbackLoop(track: AudioTrack) {
        val sock = socket ?: return
        val buf = ByteArray(FRAME_BYTES * 2)
        val latest = ByteArray(FRAME_BYTES * 2)
        val packet = DatagramPacket(buf, buf.size)
        while (speakerEnabled) {
            // Block (with the flag-poll timeout) for the next packet.
            var len = try {
                packet.setData(buf)
                sock.receive(packet)
                packet.length
            } catch (_: SocketTimeoutException) {
                continue                     // no audio yet; re-check the flag
            } catch (_: Exception) {
                break
            }
            if (len <= 0) continue
            System.arraycopy(buf, 0, latest, 0, len)

            // Shed any already-queued backlog with a very short timeout, keeping
            // only the freshest frame so playback stays current.
            try {
                sock.soTimeout = DRAIN_TIMEOUT_MS
                while (true) {
                    packet.setData(buf)
                    sock.receive(packet)
                    if (packet.length > 0) {
                        len = packet.length
                        System.arraycopy(buf, 0, latest, 0, len)
                    }
                }
            } catch (_: SocketTimeoutException) {
                // Backlog drained.
            } catch (_: Exception) {
                break
            } finally {
                try { sock.soTimeout = SOCKET_TIMEOUT_MS } catch (_: Exception) {}
            }

            try {
                track.write(latest, 0, len)
            } catch (_: Exception) {
                break
            }
        }
    }

    private fun stopPlayback() {
        playbackThread?.let { t ->
            t.interrupt()
            try { t.join(JOIN_MS) } catch (_: InterruptedException) {}
        }
        playbackThread = null
        audioTrack?.let { tr ->
            try { if (tr.playState == AudioTrack.PLAYSTATE_PLAYING) tr.stop() } catch (_: Exception) {}
            try { tr.flush() } catch (_: Exception) {}
            try { tr.release() } catch (_: Exception) {}
        }
        audioTrack = null
    }

    companion object {
        private const val TAG = "AudioLinkController"

        // Must match the Python script (PEER_IP / PORT / RATE / FRAME_SIZE).
        private const val HOST = "192.168.144.100"
        private const val PORT = 5555
        private const val RATE = 16000
        private const val FRAME_BYTES = 512          // 256 samples * 2 bytes (int16, mono)

        // Software boost applied to captured voice AFTER noise cancellation. 1f = off,
        // 2f ≈ +6 dB. Tuned for the external USB mic, which runs hotter and cleaner than
        // the MK32's internal one, so no boost is needed. Raise cautiously — too high
        // clips/distorts loud speech.
        private const val MIC_GAIN = 1.0f

        // Frames whose peak (|sample|, int16 scale 0..32767) is below this are treated as
        // background noise and silenced instead of amplified. Lower it if quiet speech
        // gets cut; raise it if noise still leaks through.
        private const val MIC_NOISE_GATE = 400

        private const val ENCODING = AudioFormat.ENCODING_PCM_16BIT
        private const val IN_CHANNEL = AudioFormat.CHANNEL_IN_MONO
        private const val OUT_CHANNEL = AudioFormat.CHANNEL_OUT_MONO

        // getRoutedDevice() briefly reports the built-in mic after startRecording()
        // while the USB route is still being established — poll for up to this long.
        private const val ROUTE_SETTLE_MS = 400
        private const val ROUTE_POLL_MS = 20

        private const val SOCKET_TIMEOUT_MS = 20     // playback poll: small = fast stop on toggle
        private const val DRAIN_TIMEOUT_MS = 2       // backlog-drain poll (drop-oldest)
        private const val JOIN_MS = 500L
    }
}
