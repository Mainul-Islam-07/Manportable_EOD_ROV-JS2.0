package com.avatarrobot.camswitcher

import android.annotation.SuppressLint
import android.content.Context
import android.media.AudioAttributes
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

    /** Release everything: stop both directions, close the socket, join threads. */
    fun close() {
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
            enterCommunicationMode()
            startPlayback()
        } else {
            stopPlayback()
            maybeLeaveCommunicationMode()
        }
    }

    private fun applyMic(enabled: Boolean) {
        if (enabled == micEnabled) return
        micEnabled = enabled
        if (enabled) {
            ensureSocket()
            enterCommunicationMode()
            if (!startCapture()) {            // device/permission failure
                micEnabled = false
                maybeLeaveCommunicationMode()
                mainHandler.post { onMicFailed?.invoke() }
            }
        } else {
            stopCapture()
            maybeLeaveCommunicationMode()
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

    // VOICE_COMMUNICATION routing only engages the platform AEC reliably while
    // the audio mode is MODE_IN_COMMUNICATION. Set it once whenever either side
    // is active; restore MODE_NORMAL when both are off.
    //
    // In communication mode Android defaults playback to the earpiece, so we
    // explicitly force it onto the built-in loudspeaker (hands-free), which keeps
    // the AEC reference intact while making incoming audio actually audible.
    private fun enterCommunicationMode() {
        try {
            if (audioManager.mode != AudioManager.MODE_IN_COMMUNICATION) {
                audioManager.mode = AudioManager.MODE_IN_COMMUNICATION
            }
            routeToLoudspeaker()
        } catch (_: Exception) { /* best-effort routing */ }
    }

    private fun maybeLeaveCommunicationMode() {
        if (micEnabled || speakerEnabled) return
        try {
            clearLoudspeakerRoute()
            audioManager.mode = AudioManager.MODE_NORMAL
        } catch (_: Exception) { /* best-effort routing */ }
    }

    @Suppress("DEPRECATION")
    private fun routeToLoudspeaker() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val speaker = audioManager.availableCommunicationDevices
                .firstOrNull { it.type == AudioDeviceInfo.TYPE_BUILTIN_SPEAKER }
            if (speaker != null) audioManager.setCommunicationDevice(speaker)
        } else {
            audioManager.isSpeakerphoneOn = true
        }
    }

    @Suppress("DEPRECATION")
    private fun clearLoudspeakerRoute() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            audioManager.clearCommunicationDevice()
        } else {
            audioManager.isSpeakerphoneOn = false
        }
    }

    // -- Capture (mic -> UDP) -------------------------------------------------

    @SuppressLint("MissingPermission")   // caller gates on RECORD_AUDIO
    private fun startCapture(): Boolean {
        val minBuf = AudioRecord.getMinBufferSize(RATE, IN_CHANNEL, ENCODING)
        if (minBuf <= 0) return false
        val bufSize = maxOf(minBuf, FRAME_BYTES * 4)

        val record = try {
            AudioRecord(
                MediaRecorder.AudioSource.VOICE_COMMUNICATION,
                RATE, IN_CHANNEL, ENCODING, bufSize
            )
        } catch (_: Exception) {
            return false
        }
        if (record.state != AudioRecord.STATE_INITIALIZED) {
            record.release()
            return false
        }

        // Built-in echo cancellation + noise suppression, where the device offers it.
        if (AcousticEchoCanceler.isAvailable()) {
            aec = try { AcousticEchoCanceler.create(record.audioSessionId)?.apply { enabled = true } }
                  catch (_: Exception) { null }
        }
        if (NoiseSuppressor.isAvailable()) {
            ns = try { NoiseSuppressor.create(record.audioSessionId)?.apply { enabled = true } }
                 catch (_: Exception) { null }
        }

        audioRecord = record
        record.startRecording()

        captureThread = Thread({ captureLoop(record) }, "audio-capture").apply {
            isDaemon = true
            start()
        }
        return true
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

    // In-place on 16-bit LE PCM: if the frame's peak is below NOISE_GATE it is
    // near-silent (background noise) and zeroed; otherwise it is voice and gets
    // boosted by MIC_GAIN, clamped to the int16 range so it clips rather than
    // wraps. Runs on the already noise-cancelled samples from record.read().
    private fun gateAndAmplify(buf: ByteArray, len: Int) {
        var peak = 0
        var i = 0
        while (i + 1 < len) {
            val s = (buf[i].toInt() and 0xFF) or (buf[i + 1].toInt() shl 8)  // signed LE
            val abs = if (s < 0) -s else s
            if (abs > peak) peak = abs
            i += 2
        }

        if (peak < NOISE_GATE) {
            for (j in 0 until len) buf[j] = 0          // silence the noise floor
            return
        }
        if (MIC_GAIN == 1f) return

        i = 0
        while (i + 1 < len) {
            val s = (buf[i].toInt() and 0xFF) or (buf[i + 1].toInt() shl 8)
            val scaled = (s * MIC_GAIN).toInt().coerceIn(-32768, 32767)
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
                        .setUsage(AudioAttributes.USAGE_VOICE_COMMUNICATION)
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
        // Must match the Python script (PEER_IP / PORT / RATE / FRAME_SIZE).
        private const val HOST = "192.168.144.100"
        private const val PORT = 5555
        private const val RATE = 16000
        private const val FRAME_BYTES = 512          // 256 samples * 2 bytes (int16, mono)

        // Software boost applied to captured voice AFTER noise cancellation. 1f =
        // off. 2f ≈ +6 dB. Raise cautiously — too high clips/distorts loud speech.
        private const val MIC_GAIN = 1.5f

        // Frames whose peak (|sample|, int16 scale 0..32767) is below this are
        // treated as background noise and silenced instead of amplified. Lower it
        // if quiet speech gets cut; raise it if noise still leaks through.
        private const val NOISE_GATE = 600

        private const val ENCODING = AudioFormat.ENCODING_PCM_16BIT
        private const val IN_CHANNEL = AudioFormat.CHANNEL_IN_MONO
        private const val OUT_CHANNEL = AudioFormat.CHANNEL_OUT_MONO

        private const val SOCKET_TIMEOUT_MS = 20     // playback poll: small = fast stop on toggle
        private const val DRAIN_TIMEOUT_MS = 2       // backlog-drain poll (drop-oldest)
        private const val JOIN_MS = 500L
    }
}
