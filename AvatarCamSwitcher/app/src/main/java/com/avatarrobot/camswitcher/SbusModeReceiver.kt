package com.avatarrobot.camswitcher

import android.util.Log
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import kotlin.concurrent.thread

/**
 * Listens for the operator's current SBUS mode, broadcast by the Pi's
 * `sbus_mode_udp_bridge.py` as compact JSON to UDP **:9871** at ~5 Hz, e.g.
 * `{"seq":12,"mode":"DRIVE","control_mode":2,"operation_mode":0,"rx_age_ms":40}`.
 *
 * Only the `mode` string is consumed here (DISARM | STAIR | HOME | ARM | DRIVE); it is
 * shown beside the FIRING button. This is a SEPARATE port from the 9870 telemetry
 * receiver, so the two sockets never conflict. Same coexistence convention:
 * started in onResume, stopped in onPause, SO_REUSEADDR + bind retry to ride out
 * the handoff window.
 *
 * [onMode] is invoked on the background receive thread — the caller marshals to
 * the UI thread.
 */
class SbusModeReceiver(
    private val port: Int = 9871,
    private val onMode: (String) -> Unit,
) {
    @Volatile private var running = false
    private var worker: Thread? = null
    private var socket: DatagramSocket? = null

    fun start() {
        if (running) return
        running = true
        worker = thread(name = "udp-sbus-mode", isDaemon = true) { loop() }
    }

    fun stop() {
        running = false
        try { socket?.close() } catch (_: Exception) {}
        worker = null
    }

    private fun bindWithRetry(): DatagramSocket? {
        repeat(3) { attempt ->
            try {
                val s = DatagramSocket(null)
                s.reuseAddress = true
                s.bind(InetSocketAddress(port))
                return s
            } catch (e: Exception) {
                Log.w(TAG, "bind attempt ${attempt + 1} on :$port failed: ${e.message}")
                try { Thread.sleep(150) } catch (_: InterruptedException) {}
            }
        }
        Log.e(TAG, "could not bind :$port after 3 attempts")
        return null
    }

    private fun loop() {
        val buf = ByteArray(4 * 1024)               // the mode datagram is tiny
        val sock = bindWithRetry() ?: run { running = false; return }
        socket = sock
        sock.soTimeout = 1000
        sock.use {
            while (running) {
                val pkt = DatagramPacket(buf, buf.size)
                try {
                    it.receive(pkt)
                } catch (e: SocketTimeoutException) {
                    continue
                } catch (e: Exception) {
                    if (running) { Log.w(TAG, "recv error: ${e.message}"); continue } else break
                }
                val text = String(pkt.data, 0, pkt.length, Charsets.UTF_8)
                val mode = try {
                    JSONObject(text).optString("mode", "")
                } catch (_: Exception) {
                    continue                        // malformed — drop
                }
                if (mode.isNotEmpty()) onMode(mode)
            }
        }
    }

    companion object { private const val TAG = "SbusMode" }
}
