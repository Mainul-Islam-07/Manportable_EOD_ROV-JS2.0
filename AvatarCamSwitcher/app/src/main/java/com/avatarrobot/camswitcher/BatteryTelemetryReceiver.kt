package com.avatarrobot.camswitcher

import android.util.Log
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.SocketTimeoutException
import kotlin.concurrent.thread

/**
 * Minimal UDP listener for the rover's telemetry stream (telemetry_udp_bridge.py).
 *
 * The bridge sends a complete JSON snapshot to port 9870 at ~5 Hz. We only care
 * about the main-pack charge ("battery_pct", 0-100, or null when the bridge had
 * no fresh voltage) — every other field (including the encoder memory-battery
 * voltages) is ignored here. Each datagram delivers the latest value via
 * [onBattery]; the caller is responsible for marshalling onto the UI thread.
 *
 * One background daemon thread owns the socket and blocks on receive() with a
 * timeout so [stop] can shut it down cleanly. This mirrors the dashboard's
 * UdpTelemetryReceiver, trimmed to a single field.
 */
class BatteryTelemetryReceiver(
    private val port: Int = 9870,
    private val onBattery: (Int) -> Unit,
) {

    @Volatile private var running = false
    private var worker: Thread? = null
    private var socket: DatagramSocket? = null

    fun start() {
        if (running) return
        running = true
        worker = thread(name = "udp-battery", isDaemon = true) { loop() }
    }

    fun stop() {
        running = false
        try { socket?.close() } catch (_: Exception) {}
        worker = null
    }

    private fun loop() {
        // Buffer large enough for the full snapshot (~12 motors) even though we
        // only read one field out of it.
        val buf = ByteArray(64 * 1024)
        try {
            DatagramSocket(port).also { socket = it }.use { sock ->
                sock.soTimeout = 1000
                while (running) {
                    val pkt = DatagramPacket(buf, buf.size)
                    try {
                        sock.receive(pkt)
                    } catch (e: SocketTimeoutException) {
                        continue  // lets us re-check `running`
                    } catch (e: Exception) {
                        if (running) Log.w(TAG, "recv error: ${e.message}")
                        continue
                    }
                    val text = String(pkt.data, 0, pkt.length, Charsets.UTF_8)
                    val pct = runCatching { parsePct(text) }.getOrNull()
                    if (pct != null) onBattery(pct)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "socket open failed on :$port — ${e.message}")
        }
    }

    /** -1 when the snapshot reports no fresh battery reading. */
    private fun parsePct(text: String): Int {
        val root = JSONObject(text)
        return if (root.isNull("battery_pct")) -1 else root.optInt("battery_pct", -1)
    }

    companion object { private const val TAG = "UdpBattery" }
}
