package com.avatarrobot.camswitcher

import android.util.Log
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketTimeoutException
import kotlin.concurrent.thread

/**
 * Single UDP listener for the rover's telemetry stream (telemetry_udp_bridge.py),
 * which sends one complete JSON snapshot to port 9870 at ~5 Hz.
 *
 * This app now hosts BOTH the battery chip and the 3D digital twin, which used to
 * be two separate apps each binding :9870. Two sockets cannot share that port in
 * one process, so this receiver parses each datagram ONCE and fans it out:
 *   - [onBattery]  ← the "battery_pct" field (0-100, or -1 when no fresh reading)
 *   - [onJoints]   ← the whole snapshot, for JointState.update (joints + seq)
 * Both callbacks fire on the background receive thread; callers marshal to the UI
 * thread themselves.
 *
 * COEXISTENCE: AvatarDashboard still binds :9870 when it is foreground. The
 * convention is "the foreground app owns the port; background apps release it",
 * so this is started from onResume and stopped from onPause. To survive the brief
 * handoff window where the outgoing app's socket may not have fully closed, the
 * socket is created with SO_REUSEADDR *before* binding and the bind is retried a
 * few times (logic carried over from the twin's TelemetryReceiver).
 */
class UdpTelemetryReceiver(
    private val port: Int = 9870,
    private val onBattery: (Int) -> Unit,
    private val onJoints: (JSONObject) -> Unit,
) {

    @Volatile private var running = false
    private var worker: Thread? = null
    private var socket: DatagramSocket? = null

    fun start() {
        if (running) return
        running = true
        worker = thread(name = "udp-telemetry", isDaemon = true) { loop() }
    }

    fun stop() {
        running = false
        try { socket?.close() } catch (_: Exception) {}
        worker = null
    }

    /** Create a SO_REUSEADDR socket and bind, retrying across the handoff window. */
    private fun bindWithRetry(): DatagramSocket? {
        repeat(3) { attempt ->
            try {
                val s = DatagramSocket(null)        // unbound
                s.reuseAddress = true               // must be set BEFORE bind
                s.bind(InetSocketAddress(port))
                return s
            } catch (e: Exception) {
                Log.w(TAG, "bind attempt ${attempt + 1} on :$port failed: ${e.message}")
                try { Thread.sleep(150) } catch (_: InterruptedException) {}
            }
        }
        Log.e(TAG, "could not bind :$port after 3 attempts — another app may hold it")
        return null
    }

    private fun loop() {
        // Buffer large enough for the full snapshot (~12 motors + joints).
        val buf = ByteArray(64 * 1024)
        val sock = bindWithRetry() ?: run { running = false; return }
        socket = sock
        sock.soTimeout = 1000                       // wake ~1 Hz so stop() is prompt
        sock.use {
            while (running) {
                val pkt = DatagramPacket(buf, buf.size)
                try {
                    it.receive(pkt)
                } catch (e: SocketTimeoutException) {
                    continue                         // re-check `running`
                } catch (e: Exception) {
                    if (running) { Log.w(TAG, "recv error: ${e.message}"); continue } else break
                }
                val text = String(pkt.data, 0, pkt.length, Charsets.UTF_8)
                val json = try {
                    JSONObject(text)
                } catch (_: Exception) {
                    continue                         // malformed / short / truncated — drop
                }
                // Fan out: battery first (cheap field), then the full snapshot.
                val pct = if (json.isNull("battery_pct")) -1 else json.optInt("battery_pct", -1)
                if (pct >= 0) onBattery(pct)
                onJoints(json)
            }
        }
    }

    companion object { private const val TAG = "UdpTelemetry" }
}
