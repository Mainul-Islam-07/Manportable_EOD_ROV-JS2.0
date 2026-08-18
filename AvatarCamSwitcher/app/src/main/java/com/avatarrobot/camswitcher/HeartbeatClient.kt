package com.avatarrobot.camswitcher

import android.util.Log
import java.io.BufferedReader
import java.io.InputStreamReader
import java.net.InetSocketAddress
import java.net.Socket
import java.net.SocketTimeoutException
import java.nio.charset.StandardCharsets
import kotlin.concurrent.thread

/**
 * Presence heartbeat to the Pi fire-control server (fire_server.py, port 5006).
 *
 * While [start]ed, a daemon thread holds a persistent TCP connection and sends
 * "PING\n" once a second (the server replies "PONG\n"). The server publishes ROS
 * `/fire_mode = 1` while pings are recent and `0` after ~3 s of silence — so this
 * heartbeat IS "fire mode 1". Calling [stop] (or the process dying) lets the
 * server time the presence out back to `/fire_mode = 0`.
 *
 * It is a process-level singleton on purpose: the FIRING→AIM flow navigates
 * between Activities (which pauses MainActivity), and fire mode must stay 1 across
 * that until the operator presses RESET.
 */
object HeartbeatClient {

    private const val TAG = "FcsHeartbeat"
    private const val CONNECT_TIMEOUT_MS = 3000
    private const val READ_TIMEOUT_MS = 1500
    private const val PING_PERIOD_MS = 1000L
    private const val RECONNECT_DELAY_MS = 1000L

    @Volatile var host: String = ArmFireClient.DEFAULT_IP
    @Volatile var port: Int = 5006

    @Volatile private var running = false
    private var worker: Thread? = null

    val isActive: Boolean get() = running

    fun start() {
        if (running) return
        running = true
        worker = thread(name = "fcs-heartbeat", isDaemon = true) { loop() }
    }

    fun stop() {
        running = false
        worker?.interrupt()
        worker = null
    }

    private fun loop() {
        while (running) {
            var socket: Socket? = null
            try {
                socket = Socket()
                socket.connect(InetSocketAddress(host, port), CONNECT_TIMEOUT_MS)
                socket.soTimeout = READ_TIMEOUT_MS
                val out = socket.getOutputStream()
                val reader = BufferedReader(
                    InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8))
                Log.i(TAG, "heartbeat connected to $host:$port")
                while (running) {
                    out.write("PING\n".toByteArray(StandardCharsets.UTF_8)); out.flush()
                    // Drain the PONG so the socket buffer never grows; ignore timeout.
                    try {
                        if (reader.readLine() == null) break    // peer closed → reconnect
                    } catch (_: SocketTimeoutException) { /* no reply yet — fine */ }
                    Thread.sleep(PING_PERIOD_MS)
                }
            } catch (_: InterruptedException) {
                break                                            // stop() interrupted us
            } catch (e: Exception) {
                if (running) Log.w(TAG, "heartbeat link error: ${e.message}")
            } finally {
                try { socket?.close() } catch (_: Exception) {}
            }
            if (running) {
                try { Thread.sleep(RECONNECT_DELAY_MS) } catch (_: InterruptedException) { break }
            }
        }
        Log.i(TAG, "heartbeat stopped")
    }
}
