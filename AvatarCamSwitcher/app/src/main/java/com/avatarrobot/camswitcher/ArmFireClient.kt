package com.avatarrobot.camswitcher

import android.os.Handler
import android.os.Looper
import java.io.BufferedReader
import java.io.InputStreamReader
import java.net.InetSocketAddress
import java.net.Socket
import java.nio.charset.StandardCharsets
import java.util.concurrent.Executors

/**
 * Tiny TCP client for the rover FIRE endpoint (fire_server.py, port 5005), ported
 * from the old ARMSWITCH app. It opens a short-lived socket to <ip:port>, writes
 * one ASCII line "<command>\n", and succeeds iff the reply line trims to "OK".
 *
 * Used only for "FIRE <ms>" (the AIM-window FIRE button): the server replies "OK"
 * and pulses GPIO17 for <ms>; after OK we also hold the socket open for [holdMs]
 * before closing, mirroring the original ARMSWITCH "hold" semantics. (Fire mode
 * 1/0 is NOT a command here — it is the presence heartbeat; see HeartbeatClient.)
 *
 * [onResult] is always delivered on the main thread.
 */
object ArmFireClient {

    const val DEFAULT_IP = "192.168.144.100"
    const val DEFAULT_PORT = 5005

    private const val CONNECT_TIMEOUT = 3000
    private const val READ_TIMEOUT = 5000

    private val executor = Executors.newSingleThreadExecutor()
    private val ui = Handler(Looper.getMainLooper())

    /** Send a command; succeed iff the peer replies "OK". Result posts to the UI thread. */
    fun send(
        ip: String,
        port: Int,
        command: String,
        holdMs: Int = 0,
        onResult: (ok: Boolean, message: String) -> Unit,
    ) {
        executor.submit {
            var ok = false
            var message: String
            var socket: Socket? = null
            try {
                socket = Socket()
                socket.connect(InetSocketAddress(ip, port), CONNECT_TIMEOUT)
                socket.soTimeout = READ_TIMEOUT

                socket.getOutputStream().apply {
                    write((command + "\n").toByteArray(StandardCharsets.UTF_8)); flush()
                }
                val reply = BufferedReader(
                    InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8)
                ).readLine()

                when {
                    reply == null -> message = "peer closed connection"
                    reply.trim() == "OK" -> {
                        ok = true; message = "OK"
                        // Hold the connection open for the fire duration, then close.
                        if (holdMs > 0) {
                            val start = System.currentTimeMillis()
                            while (System.currentTimeMillis() - start < holdMs) {
                                try { Thread.sleep(20) } catch (_: InterruptedException) { break }
                            }
                        }
                    }
                    else -> message = "peer replied: ${reply.trim()}"
                }
            } catch (e: Exception) {
                message = "${e.javaClass.simpleName}: ${e.message}"
            } finally {
                try { socket?.close() } catch (_: Exception) {}
            }
            ui.post { onResult(ok, message) }
        }
    }
}
