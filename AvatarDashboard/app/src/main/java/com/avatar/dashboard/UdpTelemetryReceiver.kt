package com.avatar.dashboard

import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONObject
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.SocketTimeoutException
import kotlin.concurrent.thread

/**
 * Listens for UDP telemetry datagrams from telemetry_udp_bridge.py and
 * exposes the latest decoded [Telemetry] via a [StateFlow].
 *
 * One background thread owns the socket; it blocks on receive() with a
 * timeout so it can be cleanly stopped.  Each datagram is a COMPLETE
 * snapshot, so we simply replace state on every packet — no merging,
 * no accumulation, no leaks.
 */
class UdpTelemetryReceiver(private val port: Int = 9870) {

    private val _state = MutableStateFlow(Telemetry())
    val state: StateFlow<Telemetry> = _state.asStateFlow()

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

    private fun loop() {
        // Buffer large enough for the full snapshot of ~12 motors.
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
                    val parsed = runCatching { parse(text) }.getOrNull()
                    if (parsed != null) _state.value = parsed
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "socket open failed on :$port — ${e.message}")
        }
    }

    private fun parse(text: String): Telemetry {
        val root = JSONObject(text)

        // -- diagnostics --
        val diags = mutableListOf<MotorDiag>()
        root.optJSONObject("diagnostics")?.let { d ->
            val it = d.keys()
            while (it.hasNext()) {
                val name = it.next()
                val o = d.getJSONObject(name)
                diags.add(
                    MotorDiag(
                        name = name,
                        level = o.optString("level", "STALE"),
                        message = o.optString("msg", ""),
                        hwId = o.optString("hw_id", "?"),
                        bus = o.optString("bus", ""),
                        voltage = o.optString("voltage", "--"),
                        current = o.optString("current", "--"),
                        coilTemp = o.optString("coil_t", "--"),
                        boardTemp = o.optString("board_t", "--"),
                        statusword = o.optString("sw", "--"),
                        swState = o.optString("sw_state", "--"),
                        swFlags = o.optString("sw_flags", "--"),
                        errorReg = o.optString("er", "--"),
                        errorRegFlags = o.optString("er_flags", "--"),
                        errorCode = o.optString("ec", "--"),
                        errorCodeFlags = o.optString("ec_flags", "--"),
                        hbState = o.optString("hb_state", "--"),
                        hbCount = o.optString("hb_count", "--"),
                        fresh = o.optBoolean("fresh", true),
                        ageMs = o.optInt("age_ms", -1),
                    )
                )
            }
        }
        diags.sortBy { it.name }

        // -- joints --
        val joints = mutableListOf<JointVal>()
        root.optJSONObject("joints")?.let { j ->
            val it = j.keys()
            while (it.hasNext()) {
                val name = it.next()
                joints.add(JointVal(name, j.optDouble(name, 0.0)))
            }
        }

        // -- active lists --
        fun strList(key: String): List<String> {
            val arr = root.optJSONArray(key) ?: return emptyList()
            return (0 until arr.length()).map { arr.optString(it) }
        }

        return Telemetry(
            seq = root.optLong("seq", 0),
            stamp = root.optDouble("stamp", 0.0),
            batteryPct = if (root.isNull("battery_pct")) -1
                         else root.optInt("battery_pct", -1),
            batteryVolts = if (root.isNull("battery_volts")) Double.NaN
                           else root.optDouble("battery_volts", Double.NaN),
            driveMemVolts = if (root.isNull("drive_mem_volts")) Double.NaN
                            else root.optDouble("drive_mem_volts", Double.NaN),
            armMemVolts = if (root.isNull("arm_mem_volts")) Double.NaN
                          else root.optDouble("arm_mem_volts", Double.NaN),
            driveMemOk = if (root.isNull("drive_mem_ok")) null
                         else root.optBoolean("drive_mem_ok"),
            armMemOk = if (root.isNull("arm_mem_ok")) null
                       else root.optBoolean("arm_mem_ok"),
            diagnostics = diags,
            joints = joints,
            armActive = strList("arm_active"),
            driveActive = strList("drive_active"),
            rxAtMs = System.currentTimeMillis(),
        )
    }

    companion object { private const val TAG = "UdpTelemetry" }
}
