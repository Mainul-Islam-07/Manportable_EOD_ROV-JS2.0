package com.avatarrobot.camswitcher

import android.os.Handler
import android.os.HandlerThread
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress

/**
 * Minimal UDP client for the SIYI Gimbal Camera External SDK (A8 mini).
 *
 * Aims the gimbal over Ethernet by sending SDK command frames to the camera at
 * 192.168.144.25:37260. All socket I/O runs on a dedicated [HandlerThread] so it
 * never touches the UI thread (which would throw NetworkOnMainThreadException);
 * this mirrors the rest of the app's Handler-based, coroutine-free style.
 *
 * Frame layout (all multi-byte fields little-endian):
 *   STX(0x55 0x66) | CTRL(1) | DATA_LEN(2) | SEQ(2) | CMD_ID(1) | DATA(n) | CRC16(2)
 * CRC16 is CRC-16/XMODEM over STX..DATA, appended low byte first.
 */
class SiyiGimbalController {

    private val netThread = HandlerThread("siyi-net").apply { start() }
    private val netHandler = Handler(netThread.looper)

    private var socket: DatagramSocket? = null
    private val address: InetAddress = InetAddress.getByName(HOST)
    private var seq = 0

    // Repeats the current rotation command while a D-pad key is held: keeps the
    // gimbal moving and survives the odd dropped UDP packet. Cleared by stopMove().
    private var moveYaw = 0
    private var movePitch = 0
    private val repeatRunnable = object : Runnable {
        override fun run() {
            sendFrame(CMD_ROTATE, byteArrayOf(clampSpeed(moveYaw), clampSpeed(movePitch)))
            netHandler.postDelayed(this, REPEAT_MS)
        }
    }

    /** Begin moving at the given yaw/pitch speeds (-100..100); 0 holds that axis. */
    fun startMove(yaw: Int, pitch: Int) {
        netHandler.post {
            moveYaw = yaw
            movePitch = pitch
            netHandler.removeCallbacks(repeatRunnable)
            netHandler.post(repeatRunnable)
        }
    }

    /** Stop all motion: cancel the repeat and send a zero-speed frame (twice). */
    fun stopMove() {
        netHandler.post {
            moveYaw = 0
            movePitch = 0
            netHandler.removeCallbacks(repeatRunnable)
            val stop = byteArrayOf(0, 0)
            sendFrame(CMD_ROTATE, stop)
            sendFrame(CMD_ROTATE, stop)
        }
    }

    /** Recenter the gimbal (yaw + pitch to home). */
    fun center() {
        netHandler.post { sendFrame(CMD_CENTER, byteArrayOf(0x01)) }
    }

    /** Release the socket and stop the background thread. */
    fun close() {
        netHandler.post {
            netHandler.removeCallbacks(repeatRunnable)
            socket?.close()
            socket = null
        }
        netThread.quitSafely()
    }

    // -- internals (all run on netHandler / netThread) ------------------------

    private fun sendFrame(cmdId: Int, data: ByteArray) {
        try {
            val frame = buildFrame(cmdId, data)
            val sock = socket ?: DatagramSocket().also { socket = it }
            sock.send(DatagramPacket(frame, frame.size, address, PORT))
        } catch (_: Exception) {
            // Best-effort teleop: a dropped command is recoverable (the repeat
            // loop resends, and the user simply presses again). Don't crash.
        }
    }

    private fun buildFrame(cmdId: Int, data: ByteArray): ByteArray {
        val len = data.size
        val frame = ByteArray(HEADER_LEN + len + CRC_LEN)
        frame[0] = 0x55
        frame[1] = 0x66
        frame[2] = 0x01                       // CTRL: need_ack
        frame[3] = (len and 0xFF).toByte()    // DATA_LEN (LE)
        frame[4] = ((len shr 8) and 0xFF).toByte()
        val s = seq++ and 0xFFFF
        frame[5] = (s and 0xFF).toByte()      // SEQ (LE)
        frame[6] = ((s shr 8) and 0xFF).toByte()
        frame[7] = (cmdId and 0xFF).toByte()  // CMD_ID
        System.arraycopy(data, 0, frame, HEADER_LEN, len)

        val crc = crc16(frame, HEADER_LEN + len)
        frame[HEADER_LEN + len] = (crc and 0xFF).toByte()         // CRC (LE)
        frame[HEADER_LEN + len + 1] = ((crc shr 8) and 0xFF).toByte()
        return frame
    }

    /** CRC-16/XMODEM (poly 0x1021, init 0x0000, no reflection, no final XOR). */
    private fun crc16(buf: ByteArray, len: Int): Int {
        var crc = 0
        for (i in 0 until len) {
            crc = crc xor ((buf[i].toInt() and 0xFF) shl 8)
            for (b in 0 until 8) {
                crc = if (crc and 0x8000 != 0) (crc shl 1) xor 0x1021 else crc shl 1
                crc = crc and 0xFFFF
            }
        }
        return crc and 0xFFFF
    }

    private fun clampSpeed(v: Int): Byte = v.coerceIn(-100, 100).toByte()

    companion object {
        private const val HOST = "192.168.144.25"
        private const val PORT = 37260
        private const val HEADER_LEN = 8
        private const val CRC_LEN = 2
        private const val CMD_ROTATE = 0x07
        private const val CMD_CENTER = 0x08
        private const val REPEAT_MS = 120L

        // Fixed teleop speed for a held arrow (-100..100). Tune to taste.
        const val MOVE_SPEED = 50
    }
}
