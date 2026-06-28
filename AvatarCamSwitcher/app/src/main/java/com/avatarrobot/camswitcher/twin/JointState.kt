package com.avatarrobot.camswitcher.twin

import org.json.JSONObject

/**
 * Thread-safe latest-known joint state shared between the UDP receive thread
 * (writer, via [update]) and the GL render thread + UI handler (readers).
 *
 * Joint values are held by name exactly as published (e.g. "turret_Joint").
 * All angles are RADIANS except `telescope_Joint`, which is METERS of linear
 * extension. A joint absent from a packet is treated as UNKNOWN and its last
 * value is HELD (never reset to zero) — per the telemetry contract.
 *
 * Staleness is derived from `seq`: if the sequence number stops changing for
 * longer than [STALE_AFTER_MS], the link is considered stale. A robot restart
 * resets seq to a low number; any change (up or down) counts as "alive".
 *
 * Ported verbatim from JS2.0_Digital_Twin (only the package changed) so the
 * digital twin can run inside the camera switcher.
 */
class JointState {

    // Snapshot-replacement concurrency: readers grab the volatile reference;
    // the writer publishes a fresh immutable map. No locks on the hot path.
    @Volatile private var joints: Map<String, Float> = emptyMap()

    @Volatile var seq: Long = -1L
        private set
    @Volatile var lastSeqChangeMs: Long = 0L
        private set
    @Volatile var everReceived: Boolean = false
        private set

    /** Parse one full snapshot. Called off the UI thread. Defensive throughout. */
    fun update(json: JSONObject, nowMs: Long) {
        // -- joints: merge over the held map so missing keys keep their value --
        json.optJSONObject("joints")?.let { j ->
            val merged = HashMap(joints)
            val keys = j.keys()
            while (keys.hasNext()) {
                val name = keys.next()
                // optDouble skips nulls/non-numbers gracefully.
                val v = j.optDouble(name, Double.NaN)
                if (!v.isNaN()) merged[name] = v.toFloat()
            }
            joints = merged
        }

        // -- seq / staleness bookkeeping --
        val newSeq = json.optLong("seq", seq)
        if (newSeq != seq) {
            seq = newSeq
            lastSeqChangeMs = nowMs
        }
        everReceived = true
    }

    /** Current joint values (immutable snapshot — safe to read on any thread). */
    fun snapshot(): Map<String, Float> = joints

    /** True once data has arrived but seq has not advanced within the window. */
    fun isStale(nowMs: Long): Boolean =
        everReceived && (nowMs - lastSeqChangeMs) > STALE_AFTER_MS

    /** True before any packet has ever been received. */
    fun isWaiting(): Boolean = !everReceived

    companion object { const val STALE_AFTER_MS = 1500L }
}
