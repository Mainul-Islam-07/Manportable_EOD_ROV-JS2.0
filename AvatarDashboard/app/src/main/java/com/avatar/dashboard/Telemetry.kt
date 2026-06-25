package com.avatar.dashboard

/**
 * Plain immutable holders for one telemetry snapshot decoded from a UDP
 * datagram sent by telemetry_udp_bridge.py.  Kept dependency-free (manual
 * JSON parsing) so the app has no networking/serialization libraries to
 * pull in — important for a small, locked-down MK32 device.
 */

data class MotorDiag(
    val name: String,
    val level: String,      // OK | WARN | FAULT | STALE
    val message: String,
    val hwId: String,
    val bus: String,
    val voltage: String,
    val current: String,
    val coilTemp: String,
    val boardTemp: String,
    val statusword: String,
    val swState: String,
    val swFlags: String,
    val errorReg: String,
    val errorRegFlags: String,
    val errorCode: String,
    val errorCodeFlags: String,
    val hbState: String,
    val hbCount: String,
    val fresh: Boolean = true,   // updated within the bridge's fresh window
    val ageMs: Int = -1,         // ms since this motor's last diagnostics
)

data class JointVal(
    val name: String,
    val position: Double,
)

/** One fully-decoded snapshot. `null` fields mean that section was absent. */
data class Telemetry(
    val seq: Long = 0,
    val stamp: Double = 0.0,
    /** Battery charge 0-100, or -1 when the bridge had no fresh voltage. */
    val batteryPct: Int = -1,
    /** Source bus voltage for [batteryPct], or NaN when unavailable. */
    val batteryVolts: Double = Double.NaN,
    /** Drive_CAN encoder memory-battery voltage, or NaN when unavailable. */
    val driveMemVolts: Double = Double.NaN,
    /** Arm_CAN encoder memory-battery voltage, or NaN when unavailable. */
    val armMemVolts: Double = Double.NaN,
    /** Drive memory battery above threshold; null until a reading arrives. */
    val driveMemOk: Boolean? = null,
    /** Arm memory battery above threshold; null until a reading arrives. */
    val armMemOk: Boolean? = null,
    val diagnostics: List<MotorDiag> = emptyList(),
    val joints: List<JointVal> = emptyList(),
    val armActive: List<String> = emptyList(),
    val driveActive: List<String> = emptyList(),
    /** Wall-clock millis when THIS device received the packet. */
    val rxAtMs: Long = 0L,
)
