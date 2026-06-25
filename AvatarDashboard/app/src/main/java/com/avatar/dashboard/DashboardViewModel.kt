package com.avatar.dashboard

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.delay

enum class LinkState { LIVE, STALE, NO_LINK }

data class UiState(
    val telemetry: Telemetry = Telemetry(),
    val link: LinkState = LinkState.NO_LINK,
    val ageMs: Long = -1,
    val anyFault: Boolean = false,
    /** Battery charge 0-100, or -1 when unknown. */
    val batteryPct: Int = -1,
    /** Source bus voltage, or NaN when unknown. */
    val batteryVolts: Double = Double.NaN,
    /** Drive_CAN encoder memory-battery voltage, or NaN when unknown. */
    val driveMemVolts: Double = Double.NaN,
    /** Arm_CAN encoder memory-battery voltage, or NaN when unknown. */
    val armMemVolts: Double = Double.NaN,
    /** Drive memory battery above threshold; null when unknown. */
    val driveMemOk: Boolean? = null,
    /** Arm memory battery above threshold; null when unknown. */
    val armMemOk: Boolean? = null,
)

/**
 * Holds the [UdpTelemetryReceiver] for the whole app lifetime and folds the
 * raw telemetry together with a 4 Hz "clock" so the UI can show packet age
 * and flip to STALE / NO_LINK when datagrams stop arriving.
 */
class DashboardViewModel : ViewModel() {

    private val receiver = UdpTelemetryReceiver(port = LISTEN_PORT)

    init { receiver.start() }

    // Emits every 1 s so the age readout advances in clean whole-second
    // steps (no sub-second jitter).  Link thresholds are 3 s / 8 s, so a
    // 1 s cadence is still plenty responsive for LIVE/STALE/NO_LINK.
    private val ticker = flow {
        while (true) { emit(System.currentTimeMillis()); delay(1000) }
    }

    val ui: StateFlow<UiState> =
        combine(receiver.state, ticker) { tel, now ->
            val age = if (tel.rxAtMs == 0L) -1 else now - tel.rxAtMs
            val link = when {
                age < 0           -> LinkState.NO_LINK
                age <= STALE_MS   -> LinkState.LIVE
                age <= NO_LINK_MS -> LinkState.STALE
                else              -> LinkState.NO_LINK
            }
            val fault = tel.diagnostics.any {
                it.level.equals("FAULT", true) || it.level.equals("ERROR", true)
            }
            // Battery is held forever: always pass through the last value the
            // bridge sent, even when the UDP link is down.  It only stays
            // unknown (-1) before any reading has ever arrived.
            UiState(telemetry = tel, link = link, ageMs = age, anyFault = fault,
                    batteryPct = tel.batteryPct, batteryVolts = tel.batteryVolts,
                    driveMemVolts = tel.driveMemVolts, armMemVolts = tel.armMemVolts,
                    driveMemOk = tel.driveMemOk, armMemOk = tel.armMemOk)
        }.stateIn(viewModelScope, SharingStarted.Eagerly, UiState())

    override fun onCleared() {
        receiver.stop()
        super.onCleared()
    }

    companion object {
        const val LISTEN_PORT = 9870
        // Windows are generous so a SLOW bridge rate (e.g. 2 Hz = one packet
        // every 500 ms) plus network jitter never flickers the indicator:
        //   LIVE    : a packet within the last 3 s
        //   STALE   : 3–8 s since the last packet
        //   NO LINK : nothing for over 8 s (bridge down / network lost)
        private const val STALE_MS = 3000L
        private const val NO_LINK_MS = 8000L
    }
}
