package com.avatar.dashboard

import androidx.compose.animation.animateColorAsState
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/** Persistent top strip: link state, packet age, global fault flag. */
@Composable
fun StatusStrip(ui: UiState) {
    val bg by animateColorAsState(
        if (ui.anyFault) StatusFault.copy(alpha = 0.18f) else SurfaceHi,
        label = "stripBg"
    )

    Row(
        Modifier
            .fillMaxWidth()
            .background(bg)
            .padding(horizontal = 10.dp, vertical = 5.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        BatteryChip(pct = ui.batteryPct, volts = ui.batteryVolts)

        Spacer(Modifier.width(8.dp))
        MemBatteryChip(label = "DRV mem", volts = ui.driveMemVolts, ok = ui.driveMemOk)
        Spacer(Modifier.width(6.dp))
        MemBatteryChip(label = "ARM mem", volts = ui.armMemVolts, ok = ui.armMemOk)

        Spacer(Modifier.weight(1f))

        // Coarse, whole-second age so the readout doesn't jitter every tick.
        // (Link state still recomputes fast in the ViewModel; only the text
        //  shown here is rounded.)
        val ageText = when {
            ui.ageMs < 0    -> "—"
            ui.ageMs < 1000 -> "<1 s"
            else            -> "${ui.ageMs / 1000} s"
        }
        Text("age $ageText", color = TextDim, fontSize = 11.sp)

        if (ui.anyFault) {
            Spacer(Modifier.width(12.dp))
            Box(
                Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(StatusFault)
                    .padding(horizontal = 8.dp, vertical = 2.dp)
            ) {
                Text("FAULT", color = Color.White,
                    fontWeight = FontWeight.Bold, fontSize = 11.sp)
            }
        }
    }
}

/**
 * Top-left battery indicator.
 *   pct  : 0-100, or -1 when unknown ("--")
 *   volts: source voltage, shown as "49.2V" when finite
 * Colour: green >=50, amber >=20, red below, grey when unknown.
 */
@Composable
fun BatteryChip(pct: Int, volts: Double) {
    val known = pct in 0..100
    val color = when {
        !known      -> StatusStale
        pct >= 50   -> StatusOk
        pct >= 20   -> StatusWarn
        else        -> StatusFault
    }
    val pctText = if (known) "$pct%" else "--"
    val voltText = if (volts.isFinite()) "  ${"%.1f".format(volts)}V" else ""

    Row(
        Modifier
            .clip(RoundedCornerShape(6.dp))
            .background(SurfaceDark)
            .padding(horizontal = 8.dp, vertical = 3.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            Modifier
                .size(width = 9.dp, height = 6.dp)
                .clip(RoundedCornerShape(1.dp))
                .background(color)
        )
        Spacer(Modifier.width(6.dp))
        Text(
            "$pctText$voltText",
            color = if (known) TextHi else TextDim,
            fontSize = 12.sp,
            fontWeight = FontWeight.SemiBold,
        )
    }
}


/**
 * Encoder memory-battery readout for one CAN bus, shown in the top strip.
 *   label : e.g. "DRV mem" / "ARM mem"
 *   volts : memory-cell voltage, shown as "3.95V" when finite, else "--"
 *   ok    : true = above threshold (green), false = low (red), null = unknown (grey)
 */
@Composable
fun MemBatteryChip(label: String, volts: Double, ok: Boolean?) {
    val color = when (ok) {
        true  -> StatusOk
        false -> StatusFault
        null  -> StatusStale
    }
    val voltText = if (volts.isFinite()) "${"%.2f".format(volts)}V" else "--"

    Row(
        Modifier
            .clip(RoundedCornerShape(6.dp))
            .background(SurfaceDark)
            .padding(horizontal = 8.dp, vertical = 3.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            Modifier
                .size(width = 9.dp, height = 6.dp)
                .clip(RoundedCornerShape(1.dp))
                .background(color)
        )
        Spacer(Modifier.width(6.dp))
        Text(
            "$label $voltText",
            color = if (ok == null) TextDim else TextHi,
            fontSize = 12.sp,
            fontWeight = FontWeight.SemiBold,
        )
    }
}


/** Small label/value row used inside cards. */
@Composable
fun InfoRow(label: String, value: String, valueColor: Color = TextHi) {
    Row(
        Modifier.fillMaxWidth().padding(vertical = 1.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(label, color = TextDim, fontSize = 11.sp)
        Text(value, color = valueColor, fontSize = 11.sp,
            fontWeight = FontWeight.Medium)
    }
}
