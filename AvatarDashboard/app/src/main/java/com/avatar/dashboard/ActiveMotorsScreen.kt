package com.avatar.dashboard

import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Full known motor roster per bus (from motor_settings.xlsx).  A motor that
 * appears in the bridge's active list shows a BLINKING RED light (alive);
 * the rest show a STEADY GREY light (dropped / missing).
 *
 * Landscape 7" layout: the two buses sit SIDE BY SIDE, each its own column,
 * so the whole fleet is visible at a glance without scrolling.
 */
private val ARM_ROSTER = listOf(
    "Turret", "Left_Differential", "Right_Differential",
    "Telescopic", "Wrist", "Gripper_360", "Gripper"
)
private val DRIVE_ROSTER = listOf(
    "Left_Drive", "Right_Drive", "Front_Flipper", "Rear_Flipper"
)

@Composable
fun ActiveMotorsScreen(ui: UiState) {
    val arm = ui.telemetry.armActive.toSet()
    val drive = ui.telemetry.driveActive.toSet()

    Row(
        Modifier.fillMaxSize().padding(10.dp),
        horizontalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        BusColumn("ARM BUS", ARM_ROSTER, arm, Modifier.weight(1f))
        BusColumn("DRIVE BUS", DRIVE_ROSTER, drive, Modifier.weight(1f))
    }
}

@Composable
private fun BusColumn(
    title: String,
    roster: List<String>,
    active: Set<String>,
    modifier: Modifier = Modifier
) {
    val aliveCount = roster.count { it in active }
    Column(modifier) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 3.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(title, color = TextHi, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Spacer(Modifier.weight(1f))
            Text("$aliveCount / ${roster.size}",
                color = if (aliveCount == roster.size) StatusOk else StatusWarn,
                fontWeight = FontWeight.SemiBold, fontSize = 12.sp)
        }
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            roster.forEach { name ->
                MotorChip(name, alive = name in active)
            }
        }
    }
}

@Composable
private fun MotorChip(name: String, alive: Boolean) {
    Row(
        Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(if (alive) SurfaceHi else SurfaceDark)
            .padding(horizontal = 8.dp, vertical = 6.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        StatusLight(alive)
        Spacer(Modifier.width(7.dp))
        Text(
            name.replace('_', ' '),
            color = if (alive) TextHi else TextDim,
            fontSize = 11.sp, maxLines = 1,
            fontWeight = if (alive) FontWeight.SemiBold else FontWeight.Normal
        )
    }
}

/** Active = steady red light.  Inactive = steady grey light. */
@Composable
private fun StatusLight(alive: Boolean) {
    val color = if (alive) StatusFault else StatusStale
    Box(Modifier.size(11.dp).clip(CircleShape).background(color))
}
