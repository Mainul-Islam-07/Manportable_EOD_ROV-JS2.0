package com.avatar.dashboard

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.text.font.FontWeight

/**
 * Telescope joint is published in METERS (linear); everything else is in
 * RADIANS.  Rotary joints are shown primarily in DEGREES (large), with the
 * raw radian value as a small secondary line.  The telescope shows mm / m.
 */
private fun isLinear(name: String) =
    name.contains("telescope", true) || name.contains("telescop", true)

@Composable
fun JointsScreen(ui: UiState) {
    val joints = ui.telemetry.joints
    if (joints.isEmpty()) {
        EmptyState("Waiting for /joint_states…")
        return
    }
    // Landscape 7" screen: two columns so all joints are visible at once.
    LazyVerticalGrid(
        columns = GridCells.Fixed(2),
        modifier = Modifier.fillMaxSize().padding(horizontal = 10.dp),
        contentPadding = PaddingValues(vertical = 10.dp),
        verticalArrangement = Arrangement.spacedBy(6.dp),
        horizontalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        items(joints, key = { it.name }) { JointRow(it) }
    }
}

@Composable
private fun JointRow(j: JointVal) {
    val linear = isLinear(j.name)
    val primary: String
    val secondary: String
    if (linear) {
        primary = "%.0f mm".format(j.position * 1000.0)
        secondary = "%.3f m".format(j.position)
    } else {
        primary = "%.1f°".format(Math.toDegrees(j.position))
        secondary = "%.3f rad".format(j.position)
    }

    Row(
        Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(10.dp))
            .background(SurfaceDark)
            .padding(horizontal = 12.dp, vertical = 11.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(prettyName(j.name), color = TextHi,
            fontWeight = FontWeight.SemiBold, fontSize = 14.sp, maxLines = 1)
        Column(horizontalAlignment = Alignment.End) {
            Text(primary, color = Accent, fontWeight = FontWeight.Bold,
                fontSize = 19.sp, maxLines = 1)
            Text(secondary, color = TextDim, fontSize = 11.sp)
        }
    }
}

private fun prettyName(raw: String): String =
    raw.replace('_', ' ')
       .split(' ')
       .joinToString(" ") { it.replaceFirstChar { c -> c.uppercase() } }
