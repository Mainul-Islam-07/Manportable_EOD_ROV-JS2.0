package com.avatar.dashboard

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.lazy.grid.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun DiagnosticsScreen(ui: UiState) {
    val diags = ui.telemetry.diagnostics
    if (diags.isEmpty()) {
        EmptyState("Waiting for /motor_diagnostics…")
        return
    }
    // Landscape 7" screen: two columns so all ~12 motors fit with little
    // or no scrolling.
    LazyVerticalGrid(
        columns = GridCells.Fixed(2),
        modifier = Modifier.fillMaxSize().padding(horizontal = 10.dp),
        contentPadding = PaddingValues(vertical = 6.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp),
        horizontalArrangement = Arrangement.spacedBy(4.dp),
    ) {
        items(diags, key = { it.name }) { MotorCard(it) }
    }
}

@Composable
private fun MotorCard(d: MotorDiag) {
    var expanded by remember { mutableStateOf(false) }
    val sc = statusColor(d.level)

    Column(
        Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(10.dp))
            .background(SurfaceDark)
            .alpha(if (d.fresh) 1f else 0.45f)   // dim if data is stale (>fresh window)
            .clickable { expanded = !expanded }
            .padding(0.dp)
    ) {
        // Header row with a colored left bar feel via background tint.
        Row(
            Modifier
                .fillMaxWidth()
                .background(sc.copy(alpha = 0.14f))
                .padding(horizontal = 9.dp, vertical = 5.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(Modifier.width(5.dp).height(22.dp)
                .clip(RoundedCornerShape(3.dp)).background(sc))
            Spacer(Modifier.width(7.dp))
            Column(Modifier.weight(1f)) {
                Text(d.name, color = TextHi, fontWeight = FontWeight.Bold,
                    fontSize = 12.sp, maxLines = 1)
                Text(d.swState.ifBlank { "—" }, color = TextDim,
                    fontSize = 9.sp, maxLines = 1)
            }
            StatusPill(d.level, sc)
        }

        // Always-visible quick line: voltage / temps
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 3.dp),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            QuickMetric("V", d.voltage)
            QuickMetric("I", d.current)
            QuickMetric("coil°", d.coilTemp)
            QuickMetric("brd°", d.boardTemp)
        }

        AnimatedVisibility(expanded) {
            Column(Modifier.padding(horizontal = 14.dp, vertical = 8.dp)) {
                Divider()
                InfoRow("Node ID", d.hwId)
                InfoRow("Bus", d.bus.ifBlank { "—" })
                InfoRow("Statusword", d.statusword)
                InfoRow("SW flags", d.swFlags.ifBlank { "none" })
                InfoRow("Error reg", "${d.errorReg}  (${d.errorRegFlags})")
                InfoRow("Error code", "${d.errorCode}  (${d.errorCodeFlags})",
                    valueColor = if (d.errorCodeFlags.isNotBlank() &&
                        d.errorCodeFlags != "none") StatusWarn else TextHi)
                InfoRow("Heartbeat", "${d.hbState}  #${d.hbCount}")
                if (d.message.isNotBlank()) {
                    Spacer(Modifier.height(4.dp))
                    Text(d.message, color = TextDim, fontSize = 12.sp)
                }
            }
        }
    }
}

@Composable
private fun StatusPill(level: String, sc: Color) {
    Box(
        Modifier.clip(RoundedCornerShape(8.dp)).background(sc)
            .padding(horizontal = 10.dp, vertical = 4.dp)
    ) {
        Text(level.uppercase(), color = Color(0xFF0B0E13),
            fontWeight = FontWeight.Bold, fontSize = 11.sp)
    }
}

@Composable
private fun QuickMetric(label: String, value: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(value, color = TextHi, fontWeight = FontWeight.SemiBold, fontSize = 12.sp)
        Text(label, color = TextDim, fontSize = 8.sp)
    }
}

@Composable
private fun Divider() {
    Box(Modifier.fillMaxWidth().height(1.dp)
        .background(SurfaceHi).padding(vertical = 4.dp))
    Spacer(Modifier.height(4.dp))
}

@Composable
fun EmptyState(text: String) {
    Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Text(text, color = TextDim, fontSize = 13.sp)
    }
}
