package com.avatar.dashboard

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

// ---- Palette: deep near-black background, vivid status colors that
// ---- stay legible on a small low-nit panel and in sunlight. ----
val BgDark      = Color(0xFF0B0E13)
val SurfaceDark = Color(0xFF161B24)
val SurfaceHi   = Color(0xFF1F2733)
val TextHi      = Color(0xFFF2F5F8)
val TextDim     = Color(0xFF9AA6B5)
val Accent      = Color(0xFF35C4FF)

val StatusOk    = Color(0xFF2ECC71)
val StatusWarn  = Color(0xFFF5B400)
val StatusFault = Color(0xFFFF3B30)
val StatusStale = Color(0xFF5A6675)

fun statusColor(level: String): Color = when (level.uppercase()) {
    "OK"    -> StatusOk
    "WARN"  -> StatusWarn
    "FAULT", "ERROR" -> StatusFault
    else    -> StatusStale
}

private val AvatarColors = darkColorScheme(
    primary = Accent,
    background = BgDark,
    surface = SurfaceDark,
    onPrimary = BgDark,
    onBackground = TextHi,
    onSurface = TextHi,
)

// Compact type — small 7" screen, fit more without scrolling.
private val AvatarType = Typography(
    titleLarge = TextStyle(fontSize = 18.sp, fontWeight = FontWeight.Bold),
    titleMedium = TextStyle(fontSize = 15.sp, fontWeight = FontWeight.SemiBold),
    bodyLarge = TextStyle(fontSize = 14.sp),
    bodyMedium = TextStyle(fontSize = 12.sp),
    labelSmall = TextStyle(fontSize = 11.sp, fontWeight = FontWeight.Medium),
)

@Composable
fun AvatarTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = AvatarColors,
        typography = AvatarType,
        content = content,
    )
}
