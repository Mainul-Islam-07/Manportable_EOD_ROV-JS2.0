package com.avatarrobot.camswitcher

import android.app.Activity
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat

/**
 * Show the status (notification) bar but hide the navigation bar.
 *
 * The video letterboxes (actual aspect, no zoom), so there are black bars anyway —
 * the operator asked to keep the notification bar visible in that space. We leave
 * `decorFitsSystemWindows` at its default (true) so the HUD is inset below the
 * status bar and never drawn under it. The nav bar stays hidden (swipe to reveal).
 *
 * Re-applied from onWindowFocusChanged because system dialogs (MediaProjection
 * consent, RECORD_AUDIO grant) can bring the nav bar back when they dismiss.
 */
fun Activity.configureSystemBars() {
    val controller = WindowInsetsControllerCompat(window, window.decorView)
    controller.hide(WindowInsetsCompat.Type.navigationBars())   // keep the status bar
    controller.systemBarsBehavior =
        WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
}
