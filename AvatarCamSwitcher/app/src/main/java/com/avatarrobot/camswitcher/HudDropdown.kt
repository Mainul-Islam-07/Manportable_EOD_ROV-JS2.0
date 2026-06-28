package com.avatarrobot.camswitcher

import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.graphics.drawable.GradientDrawable
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.LinearLayout
import android.widget.PopupWindow

/**
 * A compact tap-to-expand dropdown for the HUD (CAMERA / ZOOM / SOUND).
 *
 * The [anchor] is a collapsed button showing the current selection. Tapping it
 * pops a vertical list of options; the currently-selected option is tinted green.
 * Choosing an option collapses the popup, updates the anchor label, and fires
 * [onSelect]. When [dropUp] is true the list opens ABOVE the anchor (for the
 * bottom-of-screen CAMERA / ZOOM chips); otherwise it drops below.
 *
 * [setCurrent] keeps the collapsed label in sync when the selection changes from
 * elsewhere (e.g. a physical key switching the camera) WITHOUT firing [onSelect].
 */
class HudDropdown<T>(
    private val anchor: Button,
    private val options: List<Pair<T, String>>,   // value to label
    private val dropUp: Boolean = false,
    // Maps (value, default option label) → the COLLAPSED chip text only; the
    // expanded list always uses the option label. e.g. SOUND shows "SOUND" at rest.
    private val collapsedLabel: ((T, String) -> String)? = null,
    private val onSelect: (T) -> Unit,
) {
    private var current: T = options.first().first
    private var popup: PopupWindow? = null

    init {
        anchor.text = collapsedFor(current)
        anchor.setOnClickListener { toggle() }
    }

    /** Update the shown selection without invoking [onSelect]. */
    fun setCurrent(value: T) {
        current = value
        anchor.text = collapsedFor(value)
    }

    private fun labelFor(value: T): String =
        options.firstOrNull { it.first == value }?.second ?: value.toString()

    private fun collapsedFor(value: T): String =
        collapsedLabel?.invoke(value, labelFor(value)) ?: labelFor(value)

    private fun toggle() {
        if (popup?.isShowing == true) { popup?.dismiss(); return }
        showPopup()
    }

    private fun showPopup() {
        val ctx = anchor.context
        val list = LinearLayout(ctx).apply {
            orientation = LinearLayout.VERTICAL
            background = GradientDrawable().apply {
                setColor(Color.parseColor("#CC202020"))
                cornerRadius = dp(6f)
                setStroke(dp(1f).toInt(), Color.parseColor("#55FFFFFF"))
            }
            val pad = dp(4f).toInt()
            setPadding(pad, pad, pad, pad)
        }

        for ((value, label) in options) {
            val selected = value == current
            val item = Button(ctx).apply {
                text = label
                isAllCaps = true
                setTextColor(Color.WHITE)
                textSize = 11f
                gravity = Gravity.CENTER
                minWidth = 0; minHeight = 0
                setPadding(dp(10f).toInt(), dp(5f).toInt(), dp(10f).toInt(), dp(5f).toInt())
                stateListAnimator = null
                background = GradientDrawable().apply {
                    setColor(if (selected) Color.parseColor("#FF2E7D32") else Color.parseColor("#33FFFFFF"))
                    cornerRadius = dp(4f)
                    if (selected) setStroke(dp(1f).toInt(), Color.parseColor("#FF66BB6A"))
                }
                setOnClickListener {
                    setCurrent(value)
                    popup?.dismiss()
                    onSelect(value)
                }
            }
            val lp = LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT
            ).apply { topMargin = dp(2f).toInt(); bottomMargin = dp(2f).toInt() }
            list.addView(item, lp)
        }

        popup = PopupWindow(
            list,
            ViewGroup.LayoutParams.WRAP_CONTENT,
            ViewGroup.LayoutParams.WRAP_CONTENT,
            true
        ).apply {
            // Transparent backing so only our rounded list shows; dismiss on outside tap.
            setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
            isOutsideTouchable = true
            elevation = dp(8f)
            if (dropUp) {
                // Measure the list so we can offset it fully above the anchor.
                list.measure(
                    View.MeasureSpec.makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED),
                    View.MeasureSpec.makeMeasureSpec(0, View.MeasureSpec.UNSPECIFIED))
                val yOff = -(anchor.height + list.measuredHeight + dp(2f).toInt())
                showAsDropDown(anchor, 0, yOff)
            } else {
                showAsDropDown(anchor, 0, dp(2f).toInt())
            }
        }
    }

    private fun dp(v: Float): Float = v * anchor.resources.displayMetrics.density
}
