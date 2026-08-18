package com.avatarrobot.camswitcher.twin

import android.content.Context
import android.graphics.PixelFormat
import android.opengl.GLSurfaceView
import android.view.MotionEvent

/**
 * A [GLSurfaceView] that hosts the [RobotRenderer] as a TRANSPARENT overlay on
 * top of the camera feed. Touch spins the model around its vertical (Z) axis
 * only — there is no tilt and no pinch-zoom (both removed from the original
 * JS2.0_Digital_Twin orbit rig for the HUD use-case). Renders continuously so
 * live joint updates appear smoothly.
 *
 * Transparency requires three things, all set before [setRenderer]:
 *  - an EGL config with an 8-bit alpha channel,
 *  - a TRANSLUCENT surface pixel format,
 *  - z-order as a media overlay so it composites above the camera SurfaceView.
 */
class OrbitGLSurfaceView(
    context: Context,
    private val robotRenderer: RobotRenderer,
) : GLSurfaceView(context) {

    private var lastX = 0f

    init {
        setEGLContextClientVersion(2)
        // RGBA_8888 + 16-bit depth, no stencil — gives us a real alpha channel.
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        // Composite above the camera RtspSurfaceView (which is the bottom surface).
        setZOrderMediaOverlay(true)
        holder.setFormat(PixelFormat.TRANSLUCENT)
        setRenderer(robotRenderer)
        renderMode = RENDERMODE_CONTINUOUSLY
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> lastX = e.x
            MotionEvent.ACTION_MOVE -> {
                val dx = e.x - lastX
                robotRenderer.orbit(-dx * DRAG_TO_RAD)   // Z-axis spin only
                lastX = e.x
            }
        }
        return true
    }

    companion object {
        private const val DRAG_TO_RAD = 0.006f
    }
}
