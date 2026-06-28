package com.avatarrobot.camswitcher.twin

import android.content.res.AssetManager
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.util.Log
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.PI

/**
 * OpenGL ES 2.0 renderer that draws the robot as posed solid STL meshes.
 *
 * Each frame: read the latest joint values from [jointState], run forward
 * kinematics ([Kinematics.worldTransforms]) to get a world matrix per link,
 * and draw each mesh with a single-directional-light Lambert shader. The
 * camera is an orbit rig (azimuth / elevation / distance) driven by touch.
 *
 * The robot frame is Z-up (URDF), so the camera's up vector is +Z.
 *
 * Ported from JS2.0_Digital_Twin and adapted to overlay a live camera feed:
 * the clear color is fully TRANSPARENT and the meshes are drawn slightly
 * translucent over alpha blending, so the video shows through the panel.
 */
class RobotRenderer(
    private val assets: AssetManager,
    private val jointState: JointState,
) : GLSurfaceView.Renderer {

    private class Mesh(val name: String, val vbo: Int, val count: Int,
                       val r: Float, val g: Float, val b: Float)

    private val meshes = ArrayList<Mesh>()
    private var program = 0
    private var aPos = 0
    private var aNormal = 0
    private var uMVP = 0
    private var uModel = 0
    private var uColor = 0
    private var uLightDir = 0

    private val proj = FloatArray(16)
    private val view = FloatArray(16)
    private val vp = FloatArray(16)
    private val mvp = FloatArray(16)

    // ---- orbit camera (touch-controlled; read on the GL thread) ----
    // Z-axis-only rotation: azimuth spins around the vertical (Z) axis; elevation
    // and distance are FIXED (no pinch-zoom, no tilt) per the HUD overlay spec.
    @Volatile private var azimuth = (-60.0 * PI / 180.0).toFloat()
    private val elevation = (20.0 * PI / 180.0).toFloat()
    private val distance = 1.7f

    // Look-at target: roughly the middle of the standing robot (Z-up).
    private val target = floatArrayOf(0f, -0.15f, 0.35f)

    /** Spin the model about its vertical (Z) axis only. Elevation is locked. */
    fun orbit(dAzimuthRad: Float) {
        azimuth += dAzimuthRad
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        // Transparent clear so the camera feed behind the GLSurfaceView shows through.
        GLES20.glClearColor(0f, 0f, 0f, 0f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        GLES20.glEnable(GLES20.GL_CULL_FACE)
        GLES20.glCullFace(GLES20.GL_BACK)
        // Straight alpha blending for the translucent robot over the video.
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)

        buildProgram()
        loadMeshes()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        val aspect = if (height == 0) 1f else width.toFloat() / height
        Matrix.perspectiveM(proj, 0, 45f, aspect, 0.03f, 30f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        if (program == 0 || meshes.isEmpty()) return

        // Eye position from spherical orbit around the target (Z-up).
        val ce = cos(elevation); val se = sin(elevation)
        val ca = cos(azimuth); val sa = sin(azimuth)
        val eyeX = target[0] + distance * ce * ca
        val eyeY = target[1] + distance * ce * sa
        val eyeZ = target[2] + distance * se
        Matrix.setLookAtM(view, 0, eyeX, eyeY, eyeZ, target[0], target[1], target[2], 0f, 0f, 1f)
        Matrix.multiplyMM(vp, 0, proj, 0, view, 0)

        GLES20.glUseProgram(program)
        GLES20.glUniform3f(uLightDir, 0.4f, 0.5f, 1.0f)

        val world = Kinematics.worldTransforms(jointState.snapshot())

        for (m in meshes) {
            val model = world[m.name] ?: continue
            Matrix.multiplyMM(mvp, 0, vp, 0, model, 0)
            GLES20.glUniformMatrix4fv(uMVP, 1, false, mvp, 0)
            GLES20.glUniformMatrix4fv(uModel, 1, false, model, 0)
            GLES20.glUniform3f(uColor, m.r, m.g, m.b)

            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, m.vbo)
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, STRIDE, 0)
            GLES20.glEnableVertexAttribArray(aNormal)
            GLES20.glVertexAttribPointer(aNormal, 3, GLES20.GL_FLOAT, false, STRIDE, 12)

            GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, m.count)

            GLES20.glDisableVertexAttribArray(aPos)
            GLES20.glDisableVertexAttribArray(aNormal)
        }
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    // ---- setup helpers ------------------------------------------------------

    private fun loadMeshes() {
        meshes.clear()
        for (link in Kinematics.LINKS) {
            if (link.heavy && !Kinematics.INCLUDE_BASE) continue
            val model = try {
                StlModel.fromAsset(assets, "meshes/${link.mesh}")
            } catch (e: Exception) {
                Log.e(TAG, "failed to load ${link.mesh}: ${e.message}"); continue
            }
            if (model.vertexCount == 0) continue

            val ids = IntArray(1)
            GLES20.glGenBuffers(1, ids, 0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, ids[0])
            GLES20.glBufferData(
                GLES20.GL_ARRAY_BUFFER,
                model.vertexCount * STRIDE,
                model.interleaved,
                GLES20.GL_STATIC_DRAW
            )
            meshes.add(Mesh(link.name, ids[0], model.vertexCount, link.r, link.g, link.b))
        }
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        Log.i(TAG, "loaded ${meshes.size} link meshes")
    }

    private fun buildProgram() {
        val vs = compile(GLES20.GL_VERTEX_SHADER, VERT_SRC)
        val fs = compile(GLES20.GL_FRAGMENT_SHADER, FRAG_SRC)
        if (vs == 0 || fs == 0) return
        program = GLES20.glCreateProgram()
        GLES20.glAttachShader(program, vs)
        GLES20.glAttachShader(program, fs)
        GLES20.glLinkProgram(program)
        val ok = IntArray(1)
        GLES20.glGetProgramiv(program, GLES20.GL_LINK_STATUS, ok, 0)
        if (ok[0] == 0) {
            Log.e(TAG, "link failed: ${GLES20.glGetProgramInfoLog(program)}")
            program = 0; return
        }
        aPos = GLES20.glGetAttribLocation(program, "aPos")
        aNormal = GLES20.glGetAttribLocation(program, "aNormal")
        uMVP = GLES20.glGetUniformLocation(program, "uMVP")
        uModel = GLES20.glGetUniformLocation(program, "uModel")
        uColor = GLES20.glGetUniformLocation(program, "uColor")
        uLightDir = GLES20.glGetUniformLocation(program, "uLightDir")
    }

    private fun compile(type: Int, src: String): Int {
        val s = GLES20.glCreateShader(type)
        GLES20.glShaderSource(s, src)
        GLES20.glCompileShader(s)
        val ok = IntArray(1)
        GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0)
        if (ok[0] == 0) {
            Log.e(TAG, "shader compile failed: ${GLES20.glGetShaderInfoLog(s)}")
            GLES20.glDeleteShader(s); return 0
        }
        return s
    }

    private fun cos(v: Float) = kotlin.math.cos(v.toDouble()).toFloat()
    private fun sin(v: Float) = kotlin.math.sin(v.toDouble()).toFloat()

    companion object {
        private const val TAG = "JS2Renderer"
        private const val STRIDE = 6 * 4   // 6 floats (pos+normal) per vertex

        private const val VERT_SRC = """
            uniform mat4 uMVP;
            uniform mat4 uModel;
            attribute vec3 aPos;
            attribute vec3 aNormal;
            varying vec3 vWorldN;
            void main() {
                vWorldN = mat3(uModel) * aNormal;
                gl_Position = uMVP * vec4(aPos, 1.0);
            }
        """

        // Alpha 0.9 so the robot reads as solid but the camera feed still bleeds
        // through faintly, matching the translucent-overlay HUD design.
        private const val FRAG_SRC = """
            precision mediump float;
            varying vec3 vWorldN;
            uniform vec3 uColor;
            uniform vec3 uLightDir;
            void main() {
                vec3 n = normalize(vWorldN);
                // two-sided lambert so flipped/STL normals never go fully black
                float diff = abs(dot(n, normalize(uLightDir)));
                vec3 c = uColor * (0.35 + 0.75 * diff);
                gl_FragColor = vec4(min(c, 1.0), 0.9);
            }
        """
    }
}
