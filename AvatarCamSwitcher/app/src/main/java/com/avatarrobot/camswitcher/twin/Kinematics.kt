package com.avatarrobot.camswitcher.twin

import android.opengl.Matrix
import kotlin.math.sqrt

/**
 * Forward kinematics for the part_assembly_for_urdf robot, transcribed
 * directly from its URDF (the tree is fixed, so there is no runtime XML
 * parsing). All matrices are 4x4 column-major float[16] — the OpenGL ES
 * convention — so they feed straight into the renderer's model matrix.
 *
 * Each joint's fixed frame is `T(origin.xyz) * R(origin.rpy)` (rpy is URDF
 * fixed-axis XYZ = Rz*Ry*Rx). The variable part is a rotation about `axis`
 * (revolute/continuous) or a translation along `axis` (prismatic).
 */

enum class JointType { REVOLUTE, CONTINUOUS, PRISMATIC, FIXED }

data class Joint(
    val name: String,           // matches the telemetry "joints" key
    val parent: String,
    val child: String,
    val ox: Float, val oy: Float, val oz: Float,        // origin xyz
    val roll: Float, val pitch: Float, val yaw: Float,  // origin rpy
    val type: JointType,
    val ax: Float, val ay: Float, val az: Float,        // joint axis
)

/** A renderable link: its mesh asset, base color, and whether it's heavy. */
data class LinkVisual(
    val name: String,
    val mesh: String,
    val r: Float, val g: Float, val b: Float,
    val heavy: Boolean = false,
)

object Kinematics {

    const val ROOT = "base_link"

    // ---- URDF joint tree (parents listed before children) -------------------
    val JOINTS: List<Joint> = listOf(
        Joint("turret_Joint", "base_link", "turret_Link",
            0f, 0f, 0.21647f, 3.1416f, 0f, 0f, JointType.REVOLUTE, 0f, 0f, 1f),
        Joint("shoulder_Joint", "turret_Link", "shoulder_Link",
            0f, 0f, -0.068f, -3.1416f, 0f, 0f, JointType.REVOLUTE, 1f, 0f, 0f),
        Joint("elbow_Joint", "shoulder_Link", "elbow_Link",
            0f, -0.3452f, 0f, -3.0892f, 0f, 0f, JointType.REVOLUTE, 1f, 0f, 0f),
        Joint("telescope_Joint", "elbow_Link", "telescope_Link",
            0f, 0f, -0.0915f, 0f, 0f, 0f, JointType.PRISMATIC, 0f, 1f, 0f),
        Joint("wrist_pan_Joint", "telescope_Link", "wrist_pan_Link",
            0f, -0.33238f, 0f, -0.05236f, 0f, 0f, JointType.REVOLUTE, 1f, 0f, 0f),
        Joint("wrist_roll_Joint", "wrist_pan_Link", "wrist_roll_Link",
            0.10485f, -0.037f, 0f, -3.1416f, 0f, 3.1416f, JointType.CONTINUOUS, 0f, 1f, 0f),
        Joint("gripper_Joint", "wrist_roll_Link", "gripper_Link",
            0f, -0.24015f, 0f, -1.5708f, -0.025495f, 0f, JointType.FIXED, 0f, 0f, 0f),
        Joint("front_flipper_Joint", "base_link", "front_flipper_Link",
            0f, 0.11592f, 0.085966f, 2.5747f, 0f, -3.1416f, JointType.REVOLUTE, 1f, 0f, 0f),
        Joint("rear_flipper_Joint", "base_link", "rear_flipper_Link",
            0f, -0.344081f, 0.085966f, -0.888988f, 0f, 3.141593f, JointType.REVOLUTE, 1f, 0f, 0f),
    )

    // ---- Renderable links (colors taken from the URDF <material>) -----------
    // Drawn in this order; base_link is heavy (9 MB / 188k tris) — see [INCLUDE_BASE].
    val LINKS: List<LinkVisual> = listOf(
        LinkVisual("base_link", "base_link.STL", 0.502f, 0.502f, 0.502f, heavy = true),
        LinkVisual("turret_Link", "turret_Link.STL", 1f, 0.749f, 1f),
        LinkVisual("shoulder_Link", "shoulder_Link.STL", 0.502f, 0.502f, 1f),
        LinkVisual("elbow_Link", "elbow_Link.STL", 0.502f, 1f, 1f),
        LinkVisual("telescope_Link", "telescope_Link.STL", 0.502f, 1f, 0.502f),
        LinkVisual("wrist_pan_Link", "wrist_pan_Link.STL", 1f, 1f, 0.502f),
        LinkVisual("wrist_roll_Link", "wrist_roll_Link.STL", 0.984f, 0.733f, 0.514f),
        LinkVisual("gripper_Link", "gripper_Link.STL", 0.792f, 0.820f, 0.929f),
        LinkVisual("front_flipper_Link", "front_flipper_Link.STL", 0.792f, 0.820f, 0.933f),
        LinkVisual("rear_flipper_Link", "rear_flipper_Link.STL", 1f, 1f, 1f),
    )

    /** Set false to drop the heavy base_link (and keep the focus on the arm). */
    const val INCLUDE_BASE = true

    /** Telescope sign convention. Flip to -1f if extension renders the wrong way. */
    const val TELESCOPE_SIGN = 1f

    /**
     * Compute a world-space model matrix for every link, given current joint
     * values (radians; meters for telescope). Missing joints default to 0 —
     * [JointState] already holds last-known values across packets, so this only
     * matters before a joint has ever been seen.
     */
    fun worldTransforms(values: Map<String, Float>): HashMap<String, FloatArray> {
        val out = HashMap<String, FloatArray>(LINKS.size * 2)
        out[ROOT] = identity()

        val origin = FloatArray(16)
        val motion = FloatArray(16)
        val local = FloatArray(16)
        for (j in JOINTS) {
            val parentW = out[j.parent] ?: identity()

            // Fixed frame: T(xyz) * R(rpy), built directly into one matrix.
            originMatrix(j, origin)

            // Variable motion from the joint value.
            val v = values[j.name] ?: 0f
            when (j.type) {
                JointType.REVOLUTE, JointType.CONTINUOUS ->
                    axisRotation(v, j.ax, j.ay, j.az, motion)
                JointType.PRISMATIC -> {
                    val d = v * TELESCOPE_SIGN
                    Matrix.setIdentityM(motion, 0)
                    motion[12] = j.ax * d; motion[13] = j.ay * d; motion[14] = j.az * d
                }
                JointType.FIXED -> Matrix.setIdentityM(motion, 0)
            }

            Matrix.multiplyMM(local, 0, origin, 0, motion, 0)       // origin * motion
            val world = FloatArray(16)
            Matrix.multiplyMM(world, 0, parentW, 0, local, 0)       // parent * local
            out[j.child] = world
        }
        return out
    }

    // ---- matrix helpers (column-major float[16]) ----------------------------

    private fun identity(): FloatArray {
        val m = FloatArray(16); Matrix.setIdentityM(m, 0); return m
    }

    /** Build T(xyz) * R(rpy) where R = Rz(yaw)*Ry(pitch)*Rx(roll). */
    private fun originMatrix(j: Joint, m: FloatArray) {
        val cr = cos(j.roll); val sr = sin(j.roll)
        val cp = cos(j.pitch); val sp = sin(j.pitch)
        val cy = cos(j.yaw); val sy = sin(j.yaw)
        // Row-major 3x3 R, then stored column-major.
        val r00 = cy * cp;            val r01 = cy * sp * sr - sy * cr; val r02 = cy * sp * cr + sy * sr
        val r10 = sy * cp;            val r11 = sy * sp * sr + cy * cr; val r12 = sy * sp * cr - cy * sr
        val r20 = -sp;                val r21 = cp * sr;                val r22 = cp * cr
        m[0] = r00; m[1] = r10; m[2] = r20; m[3] = 0f
        m[4] = r01; m[5] = r11; m[6] = r21; m[7] = 0f
        m[8] = r02; m[9] = r12; m[10] = r22; m[11] = 0f
        m[12] = j.ox; m[13] = j.oy; m[14] = j.oz; m[15] = 1f
    }

    /** Rotation by `angle` rad about (ax,ay,az), via Rodrigues, into column-major m. */
    private fun axisRotation(angle: Float, ax: Float, ay: Float, az: Float, m: FloatArray) {
        var x = ax; var y = ay; var z = az
        val len = sqrt(x * x + y * y + z * z)
        if (len > 1e-9f) { x /= len; y /= len; z /= len }
        val c = cos(angle); val s = sin(angle); val t = 1f - c
        val r00 = t * x * x + c;     val r01 = t * x * y - s * z; val r02 = t * x * z + s * y
        val r10 = t * x * y + s * z; val r11 = t * y * y + c;     val r12 = t * y * z - s * x
        val r20 = t * x * z - s * y; val r21 = t * y * z + s * x; val r22 = t * z * z + c
        m[0] = r00; m[1] = r10; m[2] = r20; m[3] = 0f
        m[4] = r01; m[5] = r11; m[6] = r21; m[7] = 0f
        m[8] = r02; m[9] = r12; m[10] = r22; m[11] = 0f
        m[12] = 0f; m[13] = 0f; m[14] = 0f; m[15] = 1f
    }

    private fun cos(v: Float) = kotlin.math.cos(v.toDouble()).toFloat()
    private fun sin(v: Float) = kotlin.math.sin(v.toDouble()).toFloat()
}
