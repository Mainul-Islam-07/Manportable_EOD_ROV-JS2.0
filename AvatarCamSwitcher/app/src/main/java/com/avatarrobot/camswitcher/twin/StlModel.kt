package com.avatarrobot.camswitcher.twin

import android.content.res.AssetManager
import java.io.BufferedInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.sqrt

/**
 * A parsed binary STL mesh, ready to upload to a GL vertex buffer.
 *
 * Layout in [interleaved] is 6 floats per vertex: position (x,y,z) then
 * normal (nx,ny,nz). The face normal stored in the STL is reused for all
 * three vertices; if it is zero/degenerate we derive one from the triangle.
 * Coordinates are in METERS (SolidWorks export) in each link's own frame.
 */
class StlModel private constructor(
    val interleaved: FloatBuffer,
    val vertexCount: Int,
) {
    companion object {
        private const val FLOATS_PER_VERTEX = 6
        private const val HEADER_BYTES = 80
        private const val TRI_BYTES = 50            // 12 normal + 36 verts + 2 attr

        /** Parse a binary STL from `assets/<path>`. */
        fun fromAsset(assets: AssetManager, path: String): StlModel {
            // Read the whole asset; meshes are stored uncompressed (noCompress).
            val bytes = BufferedInputStream(assets.open(path)).use { it.readBytes() }
            val bb = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)

            bb.position(HEADER_BYTES)
            val triCount = bb.int                  // unsigned in spec; counts here fit in int

            // Sanity check against the file size; bail to an empty mesh if absurd.
            val expected = HEADER_BYTES + 4 + triCount.toLong() * TRI_BYTES
            if (triCount <= 0 || expected > bytes.size.toLong()) {
                return StlModel(
                    ByteBuffer.allocateDirect(0).order(ByteOrder.nativeOrder()).asFloatBuffer(), 0
                )
            }

            val vertexCount = triCount * 3
            val fb = ByteBuffer
                .allocateDirect(vertexCount * FLOATS_PER_VERTEX * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()

            repeat(triCount) {
                var nx = bb.float; var ny = bb.float; var nz = bb.float
                val ax = bb.float; val ay = bb.float; val az = bb.float
                val bx = bb.float; val by = bb.float; val bz = bb.float
                val cx = bb.float; val cy = bb.float; val cz = bb.float
                bb.short                            // attribute byte count (ignored)

                if (nx == 0f && ny == 0f && nz == 0f) {
                    // Derive normal from the winding: (B-A) x (C-A).
                    val ux = bx - ax; val uy = by - ay; val uz = bz - az
                    val vx = cx - ax; val vy = cy - ay; val vz = cz - az
                    nx = uy * vz - uz * vy
                    ny = uz * vx - ux * vz
                    nz = ux * vy - uy * vx
                    val len = sqrt(nx * nx + ny * ny + nz * nz)
                    if (len > 1e-12f) { nx /= len; ny /= len; nz /= len }
                }

                fb.put(ax); fb.put(ay); fb.put(az); fb.put(nx); fb.put(ny); fb.put(nz)
                fb.put(bx); fb.put(by); fb.put(bz); fb.put(nx); fb.put(ny); fb.put(nz)
                fb.put(cx); fb.put(cy); fb.put(cz); fb.put(nx); fb.put(ny); fb.put(nz)
            }
            fb.position(0)
            return StlModel(fb, vertexCount)
        }
    }
}
