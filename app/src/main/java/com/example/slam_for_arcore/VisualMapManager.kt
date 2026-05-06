package com.example.slam_for_arcore

import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * カメラ画素を空間にマッピングしたビジュアルマップ。
 * 各点は (x, y, z, r, g, b) の6要素で表現される。
 */
object VisualMapManager {

    private val voxelMap = HashMap<Long, FloatArray>(65536)
    private val lock = Any()

    // 2cm ボクセルグリッド
    private const val VOXEL_SIZE = 0.02f
    private const val MAX_POINTS = 600_000

    @Volatile private var cachedBuffer: FloatBuffer? = null
    @Volatile private var isDirty = false

    /**
     * RGB着色済み点のリストを追加する。
     * @param pts 各要素は FloatArray(x, y, z, r, g, b)、r/g/bは0〜1
     */
    fun addColoredPoints(pts: List<FloatArray>) {
        synchronized(lock) {
            if (voxelMap.size >= MAX_POINTS) return
            for (pt in pts) {
                val key = voxelKey(
                    (pt[0] / VOXEL_SIZE).toInt(),
                    (pt[1] / VOXEL_SIZE).toInt(),
                    (pt[2] / VOXEL_SIZE).toInt()
                )
                if (!voxelMap.containsKey(key)) {
                    voxelMap[key] = pt
                    isDirty = true
                }
            }
        }
    }

    fun getPointCount(): Int = synchronized(lock) { voxelMap.size }

    /**
     * (x,y,z,r,g,b) × N のFloatBufferを返す。
     * 変化がなければキャッシュを再利用する。
     */
    fun getPointBuffer(): FloatBuffer {
        synchronized(lock) {
            if (!isDirty) return cachedBuffer ?: FloatBuffer.wrap(FloatArray(0))
            val arr = FloatArray(voxelMap.size * 6)
            var i = 0
            for ((_, v) in voxelMap) {
                arr[i++] = v[0]; arr[i++] = v[1]; arr[i++] = v[2]
                arr[i++] = v[3]; arr[i++] = v[4]; arr[i++] = v[5]
            }
            val buf = ByteBuffer.allocateDirect(arr.size * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()
                .apply { put(arr); rewind() }
            cachedBuffer = buf
            isDirty = false
            return buf
        }
    }

    fun clear() {
        synchronized(lock) {
            voxelMap.clear()
            cachedBuffer = null
            isDirty = false
        }
    }

    // 3軸 ±81.92m (0.02m単位で ±4096) をカバーするボクセルキー
    private fun voxelKey(x: Int, y: Int, z: Int): Long {
        val xi = (x + 4096).toLong() and 0x1FFF
        val yi = (y + 4096).toLong() and 0x1FFF
        val zi = (z + 4096).toLong() and 0x1FFF
        return xi or (yi shl 13) or (zi shl 26)
    }
}
