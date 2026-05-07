package com.example.slam_for_arcore

import com.google.ar.core.Pose
import kotlin.math.*

/**
 * スキャン済み領域を追跡し、未探索フロンティアへの移動方向を計算する。
 *
 * アルゴリズム:
 *   - 水平面を 40cm グリッドで管理
 *   - 点群が MIN_POINTS_PER_CELL 以上あるセルを「スキャン済み」とみなす
 *   - スキャン済みセルに隣接する未スキャンセルを「フロンティア」とする
 *   - フロンティアを 8 方位セクターに集計し、最多方向を提示する
 *   - 方向はカメラ前方を基準とした相対角度（時計回り）で返す
 *
 * パスファインディング用の占有グリッドも提供する:
 *   - 床面レベル (floorY ± FLOOR_TOLERANCE) のセル → 走行可能
 *   - それ以外のセル → 障害物
 */
object CoverageAnalyzer {

    // ── データクラス ─────────────────────────────────────────────────────

    data class NavHint(
        val message: String,
        /** カメラ前方を 0° として時計回りの相対角度。null = 移動不要（完了） */
        val arrowAngleDeg: Float?,
        val coveragePercent: Int
    ) {
        val arrowChar: String
            get() = when {
                arrowAngleDeg == null -> "✓"
                arrowAngleDeg < 22.5f || arrowAngleDeg >= 337.5f -> "↑"
                arrowAngleDeg < 67.5f  -> "↗"
                arrowAngleDeg < 112.5f -> "→"
                arrowAngleDeg < 157.5f -> "↘"
                arrowAngleDeg < 202.5f -> "↓"
                arrowAngleDeg < 247.5f -> "↙"
                arrowAngleDeg < 292.5f -> "←"
                else                   -> "↖"
            }
    }

    // ── 設定 ─────────────────────────────────────────────────────────────

    private const val GRID_RES = 0.4f          // セル幅 40cm
    private const val SCAN_RADIUS = 6f         // ナビ判定半径 6m
    private const val MIN_POINTS_PER_CELL = 4  // スキャン済みとみなす最低点数
    private const val FLOOR_TOLERANCE = 0.15f  // 床面判定の垂直許容幅

    // ── 状態 ─────────────────────────────────────────────────────────────

    // key = packGrid(gx, gz), value = 点数
    private val gridCounts = HashMap<Long, Int>(2048)
    private val lock = Any()

    // 床面Y（世界座標）を推定して保持（pathfinding 用）
    @Volatile var estimatedFloorY: Float? = null
        private set

    @Volatile private var lastHint = NavHint("カメラを周囲に向けてスキャンしてください", null, 0)

    // ── 外部 API ─────────────────────────────────────────────────────────

    /** キーフレームごとに点群を登録する（GLスレッドから呼ぶ） */
    fun recordKeyframe(pts: List<FloatArray>) {
        if (pts.isEmpty()) return
        synchronized(lock) {
            for (pt in pts) {
                val key = packGrid(
                    floor(pt[0] / GRID_RES).toInt(),
                    floor(pt[2] / GRID_RES).toInt()
                )
                gridCounts[key] = (gridCounts[key] ?: 0) + 1
            }
            updateFloorEstimate(pts)
        }
    }

    /** ナビゲーションヒントを再計算する（10フレームに1回程度呼ぶ） */
    fun updateHint(pose: Pose) {
        val camPos = pose.translation   // float[3]

        // カメラ前方ベクトルを world 空間で取得
        // ARCore depth 空間では +Z が前方なので (0,0,1) を変換
        val aheadWorld = pose.transformPoint(floatArrayOf(0f, 0f, 1f))
        val fwdX = aheadWorld[0] - camPos[0]
        val fwdZ = aheadWorld[2] - camPos[2]
        // world 空間での前方角 (atan2 で X, Z 平面)
        val forwardAngleRad = atan2(fwdX.toDouble(), fwdZ.toDouble()).toFloat()

        val cx = floor(camPos[0] / GRID_RES).toInt()
        val cz = floor(camPos[2] / GRID_RES).toInt()
        val iRadius = ceil(SCAN_RADIUS / GRID_RES).toInt()

        var covered = 0
        var total = 0
        val sectorWeight = FloatArray(8)

        for (dz in -iRadius..iRadius) {
            for (dx in -iRadius..iRadius) {
                val dist = sqrt((dx * dx + dz * dz).toFloat()) * GRID_RES
                if (dist > SCAN_RADIUS) continue
                total++

                val gx = cx + dx; val gz = cz + dz
                val key = packGrid(gx, gz)
                val cnt = synchronized(lock) { gridCounts[key] ?: 0 }

                if (cnt >= MIN_POINTS_PER_CELL) {
                    covered++
                } else if (synchronized(lock) { hasAdjacentCovered(gx, gz) }) {
                    // フロンティア: 近いほど重みを大きく
                    val weight = 1f / max(dist, GRID_RES)
                    // world 角度 → カメラ前方からの相対角 → 8セクター
                    val cellAngleRad = atan2(dx.toDouble(), dz.toDouble()).toFloat()
                    val relAngleDeg = ((Math.toDegrees(
                        (cellAngleRad - forwardAngleRad).toDouble()
                    ) + 360.0) % 360.0).toFloat()
                    val sector = (relAngleDeg / 45f).toInt().coerceIn(0, 7)
                    sectorWeight[sector] += weight
                }
            }
        }

        val percent = if (total > 0) (covered * 100 / total).coerceIn(0, 100) else 0

        if (sectorWeight.sum() < 0.01f) {
            lastHint = NavHint("このエリアのスキャン完了 ($percent%)", null, percent)
            return
        }

        val best = sectorWeight.indices.maxByOrNull { sectorWeight[it] } ?: 0
        val angleDeg = best * 45f
        lastHint = NavHint(
            message = "${SECTOR_LABELS[best]}を探索してください ($percent%)",
            arrowAngleDeg = angleDeg,
            coveragePercent = percent
        )
    }

    fun getHint(): NavHint = lastHint

    fun clear() {
        synchronized(lock) { gridCounts.clear() }
        estimatedFloorY = null
        lastHint = NavHint("カメラを周囲に向けてスキャンしてください", null, 0)
    }

    // ── パスファインディング補助 ──────────────────────────────────────────

    /**
     * 指定 (worldX, worldZ) のセルが走行可能かを返す（床面検出済みの場合）。
     * true = 走行可能（障害物なし）, false = 障害物あり or 未スキャン
     */
    fun isNavigable(worldX: Float, worldZ: Float): Boolean {
        val floorY = estimatedFloorY ?: return false
        val pts = VisualMapManager.getPointBuffer()
        val count = VisualMapManager.getPointCount()
        val gx = floor(worldX / GRID_RES).toInt()
        val gz = floor(worldZ / GRID_RES).toInt()
        // 同セルに床面より十分上の点があれば障害物
        // 簡易実装: VisualMapManager の点群を走査して判定
        pts.rewind()
        for (i in 0 until count) {
            val px = pts.get(); val py = pts.get(); val pz = pts.get()
            pts.get(); pts.get(); pts.get()  // skip rgb
            if (floor(px / GRID_RES).toInt() == gx &&
                floor(pz / GRID_RES).toInt() == gz &&
                py > floorY + FLOOR_TOLERANCE + 0.3f) {
                return false
            }
        }
        return synchronized(lock) { (gridCounts[packGrid(gx, gz)] ?: 0) >= MIN_POINTS_PER_CELL }
    }

    // ── 内部ユーティリティ ────────────────────────────────────────────────

    // 呼び出し元で lock 済みであること
    private fun hasAdjacentCovered(gx: Int, gz: Int): Boolean {
        for (dz in -1..1) for (dx in -1..1) {
            if (dx == 0 && dz == 0) continue
            if ((gridCounts[packGrid(gx + dx, gz + dz)] ?: 0) >= MIN_POINTS_PER_CELL) return true
        }
        return false
    }

    /** Y ヒストグラムで床面を推定する（10cm 解像度）*/
    private fun updateFloorEstimate(pts: List<FloatArray>) {
        val hist = HashMap<Int, Int>(64)
        for (pt in pts) {
            val bin = floor(pt[1] / 0.1f).toInt()
            hist[bin] = (hist[bin] ?: 0) + 1
        }
        // 最頻 Y ビンを床面とする（最も低い密集層）
        val minBin = hist.entries
            .filter { it.value >= 5 }
            .minByOrNull { it.key }?.key ?: return
        estimatedFloorY = minBin * 0.1f
    }

    private fun packGrid(gx: Int, gz: Int): Long {
        val xi = (gx + 32768L) and 0xFFFF
        val zi = (gz + 32768L) and 0xFFFF
        return xi or (zi shl 16)
    }

    private val SECTOR_LABELS = arrayOf(
        "正面", "右前方", "右側", "右後方",
        "後方", "左後方", "左側", "左前方"
    )
}
