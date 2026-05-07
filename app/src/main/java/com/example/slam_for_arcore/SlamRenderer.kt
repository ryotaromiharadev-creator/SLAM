package com.example.slam_for_arcore

import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.opengl.Matrix
import com.google.ar.core.Coordinates2d
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class SlamRenderer : android.opengl.GLSurfaceView.Renderer {

    @Volatile private var pendingSession: Session? = null
    private var session: Session? = null
    private var viewportWidth = 1
    private var viewportHeight = 1

    private var cameraTextureId = -1
    private var bgProgram = -1
    private var pcProgram = -1

    var onTrackingState: ((TrackingState, Int) -> Unit)? = null
    var onNavigationHint: ((CoverageAnalyzer.NavHint) -> Unit)? = null

    /** サンプリングステップ: 1=最密, 8=最粗。UIから変更可能。 */
    @Volatile var samplingStep: Int = 4
    /** キーフレーム間隔: N フレームに1回投影処理を行う。 */
    @Volatile var keyframeInterval: Int = 5

    private val ndcCoords = floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f)
    private val cameraTexCoords = FloatArray(8)
    private lateinit var ndcBuf: FloatBuffer
    private lateinit var texBuf: FloatBuffer

    private var frameCounter = 0

    fun setSession(session: Session) {
        pendingSession = session
    }

    // ─── GLSurfaceView.Renderer ────────────────────────────────────────────

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)

        val texIds = IntArray(1)
        GLES20.glGenTextures(1, texIds, 0)
        cameraTextureId = texIds[0]
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, cameraTextureId)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)

        bgProgram = createProgram(BG_VERT, BG_FRAG)
        pcProgram = createProgram(PC_VERT, PC_FRAG)

        ndcBuf = makeFloatBuffer(ndcCoords)
        texBuf = makeFloatBuffer(cameraTexCoords)
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportWidth = width
        viewportHeight = height
    }

    override fun onDrawFrame(gl: GL10?) {
        pendingSession?.let { sess ->
            sess.setCameraTextureName(cameraTextureId)
            session = sess
            pendingSession = null
        }

        val sess = session ?: run {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
            return
        }

        try {
            sess.setDisplayGeometry(0, viewportWidth, viewportHeight)
            val frame = sess.update()
            val camera = frame.camera

            drawBackground(frame)

            if (camera.trackingState != TrackingState.TRACKING) {
                onTrackingState?.invoke(camera.trackingState, VisualMapManager.getPointCount())
                return
            }

            GLES20.glEnable(GLES20.GL_DEPTH_TEST)
            GLES20.glDepthMask(true)

            val projMatrix = FloatArray(16)
            val viewMatrix = FloatArray(16)
            camera.getProjectionMatrix(projMatrix, 0, 0.1f, 100f)
            camera.getViewMatrix(viewMatrix, 0)

            // リアルタイム確認用：ARCoreフィーチャーポイントオーバーレイ
            frame.acquirePointCloud().use { cloud ->
                if (cloud.ids.limit() > 0) {
                    drawFeaturePoints(cloud.points, cloud.ids.limit(), viewMatrix, projMatrix)
                }
            }

            // キーフレームごとに深度×色で画素を空間マッピング
            frameCounter++
            if (frameCounter % keyframeInterval == 0) {
                val pts = extractColoredPixels(frame, camera)
                if (pts.isNotEmpty()) {
                    VisualMapManager.addColoredPoints(pts)
                    CoverageAnalyzer.recordKeyframe(pts)
                }
            }

            // 10フレームに1回ナビゲーションヒントを更新
            if (frameCounter % 10 == 0) {
                CoverageAnalyzer.updateHint(camera.pose)
                onNavigationHint?.invoke(CoverageAnalyzer.getHint())
            }

            onTrackingState?.invoke(camera.trackingState, VisualMapManager.getPointCount())
        } catch (_: Exception) {
            // フレームスキップ
        }
    }

    // ─── 深度×色の画素逆投影 ─────────────────────────────────────────────

    /**
     * ARCore Depth API から深度画像とカメラ画像を取得し、
     * 各深度画素をカメラ内部パラメータで逆投影してワールド座標のRGB点群を生成する。
     *
     * 座標系変換:
     *   深度 D (m) + 画素 (u, v) → カメラ空間 (X, -Y, -D) → ワールド空間
     *   ※ ARCore カメラ空間は右手系・Y上・前方が -Z (OpenGL準拠)
     *      画像 v (下増加) → カメラ Y (上増加) なので符号反転
     */
    private fun extractColoredPixels(frame: Frame, camera: com.google.ar.core.Camera): List<FloatArray> {
        val depthImage = try {
            frame.acquireDepthImage16Bits()
        } catch (_: Exception) {
            return emptyList()
        }

        val cameraImage = try {
            frame.acquireCameraImage()
        } catch (_: Exception) {
            depthImage.close()
            return emptyList()
        }

        val result = ArrayList<FloatArray>(1024)

        try {
            val pose = camera.pose
            val intr = camera.imageIntrinsics
            val fx = intr.focalLength[0]
            val fy = intr.focalLength[1]
            val cx = intr.principalPoint[0]
            val cy = intr.principalPoint[1]

            val depW = depthImage.width
            val depH = depthImage.height
            val imgW = cameraImage.width
            val imgH = cameraImage.height

            // 深度画像解像度にスケールした焦点距離・主点
            val sx = depW.toFloat() / imgW
            val sy = depH.toFloat() / imgH
            val dfx = fx * sx
            val dfy = fy * sy
            val dcx = cx * sx
            val dcy = cy * sy

            val depPlane = depthImage.planes[0]
            val depRowStride = depPlane.rowStride / 2  // bytes → shorts
            val depBuf = depPlane.buffer.asShortBuffer()

            val yBuf = cameraImage.planes[0].buffer
            val uBuf = cameraImage.planes[1].buffer
            val vBuf = cameraImage.planes[2].buffer
            val yRowStride = cameraImage.planes[0].rowStride
            val uvPixelStride = cameraImage.planes[1].pixelStride
            val uvRowStride = cameraImage.planes[1].rowStride

            val step = samplingStep
            for (dv in 0 until depH step step) {
                for (du in 0 until depW step step) {
                    val depMm = depBuf[dv * depRowStride + du].toInt() and 0xFFFF
                    // 有効深度範囲: 0.15m〜5m
                    if (depMm < 150 || depMm > 5000) continue

                    val depM = depMm / 1000f

                    // 逆投影: 深度画素 → ARCoreカメラ空間 (CV慣習: X右, Y下増加, Z前方正)
                    // camera.pose.transformPoint() がこのCV空間からworld空間へ変換する
                    val Xc = (du - dcx) * depM / dfx
                    val Yc = (dv - dcy) * depM / dfy   // 符号反転なし: CV慣習でY下向き正
                    val Zc = depM                        // 符号反転なし: CV慣習でZ前方正

                    // カメラ空間 (CV) → ワールド空間
                    val w = pose.transformPoint(floatArrayOf(Xc, Yc, Zc))

                    // カメラ画像から画素色を取得 (YUV → RGB)
                    val iu = (du / sx).toInt().coerceIn(0, imgW - 1)
                    val iv = (dv / sy).toInt().coerceIn(0, imgH - 1)

                    val yVal = yBuf[iv * yRowStride + iu].toInt() and 0xFF
                    val uIdx = (iv / 2) * uvRowStride + (iu / 2) * uvPixelStride
                    val vIdx = uIdx  // U/V バッファは別プレーン
                    val uVal = (uBuf[uIdx].toInt() and 0xFF) - 128
                    val vVal = (vBuf[vIdx].toInt() and 0xFF) - 128

                    val r = (yVal + 1.402f * vVal).toInt().coerceIn(0, 255) / 255f
                    val g = (yVal - 0.344f * uVal - 0.714f * vVal).toInt().coerceIn(0, 255) / 255f
                    val b = (yVal + 1.772f * uVal).toInt().coerceIn(0, 255) / 255f

                    result.add(floatArrayOf(w[0], w[1], w[2], r, g, b))
                }
            }
        } finally {
            depthImage.close()
            cameraImage.close()
        }

        return result
    }

    // ─── 描画ヘルパー ─────────────────────────────────────────────────────

    private fun drawBackground(frame: Frame) {
        frame.transformCoordinates2d(
            Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES, ndcCoords,
            Coordinates2d.TEXTURE_NORMALIZED, cameraTexCoords
        )
        texBuf.put(cameraTexCoords).rewind()

        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glDepthMask(false)
        GLES20.glUseProgram(bgProgram)

        val posAttr = GLES20.glGetAttribLocation(bgProgram, "a_Position")
        val texAttr = GLES20.glGetAttribLocation(bgProgram, "a_TexCoord")
        val texUnif = GLES20.glGetUniformLocation(bgProgram, "u_Texture")

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, cameraTextureId)
        GLES20.glUniform1i(texUnif, 0)

        ndcBuf.rewind(); texBuf.rewind()
        GLES20.glVertexAttribPointer(posAttr, 2, GLES20.GL_FLOAT, false, 0, ndcBuf)
        GLES20.glEnableVertexAttribArray(posAttr)
        GLES20.glVertexAttribPointer(texAttr, 2, GLES20.GL_FLOAT, false, 0, texBuf)
        GLES20.glEnableVertexAttribArray(texAttr)

        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
        GLES20.glDisableVertexAttribArray(posAttr)
        GLES20.glDisableVertexAttribArray(texAttr)
    }

    private fun drawFeaturePoints(points: FloatBuffer, count: Int, viewMatrix: FloatArray, projMatrix: FloatArray) {
        val mvp = FloatArray(16)
        Matrix.multiplyMM(mvp, 0, projMatrix, 0, viewMatrix, 0)

        GLES20.glUseProgram(pcProgram)
        val mvpUnif = GLES20.glGetUniformLocation(pcProgram, "u_MVP")
        val posAttr = GLES20.glGetAttribLocation(pcProgram, "a_Position")

        GLES20.glUniformMatrix4fv(mvpUnif, 1, false, mvp, 0)
        points.rewind()
        GLES20.glVertexAttribPointer(posAttr, 4, GLES20.GL_FLOAT, false, 0, points)
        GLES20.glEnableVertexAttribArray(posAttr)
        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count)
        GLES20.glDisableVertexAttribArray(posAttr)
    }

    // ─── OpenGL ユーティリティ ────────────────────────────────────────────

    private fun createProgram(vertSrc: String, fragSrc: String): Int {
        fun shader(type: Int, src: String) = GLES20.glCreateShader(type).also {
            GLES20.glShaderSource(it, src); GLES20.glCompileShader(it)
        }
        val vert = shader(GLES20.GL_VERTEX_SHADER, vertSrc)
        val frag = shader(GLES20.GL_FRAGMENT_SHADER, fragSrc)
        return GLES20.glCreateProgram().also {
            GLES20.glAttachShader(it, vert); GLES20.glAttachShader(it, frag)
            GLES20.glLinkProgram(it)
        }
    }

    private fun makeFloatBuffer(data: FloatArray): FloatBuffer =
        ByteBuffer.allocateDirect(data.size * 4).order(ByteOrder.nativeOrder())
            .asFloatBuffer().apply { put(data); rewind() }

    // ─── シェーダー ────────────────────────────────────────────────────────

    companion object {
        private const val BG_VERT = """
            attribute vec4 a_Position;
            attribute vec2 a_TexCoord;
            varying vec2 v_TexCoord;
            void main() { gl_Position = a_Position; v_TexCoord = a_TexCoord; }
        """
        private const val BG_FRAG = """
            #extension GL_OES_EGL_image_external : require
            precision mediump float;
            uniform samplerExternalOES u_Texture;
            varying vec2 v_TexCoord;
            void main() { gl_FragColor = texture2D(u_Texture, v_TexCoord); }
        """
        // フィーチャーポイントオーバーレイ（黄緑の点）
        private const val PC_VERT = """
            uniform mat4 u_MVP;
            attribute vec4 a_Position;
            void main() {
                gl_Position = u_MVP * vec4(a_Position.xyz, 1.0);
                gl_PointSize = 8.0;
            }
        """
        private const val PC_FRAG = """
            precision mediump float;
            void main() { gl_FragColor = vec4(0.4, 1.0, 0.3, 1.0); }
        """
    }
}
