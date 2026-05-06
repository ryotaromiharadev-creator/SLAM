package com.example.slam_for_arcore

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

class PointCloudViewerRenderer : GLSurfaceView.Renderer {

    // タッチ操作で更新されるカメラパラメータ
    @Volatile var azimuth = 0f
    @Volatile var elevation = 25f
    @Volatile var distance = 4f

    private var program = -1
    private var mvpHandle = -1
    private var posHandle = -1
    private var colorHandle = -1

    private var viewportAspect = 1f

    // 点群バッファ（x,y,z,r,g,b 形式、6 floats/点）
    @Volatile private var pointBuffer: FloatBuffer? = null
    @Volatile private var pointCount = 0

    private var centerX = 0f
    private var centerY = 0f
    private var centerZ = 0f

    /** VisualMapManagerから最新データを取り込む（GLスレッド上で呼ぶこと）*/
    fun refreshPoints() {
        val count = VisualMapManager.getPointCount()
        if (count == 0) {
            pointBuffer = null
            pointCount = 0
            return
        }

        val buf = VisualMapManager.getPointBuffer()
        pointCount = count

        // 重心計算（大量点でもO(N)、サンプリングで高速化）
        val step = maxOf(1, count / 50_000)
        var sx = 0.0; var sy = 0.0; var sz = 0.0; var n = 0
        buf.rewind()
        var i = 0
        while (i < count && buf.remaining() >= 6) {
            val x = buf.get(); val y = buf.get(); val z = buf.get()
            buf.get(); buf.get(); buf.get()  // r, g, b をスキップ
            if (i % step == 0) { sx += x; sy += y; sz += z; n++ }
            i++
        }
        if (n > 0) {
            centerX = (sx / n).toFloat()
            centerY = (sy / n).toFloat()
            centerZ = (sz / n).toFloat()
        }

        buf.rewind()
        pointBuffer = buf
    }

    // ─── GLSurfaceView.Renderer ────────────────────────────────────────────

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.05f, 0.05f, 0.12f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)

        program = createProgram(VERTEX_SHADER, FRAGMENT_SHADER)
        mvpHandle = GLES20.glGetUniformLocation(program, "u_MVP")
        posHandle = GLES20.glGetAttribLocation(program, "a_Position")
        colorHandle = GLES20.glGetAttribLocation(program, "a_Color")

        refreshPoints()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES20.glViewport(0, 0, width, height)
        viewportAspect = if (height > 0) width.toFloat() / height.toFloat() else 1f
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        val buf = pointBuffer
        if (buf == null || pointCount == 0) return

        val proj = FloatArray(16)
        val view = FloatArray(16)
        val mvp = FloatArray(16)

        Matrix.perspectiveM(proj, 0, 60f, viewportAspect, 0.05f, 500f)

        val azRad = Math.toRadians(azimuth.toDouble()).toFloat()
        val elRad = Math.toRadians(elevation.toDouble()).toFloat()
        val camX = centerX + distance * cos(elRad) * sin(azRad)
        val camY = centerY + distance * sin(elRad)
        val camZ = centerZ + distance * cos(elRad) * cos(azRad)
        Matrix.setLookAtM(view, 0, camX, camY, camZ, centerX, centerY, centerZ, 0f, 1f, 0f)
        Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)

        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(mvpHandle, 1, false, mvp, 0)

        // インターリーブバッファ: [x, y, z, r, g, b] × N
        // stride = 6 floats × 4 bytes = 24 bytes
        val stride = 24

        buf.rewind()
        GLES20.glVertexAttribPointer(posHandle, 3, GLES20.GL_FLOAT, false, stride, buf)
        GLES20.glEnableVertexAttribArray(posHandle)

        // 色属性: 先頭から 3 floats (12 bytes) オフセット
        buf.position(3)
        GLES20.glVertexAttribPointer(colorHandle, 3, GLES20.GL_FLOAT, false, stride, buf)
        GLES20.glEnableVertexAttribArray(colorHandle)

        GLES20.glDrawArrays(GLES20.GL_POINTS, 0, pointCount)

        GLES20.glDisableVertexAttribArray(posHandle)
        GLES20.glDisableVertexAttribArray(colorHandle)
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

    // ─── シェーダー ────────────────────────────────────────────────────────

    companion object {
        // 実際のカメラ画素色をそのまま頂点カラーとして使用
        private const val VERTEX_SHADER = """
            uniform mat4 u_MVP;
            attribute vec3 a_Position;
            attribute vec3 a_Color;
            varying vec3 v_Color;
            void main() {
                gl_Position = u_MVP * vec4(a_Position, 1.0);
                v_Color = a_Color;
                gl_PointSize = 3.0;
            }
        """

        private const val FRAGMENT_SHADER = """
            precision mediump float;
            varying vec3 v_Color;
            void main() {
                gl_FragColor = vec4(v_Color, 1.0);
            }
        """
    }
}
