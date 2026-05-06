package com.example.slam_for_arcore

import android.opengl.GLSurfaceView
import android.os.Bundle
import android.view.MotionEvent
import android.widget.Button
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import kotlin.math.hypot

class PointCloudViewerActivity : AppCompatActivity() {

    private lateinit var glSurfaceView: GLSurfaceView
    private lateinit var renderer: PointCloudViewerRenderer
    private lateinit var tvPointCount: TextView

    // タッチ追跡
    private var lastX = 0f
    private var lastY = 0f
    private var pinchStartDistance = 0f
    private var pinchStartCamDist = 0f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_point_cloud_viewer)

        tvPointCount = findViewById(R.id.tv_point_count)
        glSurfaceView = findViewById(R.id.viewer_gl_surface_view)

        renderer = PointCloudViewerRenderer()
        glSurfaceView.setEGLContextClientVersion(2)
        glSurfaceView.setRenderer(renderer)
        glSurfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        updateCountDisplay()

        glSurfaceView.setOnTouchListener { _, event ->
            handleTouch(event)
            true
        }

        findViewById<Button>(R.id.btn_refresh).setOnClickListener {
            glSurfaceView.queueEvent { renderer.refreshPoints() }
            updateCountDisplay()
        }

        findViewById<Button>(R.id.btn_clear_map).setOnClickListener {
            VisualMapManager.clear()
            glSurfaceView.queueEvent { renderer.refreshPoints() }
            updateCountDisplay()
        }
    }

    override fun onResume() {
        super.onResume()
        glSurfaceView.onResume()
        // 最新データを反映
        glSurfaceView.queueEvent { renderer.refreshPoints() }
        updateCountDisplay()
    }

    override fun onPause() {
        super.onPause()
        glSurfaceView.onPause()
    }

    private fun updateCountDisplay() {
        tvPointCount.text = "Points: ${VisualMapManager.getPointCount()}"
    }

    // ─── タッチ処理 ───────────────────────────────────────────────────────

    private fun handleTouch(event: MotionEvent) {
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                lastX = event.x
                lastY = event.y
            }

            MotionEvent.ACTION_MOVE -> {
                when (event.pointerCount) {
                    1 -> {
                        // 1本指ドラッグ → オービット回転
                        val dx = event.x - lastX
                        val dy = event.y - lastY
                        renderer.azimuth = (renderer.azimuth - dx * 0.4f) % 360f
                        renderer.elevation = (renderer.elevation + dy * 0.4f).coerceIn(-89f, 89f)
                        lastX = event.x
                        lastY = event.y
                    }
                    2 -> {
                        // 2本指ピンチ → ズーム
                        val curDist = fingerDistance(event)
                        if (pinchStartDistance > 0f) {
                            renderer.distance = (pinchStartCamDist * pinchStartDistance / curDist)
                                .coerceIn(0.3f, 100f)
                        }
                    }
                }
            }

            MotionEvent.ACTION_POINTER_DOWN -> {
                if (event.pointerCount == 2) {
                    pinchStartDistance = fingerDistance(event)
                    pinchStartCamDist = renderer.distance
                }
            }

            MotionEvent.ACTION_POINTER_UP -> {
                if (event.pointerCount == 2) {
                    // ピンチ終了後に1本指の基準座標をリセット
                    val idx = if (event.actionIndex == 0) 1 else 0
                    lastX = event.getX(idx)
                    lastY = event.getY(idx)
                }
                pinchStartDistance = 0f
            }
        }
    }

    private fun fingerDistance(event: MotionEvent): Float =
        hypot(
            (event.getX(0) - event.getX(1)).toDouble(),
            (event.getY(0) - event.getY(1)).toDouble()
        ).toFloat()
}
