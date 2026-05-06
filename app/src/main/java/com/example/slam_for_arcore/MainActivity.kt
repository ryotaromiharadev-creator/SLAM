package com.example.slam_for_arcore

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.widget.Button
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.updatePadding
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.TrackingState

class MainActivity : AppCompatActivity() {

    private lateinit var glSurfaceView: GLSurfaceView
    private lateinit var renderer: SlamRenderer
    private lateinit var tvTrackingState: TextView
    private lateinit var tvPointCount: TextView
    private lateinit var tvSensitivityValue: TextView

    private var session: Session? = null
    private var installRequested = false

    companion object {
        private const val CAMERA_PERMISSION_REQUEST = 0

        // SeekBar の progress → (samplingStep, keyframeInterval, label文字列リソース) 対応
        private val SENSITIVITY_LEVELS = arrayOf(
            Triple(8, 8, R.string.sensitivity_low),
            Triple(4, 5, R.string.sensitivity_mid),
            Triple(2, 3, R.string.sensitivity_high),
            Triple(1, 2, R.string.sensitivity_ultra)
        )
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        // エッジツーエッジ描画を有効にしてナビゲーションバー/ステータスバーの
        // 高さを ViewCompat のインセット API で取得できるようにする
        WindowCompat.setDecorFitsSystemWindows(window, false)
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        tvTrackingState = findViewById(R.id.tv_tracking_state)
        tvPointCount = findViewById(R.id.tv_point_count)
        tvSensitivityValue = findViewById(R.id.tv_sensitivity_value)
        glSurfaceView = findViewById(R.id.gl_surface_view)

        renderer = SlamRenderer()
        renderer.onTrackingState = { state, count ->
            runOnUiThread {
                tvTrackingState.text = when (state) {
                    TrackingState.TRACKING -> getString(R.string.state_tracking)
                    TrackingState.PAUSED -> "トラッキング一時停止"
                    TrackingState.STOPPED -> getString(R.string.state_lost)
                }
                tvPointCount.text = "Points: $count"
            }
        }

        glSurfaceView.setEGLContextClientVersion(2)
        glSurfaceView.setRenderer(renderer)
        glSurfaceView.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY

        // ─── 感度 SeekBar ────────────────────────────────────────────────
        val seekSensitivity = findViewById<SeekBar>(R.id.seek_sensitivity)
        seekSensitivity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar, progress: Int, fromUser: Boolean) {
                val (step, interval, labelRes) = SENSITIVITY_LEVELS[progress]
                renderer.samplingStep = step
                renderer.keyframeInterval = interval
                tvSensitivityValue.text = getString(labelRes)
            }
            override fun onStartTrackingTouch(bar: SeekBar) {}
            override fun onStopTrackingTouch(bar: SeekBar) {}
        })

        // ─── ボタン ──────────────────────────────────────────────────────
        findViewById<Button>(R.id.btn_view_map).setOnClickListener {
            startActivity(Intent(this, PointCloudViewerActivity::class.java))
        }
        findViewById<Button>(R.id.btn_clear).setOnClickListener {
            VisualMapManager.clear()
            tvPointCount.text = "Points: 0"
        }

        // ─── ウィンドウインセット（ナビゲーションバー・ステータスバー対応） ─────
        val dp8 = (8 * resources.displayMetrics.density).toInt()
        val dp6 = (6 * resources.displayMetrics.density).toInt()

        // 下部バー: ナビゲーションバーの高さ分だけ padding を追加
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.bottom_bar)) { view, insets ->
            val navBottom = insets.getInsets(WindowInsetsCompat.Type.navigationBars()).bottom
            view.updatePadding(bottom = navBottom + dp8)
            insets
        }

        // 上部バー: ステータスバーの高さ分だけ padding を追加
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.top_bar)) { view, insets ->
            val statusTop = insets.getInsets(WindowInsetsCompat.Type.statusBars()).top
            view.updatePadding(top = statusTop + dp6)
            insets
        }
    }

    override fun onResume() {
        super.onResume()

        if (!hasCameraPermission()) {
            requestCameraPermission()
            return
        }

        try {
            when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                    installRequested = true
                    return
                }
                ArCoreApk.InstallStatus.INSTALLED -> Unit
            }

            if (session == null) {
                session = Session(this).also { sess ->
                    val config = Config(sess).apply {
                        depthMode = Config.DepthMode.AUTOMATIC
                        updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                        focusMode = Config.FocusMode.AUTO
                    }
                    sess.configure(config)
                }
            }
        } catch (e: Exception) {
            Toast.makeText(this, "ARCore: ${e.message}", Toast.LENGTH_LONG).show()
            return
        }

        session?.let {
            renderer.setSession(it)
            it.resume()
        }
        glSurfaceView.onResume()
    }

    override fun onPause() {
        super.onPause()
        glSurfaceView.onPause()
        session?.pause()
    }

    override fun onDestroy() {
        super.onDestroy()
        session?.close()
        session = null
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == CAMERA_PERMISSION_REQUEST &&
            grantResults.firstOrNull() != PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(this, "カメラ権限が必要です", Toast.LENGTH_LONG).show()
            finish()
        }
    }

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    private fun requestCameraPermission() =
        ActivityCompat.requestPermissions(
            this, arrayOf(Manifest.permission.CAMERA), CAMERA_PERMISSION_REQUEST
        )
}
