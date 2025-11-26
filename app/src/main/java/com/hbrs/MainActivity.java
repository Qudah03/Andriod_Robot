package com.hbrs;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Build;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.core.Preview;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.cardview.widget.CardView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.slider.Slider;
import com.google.common.util.concurrent.ListenableFuture;
import com.hbrs.Bluetooth.BT_DeviceListActivity;
import com.hbrs.ORB.ORB;

import java.nio.ByteBuffer;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends AppCompatActivity {

    // Request codes
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_BT_PERMISSIONS = 2;
    private static final int REQUEST_CAMERA_PERMISSION = 3;

    // UI Elements
    private Button connectButton, batteryButton, followLineButton, followBallButton;
    private TextView statusText;
    private View joystickBase, joystickHandle;
    private Slider speedSlider;
    private CardView cameraCard;
    private PreviewView previewView;
    private ImageView imageView;

    // ORB object
    private ORB orb;

    // State tracking
    private boolean isConnected = false;
    private boolean isConnecting = false;
    private float joystickBaseCenterX, joystickBaseCenterY, joystickBaseRadius;

    // CameraX variables
    private ProcessCameraProvider cameraProvider;
    private boolean isCameraPreviewOn = false;
    private ImageAnalysis imageAnalysis;
    private ExecutorService cameraExecutor;

    // Logic Modes
    private boolean isLineMode = false;
    private boolean isBallMode = false;

    // Configuration
    private final int AUTO_SPEED = 600;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        cameraExecutor = Executors.newSingleThreadExecutor();

        // Initialize all UI elements
        connectButton = findViewById(R.id.btn_connect);
        batteryButton = findViewById(R.id.btn_get_battery);
        followLineButton = findViewById(R.id.btn_follow_line);
        followBallButton = findViewById(R.id.btn_follow_ball);
        statusText = findViewById(R.id.status_text);
        joystickBase = findViewById(R.id.joystick_base);
        joystickHandle = findViewById(R.id.joystick_handle);
        speedSlider = findViewById(R.id.speed_slider);
        cameraCard = findViewById(R.id.camera_card);
        previewView = findViewById(R.id.previewView);
        imageView = findViewById(R.id.imageView);

        orb = new ORB(this);
        setupJoystickListener();
        updateUI();
    }

    // --- BUTTON CLICKS ---

    public void onClickToggleCamera(View view) {
        if (isCameraPreviewOn) {
            // Turn Camera OFF
            if (cameraProvider != null) {
                cameraProvider.unbindAll();
            }
            isCameraPreviewOn = false;

            // Also stop any active vision modes when turning off camera
            stopVision();
        } else {
            // Turn Camera ON
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                startCamera();
                // We don't start a specific mode yet; we just show the preview
                // The user will press "Line" or "Ball" next.
            } else {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            }
        }

        updateUI();
    }

    public void onClickToggleLineFollowing(View v) {
        if (isLineMode) {
            stopVision();
        } else {
            startLineFollowing();
        }
        updateUI();
    }

    public void onClickToggleBallFollowing(View v) {
        if (isBallMode) {
            stopVision();
        } else {
            startBallFollowing();
        }
        updateUI();
    }

    private void startLineFollowing() {
        stopVision(); // reset others
        isLineMode = true;
        if (!isCameraPreviewOn) startCamera();

        // Update Analyzer
        if (imageAnalysis != null) {
            imageAnalysis.setAnalyzer(cameraExecutor,
                    new LineFollowerAnalyzer(orb, imageView, isConnected, this::stopRobot));
        }
    }

    private void startBallFollowing() {
        stopVision(); // reset others
        isBallMode = true;
        if (!isCameraPreviewOn) startCamera();

        // Update Analyzer
        if (imageAnalysis != null) {

            // We pass 'orb', 'imageView', the current 'isConnected' state, and the 'stopRobot' function.
            imageAnalysis.setAnalyzer(cameraExecutor,
                    new BallFollowerAnalyzer(orb, imageView, isConnected, this::stopRobot));
        }
    }

    private void stopVision() {
        isLineMode = false;
        isBallMode = false;
        if (imageAnalysis != null) {
            imageAnalysis.clearAnalyzer();
        }
        imageView.setImageBitmap(null); // Clear overlay
        stopRobot();
    }

    // --- CAMERA SETUP ---

    private void startCamera() {
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                cameraProvider = cameraProviderFuture.get();
                bindCameraUseCases();
                isCameraPreviewOn = true;

                // Restore analyzer if mode is active
                if (isLineMode) startLineFollowing();
                if (isBallMode) startBallFollowing();

                updateUI();
            } catch (ExecutionException | InterruptedException e) {
                Toast.makeText(this, "Error starting camera: " + e.getMessage(), Toast.LENGTH_SHORT).show();
            }
        }, ContextCompat.getMainExecutor(this));
    }

    private void bindCameraUseCases() {
        if (cameraProvider == null) return;
        cameraProvider.unbindAll();

        Preview preview = new Preview.Builder().build();
        preview.setSurfaceProvider(previewView.getSurfaceProvider());

        imageAnalysis = new ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build();

        CameraSelector cameraSelector = new CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .build();

        cameraProvider.bindToLifecycle(this, cameraSelector, imageAnalysis, preview);
    }

    // --- UI & BLUETOOTH ---

    @SuppressLint("SetTextI18n")
    private void updateUI() {
        if (isConnected) {
            statusText.setText("Status: Connected");
            connectButton.setText("Disconnect");
            batteryButton.setEnabled(true);
        } else {
            statusText.setText(isConnecting ? "Status: Connecting..." : "Status: Not Connected");
            connectButton.setText("Connect");
            batteryButton.setEnabled(false);
        }

        cameraCard.setVisibility(isCameraPreviewOn ? View.VISIBLE : View.GONE);

        // Mode Buttons
        int modeButtonVisibility = isCameraPreviewOn ? View.VISIBLE : View.GONE;
        followLineButton.setVisibility(modeButtonVisibility);
        followBallButton.setVisibility(modeButtonVisibility);

        followLineButton.setText(isLineMode ? "Stop Line" : "Start Line");
        followBallButton.setText(isBallMode ? "Stop Ball" : "Start Ball");

        followLineButton.setEnabled(!isBallMode);
        followBallButton.setEnabled(!isLineMode);

        // Joystick only works if no auto-mode is active
        boolean manualEnabled = !isLineMode && !isBallMode;
        joystickBase.setAlpha(manualEnabled ? 1.0f : 0.3f);
        joystickHandle.setAlpha(manualEnabled ? 1.0f : 0.3f);
        speedSlider.setEnabled(manualEnabled);
    }

    public void onClickConnect(View v) {
        if (isConnected) {
            orb.close();
            isConnected = false;
            updateUI();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            startDeviceListActivity();
        } else {
            BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.S)
    private void startDeviceListActivity() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED ||
                ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN}, REQUEST_BT_PERMISSIONS);
        } else {
            BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
        }
    }

    private void connectToDevice(final BluetoothDevice device) {
        isConnecting = true;
        updateUI();
        new Thread(() -> {
            final boolean success = orb.openBT(device);
            if (success) {
                orb.configMotor(ORB.M1, 144, 50, 50, 30);
                orb.configMotor(ORB.M4, 144, 50, 50, 30);
            }
            runOnUiThread(() -> {
                isConnected = success;
                isConnecting = false;
                updateUI();
                if (!success) Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_LONG).show();
            });
        }).start();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_OK && data != null) {
            BluetoothDevice device = BT_DeviceListActivity.getDeviceFromIntent(data);
            if (device != null) connectToDevice(device);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_CAMERA_PERMISSION && grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startCamera();
        }
    }

    public void onClickGetBattery(View v) {
        if (isConnected) Toast.makeText(this, "Battery: " + orb.getVcc() + "V", Toast.LENGTH_SHORT).show();
    }

    // --- JOYSTICK ---

    @SuppressLint("ClickableViewAccessibility")
    private void setupJoystickListener() {
        joystickBase.getViewTreeObserver().addOnGlobalLayoutListener(() -> {
            joystickBaseCenterX = joystickBase.getX() + joystickBase.getWidth() / 2f;
            joystickBaseCenterY = joystickBase.getY() + joystickBase.getHeight() / 2f;
            joystickBaseRadius = joystickBase.getWidth() / 2f;
        });
        joystickBase.setOnTouchListener((view, event) -> {
            if (!isConnected || isLineMode || isBallMode) return false;

            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_MOVE:
                    float dx = event.getX() - (joystickBase.getWidth() / 2f);
                    float dy = event.getY() - (joystickBase.getHeight() / 2f);
                    float distance = (float) Math.sqrt(dx * dx + dy * dy);
                    float clampedX = (distance > joystickBaseRadius) ? (dx / distance) * joystickBaseRadius : dx;
                    float clampedY = (distance > joystickBaseRadius) ? (dy / distance) * joystickBaseRadius : dy;

                    joystickHandle.setX(joystickBase.getX() + (joystickBase.getWidth() / 2f) + clampedX - (joystickHandle.getWidth() / 2f));
                    joystickHandle.setY(joystickBase.getY() + (joystickBase.getHeight() / 2f) + clampedY - (joystickHandle.getHeight() / 2f));
                    updateRobotMovement(clampedX, clampedY);
                    return true;
                case MotionEvent.ACTION_UP:
                    joystickHandle.setX(joystickBaseCenterX - joystickHandle.getWidth() / 2f);
                    joystickHandle.setY(joystickBaseCenterY - joystickHandle.getHeight() / 2f);
                    stopRobot();
                    return true;
            }
            return false;
        });
    }

    private void updateRobotMovement(float dx, float dy) {
        float normalizedX = dx / joystickBaseRadius;
        float normalizedY = -dy / joystickBaseRadius;
        int maxSpeed = (int) speedSlider.getValue();
        int leftSpeed = (int) (maxSpeed * (normalizedY + normalizedX));
        int rightSpeed = (int) (maxSpeed * (normalizedY - normalizedX));
        leftSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, leftSpeed));
        rightSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, rightSpeed));
        orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
        orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);
    }

    private void stopRobot() {
        if (isConnected) {
            orb.setMotor(ORB.M1, ORB.BRAKE_MODE, 0, 0);
            orb.setMotor(ORB.M4, ORB.BRAKE_MODE, 0, 0);
        }
    }

    // ============================================================================================
    // ANALYZER 1: LINE FOLLOWER (PCA + ROTATION)
    // ============================================================================================
    private class LineAnalyzer implements ImageAnalysis.Analyzer {
        private Bitmap debugBitmap;
        private Canvas debugCanvas;
        private Paint pointPaint, linePaint, boxPaint;
        private float smoothedAngle = 0f;
        private boolean hasSmoothed = false;

        public LineAnalyzer() {
            pointPaint = new Paint(); pointPaint.setColor(Color.GREEN); pointPaint.setStrokeWidth(5);
            linePaint = new Paint(); linePaint.setColor(Color.RED); linePaint.setStrokeWidth(8);
            boxPaint = new Paint(); boxPaint.setColor(Color.BLUE); boxPaint.setStyle(Paint.Style.STROKE); boxPaint.setStrokeWidth(3);
        }

        @Override
        public void analyze(@NonNull ImageProxy image) {
            if (!isLineMode) { image.close(); return; }

            ByteBuffer buffer = image.getPlanes()[0].getBuffer();
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);

            int srcWidth = image.getWidth();
            int srcHeight = image.getHeight();

            // Rotate 90 Clockwise for Portrait Mode
            byte[] rotated = new byte[data.length];
            rotateY90Clockwise(data, rotated, srcWidth, srcHeight);

            int width = srcHeight;
            int height = srcWidth;

            if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
                debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                debugCanvas = new Canvas(debugBitmap);
            }
            debugBitmap.eraseColor(Color.TRANSPARENT);

            // ROI: Bottom Half
            int startRow = 0;
            int endRow = (int) (height * 0.5f);
            debugCanvas.drawRect(0, startRow, width, endRow, boxPaint);

            long sumX = 0, sumY = 0;
            int count = 0;
            int threshold = 60;
            int step = 10;

            for (int y = startRow; y < endRow; y += step) {
                int rowOffset = y * width;
                for (int x = 0; x < width; x += step) {
                    int index = rowOffset + x;
                    if (index >= rotated.length) continue;
                    if ((rotated[index] & 0xFF) < threshold) {
                        sumX += x; sumY += y; count++;
                        debugCanvas.drawPoint(x, y, pointPaint);
                    }
                }
            }

            int steering = 0;
            if (count > 30) {
                float meanX = (float) sumX / count;
                float meanY = (float) sumY / count;

                double covXX = 0, covYY = 0, covXY = 0;
                for (int y = startRow; y < endRow; y += step) {
                    int rowOffset = y * width;
                    for (int x = 0; x < width; x += step) {
                        int index = rowOffset + x;
                        if (index >= rotated.length) continue;
                        if ((rotated[index] & 0xFF) < threshold) {
                            double dx = x - meanX;
                            double dy = y - meanY;
                            covXX += dx * dx; covYY += dy * dy; covXY += dx * dy;
                        }
                    }
                }

                double angleRad = 0.5 * Math.atan2(2.0 * covXY, covXX - covYY);
                double desiredHeading = -Math.PI / 2.0;
                double errorAngle = angleRad - desiredHeading;

                // Normalize angle [-PI, PI]
                while (errorAngle > Math.PI) errorAngle -= 2 * Math.PI;
                while (errorAngle < -Math.PI) errorAngle += 2 * Math.PI;

                // Smoothing
                float angleDeg = (float) Math.toDegrees(angleRad);
                if (!hasSmoothed) { smoothedAngle = angleDeg; hasSmoothed = true; }
                else { smoothedAngle = smoothedAngle * 0.8f + angleDeg * 0.2f; }

                // Visualization Line
                float smoothedRad = (float) Math.toRadians(smoothedAngle);
                float len = Math.min(width, height) * 0.9f;
                float endX = meanX + len * (float) Math.cos(smoothedRad);
                float endY = meanY + len * (float) Math.sin(smoothedRad);
                debugCanvas.drawLine(meanX, meanY, endX, endY, linePaint);

                // Control
                float KP_POS = 0.63f;
                float KP_ANG = 0.11f;
                float errorPos = meanX - (width / 2.0f);
                float errorAngleDeg = (float) Math.toDegrees(errorAngle);

                steering = Math.round(KP_POS * errorPos + KP_ANG * errorAngleDeg);
                steering = Math.max(-800, Math.min(800, steering));
            }

            runOnUiThread(() -> {
                imageView.setImageBitmap(debugBitmap);
                imageView.setRotation(0);
            });

            if (isConnected) {
                if (count > 30) {
                    // Auto speed movement
                    driveRobot(AUTO_SPEED - steering, AUTO_SPEED + steering);
                } else {
                    stopRobot();
                }
            }
            image.close();
        }

        private void rotateY90Clockwise(byte[] src, byte[] dst, int srcWidth, int srcHeight) {
            int dstWidth = srcHeight;
            for (int y = 0; y < srcHeight; y++) {
                int rowOffset = y * srcWidth;
                for (int x = 0; x < srcWidth; x++) {
                    dst[x * dstWidth + (dstWidth - 1 - y)] = src[rowOffset + x];
                }
            }
        }

        // Helper for autonomous driving
        private void driveRobot(int left, int right) {
            if (!isConnected) return;
            left = Math.max(-1000, Math.min(1000, left));
            right = Math.max(-1000, Math.min(1000, right));
            orb.setMotor(ORB.M1, ORB.SPEED_MODE, -left, 0);
            orb.setMotor(ORB.M4, ORB.SPEED_MODE, right, 0);
        }
    }



}
