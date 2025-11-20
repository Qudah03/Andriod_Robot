package com.hbrs;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.os.Build;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
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

public class MainActivity extends AppCompatActivity {

    // Request codes
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_BT_PERMISSIONS = 2;
    private static final int REQUEST_CAMERA_PERMISSION = 3;

    // UI Elements
    private Button connectButton, batteryButton, followLineButton;
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

    // Line Following control
    private boolean isLineFollowingActive = false;
    // Adjusted base speed (lower is safer for testing)
    private final int LINE_FOLLOW_SPEED = 600;
    private final float P_GAIN = 1.5f; // Sensitivity

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize all UI elements
        connectButton = findViewById(R.id.btn_connect);
        batteryButton = findViewById(R.id.btn_get_battery);
        followLineButton = findViewById(R.id.btn_follow_line);
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

    // --- IMPROVED FAST ANALYZER ---
    // --- DEBUG ANALYZER WITH VISUALIZATION ---

    private class MyAnalyzer implements ImageAnalysis.Analyzer {

        private Bitmap debugBitmap;
        private android.graphics.Canvas debugCanvas;
        private android.graphics.Paint pointPaint;        private android.graphics.Paint linePaint;
        private android.graphics.Paint boxPaint;

        public MyAnalyzer() {
            pointPaint = new android.graphics.Paint();
            pointPaint.setColor(android.graphics.Color.GREEN);
            pointPaint.setStrokeWidth(5);

            linePaint = new android.graphics.Paint();
            linePaint.setColor(android.graphics.Color.RED);
            linePaint.setStrokeWidth(5);

            boxPaint = new android.graphics.Paint();
            boxPaint.setColor(android.graphics.Color.BLUE);
            boxPaint.setStyle(android.graphics.Paint.Style.STROKE);
            boxPaint.setStrokeWidth(3);
        }

        @Override
        public void analyze(@NonNull ImageProxy image) {
            if (!isLineFollowingActive) {
                image.close();
                return;
            }

            // 1. Get Image Data
            ByteBuffer buffer = image.getPlanes()[0].getBuffer();
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);

            int width = image.getWidth();
            int height = image.getHeight();

            // 2. Initialize Debug Bitmap
            if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
                debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                debugCanvas = new android.graphics.Canvas(debugBitmap);
            }

            // Clear previous drawing
            debugBitmap.eraseColor(android.graphics.Color.TRANSPARENT);

            // 3. Define Region of Interest (FULL SCREEN SCAN as requested)
            int startRow = 0;
            int endRow = height - 1;

            // Draw the ROI Box (Blue) - Covering the whole area
            debugCanvas.drawRect(0, startRow, width, endRow, boxPaint);

            long sumX = 0;
            int count = 0;
            int threshold = 100; // Tune this threshold (0-255)

            // 4. Scan Loop
            int step = 10; // Skip pixels for speed
            for (int y = startRow; y < endRow; y += step) {
                int rowOffset = y * width;
                for (int x = 0; x < width; x += step) {
                    int index = rowOffset + x;
                    if (index >= data.length) continue;

                    int pixelVal = data[index] & 0xFF;

                    if (pixelVal < threshold) {
                        sumX += x;
                        count++;
                        // DRAW GREEN DOT
                        debugCanvas.drawPoint(x, y, pointPaint);
                    }
                }
            }

            // 5. Control Logic
            int steering = 0;
            if (count > 10) {
                int avgX = (int) (sumX / count);

                // DRAW RED LINE at center
                debugCanvas.drawLine(avgX, startRow, avgX, endRow, linePaint);

                int error = avgX - (width / 2);
                steering = (int) (error * P_GAIN);
            }

            // 6. Update UI
            runOnUiThread(() -> {
                imageView.setImageBitmap(debugBitmap);
                // If the image looks sideways, you can uncomment this:
                  imageView.setRotation(90);
            });

            // 7. Send Motor Commands
            if (isConnected) {
                if (count > 10) {
                    int baseSpeed = LINE_FOLLOW_SPEED;
                    int leftSpeed = baseSpeed + steering;
                    int rightSpeed = baseSpeed - steering;

                    leftSpeed = Math.max(-1000, Math.min(1000, leftSpeed));
                    rightSpeed = Math.max(-1000, Math.min(1000, rightSpeed));

                    orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                    orb.setMotor(ORB.M4, ORB.SPEED_MODE, rightSpeed, 0);
                } else {
                    stopRobot();
                }
            }

            image.close();
        }
    }



    // --- CAMERA LOGIC ---

    public void onClickToggleCamera(View v) {
        if (isCameraPreviewOn) {
            stopCamera();
        } else {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                startCamera();
            } else {
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            }
        }
    }

    public void onClickToggleLineFollowing(View v) {
        isLineFollowingActive = !isLineFollowingActive;
        if (!isLineFollowingActive && isConnected) {
            stopRobot();
        }
        updateUI();
    }

    private void startCamera() {
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                cameraProvider = cameraProviderFuture.get();
                bindCameraUseCases();
                isCameraPreviewOn = true;
                updateUI();
            } catch (ExecutionException | InterruptedException e) {
                Toast.makeText(this, "Error starting camera: " + e.getMessage(), Toast.LENGTH_SHORT).show();
            }
        }, ContextCompat.getMainExecutor(this));
    }

    private void stopCamera() {
        if (cameraProvider != null) {
            cameraProvider.unbindAll();
        }
        isCameraPreviewOn = false;
        isLineFollowingActive = false;
        if (isConnected) stopRobot();
        updateUI();
    }

    private void bindCameraUseCases() {
        if (cameraProvider == null) return;

        cameraProvider.unbindAll();

        Preview preview = new Preview.Builder().build();
        preview.setSurfaceProvider(previewView.getSurfaceProvider());

        imageAnalysis = new ImageAnalysis.Builder()
                // STRATEGY_KEEP_ONLY_LATEST ensures we process only the newest frame
                // and drop old ones if processing is slow.
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build();

        imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(this), new MyAnalyzer());

        CameraSelector cameraSelector = new CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .build();

        cameraProvider.bindToLifecycle(this, cameraSelector, imageAnalysis, preview);
    }

    // --- UI and State Management ---

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
        followLineButton.setVisibility(isCameraPreviewOn && isConnected ? View.VISIBLE : View.GONE);

        if (isCameraPreviewOn) {
            boolean manualControlEnabled = !isLineFollowingActive;
            joystickBase.setAlpha(manualControlEnabled ? 1.0f : 0.3f);
            joystickHandle.setAlpha(manualControlEnabled ? 1.0f : 0.3f);
            speedSlider.setEnabled(manualControlEnabled);
            followLineButton.setText(isLineFollowingActive ? "Stop Line Follow" : "Start Line Follow");
        } else {
            joystickBase.setAlpha(isConnected ? 1.0f : 0.3f);
            joystickHandle.setAlpha(isConnected ? 1.0f : 0.3f);
            speedSlider.setEnabled(isConnected);
        }
    }


    // --- PERMISSIONS ---
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startCamera();
            } else {
                Toast.makeText(this, "Camera permission is required.", Toast.LENGTH_LONG).show();
            }
        } else if (requestCode == REQUEST_BT_PERMISSIONS) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
            } else {
                Toast.makeText(this, "Bluetooth permissions are required.", Toast.LENGTH_LONG).show();
            }
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_OK && data != null) {
            BluetoothDevice device = BT_DeviceListActivity.getDeviceFromIntent(data);
            if (device != null) connectToDevice(device);
        }
    }

    // --- ROBOT LOGIC ---
    public void onClickConnect(View v) {
        if (isConnected) {
            orb.close();
            isConnected = false;
            updateUI();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            startDeviceListActivity();
        } else {
            // Fallback for older Android versions if needed
            BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.S)
    private void startDeviceListActivity() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED || ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
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
                if (!success) Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_LONG).show();
                updateUI();
            });
        }).start();
    }

    @SuppressLint("DefaultLocale")
    public void onClickGetBattery(View v) {
        if (isConnected && orb != null) {
            Toast.makeText(this, String.format("Battery: %.2fV", orb.getVcc()), Toast.LENGTH_LONG).show();
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    private void setupJoystickListener() {
        joystickBase.getViewTreeObserver().addOnGlobalLayoutListener(() -> {
            joystickBaseCenterX = joystickBase.getX() + joystickBase.getWidth() / 2f;
            joystickBaseCenterY = joystickBase.getY() + joystickBase.getHeight() / 2f;
            joystickBaseRadius = joystickBase.getWidth() / 2f;
        });
        joystickBase.setOnTouchListener((view, event) -> {
            if (!isConnected || isLineFollowingActive) return false;

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
        if (orb != null && isConnected) {
            orb.setMotor(ORB.M1, ORB.BRAKE_MODE, 0, 0);
            orb.setMotor(ORB.M4, ORB.BRAKE_MODE, 0, 0);
        }
    }
}
