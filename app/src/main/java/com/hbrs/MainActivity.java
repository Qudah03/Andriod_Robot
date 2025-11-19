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
import androidx.lifecycle.LifecycleOwner;

import com.google.android.material.slider.Slider;
import com.google.common.util.concurrent.ListenableFuture;
import com.hbrs.Bluetooth.BT_DeviceListActivity;
import com.hbrs.ORB.ORB;

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
    private ImageView imageView; // Overlay for drawing

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
    private final int LINE_FOLLOW_SPEED = 1000;
    private final float P_GAIN = 0.8f; // Proportional gain for steering (configurable)

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

    // --- Image Analyzer with Optimizations ---
    // --- Image Analyzer with ROBUST detection and CLEAN UI (No Drawing) ---
/***    private class MyAnalyzer implements ImageAnalysis.Analyzer {

        @SuppressLint("UnsafeOptInUsageError")
        @Override
        public void analyze(@NonNull ImageProxy imageProxy) {
            try {
                // We don't need the overlay for drawing anymore, but the structure is kept for potential future use.
                // The main logic is now just detection and control.

                int height = imageProxy.getHeight();
                int width = imageProxy.getWidth();
                int[] scanlinesY = {
                        (int) (height * 0.4), // Top scan
                        (int) (height * 0.6), // Middle scan
                        (int) (height * 0.8)  // Bottom scan
                };

                int totalPoints = 0;
                int averageCenterX = 0;

                // --- Scan for the line at multiple points ---
                for (int y : scanlinesY) {
                    int trackCenter = findTrackCenterAdaptive(imageProxy, y);
                    if (trackCenter != -1) {
                        averageCenterX += trackCenter;
                        totalPoints++;
                    }
                }

                int targetX = -1;
                if (totalPoints > 0) {
                    // The target X is the average of all detected center points.
                    targetX = averageCenterX / totalPoints;
                }

                // --- Robot Control ---
                if (isConnected && isLineFollowingActive) {
                    if (targetX != -1) {
                        float error = (width / 2.0f) - targetX;
                        int steeringCorrection = (int) (error * P_GAIN);
                        int leftSpeed = LINE_FOLLOW_SPEED + steeringCorrection;
                        int rightSpeed = LINE_FOLLOW_SPEED - steeringCorrection;

                        orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                        orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);
                    } else {
                        // Line is completely lost, stop the robot for safety.
                        stopRobot();
                    }
                }

                // --- VISUALIZATION IS REMOVED ---
                // No drawing calls to overlayCanvas.
                // No update to the imageView, so it remains transparent.

            } finally {
                // CRITICAL: Always close the imageProxy
                imageProxy.close();
            }
        }


         // Finds the center of a dark track using an adaptive threshold.
         // This is much more robust to lighting changes than a fixed threshold.

        @SuppressLint("UnsafeOptInUsageError")
        private int findTrackCenterAdaptive(ImageProxy image, int y) {
            Bitmap bm = image.toBitmap();
            int width = bm.getWidth();
            int startOfTrack = -1;
            int endOfTrack = -1;

            int minBrightness = 255;
            int maxBrightness = 0;

            // 1. First Pass: Find min/max brightness
            for (int x = 0; x < width; x++) {
                int pixel = bm.getPixel(x, y);
                int brightness = (Color.red(pixel) + Color.green(pixel) + Color.blue(pixel)) / 3;
                if (brightness < minBrightness) minBrightness = brightness;
                if (brightness > maxBrightness) maxBrightness = brightness;
            }

            // 2. Calculate adaptive threshold
            int threshold = minBrightness + (maxBrightness - minBrightness) / 2;

            // 3. Second Pass: Find the track using the threshold
            for (int x = 0; x < width; x++) {
                int pixel = bm.getPixel(x, y);
                int brightness = (Color.red(pixel) + Color.green(pixel) + Color.blue(pixel)) / 3;

                if (brightness < threshold) {
                    if (startOfTrack == -1) {
                        startOfTrack = x;
                    }
                    endOfTrack = x;
                }
            }

            if (startOfTrack != -1 && (endOfTrack - startOfTrack) > 10) {
                return startOfTrack + ((endOfTrack - startOfTrack) / 2);
            }

            return -1; // Track not found
        }
    }
***/

// --- Image Analyzer with ADVANCED TARGETING VISUALIZATION ---
private class MyAnalyzer implements ImageAnalysis.Analyzer {@SuppressLint("UnsafeOptInUsageError")
@Override
public void analyze(@NonNull ImageProxy imageProxy) {
    try {
        // The main logic is now detection, targeting, and control.
        // Visualization is removed for a clean UI.

        int height = imageProxy.getHeight();
        int width = imageProxy.getWidth();

        // 1. Scan the image at two points to determine the line's angle
        int topY = (int) (height * 0.4);
        int bottomY = (int) (height * 0.8);

        int topX = findTrackCenterAdaptive(imageProxy, topY);
        int bottomX = findTrackCenterAdaptive(imageProxy, bottomY);

        // 2. Determine the robot's target point and control logic
        if (isConnected && isLineFollowingActive) {
            if (topX != -1 && bottomX != -1) {
                // --- We have two points, so we can calculate the line's angle and position ---

                // Calculate the horizontal offset of the line from the center at the bottom of the view
                float currentOffset = bottomX - (width / 2.0f);

                // Calculate the angle of the line. A positive angle means the line is pointing to the right.
                // A small angle correction factor might be needed depending on camera lens distortion.
                float angle = (float) Math.toDegrees(Math.atan2(topX - bottomX, topY - bottomY));

                // --- ADVANCED CONTROL ---
                // The error is a combination of the current offset and the angle of the line.
                // This acts like a "PD" (Proportional-Derivative) controller.
                // The 'offset' is the P term (where are we now?)
                // The 'angle' is the D term (where are we going?)
                float error = currentOffset * 0.5f + angle * 1.5f;

                int steeringCorrection = (int) (error * P_GAIN);
                int leftSpeed = LINE_FOLLOW_SPEED + steeringCorrection;
                int rightSpeed = LINE_FOLLOW_SPEED - steeringCorrection;

                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);

            } else if (bottomX != -1) {
                // --- We only see the bottom of the line, use simple control ---
                float error = (width / 2.0f) - bottomX;
                int steeringCorrection = (int) (error * P_GAIN);
                int leftSpeed = LINE_FOLLOW_SPEED + steeringCorrection;
                int rightSpeed = LINE_FOLLOW_SPEED - steeringCorrection;
                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);
            } else {
                // Line is completely lost, stop the robot for safety.
                stopRobot();
            }
        }

        // --- VISUALIZATION IS REMOVED as per the previous request ---

    } finally {
        // CRITICAL: Always close the imageProxy
        imageProxy.close();
    }
}

    /**
     * Finds the center of a dark track using an adaptive threshold.
     * This is much more robust to lighting changes than a fixed threshold.
     */
    @SuppressLint("UnsafeOptInUsageError")
    private int findTrackCenterAdaptive(ImageProxy image, int y) {
        Bitmap bm = image.toBitmap();
        int width = bm.getWidth();
        int startOfTrack = -1;
        int endOfTrack = -1;

        int minBrightness = 255;
        int maxBrightness = 0;

        // 1. First Pass: Find min/max brightness
        for (int x = 0; x < width; x++) {
            int pixel = bm.getPixel(x, y);
            int brightness = (Color.red(pixel) + Color.green(pixel) + Color.blue(pixel)) / 3;
            if (brightness < minBrightness) minBrightness = brightness;
            if (brightness > maxBrightness) maxBrightness = brightness;
        }

        // 2. Calculate adaptive threshold
        int threshold = minBrightness + (maxBrightness - minBrightness) / 2;

        // 3. Second Pass: Find the track using the threshold
        for (int x = 0; x < width; x++) {
            int pixel = bm.getPixel(x, y);
            int brightness = (Color.red(pixel) + Color.green(pixel) + Color.blue(pixel)) / 3;

            if (brightness < threshold) {
                if (startOfTrack == -1) {
                    startOfTrack = x;
                }
                endOfTrack = x;
            }
        }

        if (startOfTrack != -1 && (endOfTrack - startOfTrack) > 10) {
            return startOfTrack + ((endOfTrack - startOfTrack) / 2);
        }

        return -1; // Track not found
    }
}
    // --- CAMERA AND LINE FOLLOWING LOGIC ---

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
        isLineFollowingActive = !isLineFollowingActive; // Toggle the state
        if (!isLineFollowingActive && isConnected) {
            stopRobot(); // Stop motors when switching to manual
        }
        updateUI(); // Update button text and joystick state
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
        isLineFollowingActive = false; // Always turn off line following when camera is off
        if (isConnected) stopRobot();
        updateUI();
        Toast.makeText(this, "Camera Off", Toast.LENGTH_SHORT).show();
    }

    private void bindCameraUseCases() {
        cameraProvider.unbindAll();

        Preview preview = new Preview.Builder().build();
        preview.setSurfaceProvider(previewView.getSurfaceProvider());

        imageAnalysis = new ImageAnalysis.Builder()
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
        // Bluetooth connection state
        if (isConnected) {
            statusText.setText("Status: Connected to " + orb.getDeviceName());
            connectButton.setText("Disconnect");
            batteryButton.setEnabled(true);
        } else {
            statusText.setText(isConnecting ? "Status: Connecting..." : "Status: Not Connected");
            connectButton.setText("Connect");
            batteryButton.setEnabled(false);
        }

        // Camera and Line Following state
        cameraCard.setVisibility(isCameraPreviewOn ? View.VISIBLE : View.GONE);
        followLineButton.setVisibility(isCameraPreviewOn && isConnected ? View.VISIBLE : View.GONE);

        if (isCameraPreviewOn) {
            // Manual vs. Automatic control
            boolean manualControlEnabled = !isLineFollowingActive;
            joystickBase.setAlpha(manualControlEnabled ? 1.0f : 0.3f);
            joystickHandle.setAlpha(manualControlEnabled ? 1.0f : 0.3f);
            speedSlider.setEnabled(manualControlEnabled);
            followLineButton.setText(isLineFollowingActive ? "Stop Line Follow" : "Start Line Follow");
        } else {
            // When camera is off, joystick is enabled if connected
            joystickBase.setAlpha(isConnected ? 1.0f : 0.3f);
            joystickHandle.setAlpha(isConnected ? 1.0f : 0.3f);
            speedSlider.setEnabled(isConnected);
        }
    }


    // --- PERMISSION AND ACTIVITY RESULTS ---
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
            else Toast.makeText(this, "Failed to get device.", Toast.LENGTH_SHORT).show();
        }
    }

    // --- ROBOT AND JOYSTICK CONTROL LOGIC ---
    public void onClickConnect(View v) {
        if (isConnected) {
            orb.close();
            isConnected = false;
            updateUI();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            startDeviceListActivity();
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
            if (!isConnected || isLineFollowingActive) return false; // Disable if not connected OR line following

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
