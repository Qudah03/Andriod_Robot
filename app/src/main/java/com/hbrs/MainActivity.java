package com.hbrs;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.core.CameraSelector;
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

    // Request codes for activities and permissions
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_BT_PERMISSIONS = 2;
    private static final int REQUEST_CAMERA_PERMISSION = 3;

    // UI Elements
    private Button connectButton, batteryButton;
    private TextView statusText;
    private View joystickBase, joystickHandle;
    private Slider speedSlider;
    private CardView cameraCard;
    private PreviewView previewView;

    // ORB object for robot communication
    private ORB orb;

    // State tracking
    private boolean isConnected = false;
    private boolean isConnecting = false;

    // Joystick properties
    private float joystickBaseCenterX, joystickBaseCenterY, joystickBaseRadius;

    // CameraX variables
    private ProcessCameraProvider cameraProvider;
    private boolean isCameraPreviewOn = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize all UI elements
        connectButton = findViewById(R.id.btn_connect);
        batteryButton = findViewById(R.id.btn_get_battery);
        statusText = findViewById(R.id.status_text);
        joystickBase = findViewById(R.id.joystick_base);
        joystickHandle = findViewById(R.id.joystick_handle);
        speedSlider = findViewById(R.id.speed_slider);
        cameraCard = findViewById(R.id.camera_card);
        previewView = findViewById(R.id.previewView);

        // Initialize the ORB communication handler
        orb = new ORB(this);

        // Set up listeners and initial UI state
        setupJoystickListener();
        updateUI();
    }

    // --- CAMERA LOGIC ---

    public void onClickToggleCamera(View v) {
        if (isCameraPreviewOn) {
            stopCamera();
        } else {
            // Check for camera permission first
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                startCamera(); // Permission already granted
            } else {
                // Request permission
                ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, REQUEST_CAMERA_PERMISSION);
            }
        }
    }

    private void startCamera() {
        // Show the camera view container
        cameraCard.setVisibility(View.VISIBLE);

        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                cameraProvider = cameraProviderFuture.get();
                bindPreview(cameraProvider);
                isCameraPreviewOn = true;
            } catch (ExecutionException | InterruptedException e) {
                Toast.makeText(this, "Error starting camera: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                cameraCard.setVisibility(View.INVISIBLE); // Hide on error
            }
        }, ContextCompat.getMainExecutor(this));
    }

    private void stopCamera() {
        if (cameraProvider != null) {
            cameraProvider.unbindAll(); // This detaches the camera from the view
            isCameraPreviewOn = false;
            cameraCard.setVisibility(View.INVISIBLE); // Hide the view
            Toast.makeText(this, "Camera Off", Toast.LENGTH_SHORT).show();
        }
    }

    private void bindPreview(@NonNull ProcessCameraProvider cameraProvider) {
        // Unbind any old use cases before rebinding
        cameraProvider.unbindAll();

        Preview preview = new Preview.Builder().build();

        CameraSelector cameraSelector = new CameraSelector.Builder()
                .requireLensFacing(CameraSelector.LENS_FACING_BACK)
                .build();

        preview.setSurfaceProvider(previewView.getSurfaceProvider());

        // Bind all use cases to the camera provider
        cameraProvider.bindToLifecycle((LifecycleOwner) this, cameraSelector, preview);

        Toast.makeText(this, "Camera On", Toast.LENGTH_SHORT).show();
    }

    // --- PERMISSION AND ACTIVITY RESULTS ---

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                startCamera(); // Permission was granted, now start the camera
            } else {
                Toast.makeText(this, "Camera permission is required to show the preview.", Toast.LENGTH_LONG).show();
            }
        } else if (requestCode == REQUEST_BT_PERMISSIONS) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
            } else {
                Toast.makeText(this, "Bluetooth permissions are required to connect.", Toast.LENGTH_LONG).show();
            }
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_ENABLE_BT) {
            if (resultCode == RESULT_OK && data != null) {
                BluetoothDevice device = BT_DeviceListActivity.getDeviceFromIntent(data);
                if (device != null) {
                    connectToDevice(device);
                } else {
                    Toast.makeText(this, "Failed to get device.", Toast.LENGTH_SHORT).show();
                }
            } else if (resultCode == BT_DeviceListActivity.RESULT_NOPERMISSION) {
                Toast.makeText(this, "Bluetooth permissions not granted.", Toast.LENGTH_LONG).show();
            }
        }
    }

    // --- BLUETOOTH AND ROBOT CONTROL LOGIC (UNCHANGED) ---

    public void onClickConnect(View v) {
        if (isConnected) {
            orb.close();
            isConnected = false;
            updateUI();
        } else {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                startDeviceListActivity();
            }
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.S)
    private void startDeviceListActivity() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED ||
                ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN},
                    REQUEST_BT_PERMISSIONS);
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
                final int TICS_PER_ROTATION = 144;
                final int ACCELERATION = 50;
                final int KP = 50;
                final int KI = 30;
                orb.configMotor(ORB.M1, TICS_PER_ROTATION, ACCELERATION, KP, KI);
                orb.configMotor(ORB.M4, TICS_PER_ROTATION, ACCELERATION, KP, KI);
            }
            runOnUiThread(() -> {
                isConnected = success;
                isConnecting = false;
                if (!success) {
                    Toast.makeText(MainActivity.this, "Connection Failed!", Toast.LENGTH_LONG).show();
                }
                updateUI();
            });
        }).start();
    }

    @SuppressLint("DefaultLocale")
    public void onClickGetBattery(View v) {
        if (isConnected && orb != null) {
            float voltage = orb.getVcc();
            Toast.makeText(this, String.format("Battery Voltage: %.2fV", voltage), Toast.LENGTH_LONG).show();
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
            if (!isConnected) return false;
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_MOVE:
                    float dx = event.getX() - (joystickBase.getWidth() / 2f);
                    float dy = event.getY() - (joystickBase.getHeight() / 2f);
                    float distance = (float) Math.sqrt(dx * dx + dy * dy);
                    float clampedDx = dx;
                    float clampedDy = dy;
                    if (distance > joystickBaseRadius) {
                        clampedDx = (dx / distance) * joystickBaseRadius;
                        clampedDy = (dy / distance) * joystickBaseRadius;
                    }
                    joystickHandle.setX(joystickBase.getX() + (joystickBase.getWidth() / 2f) + clampedDx - (joystickHandle.getWidth() / 2f));
                    joystickHandle.setY(joystickBase.getY() + (joystickBase.getHeight() / 2f) + clampedDy - (joystickHandle.getHeight() / 2f));
                    updateRobotMovement(clampedDx, clampedDy);
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
        final int maxSpeed = (int) speedSlider.getValue();
        int leftSpeed = (int) (maxSpeed * (normalizedY + normalizedX));
        int rightSpeed = (int) (maxSpeed * (normalizedY - normalizedX));
        leftSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, leftSpeed));
        rightSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, rightSpeed));
        orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
        orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);
    }

    private void stopRobot() {
        orb.setMotor(ORB.M1, ORB.BRAKE_MODE, 0, 0);
        orb.setMotor(ORB.M4, ORB.BRAKE_MODE, 0, 0);
    }

    @SuppressLint("SetTextI18n")
    private void updateUI() {
        if (isConnected) {
            statusText.setText("Status: Connected to " + orb.getDeviceName());
            connectButton.setText("Disconnect");
            joystickBase.setAlpha(1.0f);
            joystickHandle.setAlpha(1.0f);
            speedSlider.setEnabled(true);
            batteryButton.setEnabled(true);
        } else {
            connectButton.setText("Connect");
            joystickBase.setAlpha(0.3f);
            joystickHandle.setAlpha(0.3f);
            speedSlider.setEnabled(false);
            batteryButton.setEnabled(false);
            if (isConnecting) {
                statusText.setText("Status: Connecting...");
            } else {
                statusText.setText("Status: Not Connected");
            }
        }
    }
}
