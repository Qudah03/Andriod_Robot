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
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.slider.Slider; // <-- IMPORT SLIDER
import com.hbrs.Bluetooth.BT_DeviceListActivity;
import com.hbrs.ORB.ORB;

public class MainActivity extends AppCompatActivity {

    // Request codes for activities and permissions
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_BT_PERMISSIONS = 2;

    // UI Elements
    private Button connectButton;
    private Button batteryButton;
    private TextView statusText;
    private View joystickBase;
    private View joystickHandle;
    private Slider speedSlider; // <-- 1. DECLARE SLIDER VARIABLE

    // ORB object for robot communication
    private ORB orb;

    // State tracking
    private boolean isConnected = false;
    private boolean isConnecting = false;

    // Joystick properties
    private float joystickBaseCenterX;
    private float joystickBaseCenterY;
    private float joystickBaseRadius;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize UI elements by finding them in the layout
        connectButton = findViewById(R.id.btn_connect);
        batteryButton = findViewById(R.id.btn_get_battery); // <-- 2. INITIALIZE BATTERY BUTTON
        statusText = findViewById(R.id.status_text);
        joystickBase = findViewById(R.id.joystick_base);
        joystickHandle = findViewById(R.id.joystick_handle);
        speedSlider = findViewById(R.id.speed_slider); // <-- INITIALIZE SLIDER

        // Initialize the ORB communication handler
        orb = new ORB(this);

        // Set up the listener for joystick movement
        setupJoystickListener();

        // Update the UI to its initial state
        updateUI();
    }

    // ... (onClickConnect, startDeviceListActivity, onActivityResult, onRequestPermissionsResult, connectToDevice methods remain the same) ...
    public void onClickConnect(View v) {
        if (isConnected) {
            // If already connected, disconnect
            orb.close();
            isConnected = false;
            updateUI();
        } else {
            // If not connected, start the connection process
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                startDeviceListActivity();
            }
        }
    }

    /**
     * Checks for Bluetooth permissions and starts the BT_DeviceListActivity.
     */
    @RequiresApi(api = Build.VERSION_CODES.S)
    private void startDeviceListActivity() {
        // Check for required Bluetooth permissions (for Android 12+)
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED ||
                ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {

            // If permissions are not granted, request them
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN},
                    REQUEST_BT_PERMISSIONS);
        } else {
            // If permissions are already granted, start the device list activity
            BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
        }
    }

    /**
     * Callback for the result from an activity (e.g., device selection or permission request).
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        if (requestCode == REQUEST_ENABLE_BT) {
            if (resultCode == RESULT_OK && data != null) {
                // A device was selected from the list, get it from the intent
                BluetoothDevice device = BT_DeviceListActivity.getDeviceFromIntent(data);
                if (device != null) {
                    // Connect to the selected device
                    connectToDevice(device);
                } else {
                    Toast.makeText(this, "Failed to get device.", Toast.LENGTH_SHORT).show();
                }
            } else if (resultCode == BT_DeviceListActivity.RESULT_NOPERMISSION) {
                Toast.makeText(this, "Bluetooth permissions not granted.", Toast.LENGTH_LONG).show();
            }
        }
    }

    /**
     * Callback for the result of a permission request.
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_BT_PERMISSIONS) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // Permissions were granted, now start the device list activity
                BT_DeviceListActivity.start(this, REQUEST_ENABLE_BT);
            } else {
                // Permissions were denied
                Toast.makeText(this, "Bluetooth permissions are required to connect.", Toast.LENGTH_LONG).show();
            }
        }
    }

    /**
     * Connects to the chosen Bluetooth device on a background thread.
     */
    private void connectToDevice(final BluetoothDevice device) {
        isConnecting = true;
        updateUI(); // Show "Connecting..." status

        new Thread(() -> {
            // Attempt to open the Bluetooth connection
            final boolean success = orb.openBT(device);

            // *** NEW CODE: CONFIGURE MOTORS ON SUCCESSFUL CONNECTION ***
            if (success) {
                // This is a one-time setup that tells the ORB controller how to handle the motors.
                // These values are typical for standard Makeblock motors. Adjust if you use different hardware.
                final int TICS_PER_ROTATION = 144;
                final int ACCELERATION      = 50;
                final int KP                = 50;
                final int KI                = 30;

                // Configure Motor 1 (assuming it's the left motor)
                orb.configMotor(ORB.M1, TICS_PER_ROTATION, ACCELERATION, KP, KI);
                // Configure Motor 2 (assuming it's the right motor)
                orb.configMotor(ORB.M4, TICS_PER_ROTATION, ACCELERATION, KP, KI);
            }

            // Update the UI on the main thread
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

    /**
     *  ADD THE OnClick HANDLER FOR THE BATTERY BUTTON
     * Handles clicks on the "Get Battery" button.
     */
    public void onClickGetBattery(View v) {
        if (isConnected && orb != null) {
            float voltage = orb.getVcc();
            // Display the voltage in a Toast message
            Toast.makeText(this, String.format("Battery Voltage: %.2fV", voltage), Toast.LENGTH_LONG).show();
        }
    }

    @SuppressLint("ClickableViewAccessibility")
    private void setupJoystickListener() {
        // Use a ViewTreeObserver to get the dimensions of the joystick base after it's drawn
        joystickBase.getViewTreeObserver().addOnGlobalLayoutListener(() -> {
            // Calculate the center of the base once the layout is complete
            joystickBaseCenterX = joystickBase.getX() + joystickBase.getWidth() / 2f;
            joystickBaseCenterY = joystickBase.getY() + joystickBase.getHeight() / 2f;
            joystickBaseRadius = joystickBase.getWidth() / 2f;
        });

        joystickBase.setOnTouchListener((view, event) -> {
            // Only process touch events if connected
            if (!isConnected) return false;

            // Get the touch coordinates relative to the joystick base
            float touchX = event.getX();
            float touchY = event.getY();

            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_MOVE:
                    // Calculate the displacement (dx, dy) from the center of the joystick base
                    float dx = touchX - (joystickBase.getWidth() / 2f);
                    float dy = touchY - (joystickBase.getHeight() / 2f);

                    // Calculate the distance from the center
                    float distance = (float) Math.sqrt(dx * dx + dy * dy);

                    // This will be the value sent to the motor control function
                    float clampedDx = dx;
                    float clampedDy = dy;

                    // If the touch is outside the joystick base, clamp the values
                    if (distance > joystickBaseRadius) {
                        clampedDx = (dx / distance) * joystickBaseRadius;
                        clampedDy = (dy / distance) * joystickBaseRadius;
                    }

                    // *** CORRECTED VISUAL POSITIONING ***
                    // Update the handle's visual position.
                    // We subtract half the handle's width/height to center it on the clamped point.
                    joystickHandle.setX(joystickBase.getX() + (joystickBase.getWidth() / 2f) + clampedDx - (joystickHandle.getWidth() / 2f));
                    joystickHandle.setY(joystickBase.getY() + (joystickBase.getHeight() / 2f) + clampedDy - (joystickHandle.getHeight() / 2f));

                    // Convert the clamped joystick position to robot movement and send commands
                    updateRobotMovement(clampedDx, clampedDy);
                    return true;

                case MotionEvent.ACTION_UP:
                    // Reset the handle to the center and stop the robot
                    joystickHandle.setX(joystickBaseCenterX - joystickHandle.getWidth() / 2f);
                    joystickHandle.setY(joystickBaseCenterY - joystickHandle.getHeight() / 2f);
                    stopRobot();
                    return true;
            }
            return false;
        });
    }

    /**
     * Calculates motor speeds based on joystick position and sends them to the robot.
     * @param dx The horizontal displacement from the center.
     * @param dy The vertical displacement from the center.
     */
    private void updateRobotMovement(float dx, float dy) {
        // Normalize the values to a range of -1.0 to 1.0
        float normalizedX = dx / joystickBaseRadius;
        float normalizedY = -dy / joystickBaseRadius; // Invert Y-axis for intuitive control

        // *** 3. GET MAX SPEED FROM THE SLIDER ***
        final int maxSpeed = (int) speedSlider.getValue();

        // Calculate left and right motor speeds for differential drive steering
        int leftSpeed = (int) (maxSpeed * (normalizedY + normalizedX));
        int rightSpeed = (int) (maxSpeed * (normalizedY - normalizedX));

        // Clamp speeds to the max value
        leftSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, leftSpeed));
        rightSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, rightSpeed));

        // Send speed commands to the motors (assuming M1 is left, M2 is right)
        orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
        orb.setMotor(ORB.M4, ORB.SPEED_MODE, +rightSpeed, 0);
    }

    /**
     * Stops both motors.
     */
    private void stopRobot() {
        orb.setMotor(ORB.M1, ORB.BRAKE_MODE, 0, 0);
        orb.setMotor(ORB.M4, ORB.BRAKE_MODE, 0, 0);
    }

    /**
     * Updates the UI elements based on the current connection state.
     */
    @SuppressLint("SetTextI18n")
    private void updateUI() {
        if (isConnected) {
            statusText.setText("Status: Connected to " + orb.getDeviceName());
            connectButton.setText("Disconnect");
            joystickBase.setAlpha(1.0f); // Make joystick fully visible
            joystickHandle.setAlpha(1.0f);
            speedSlider.setEnabled(true); // Enable slider when connected
            batteryButton.setEnabled(true); // Enable battery button when connected
        } else {
            connectButton.setText("Connect");
            joystickBase.setAlpha(0.3f); // Dim the joystick when not connected
            joystickHandle.setAlpha(0.3f);
            speedSlider.setEnabled(false); // Disable slider when disconnected
            if (isConnecting) {
                statusText.setText("Status: Connecting...");
            } else {
                statusText.setText("Status: Not Connected");
            }
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Ensure the connection is closed when the app is destroyed
        if (orb != null) {
            orb.close();
        }
    }
}
