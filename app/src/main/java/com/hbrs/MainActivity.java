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
    // Gain for the positional offset (how far left/right)
    private final float P_GAIN_POS = 1.2f;
    // Gain for the angle (how sharp is the turn)
    private final float P_GAIN_ANGLE = 15.0f;

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

    // --- IMPROVED PCA ANALYZER (portrait -> rotated, ROI bottom, stable control) ---
    private class MyAnalyzer implements ImageAnalysis.Analyzer {

        private Bitmap debugBitmap;
        private android.graphics.Canvas debugCanvas;
        private android.graphics.Paint pointPaint;
        private android.graphics.Paint linePaint;
        private android.graphics.Paint boxPaint;

        // smoothing state
        private float smoothedAngle = 0f;
        private boolean hasSmoothed = false;

        public MyAnalyzer() {
            pointPaint = new android.graphics.Paint();
            pointPaint.setColor(android.graphics.Color.GREEN);
            pointPaint.setStrokeWidth(5);

            linePaint = new android.graphics.Paint();
            linePaint.setColor(android.graphics.Color.RED);
            linePaint.setStrokeWidth(8);

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

            // 1) Read Y-plane (grayscale) into byte[]
            ByteBuffer buffer = image.getPlanes()[0].getBuffer();
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);

            int srcWidth = image.getWidth();
            int srcHeight = image.getHeight();

            // NOTE: you're testing in portrait mode -> rotate the Y plane 90deg clockwise
            // so that "up" in the processed frame corresponds to robot forward
            byte[] rotated = new byte[data.length];
            rotateY90Clockwise(data, rotated, srcWidth, srcHeight);

            // After rotation, processed width/height swap
            int width = srcHeight;   // rotated width
            int height = srcWidth;   // rotated height

            // 2) Prepare debug bitmap of rotated size
            if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
                debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                debugCanvas = new android.graphics.Canvas(debugBitmap);
            }

            debugBitmap.eraseColor(android.graphics.Color.TRANSPARENT);


            // --- ROI (Region of Interest) CONTROL ---
            // The ROI determines how big the blue box is.
            // height = frame height AFTER rotation
            // width  = frame width AFTER rotation
            //
            // You can tune the ROI by changing 'startRow' and 'endRow':
            //
            // 1) startRow controls how FAR DOWN from the top the box begins.
            //    - startRow = 0 → full height (largest box)
            //    - startRow = height * 0.5 → bottom 50%
            //    - startRow = height * 0.66 → bottom 33% (small box)
            //
            // 2) endRow controls how FAR it goes downward.
            //    - endRow = height - 1 → goes to the bottom (recommended)
            //    - Smaller endRow → shorter box
            //
            // 3) If you want to also limit width (make box narrower):
            //      leftX  = width * 0.2   // 20% from the left
            //      rightX = width * 0.8   // 80% from the left
            //
            // Example:
            // int startRow = (int)(height * 0.50f); // bottom half
            // int endRow   = height - 1;            // down to the bottom
            // int leftX    = 0;                      // full width
            // int rightX   = width - 1;              // full width

            // 3) ROI: bottom third of the rotated image (closer to robot)
            int startRow = 0;
            // int endRow = height - 1;
            // int startRow = 0; full view
            int endRow = (int) (height * 0.5f);

            debugCanvas.drawRect(0, startRow, width, endRow, boxPaint);

            // 4) PCA data collection (single pass)
            long sumX = 0;
            long sumY = 0;
            int count = 0;
            final int threshold = 60;     // tuned for your image, tweak if necessary
            final int step = 10;           // pixel step for speed/coverage

            for (int y = startRow; y < endRow; y += step) {
                int rowOffset = y * width;
                for (int x = 0; x < width; x += step) {
                    int index = rowOffset + x;
                    if (index >= rotated.length) continue;
                    int pixelVal = rotated[index] & 0xFF;
                    if (pixelVal < threshold) {
                        sumX += x;
                        sumY += y;
                        count++;
                        debugCanvas.drawPoint(x, y, pointPaint);
                    }
                }
            }

            int steering = 0;

            if (count > 30) { // enough points to do PCA reliably
                // centroid
                float meanX = (float) sumX / count;
                float meanY = (float) sumY / count;

                // covariance
                double covXX = 0;
                double covYY = 0;
                double covXY = 0;

                for (int y = startRow; y < endRow; y += step) {
                    int rowOffset = y * width;
                    for (int x = 0; x < width; x += step) {
                        int index = rowOffset + x;
                        if (index >= rotated.length) continue;
                        if ((rotated[index] & 0xFF) < threshold) {
                            double dx = x - meanX;
                            double dy = y - meanY;
                            covXX += dx * dx;
                            covYY += dy * dy;
                            covXY += dx * dy;
                        }
                    }
                }

                // principal axis angle (radians) relative to X axis
                double angleRad = 0.5 * Math.atan2(2.0 * covXY, covXX - covYY);

                // Ensure a stable orientation direction:
                // We want the principal axis to point "forward" (negative Y in image).
                // The desired heading (robot forward in rotated frame) is -PI/2.
                double desiredHeading = -Math.PI / 2.0;
                double errorAngle = normalizeAngle(angleRad - desiredHeading); // [-pi, pi]

                // simple EMA smoothing for angle (prevents large jumps)
                float angleDeg = (float) Math.toDegrees(angleRad);
                if (!hasSmoothed) {
                    smoothedAngle = angleDeg;
                    hasSmoothed = true;
                } else {
                    final float alpha = 0.20f; // smoothing factor, tweak 0.1..0.4
                    smoothedAngle = smoothedAngle * (1.0f - alpha) + angleDeg * alpha;
                }

                // draw PCA orientation line (using smoothed angle)
                float smoothedRad = (float) Math.toRadians(smoothedAngle);
                float len = Math.min(width, height) * 0.9f;
                float endX = meanX + len * (float) Math.cos(smoothedRad);
                float endY = meanY + len * (float) Math.sin(smoothedRad);
                debugCanvas.drawLine(meanX, meanY, endX, endY, linePaint);

                // Control law (tuned, conservative)
                final float KP_POS = 0.63f;   // position gain (pixels -> speed)
                final float KP_ANG = 0.11f;  // angle gain (degrees -> speed)

                float errorPos = meanX - (width / 2.0f); // positive -> line is to the right

                // angle error in degrees for mixing
                float errorAngleDeg = (float) Math.toDegrees(errorAngle);

                // combine with reasonable scaling
                float steeringF = KP_POS * errorPos + KP_ANG * errorAngleDeg;

                // integer steering and clamp (you can adjust max steering)
                steering = Math.round(steeringF);
                final int MAX_STEER = 800;
                steering = Math.max(-MAX_STEER, Math.min(MAX_STEER, steering));
            }

            // Update UI (show rotated debugBitmap)
            runOnUiThread(() -> {
                imageView.setImageBitmap(debugBitmap);
                // IMPORTANT: Do not rotate the view. We already rotated the data.
                imageView.setRotation(0);
            });

            // 7) Send Motor Commands (keep your motor API calls unchanged)
            if (isConnected) {
                if (count > 30) {
                    int baseSpeed = LINE_FOLLOW_SPEED;

                    // Mixing: positive steering -> turn right: decrease left, increase right
                    // (If the robot turns the wrong way, invert steering sign here)
                    int leftSpeed = baseSpeed - steering;
                    int rightSpeed = baseSpeed + steering;

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

        // rotate Y-plane 90° clockwise: dstWidth = srcHeight, dstHeight = srcWidth
        private void rotateY90Clockwise(byte[] src, byte[] dst, int srcWidth, int srcHeight) {
            // src index: y * srcWidth + x
            // dst index: x' * dstWidth + y' where after rotation x' = y, y' = dstHeight - 1 - x
            // Equivalent mapping for 90deg CW:
            // dst[(x) * srcHeight + (srcHeight - 1 - y)] = src[y * srcWidth + x]
            int dstWidth = srcHeight;
            int dstHeight = srcWidth;
            for (int y = 0; y < srcHeight; y++) {
                int rowOffset = y * srcWidth;
                for (int x = 0; x < srcWidth; x++) {
                    int sIdx = rowOffset + x;
                    int dIdx = x * dstWidth + (dstWidth - 1 - y);
                    if (sIdx >= 0 && sIdx < src.length && dIdx >= 0 && dIdx < dst.length) {
                        dst[dIdx] = src[sIdx];
                    }
                }
            }
        }

        // normalize angle to [-PI, PI]
        private double normalizeAngle(double a) {
            while (a <= -Math.PI) a += 2.0 * Math.PI;
            while (a > Math.PI) a -= 2.0 * Math.PI;
            return a;
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
