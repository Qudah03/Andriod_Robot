package com.hbrs;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.widget.ImageView;
import androidx.annotation.NonNull;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import com.hbrs.ORB.ORB;
import java.nio.ByteBuffer;

public class BallFollowerAnalyzer implements ImageAnalysis.Analyzer {

    private final ORB orb;
    private final ImageView imageView;
    private final boolean isConnected;
    private final Runnable stopRobotAction;

    private Bitmap debugBitmap;
    private Canvas debugCanvas;
    private final Paint redPaint;
    private final Paint targetPaint;

    // Reuse this array to avoid garbage collection lag
    private final float[] hsvCache = new float[3];

    // --- HSV TUNING PARAMETERS ---
    // Red wraps around 0 (e.g., 0-25 and 335-360)
    private static final float RED_HUE_LOW_LIMIT = 25f;  // Lower band end
    private static final float RED_HUE_HIGH_LIMIT = 335f; // Upper band start
    private static final float MIN_SATURATION = 0.5f;     // Must be vibrant (0.0 - 1.0)
    private static final float MIN_VALUE = 0.4f;          // Must not be too dark (0.0 - 1.0)

    private static final int MIN_BLOB_SIZE = 25;
    private static final float TARGET_RADIUS = 150f;

    public BallFollowerAnalyzer(ORB orb, ImageView imageView, boolean isConnected, Runnable stopRobotAction) {
        this.orb = orb;
        this.imageView = imageView;
        this.isConnected = isConnected;
        this.stopRobotAction = stopRobotAction;

        redPaint = new Paint();
        redPaint.setColor(Color.RED);
        redPaint.setStyle(Paint.Style.FILL);
        redPaint.setAlpha(120);

        targetPaint = new Paint();
        targetPaint.setColor(Color.YELLOW);
        targetPaint.setStyle(Paint.Style.STROKE);
        targetPaint.setStrokeWidth(8);
    }

    @Override
    public void analyze(@NonNull ImageProxy image) {
        // We need all three planes (Y, U, V) to convert to RGB/HSV
        ByteBuffer yBuffer = image.getPlanes()[0].getBuffer();
        ByteBuffer uBuffer = image.getPlanes()[1].getBuffer();
        ByteBuffer vBuffer = image.getPlanes()[2].getBuffer();

        int srcWidth = image.getWidth();
        int srcHeight = image.getHeight();

        int uvRowStride = image.getPlanes()[1].getRowStride();
        int uvPixelStride = image.getPlanes()[1].getPixelStride();

        int width = srcHeight;
        int height = srcWidth;

        if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
            debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            debugCanvas = new Canvas(debugBitmap);
        }
        debugBitmap.eraseColor(Color.TRANSPARENT);

        long sumX = 0;
        long sumY = 0;
        int pixelCount = 0;

        int minX = width, maxX = 0;
        int minY = height, maxY = 0;

        // Step 4 is a good balance between speed and detection distance
        int step = 4;

        for (int y = 0; y < srcHeight; y += step) {
            for (int x = 0; x < srcWidth; x += step) {

                // 1. Get YUV indices
                int yIndex = y * srcWidth + x;
                int uvIndex = (y / 2) * uvRowStride + (x / 2) * uvPixelStride;

                // Safety check for buffer bounds
                if (yIndex >= yBuffer.remaining() || uvIndex >= uBuffer.remaining()) continue;

                // 2. Extract Raw YUV
                int Y = yBuffer.get(yIndex) & 0xFF;
                int U = uBuffer.get(uvIndex) & 0xFF;
                int V = vBuffer.get(uvIndex) & 0xFF;

                // 3. Convert YUV -> RGB
                // Standard integer conversion formula (faster than float)
                int r = (int) (Y + 1.370705f * (V - 128));
                int g = (int) (Y - 0.698001f * (V - 128) - 0.337633f * (U - 128));
                int b = (int) (Y + 1.732446f * (U - 128));

                // Clamp to 0-255
                r = Math.max(0, Math.min(255, r));
                g = Math.max(0, Math.min(255, g));
                b = Math.max(0, Math.min(255, b));

                // 4. Convert RGB -> HSV
                Color.RGBToHSV(r, g, b, hsvCache);
                float hue = hsvCache[0];        // 0 .. 360
                float saturation = hsvCache[1]; // 0 .. 1
                float value = hsvCache[2];      // 0 .. 1

                // 5. Check "Redness" in HSV
                // Red is tricky: it exists at the start (0-25) AND end (335-360) of the circle.
                boolean isRedHue = (hue < RED_HUE_LOW_LIMIT || hue > RED_HUE_HIGH_LIMIT);
                boolean isSaturated = saturation > MIN_SATURATION;
                boolean isBright = value > MIN_VALUE;

                if (isRedHue && isSaturated && isBright) {

                    // Coordinate Rotation & Inversion Fix
                    int rotX = width - 1 - y;
                    int rotY = width - 1 - x;

                    sumX += rotX;
                    sumY += rotY;
                    pixelCount++;

                    if (rotX < minX) minX = rotX;
                    if (rotX > maxX) maxX = rotX;
                    if (rotY < minY) minY = rotY;
                    if (rotY > maxY) maxY = rotY;

                    debugCanvas.drawPoint(rotX, rotY, redPaint);
                }
            }
        }

        boolean ballDetected = false;
        float ballRadius = 0;
        float centerX = 0;

        if (pixelCount > MIN_BLOB_SIZE) {
            centerX = (float) sumX / pixelCount;
            float centerY = (float) sumY / pixelCount;

            float widthBox = maxX - minX;
            float heightBox = maxY - minY;
            ballRadius = (widthBox + heightBox) / 4.0f;

            debugCanvas.drawCircle(centerX, centerY, ballRadius, targetPaint);
            debugCanvas.drawLine(centerX - 10, centerY, centerX + 10, centerY, targetPaint);
            debugCanvas.drawLine(centerX, centerY - 10, centerX, centerY + 10, targetPaint);

            ballDetected = true;
        }

        // --- MOTOR CONTROL LOGIC (Preserved from your working version) ---
        if (isConnected) {
            if (ballDetected) {
                // 1. Steering
                float errorX = centerX - (width / 2.0f);
                int steering = (int) (errorX * -1.2f); // Negative gain for correction

                // 2. Distance
                float radiusError = TARGET_RADIUS - ballRadius;
                int forwardSpeed = (int) (radiusError * 8.0f);

                forwardSpeed = Math.max(-600, Math.min(800, forwardSpeed));

                if (Math.abs(radiusError) < 15) forwardSpeed = 0;

                int leftSpeed = forwardSpeed - steering;
                int rightSpeed = forwardSpeed + steering;

                leftSpeed = Math.max(-1000, Math.min(1000, leftSpeed));
                rightSpeed = Math.max(-1000, Math.min(1000, rightSpeed));

                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, rightSpeed, 0);
            } else {
                // Search pattern
                int searchSpeed = 150;
                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -searchSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, searchSpeed, 0);
            }
        }

        imageView.post(() -> {
            imageView.setImageBitmap(debugBitmap);
            imageView.setRotation(0);
        });

        image.close();
    }
}