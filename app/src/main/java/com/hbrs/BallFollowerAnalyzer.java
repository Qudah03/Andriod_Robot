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

    private final float[] hsvCache = new float[3];

    // --- HSV TUNING PARAMETERS ---
    private static final float RED_HUE_LOW_LIMIT = 20f;
    private static final float RED_HUE_HIGH_LIMIT = 340f;
    private static final float MIN_SATURATION = 0.65f;
    private static final float MIN_VALUE = 0.3f;

    private static final int MIN_BLOB_SIZE = 25;
    private static final float TARGET_RADIUS = 150f;

    // --- SMOOTHING VARIABLES ---
    private float smoothedCenterX = -1;
    private float smoothedRadius = -1;

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
        ByteBuffer yBuffer = image.getPlanes()[0].getBuffer();
        ByteBuffer uBuffer = image.getPlanes()[1].getBuffer();
        ByteBuffer vBuffer = image.getPlanes()[2].getBuffer();

        // Native Dimensions (Landscape)
        int width = image.getWidth();
        int height = image.getHeight();

        int uvRowStride = image.getPlanes()[1].getRowStride();
        int uvPixelStride = image.getPlanes()[1].getPixelStride();

        // 1. Setup Bitmap (Use native Width/Height)
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

        int step = 6;

        for (int y = 0; y < height; y += step) {
            for (int x = 0; x < width; x += step) {

                int yIndex = y * width + x;
                int uvIndex = (y / 2) * uvRowStride + (x / 2) * uvPixelStride;

                if (yIndex >= yBuffer.capacity() || uvIndex >= uBuffer.capacity()) continue;

                int Y = yBuffer.get(yIndex) & 0xFF;
                int U = uBuffer.get(uvIndex) & 0xFF;
                int V = vBuffer.get(uvIndex) & 0xFF;

                // RGB Conversion
                int r = (int) (Y + 1.370705f * (V - 128));
                int g = (int) (Y - 0.698001f * (V - 128) - 0.337633f * (U - 128));
                int b = (int) (Y + 1.732446f * (U - 128));

                r = Math.max(0, Math.min(255, r));
                g = Math.max(0, Math.min(255, g));
                b = Math.max(0, Math.min(255, b));

                // HSV Conversion
                Color.RGBToHSV(r, g, b, hsvCache);
                float hue = hsvCache[0];
                float saturation = hsvCache[1];
                float value = hsvCache[2];

                boolean isRedHue = (hue < RED_HUE_LOW_LIMIT || hue > RED_HUE_HIGH_LIMIT);
                boolean isSaturated = saturation > MIN_SATURATION;
                boolean isBright = value > MIN_VALUE;

                if (isRedHue && isSaturated && isBright) {

                    // --- NO ROTATION NEEDED (Landscape) ---
                    // Direct mapping: x is x, y is y
                    sumX += x;
                    sumY += y;
                    pixelCount++;

                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;

                    debugCanvas.drawPoint(x, y, redPaint);
                }
            }
        }

        boolean ballDetected = false;

        if (pixelCount > MIN_BLOB_SIZE) {
            float rawCenterX = (float) sumX / pixelCount;
            // Radius calculation
            float widthBox = maxX - minX;
            float heightBox = maxY - minY;
            float rawRadius = (widthBox + heightBox) / 4.0f;

            // Smoothing Logic
            if (smoothedCenterX == -1) {
                smoothedCenterX = rawCenterX;
                smoothedRadius = rawRadius;
            } else {
                smoothedCenterX = smoothedCenterX * 0.7f + rawCenterX * 0.3f;
                smoothedRadius = smoothedRadius * 0.7f + rawRadius * 0.3f;
            }

            // Draw circle at exact center
            debugCanvas.drawCircle(smoothedCenterX, height / 2f, smoothedRadius, targetPaint);
            debugCanvas.drawCircle(smoothedCenterX, height / 2f, 5, targetPaint);

            ballDetected = true;
        } else {
            smoothedCenterX = -1;
        }

        if (isConnected) {
            if (ballDetected) {
                // Center of screen is width/2
                float errorX = smoothedCenterX - (width / 2.0f);

                // --- STEERING ---
                // If X is small (Left), Error is negative.
                // If X is large (Right), Error is positive.
                // Depending on motor wiring, you might need negative or positive gain.
                // Standard: Turn TOWARD error.
                int steering = (int) (errorX * 0.8f);

                // Distance Control
                float radiusError = TARGET_RADIUS - smoothedRadius;
                int forwardSpeed = (int) (radiusError * 8.0f);

                forwardSpeed = Math.max(-600, Math.min(800, forwardSpeed));
                if (Math.abs(radiusError) < 20) forwardSpeed = 0;

                // Motor Mixing
                // Note: You might need to swap + and - here depending on if your phone
                // is mounted "Camera Left" or "Camera Right".
                // Try this first:
                int leftSpeed = forwardSpeed + steering;
                int rightSpeed = forwardSpeed - steering;

                leftSpeed = Math.max(-1000, Math.min(1000, leftSpeed));
                rightSpeed = Math.max(-1000, Math.min(1000, rightSpeed));

                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, rightSpeed, 0);
            } else {
                stopRobotAction.run();
            }
        }

        imageView.post(() -> {
            imageView.setImageBitmap(debugBitmap);
            // No rotation needed for ImageView either
            imageView.setRotation(0);
            // Make sure the image fills the width
            imageView.setScaleType(ImageView.ScaleType.FIT_XY);
        });

        image.close();
    }
}
