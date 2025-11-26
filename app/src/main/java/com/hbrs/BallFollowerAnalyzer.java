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

    // Tuning Parameters - ADJUST THESE FOR YOUR LIGHTING AND BALL
    // These are optimized for a broader, more reliable red detection in YUV (YCrCb)
    private static final int RED_THRESHOLD_V_MIN = 160; // Cr (V) must be high for Red/Orange
    private static final int RED_THRESHOLD_U_MAX = 110; // Cb (U) must be low for Red
    private static final int MIN_BLOB_SIZE = 25;        // Minimum pixels to consider a "ball" (reduced for far objects)
    private static final float TARGET_RADIUS = 150f;    // Robot stops when ball looks this big

    public BallFollowerAnalyzer(ORB orb, ImageView imageView, boolean isConnected, Runnable stopRobotAction) {
        this.orb = orb;
        this.imageView = imageView;
        this.isConnected = isConnected;
        this.stopRobotAction = stopRobotAction;

        redPaint = new Paint();
        redPaint.setColor(Color.RED);
        redPaint.setStyle(Paint.Style.FILL);
        redPaint.setAlpha(120); // Slightly more opaque

        targetPaint = new Paint();
        targetPaint.setColor(Color.YELLOW); // Changed to Yellow for better contrast on screen
        targetPaint.setStyle(Paint.Style.STROKE);
        targetPaint.setStrokeWidth(8);
    }

    @Override
    public void analyze(@NonNull ImageProxy image) {
        ByteBuffer uBuffer = image.getPlanes()[1].getBuffer();
        ByteBuffer vBuffer = image.getPlanes()[2].getBuffer();

        int srcWidth = image.getWidth();
        int srcHeight = image.getHeight();

        int uvRowStride = image.getPlanes()[1].getRowStride();
        int uvPixelStride = image.getPlanes()[1].getPixelStride();

        // Prepare Debug Bitmap (Rotated for Portrait)
        int width = srcHeight; // New width is old height
        int height = srcWidth; // New height is old width

        if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
            debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            debugCanvas = new Canvas(debugBitmap);
        }
        debugBitmap.eraseColor(Color.TRANSPARENT);

        long sumX = 0;
        long sumY = 0;
        int pixelCount = 0;

        // Bounding Box Tracking
        int minX = width, maxX = 0;
        int minY = height, maxY = 0;

        int step = 4; // **Reduced step to 4** for better long-distance (small object) detection

        for (int y = 0; y < srcHeight; y += step) {
            for (int x = 0; x < srcWidth; x += step) {

                // YUV planes are sub-sampled (4:2:0), so U/V indices are based on x/2, y/2
                int uvIndex = (y / 2) * uvRowStride + (x / 2) * uvPixelStride;
                if (uvIndex >= uBuffer.remaining() || uvIndex >= vBuffer.remaining()) continue;

                int u = uBuffer.get(uvIndex) & 0xFF;
                int v = vBuffer.get(uvIndex) & 0xFF;

                // Check for "Redness" in YUV space with optimized thresholds
                if (v > RED_THRESHOLD_V_MIN && u < RED_THRESHOLD_U_MAX) {

                    // Rotate coordinates: (x, y) -> (y, width - 1 - x)
                    // The rotation seems correct for the orientation setup (portrait),
                    // so we use rotX as the horizontal (steering) axis and rotY as the vertical axis.
                    int rotX = width - 1 - y; // The Y axis of the source image becomes the X axis of the rotated image
                    int rotY = width - 1 - x; // The X axis of the source image becomes the Y axis of the rotated image

                    sumX += rotX;
                    sumY += rotY;
                    pixelCount++;

                    // Update bounding box
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

        // Filter Noise: Only react if blob is big enough
        if (pixelCount > MIN_BLOB_SIZE) {
            centerX = (float) sumX / pixelCount;
            float centerY = (float) sumY / pixelCount;

            // Calculate rough radius from bounding box
            float widthBox = maxX - minX;
            float heightBox = maxY - minY;
            // Use average dimension for a more robust "radius" against perspective distortion
            ballRadius = (widthBox + heightBox) / 4.0f;

            // Visualization
            debugCanvas.drawCircle(centerX, centerY, ballRadius, targetPaint);

            // Draw Center Crosshair
            debugCanvas.drawLine(centerX - 10, centerY, centerX + 10, centerY, targetPaint);
            debugCanvas.drawLine(centerX, centerY - 10, centerX, centerY + 10, targetPaint);

            ballDetected = true;
        }

        // --- MOTOR CONTROL LOGIC ---
        if (isConnected) {
            if (ballDetected) {
                // 1. Steering (Left/Right)
                // Positive errorX means the ball is to the RIGHT of the center (width / 2.0f).
                // Robot needs to turn RIGHT.
                // Right motor (M4) speed increases, Left motor (M1) speed decreases.
                float errorX = centerX - (width / 2.0f);
                int steering = (int) (errorX * -1.2f); // Increased P-Gain for turning response

                // 2. Distance Control (Forward/Backward)
                // Positive radiusError means TARGET_RADIUS is bigger than ballRadius -> Ball is TOO FAR.
                // Negative radiusError means ballRadius is bigger than TARGET_RADIUS -> Ball is TOO CLOSE.
                float radiusError = TARGET_RADIUS - ballRadius;
                int forwardSpeed = (int) (radiusError * 8.0f); // Increased P-Gain for distance

                // Clamp Max Speed
                forwardSpeed = Math.max(-600, Math.min(800, forwardSpeed));

                // Stop if we are "close enough" (deadband)
                if (Math.abs(radiusError) < 15) forwardSpeed = 0; // Tighter deadband

                // Apply steering
                int leftSpeed = forwardSpeed - steering;
                int rightSpeed = forwardSpeed + steering;

                // Safety Clamps
                leftSpeed = Math.max(-1000, Math.min(1000, leftSpeed));
                rightSpeed = Math.max(-1000, Math.min(1000, rightSpeed));

                // IMPORTANT: M1 has -leftSpeed, M4 has +rightSpeed, which is often correct for
                // differential drive where one motor is inverted. Keep this as it was.
                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, rightSpeed, 0);
            } else {
                // Lost the ball? Spin slowly to search.
                int searchSpeed = 150;
                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -searchSpeed, 0); // Reverse Left (Turn Right)
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, searchSpeed, 0); // Forward Right (Turn Right)
            }
        }

        // Update UI
        imageView.post(() -> {
            imageView.setImageBitmap(debugBitmap);
            imageView.setRotation(0);
        });

        image.close();
    }
}