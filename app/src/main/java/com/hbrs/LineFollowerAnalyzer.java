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

public class LineFollowerAnalyzer implements ImageAnalysis.Analyzer {

    private final ORB orb;
    private final ImageView imageView;
    private final boolean isConnected;
    private final Runnable stopRobotAction;

    private Bitmap debugBitmap;
    private Canvas debugCanvas;
    private final Paint pointPaint;
    private final Paint linePaint;
    private final Paint boxPaint;

    // State
    private float smoothedAngle = 0f;
    private boolean hasSmoothed = false;
    private byte[] cachedYuvBytes;

    public LineFollowerAnalyzer(ORB orb, ImageView imageView, boolean isConnected, Runnable stopRobotAction) {
        this.orb = orb;
        this.imageView = imageView;
        this.isConnected = isConnected;
        this.stopRobotAction = stopRobotAction;

        pointPaint = new Paint();
        pointPaint.setColor(Color.GREEN);
        pointPaint.setStrokeWidth(5);

        linePaint = new Paint();
        linePaint.setColor(Color.RED);
        linePaint.setStrokeWidth(8);

        boxPaint = new Paint();
        boxPaint.setColor(Color.BLUE);
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(3);
    }

    @Override
    public void analyze(@NonNull ImageProxy image) {
        ByteBuffer buffer = image.getPlanes()[0].getBuffer();
        // byte[] data = new byte[buffer.remaining()];
        /*
        * This creates a massive new array (e.g., 300KB) 30 times per second.
        * This fills up memory quickly, forcing Android to pause your app to clean up (Garbage Collection).
        * This causes the robot to "hiccup" or lag.
        * */
        int remaining = buffer.remaining();

        //Only allocate if null or size changed
        if (cachedYuvBytes == null || cachedYuvBytes.length != remaining) {
            cachedYuvBytes = new byte[remaining];
        }
        buffer.get(cachedYuvBytes);

        // In Landscape, Width is the long side (e.g., 640), Height is short (e.g., 480)
        int width = image.getWidth();
        int height = image.getHeight();

        if (debugBitmap == null || debugBitmap.getWidth() != width || debugBitmap.getHeight() != height) {
            debugBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            debugCanvas = new Canvas(debugBitmap);
        }
        debugBitmap.eraseColor(Color.TRANSPARENT);

        // --- ROI CONFIGURATION ---
        // We want to look at the floor in front of the robot.
        // In image coordinates, Y=0 is top, Y=height is bottom.
        // Let's scan the bottom 50% of the image to avoid seeing the horizon/walls.

        int startRow = height / 2; // Start from middle
        int endRow = height - 10;  // Go to the bottom (minus small margin)

        // Draw the Blue Box to visualize where we are looking
        // Left=0, Top=startRow, Right=width, Bottom=endRow
        debugCanvas.drawRect(0, startRow, width, endRow, boxPaint);

        long sumX = 0;
        long sumY = 0;
        int count = 0;

        final int threshold = 50;
        final int step = 5;

        // Iterate through the ROI
        for (int y = startRow; y < endRow; y += step) {
            int rowOffset = y * width;
            for (int x = 0; x < width; x += step) {
                int index = rowOffset + x;
                if (index >= cachedYuvBytes.length) continue;

                // Check pixel darkness
                if ((cachedYuvBytes[index] & 0xFF) < threshold) {
                    sumX += x;
                    sumY += y;
                    count++;
                    debugCanvas.drawPoint(x, y, pointPaint);
                }
            }
        }

        int steering = 0;
        if (count > 10) {
            float meanX = (float) sumX / count;
            float meanY = (float) sumY / count;

            double covXX = 0, covYY = 0, covXY = 0;

            for (int y = startRow; y < endRow; y += step) {
                int rowOffset = y * width;
                for (int x = 0; x < width; x += step) {
                    int index = rowOffset + x;
                    if (index >= cachedYuvBytes.length) continue;

                    if ((cachedYuvBytes[index] & 0xFF) < threshold) {
                        double dx = x - meanX;
                        double dy = y - meanY;
                        covXX += dx * dx;
                        covYY += dy * dy;
                        covXY += dx * dy;
                    }
                }
            }

            // PCA Angle calculation
            double angleRad = 0.5 * Math.atan2(2.0 * covXY, covXX - covYY);

            // In Landscape, driving straight up is -90 degrees (-PI/2)
            double desiredHeading = -Math.PI / 2.0;
            double errorAngle = normalizeAngle(angleRad - desiredHeading);

            // Smoothing
            float angleDeg = (float) Math.toDegrees(angleRad);
            if (!hasSmoothed) {
                smoothedAngle = angleDeg;
                hasSmoothed = true;
            } else {
                smoothedAngle = smoothedAngle * 0.7f + angleDeg * 0.3f;
            }

            // Visualization
            float smoothedRad = (float) Math.toRadians(smoothedAngle);
            float len = 200;
            float endX = meanX + len * (float) Math.cos(smoothedRad);
            float endY = meanY + len * (float) Math.sin(smoothedRad);
            debugCanvas.drawLine(meanX, meanY, endX, endY, linePaint);

            // CONTROL LOGIC
            final float KP_POS = 0.7f;
            final float KP_ANG = 0.15f;

            // Error is distance from center width
            float errorPos = meanX - (width / 2.0f);
            float errorAngleDeg = (float) Math.toDegrees(errorAngle);

            float steeringF = KP_POS * errorPos + KP_ANG * errorAngleDeg;
            steering = Math.round(steeringF);
            steering = Math.max(-800, Math.min(800, steering));
        }

        imageView.post(() -> {
            imageView.setImageBitmap(debugBitmap);
            imageView.setRotation(0);
            imageView.setScaleType(ImageView.ScaleType.FIT_XY);
        });

        if (isConnected) {
            if (count > 10) {
                int baseSpeed = 600;

                // --- STEERING LOGIC ---
                // Try this configuration for Landscape.
                // If it turns opposite to the line, swap + and -
                int leftSpeed = baseSpeed - steering;
                int rightSpeed = baseSpeed + steering;

                leftSpeed = Math.max(-1000, Math.min(1000, leftSpeed));
                rightSpeed = Math.max(-1000, Math.min(1000, rightSpeed));

                orb.setMotor(ORB.M1, ORB.SPEED_MODE, -leftSpeed, 0);
                orb.setMotor(ORB.M4, ORB.SPEED_MODE, rightSpeed, 0);
            } else {
                stopRobotAction.run();
            }
        }
        image.close();
    }


    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
