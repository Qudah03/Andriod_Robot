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
        byte[] data = new byte[buffer.remaining()];
        buffer.get(data);

        int srcWidth = image.getWidth();
        int srcHeight = image.getHeight();

        // Rotate logic for Portrait mode
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

        long sumX = 0;
        long sumY = 0;
        int count = 0;
        final int threshold = 60;
        final int step = 10;

        for (int y = startRow; y < endRow; y += step) {
            int rowOffset = y * width;
            for (int x = 0; x < width; x += step) {
                int index = rowOffset + x;
                if (index >= rotated.length) continue;
                if ((rotated[index] & 0xFF) < threshold) {
                    sumX += x;
                    sumY += y;
                    count++;
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
                        covXX += dx * dx;
                        covYY += dy * dy;
                        covXY += dx * dy;
                    }
                }
            }

            double angleRad = 0.5 * Math.atan2(2.0 * covXY, covXX - covYY);
            double desiredHeading = -Math.PI / 2.0;
            double errorAngle = normalizeAngle(angleRad - desiredHeading);

            float angleDeg = (float) Math.toDegrees(angleRad);
            if (!hasSmoothed) {
                smoothedAngle = angleDeg;
                hasSmoothed = true;
            } else {
                smoothedAngle = smoothedAngle * 0.8f + angleDeg * 0.2f;
            }

            float smoothedRad = (float) Math.toRadians(smoothedAngle);
            float len = Math.min(width, height) * 0.9f;
            float endX = meanX + len * (float) Math.cos(smoothedRad);
            float endY = meanY + len * (float) Math.sin(smoothedRad);
            debugCanvas.drawLine(meanX, meanY, endX, endY, linePaint);

            final float KP_POS = 0.63f;
            final float KP_ANG = 0.11f;
            float errorPos = meanX - (width / 2.0f);
            float errorAngleDeg = (float) Math.toDegrees(errorAngle);

            float steeringF = KP_POS * errorPos + KP_ANG * errorAngleDeg;
            steering = Math.round(steeringF);
            steering = Math.max(-800, Math.min(800, steering));
        }

        imageView.post(() -> {
            imageView.setImageBitmap(debugBitmap);
            imageView.setRotation(0);
        });

        if (isConnected) {
            if (count > 30) {
                int baseSpeed = 600;
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

    private void rotateY90Clockwise(byte[] src, byte[] dst, int srcWidth, int srcHeight) {
        int dstWidth = srcHeight;
        int dstHeight = srcWidth;
        for (int y = 0; y < srcHeight; y++) {
            for (int x = 0; x < srcWidth; x++) {
                dst[x * dstWidth + (dstWidth - 1 - y)] = src[y * srcWidth + x];
            }
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
