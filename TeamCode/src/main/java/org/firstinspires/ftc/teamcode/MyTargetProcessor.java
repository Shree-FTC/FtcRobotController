package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class MyTargetProcessor implements VisionProcessor {

    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private volatile double targetX = 0.0;
    private volatile boolean targetFound = false;
    public int cameraWidth = 0; // Make this public so TeleOp can access it

    // Define the color range for detection (Example: Red target in HSV)
    private static final Scalar LOWER_BOUND = new Scalar(0, 100, 100);
    private static final Scalar UPPER_BOUND = new Scalar(10, 255, 255);

    // --- Motif storage ---
    private String[] motifOrder = {"green", "purple", "green"}; // default

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraWidth = width;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        contours.clear();

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOWER_BOUND, UPPER_BOUND, mask);
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.get(0);
            double maxArea = 0;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (maxArea > 100) { // Filter small noise
                Rect boundingBox = Imgproc.boundingRect(largestContour);
                targetX = boundingBox.x + (boundingBox.width / 2.0);
                targetFound = true;
            } else {
                targetFound = false;
            }
        } else {
            targetFound = false;
        }
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (targetFound) {
            Paint paint = new Paint();
            paint.setColor(Color.GREEN);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(5);

            float scaledX = (float) (targetX * scaleBmpPxToCanvasPx);
            canvas.drawCircle(scaledX, onscreenHeight / 2f, 20, paint);
        }
    }

    // =============================
    // Existing methods
    // =============================
    public double getTargetX() {
        return targetX;
    }

    public boolean targetDetected() {
        return targetFound;
    }

    // =============================
    // Motif methods
    // =============================
    public void updateMotifOrder(int tagId) {
        // Example logic based on tag ID:
        switch (tagId) {
            case 1:
                motifOrder = new String[]{"green", "purple", "purple"};
                break;
            case 2:
                motifOrder = new String[]{"purple", "green", "purple"};
                break;
            case 3:
                motifOrder = new String[]{"purple", "purple", "green"};
                break;
            default:
                motifOrder = new String[]{"green", "purple", "green"};
        }
    }

    public String[] getMotifOrder() {
        return motifOrder;
    }

    public int getDetectedTagId() {
        // Placeholder: implement actual AprilTag detection
        return 1;
    }
}
