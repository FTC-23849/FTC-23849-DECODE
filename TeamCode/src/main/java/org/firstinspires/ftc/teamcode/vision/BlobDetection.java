package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "OpenCV Blob Detection")
public class BlobDetection extends LinearOpMode {

    public static double PURPLE_H_LOW = 117;
    public static double PURPLE_S_LOW = 20;
    public static double PURPLE_V_LOW = 30;

    public static double PURPLE_H_HIGH = 155;
    public static double PURPLE_S_HIGH = 240;
    public static double PURPLE_V_HIGH = 240;

    public static double GREEN_H_LOW = 65;
    public static double GREEN_S_LOW = 82;
    public static double GREEN_V_LOW = 30;

    public static double GREEN_H_HIGH = 89;
    public static double GREEN_S_HIGH = 255;
    public static double GREEN_V_HIGH = 220;

    public static double MIN_BLOB_AREA = 500.0;
    public static double MERGE_DISTANCE = 100.0;

    double x = 0;
    double y = 0;
    double offsetX = 0;

    private OpenCvCamera camera;

    private static final int camWidth = 640;
    private static final int camHeight = 480;

    @Override
    public void runOpMode() {
        findBlob();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(camera, 30);
        waitForStart();

        while (opModeIsActive()) {
            offsetX = x - camWidth * 0.5;
            telemetry.addData("Blob X", x);
            telemetry.addData("Blob Y", y);
            telemetry.addData("Offset X", offsetX);
            telemetry.addData("Offset CM",(135.0/346.0  )*(offsetX+26));
            telemetry.update();
        }

        camera.stopStreaming();
    }

    private void findBlob() {
        int viewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), viewId);

        camera.setPipeline(new BallDetectionPipeline());
        camera.openCameraDevice();
        camera.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPSIDE_DOWN);
        camera.showFpsMeterOnViewport(false);
    }

    class BallDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lowerPurple = new Scalar(PURPLE_H_LOW, PURPLE_S_LOW, PURPLE_V_LOW);
            Scalar upperPurple = new Scalar(PURPLE_H_HIGH, PURPLE_S_HIGH, PURPLE_V_HIGH);

            Scalar lowerGreen = new Scalar(GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW);
            Scalar upperGreen = new Scalar(GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH);

            Mat maskPurple = new Mat();
            Mat maskGreen = new Mat();

            Core.inRange(hsv, lowerPurple, upperPurple, maskPurple);
            Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);

            Mat mask = new Mat();
            Core.bitwise_or(maskPurple, maskGreen, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<Rect> rects = new ArrayList<>();
            for (MatOfPoint c : contours) {
                if (Imgproc.contourArea(c) > MIN_BLOB_AREA) {
                    rects.add(Imgproc.boundingRect(c));
                }
            }

            List<Rect> merged = mergeCloseRects(rects, MERGE_DISTANCE);

            for (Rect r : merged) {
                Imgproc.rectangle(input, r, new Scalar(0, 0, 255), 2);
            }

            if (!merged.isEmpty()) {
                Rect largest = merged.get(0);
                for (Rect r : merged) {
                    if (r.area() > largest.area()) largest = r;
                }

                x = largest.x + largest.width * 0.5;
                y = largest.y + largest.height * 0.5;
                Imgproc.circle(input, new Point(x, y), 5, new Scalar(255, 0, 0), -1);
            }

            return input;
        }

        private List<Rect> mergeCloseRects(List<Rect> rects, double maxDistance) {
            boolean[] used = new boolean[rects.size()];
            List<Rect> out = new ArrayList<>();

            for (int i = 0; i < rects.size(); i++) {
                if (used[i]) continue;
                Rect merged = rects.get(i);

                for (int j = i + 1; j < rects.size(); j++) {
                    if (used[j]) continue;
                    if (centerDistance(merged, rects.get(j)) < maxDistance) {
                        merged = union(merged, rects.get(j));
                        used[j] = true;
                    }
                }
                out.add(merged);
            }
            return out;
        }

        private double centerDistance(Rect a, Rect b) {
            double dx = (a.x + a.width * 0.5) - (b.x + b.width * 0.5);
            double dy = (a.y + a.height * 0.5) - (b.y + b.height * 0.5);
            return Math.sqrt(dx * dx + dy * dy);
        }

        private Rect union(Rect a, Rect b) {
            int x1 = Math.min(a.x, b.x);
            int y1 = Math.min(a.y, b.y);
            int x2 = Math.max(a.x + a.width, b.x + b.width);
            int y2 = Math.max(a.y + a.height, b.y + b.height);
            return new Rect(x1, y1, x2 - x1, y2 - y1);
        }
    }
}
