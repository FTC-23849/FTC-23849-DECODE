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
@TeleOp(name = "OpenCV Ball Counter")
public class RampDetectionBlob extends LinearOpMode {

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

    public static double MIN_BLOB_AREA = 10.0;
    public static double MAX_BLOB_AREA = 100.0;

    public static int ballCount = 0;

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
            telemetry.addData("Ball Count", ballCount);
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
        camera.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPRIGHT);
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

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int count = 0;
            for (MatOfPoint c : contours) {
                if (Imgproc.contourArea(c) > MIN_BLOB_AREA && Imgproc.contourArea(c) < MAX_BLOB_AREA) {
                    count++;
                    Rect r = Imgproc.boundingRect(c);
                    Imgproc.rectangle(input, r, new Scalar(0, 0, 255), 2);
                }
            }

            ballCount = count;
            return input;
        }
    }
}