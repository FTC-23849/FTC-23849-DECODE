package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

public class RampDetector {

    public double PURPLE_H_LOW = 117;
    public double PURPLE_S_LOW = 20;
    public double PURPLE_V_LOW = 30;

    public double PURPLE_H_HIGH = 155;
    public double PURPLE_S_HIGH = 254;
    public double PURPLE_V_HIGH = 240;

    public double GREEN_H_LOW = 55;
    public double GREEN_S_LOW = 82;
    public double GREEN_V_LOW = 30;

    public double GREEN_H_HIGH = 89;
    public double GREEN_S_HIGH = 255;
    public double GREEN_V_HIGH = 255;

    public double MIN_BLOB_AREA = 50.0;
    public double MAX_BLOB_AREA = 400.0;

    private volatile int ballCount = 0;

    private final OpenCvCamera camera;
    private final int camWidth;
    private final int camHeight;

    public RampDetector(HardwareMap hardwareMap, String webcamName, int width, int height) {
        this.camWidth = width;
        this.camHeight = height;

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName)
        );

        camera.setPipeline(new Pipeline());
    }

    public void start() {
        camera.openCameraDevice();
        camera.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPSIDE_DOWN);
        camera.showFpsMeterOnViewport(false);
    }

    public void stop() {
        camera.stopStreaming();
    }

    public int getBallCount() {
        return ballCount;
    }

    private class Pipeline extends OpenCvPipeline {

        private final Mat hsv = new Mat();
        private final Mat maskPurple = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat mask = new Mat();
        private final Mat kernel =
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        private final List<MatOfPoint> contours = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsv,
                    new Scalar(PURPLE_H_LOW, PURPLE_S_LOW, PURPLE_V_LOW),
                    new Scalar(PURPLE_H_HIGH, PURPLE_S_HIGH, PURPLE_V_HIGH),
                    maskPurple);

            Core.inRange(hsv,
                    new Scalar(GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
                    new Scalar(GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH),
                    maskGreen);

            Core.bitwise_or(maskPurple, maskGreen, mask);

            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

            contours.clear();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int count = 0;
            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > MIN_BLOB_AREA && area < MAX_BLOB_AREA) {
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