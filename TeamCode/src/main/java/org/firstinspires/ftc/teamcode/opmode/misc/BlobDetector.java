package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

public class BlobDetector {
    public String alliance = "Blue";

    public double PURPLE_H_LOW = 117;
    public double PURPLE_S_LOW = 75;
    public double PURPLE_V_LOW = 30;

    public double PURPLE_H_HIGH = 155;
    public double PURPLE_S_HIGH = 240;
    public double PURPLE_V_HIGH = 240;

    public double GREEN_H_LOW = 65;
    public double GREEN_S_LOW = 120;
    public double GREEN_V_LOW = 30;

    public double GREEN_H_HIGH = 89;
    public double GREEN_S_HIGH = 255;
    public double GREEN_V_HIGH = 220;

    public double MIN_BLOB_AREA = 500.0;
    public double MERGE_DISTANCE = 60.0;

    private final OpenCvCamera camera;

    private final int camWidth;
    private final int camHeight;

    private volatile double x;
    private volatile double y;

    private final double cmPerPixel;
    private final double pixelOffset;

    public BlobDetector(
            HardwareMap hardwareMap,
            String webcamName,
            int width,
            int height,
            double cmPerPixel,
            double pixelOffset
            ,String Alliance
    ) {
        this.camWidth = width;
        this.camHeight = height;
        this.cmPerPixel = cmPerPixel;
        this.pixelOffset = pixelOffset;
        this.alliance = Alliance;

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, webcamName)
        );

        camera.setPipeline(new Pipeline());
    }

    public void start() {
        camera.openCameraDevice();
        camera.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPRIGHT);
        camera.showFpsMeterOnViewport(false);

    }

    public void stop() {
        camera.stopStreaming();
    }

    public double getOffsetPixels() {
        return x - camWidth * 0.5;
    }

    public double getOffsetCm() {
        double cm = 0;
        if(alliance.equals("Red")){
            cm = -1 * (cmPerPixel * (getOffsetPixels() + pixelOffset));
        }else{
            cm = (cmPerPixel * (getOffsetPixels() + pixelOffset));
        }
        return cm;
    }

    private class Pipeline extends OpenCvPipeline {

        private final Mat hsv = new Mat();
        private final Mat maskPurple = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat mask = new Mat();
        private final Mat hierarchy = new Mat();
        private final Mat kernel =
                Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        private final List<MatOfPoint> contours = new ArrayList<>();
        private final List<Rect> rects = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(
                    hsv,
                    new Scalar(PURPLE_H_LOW, PURPLE_S_LOW, PURPLE_V_LOW),
                    new Scalar(PURPLE_H_HIGH, PURPLE_S_HIGH, PURPLE_V_HIGH),
                    maskPurple
            );

            Core.inRange(
                    hsv,
                    new Scalar(GREEN_H_LOW, GREEN_S_LOW, GREEN_V_LOW),
                    new Scalar(GREEN_H_HIGH, GREEN_S_HIGH, GREEN_V_HIGH),
                    maskGreen
            );

            Core.bitwise_or(maskPurple, maskGreen, mask);

            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            contours.clear();
            rects.clear();

            Imgproc.findContours(
                    mask,
                    contours,
                    hierarchy,
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            for (MatOfPoint c : contours) {
                if (Imgproc.contourArea(c) > MIN_BLOB_AREA) {
                    rects.add(Imgproc.boundingRect(c));
                }
            }

            Rect best = null;

            for (int i = 0; i < rects.size(); i++) {
                Rect r = rects.get(i);
                for (int j = i + 1; j < rects.size(); j++) {
                    Rect o = rects.get(j);
                    if (centerDistance(r, o) < MERGE_DISTANCE) {
                        r = union(r, o);
                    }
                }
                if (best == null || r.area() > best.area()) best = r;
            }

            if (best != null) {
                x = best.x + best.width * 0.5;
                y = best.y + best.height * 0.5;
            }

            return input;
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
