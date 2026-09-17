package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class BallDetectorProcessor implements VisionProcessor {

    // ---------------- calibration (rotated frame, 960 x 1280) ----------------

    public static final double CAL_FX = 477.9;
    public static final double CAL_FY = 479.2;
    public static final double CAL_CX = 457.2;
    public static final double CAL_CY = 641.1;
    public static final double[] CAL_DIST = {0.008501, -0.02061464, 0.00075404, -0.00012449, 0.0};
    public static final int CAL_WIDTH = 960;
    public static final int CAL_HEIGHT = 1280;

    public static final double BALL_RADIUS_CM = 3.55;
    public static final double EDGE_TRIM_FRACTION = 0.15;

    // ---------------- orientation, must match the calibration ----------------

    public static boolean FLIP_HORIZONTAL = true;
    public static boolean ROTATE_90_CCW = true;

    // ---------------- colour ----------------

    public static Scalar LOWER_THRESHOLD = new Scalar(122, 33, 0);
    public static Scalar UPPER_THRESHOLD = new Scalar(170, 255, 255);
    public static int HUE_SHIFT = -10;

    // ---------------- mounting ----------------

    public static double CAMERA_YAW = 0;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_X_OFFSET = 0;
    public static double CAMERA_Y_OFFSET = 0;

    // ---------------- tuning ----------------

    public static final int MIN_BLOB_AREA = 20;
    public static final double MARKER_THRESHOLD_FRACTION = 0.15;
    public static final double NMS_KERNEL_FRACTION = 0.75;
    public static final double MIN_PEAK_DISTANCE_FRACTION = 0.80;
    public static final double MIN_AREA_FRACTION = 0.30;
    public static final double PEAK_FLOOR_FRACTION = 0.15;
    public static final double SMALL_BLOB_RADIUS = 10.0;
    public static final int UPSCALE_FACTOR = 3;
    public static final double NECK_SEARCH_START = 0.15;
    public static final double NECK_SEARCH_END = 0.85;
    public static final double NECK_RATIO_MAX = 1.0;
    public static final int MAX_SPLIT_DEPTH = 6;

    public enum DebugView { FINAL, MASK, DISTANCE, SPLIT }

    public static DebugView debugView = DebugView.FINAL;

    private static final Scalar[] SPLIT_COLORS = {
            new Scalar(255, 100, 0), new Scalar(0, 255, 255),
            new Scalar(0, 200, 0), new Scalar(255, 0, 255),
            new Scalar(0, 128, 255), new Scalar(255, 255, 0),
            new Scalar(128, 0, 255), new Scalar(0, 0, 255)
    };

    // ---------------- result types ----------------

    public static class Ball {
        public Point[] contour;
        public double centerX;
        public double centerY;
        public double radius;
        public double area;
        public double circleArea;
        public int x;
        public int y;
        public int width;
        public int height;

        public boolean hasPosition;
        public double xReal;
        public double yReal;
        public double zReal;
        public double range;
        public double xRobot;
        public double yRobot;
        public double zRobot;
    }

    private static class Peak {
        int x;
        int y;
        double strength;
    }

    private static class CenterLine {
        double x1, y1, x2, y2, length, unitX, unitY;
    }

    private static class Neck {
        double x, y;
        int positive, negative, width;
    }

    private static class Viz {
        Mat distance;
        List<Mat> finalMasks = new ArrayList<>();

        Viz(int rows, int cols) {
            distance = Mat.zeros(rows, cols, CvType.CV_32F);
        }

        void release() {
            distance.release();
            for (Mat m : finalMasks) {
                m.release();
            }
            finalMasks.clear();
        }
    }

    // ---------------- state ----------------

    private final Mat camK = new Mat(3, 3, CvType.CV_64F);
    private final MatOfDouble camD = new MatOfDouble();

    private final Object resultLock = new Object();
    private List<Ball> latestBalls = new ArrayList<>();
    private double latestScaleRadius = 0;

    private Mat work = new Mat();
    private Mat hsvBuffer = new Mat();
    private Mat blurBuffer = new Mat();
    private Mat rotateBuffer = new Mat();
    private Mat flipBuffer = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        camK.put(0, 0, CAL_FX, 0.0, CAL_CX, 0.0, CAL_FY, CAL_CY, 0.0, 0.0, 1.0);
        camD.fromArray(CAL_DIST);
    }

    public List<Ball> getBalls() {
        synchronized (resultLock) {
            return new ArrayList<>(latestBalls);
        }
    }

    public double getScaleRadius() {
        synchronized (resultLock) {
            return latestScaleRadius;
        }
    }

    // ---------------- frame pipeline ----------------

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        orientForward(input, work);

        if (HUE_SHIFT != 0) {
            shiftHue(work, HUE_SHIFT);
        }

        Mat roughMask = makeRoughMask(work);
        double scaleRadius = measureScale(roughMask);
        roughMask.release();
        if (scaleRadius <= 0) {
            scaleRadius = 10.0;
        }

        Mat cleanMask = preprocess(work, scaleRadius);

        double minimumArea = Math.max(minBallArea(scaleRadius), MIN_BLOB_AREA);
        Viz viz = new Viz(work.rows(), work.cols());
        List<Ball> balls = new ArrayList<>();

        List<Mat> components = connectedComponentMasks(cleanMask, minimumArea);
        for (Mat component : components) {
            Mat componentDistance = new Mat();
            Imgproc.distanceTransform(component, componentDistance, Imgproc.DIST_L2, 5);
            double componentRadius = Core.minMaxLoc(componentDistance).maxVal;
            componentDistance.release();
            if (componentRadius > 0) {
                balls.addAll(processComponentScaled(component, viz, componentRadius, minimumArea));
            }
            component.release();
        }

        for (Ball ball : balls) {
            computePosition(ball);
        }

        renderDebug(work, cleanMask, viz, balls, scaleRadius);

        cleanMask.release();
        viz.release();

        orientBack(work, input);

        synchronized (resultLock) {
            latestBalls = balls;
            latestScaleRadius = scaleRadius;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // overlays are drawn into the Mat, nothing to add here
    }

    private void orientForward(Mat input, Mat out) {
        if (FLIP_HORIZONTAL) {
            Core.flip(input, rotateBuffer, 1);
        } else {
            input.copyTo(rotateBuffer);
        }
        if (ROTATE_90_CCW) {
            Core.rotate(rotateBuffer, out, Core.ROTATE_90_COUNTERCLOCKWISE);
        } else {
            rotateBuffer.copyTo(out);
        }
    }

    private void orientBack(Mat processed, Mat input) {
        if (ROTATE_90_CCW) {
            Core.rotate(processed, rotateBuffer, Core.ROTATE_90_CLOCKWISE);
        } else {
            processed.copyTo(rotateBuffer);
        }
        if (FLIP_HORIZONTAL) {
            Core.flip(rotateBuffer, flipBuffer, 1);
            flipBuffer.copyTo(input);
        } else {
            rotateBuffer.copyTo(input);
        }
    }

    private void shiftHue(Mat rgb, int shift) {
        Imgproc.cvtColor(rgb, hsvBuffer, Imgproc.COLOR_RGB2HSV);
        int total = (int) (hsvBuffer.total() * hsvBuffer.channels());
        byte[] data = new byte[total];
        hsvBuffer.get(0, 0, data);
        for (int i = 0; i < total; i += 3) {
            int hue = (data[i] & 0xFF) + shift;
            hue = ((hue % 180) + 180) % 180;
            data[i] = (byte) hue;
        }
        hsvBuffer.put(0, 0, data);
        Imgproc.cvtColor(hsvBuffer, rgb, Imgproc.COLOR_HSV2RGB);
    }

    private Mat makeRoughMask(Mat rgb) {
        Imgproc.GaussianBlur(rgb, blurBuffer, new Size(5, 5), 0);
        Imgproc.cvtColor(blurBuffer, hsvBuffer, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(hsvBuffer, LOWER_THRESHOLD, UPPER_THRESHOLD, mask);
        return mask;
    }

    private double measureScale(Mat roughMask) {
        if (Core.countNonZero(roughMask) == 0) {
            return 0.0;
        }
        Mat distance = new Mat();
        Imgproc.distanceTransform(roughMask, distance, Imgproc.DIST_L2, 5);
        double max = Core.minMaxLoc(distance).maxVal;
        distance.release();
        return max;
    }

    private Mat preprocess(Mat rgb, double radius) {
        int blurSize = radius < 15 ? 3 : 5;
        Imgproc.GaussianBlur(rgb, blurBuffer, new Size(blurSize, blurSize), 0);
        Imgproc.cvtColor(blurBuffer, hsvBuffer, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsvBuffer, LOWER_THRESHOLD, UPPER_THRESHOLD, mask);

        int kernelSize = radius < 15 ? 3 : 5;
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelSize, kernelSize));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel, new Point(-1, -1), 2);
        kernel.release();

        double minimumArea = Math.max(minBallArea(radius), MIN_BLOB_AREA);

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int labelCount = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        Mat clean = Mat.zeros(mask.size(), CvType.CV_8UC1);
        Mat single = new Mat();
        for (int label = 1; label < labelCount; label++) {
            double area = stats.get(label, Imgproc.CC_STAT_AREA)[0];
            if (area > minimumArea) {
                Core.compare(labels, new Scalar(label), single, Core.CMP_EQ);
                Core.bitwise_or(clean, single, clean);
            }
        }
        single.release();
        labels.release();
        stats.release();
        centroids.release();
        mask.release();
        return clean;
    }

    // ---------------- component splitting ----------------

    private List<Mat> connectedComponentMasks(Mat binary, double minimumArea) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int labelCount = Imgproc.connectedComponentsWithStats(binary, labels, stats, centroids, 8, CvType.CV_32S);

        List<Mat> result = new ArrayList<>();
        for (int label = 1; label < labelCount; label++) {
            double area = stats.get(label, Imgproc.CC_STAT_AREA)[0];
            if (area <= minimumArea) {
                continue;
            }
            Mat component = new Mat();
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);
            result.add(component);
        }
        labels.release();
        stats.release();
        centroids.release();
        return result;
    }

    private List<Ball> processComponentScaled(Mat component, Viz viz, double radius, double minimumArea) {
        if (radius >= SMALL_BLOB_RADIUS) {
            return processMask(component, viz, 0, radius, minimumArea);
        }

        int factor = UPSCALE_FACTOR;
        Mat big = new Mat();
        Imgproc.resize(component, big, new Size(component.cols() * factor, component.rows() * factor),
                0, 0, Imgproc.INTER_CUBIC);
        Imgproc.threshold(big, big, 127, 255, Imgproc.THRESH_BINARY);

        Viz bigViz = new Viz(big.rows(), big.cols());
        List<Ball> bigBalls = processMask(big, bigViz, 0, radius * factor, minimumArea * factor * factor);

        Mat smallDistance = new Mat();
        Imgproc.resize(bigViz.distance, smallDistance, component.size());
        Core.divide(smallDistance, new Scalar(factor), smallDistance);
        Core.max(viz.distance, smallDistance, viz.distance);
        smallDistance.release();

        for (Mat bigFinal : bigViz.finalMasks) {
            Mat smallFinal = new Mat();
            Imgproc.resize(bigFinal, smallFinal, component.size(), 0, 0, Imgproc.INTER_NEAREST);
            viz.finalMasks.add(smallFinal);
        }
        bigViz.release();
        big.release();

        for (Ball ball : bigBalls) {
            scaleBall(ball, factor);
        }
        return bigBalls;
    }

    private void scaleBall(Ball ball, int factor) {
        ball.centerX /= factor;
        ball.centerY /= factor;
        ball.radius /= factor;
        ball.circleArea = Math.PI * ball.radius * ball.radius;
        ball.area /= (double) factor * factor;
        for (int i = 0; i < ball.contour.length; i++) {
            ball.contour[i] = new Point(ball.contour[i].x / factor, ball.contour[i].y / factor);
        }
        ball.x /= factor;
        ball.y /= factor;
        ball.width /= factor;
        ball.height /= factor;
    }

    private List<Ball> processMask(Mat mask, Viz viz, int depth, double radius, double minimumArea) {
        List<Ball> result = new ArrayList<>();
        if (depth > MAX_SPLIT_DEPTH) {
            return finishAsSingleBall(mask, viz, minimumArea);
        }
        if (Core.countNonZero(mask) <= minimumArea) {
            return result;
        }

        Mat distance = markerDistance(mask, radius);
        Core.max(viz.distance, distance, viz.distance);

        double maxDistance = Core.minMaxLoc(distance).maxVal;
        Mat scaled = new Mat();
        if (maxDistance == 0) {
            distance.copyTo(scaled);
        } else {
            Core.divide(distance, new Scalar(maxDistance), scaled);
        }
        distance.release();

        List<Peak> peaks = findPeaks(scaled, radius);
        scaled.release();

        List<Peak> selected = chooseTwoPeaks(peaks, Math.max(radius * MIN_PEAK_DISTANCE_FRACTION, 2.0));
        if (selected.size() != 2) {
            return finishAsSingleBall(mask, viz, minimumArea);
        }

        CenterLine line = makeCenterLine(selected.get(0), selected.get(1));
        if (line == null) {
            return finishAsSingleBall(mask, viz, minimumArea);
        }

        int width = mask.cols();
        int height = mask.rows();
        byte[] maskData = new byte[width * height];
        mask.get(0, 0, maskData);

        Neck neck = findNeck(maskData, width, height, line);
        if (neck == null) {
            return finishAsSingleBall(mask, viz, minimumArea);
        }

        Mat split1 = Mat.zeros(mask.size(), CvType.CV_8UC1);
        Mat split2 = Mat.zeros(mask.size(), CvType.CV_8UC1);
        splitComponent(maskData, width, height, line, neck, split1, split2);

        if (Core.countNonZero(split1) <= minimumArea || Core.countNonZero(split2) <= minimumArea) {
            split1.release();
            split2.release();
            return finishAsSingleBall(mask, viz, minimumArea);
        }

        Mat[] pieces = {split1, split2};
        for (Mat piece : pieces) {
            List<Mat> parts = connectedComponentMasks(piece, minimumArea);
            for (Mat part : parts) {
                result.addAll(processMask(part, viz, depth + 1, radius, minimumArea));
                part.release();
            }
            piece.release();
        }

        if (result.isEmpty()) {
            return finishAsSingleBall(mask, viz, minimumArea);
        }
        return result;
    }

    private Mat markerDistance(Mat binary, double radius) {
        Mat distance = new Mat();
        Imgproc.distanceTransform(binary, distance, Imgproc.DIST_L2, 5);

        Mat markerMask = new Mat();
        Imgproc.threshold(distance, markerMask, radius * MARKER_THRESHOLD_FRACTION, 255, Imgproc.THRESH_BINARY);
        markerMask.convertTo(markerMask, CvType.CV_8UC1);
        distance.release();

        if (Core.countNonZero(markerMask) == 0) {
            binary.copyTo(markerMask);
        }

        Mat result = new Mat();
        Imgproc.distanceTransform(markerMask, result, Imgproc.DIST_L2, 5);
        markerMask.release();
        return result;
    }

    private List<Peak> findPeaks(Mat scaled, double radius) {
        int size = (int) (radius * NMS_KERNEL_FRACTION);
        size = Math.max(3, Math.min(61, size));
        if (size % 2 == 0) {
            size++;
        }
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size));

        Mat dilated = new Mat();
        Imgproc.dilate(scaled, dilated, kernel);
        kernel.release();

        Mat isPeak = new Mat();
        Core.compare(scaled, dilated, isPeak, Core.CMP_EQ);
        dilated.release();

        Mat isHigh = new Mat();
        Core.compare(scaled, new Scalar(PEAK_FLOOR_FRACTION), isHigh, Core.CMP_GT);

        Mat peakMask = new Mat();
        Core.bitwise_and(isPeak, isHigh, peakMask);
        isPeak.release();
        isHigh.release();

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int labelCount = Imgproc.connectedComponentsWithStats(peakMask, labels, stats, centroids, 8, CvType.CV_32S);
        peakMask.release();

        List<Peak> peaks = new ArrayList<>();
        for (int label = 1; label < labelCount; label++) {
            int x = (int) Math.round(centroids.get(label, 0)[0]);
            int y = (int) Math.round(centroids.get(label, 1)[0]);
            if (x < 0 || x >= scaled.cols() || y < 0 || y >= scaled.rows()) {
                continue;
            }
            Peak peak = new Peak();
            peak.x = x;
            peak.y = y;
            peak.strength = scaled.get(y, x)[0];
            peaks.add(peak);
        }
        labels.release();
        stats.release();
        centroids.release();

        Collections.sort(peaks, (a, b) -> Double.compare(b.strength, a.strength));
        return peaks;
    }

    private List<Peak> chooseTwoPeaks(List<Peak> peaks, double minimumDistance) {
        List<Peak> result = new ArrayList<>();
        if (peaks.size() < 2) {
            return result;
        }
        Peak first = peaks.get(0);
        for (int i = 1; i < peaks.size(); i++) {
            Peak second = peaks.get(i);
            double dx = second.x - first.x;
            double dy = second.y - first.y;
            if (Math.sqrt(dx * dx + dy * dy) >= minimumDistance) {
                result.add(first);
                result.add(second);
                return result;
            }
        }
        return result;
    }

    private CenterLine makeCenterLine(Peak a, Peak b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double length = Math.sqrt(dx * dx + dy * dy);
        if (length == 0) {
            return null;
        }
        CenterLine line = new CenterLine();
        line.x1 = a.x;
        line.y1 = a.y;
        line.x2 = b.x;
        line.y2 = b.y;
        line.length = length;
        line.unitX = dx / length;
        line.unitY = dy / length;
        return line;
    }

    private static boolean insideMask(byte[] mask, int width, int height, double x, double y) {
        int px = (int) Math.round(x);
        int py = (int) Math.round(y);
        if (px < 0 || px >= width || py < 0 || py >= height) {
            return false;
        }
        return mask[py * width + px] != 0;
    }

    private static int edgeDistance(byte[] mask, int width, int height,
                                    double x, double y, double nx, double ny, int direction) {
        int limit = Math.max(width, height);
        int last = 0;
        for (int d = 1; d < limit; d++) {
            double tx = x + nx * d * direction;
            double ty = y + ny * d * direction;
            if (!insideMask(mask, width, height, tx, ty)) {
                break;
            }
            last = d;
        }
        return last;
    }

    private Neck findNeck(byte[] mask, int width, int height, CenterLine line) {
        double nx = -line.unitY;
        double ny = line.unitX;
        int start = (int) (line.length * NECK_SEARCH_START);
        int end = (int) (line.length * NECK_SEARCH_END);

        List<Neck> measurements = new ArrayList<>();
        for (int d = start; d <= end; d++) {
            double x = line.x1 + line.unitX * d;
            double y = line.y1 + line.unitY * d;
            if (!insideMask(mask, width, height, x, y)) {
                continue;
            }
            Neck measurement = new Neck();
            measurement.x = x;
            measurement.y = y;
            measurement.positive = edgeDistance(mask, width, height, x, y, nx, ny, 1);
            measurement.negative = edgeDistance(mask, width, height, x, y, nx, ny, -1);
            measurement.width = measurement.positive + measurement.negative + 1;
            measurements.add(measurement);
        }

        if (measurements.size() < 3) {
            return null;
        }

        Neck narrowest = measurements.get(0);
        int widest = 0;
        for (Neck measurement : measurements) {
            if (measurement.width < narrowest.width) {
                narrowest = measurement;
            }
            if (measurement.width > widest) {
                widest = measurement.width;
            }
        }
        if (widest == 0) {
            return null;
        }
        if ((double) narrowest.width / widest > NECK_RATIO_MAX) {
            return null;
        }
        return narrowest;
    }

    private void splitComponent(byte[] mask, int width, int height,
                                CenterLine line, Neck neck, Mat out1, Mat out2) {
        byte[] a = new byte[width * height];
        byte[] b = new byte[width * height];
        for (int y = 0; y < height; y++) {
            int row = y * width;
            double dyTerm = (y - neck.y) * line.unitY;
            for (int x = 0; x < width; x++) {
                if (mask[row + x] == 0) {
                    continue;
                }
                double side = (x - neck.x) * line.unitX + dyTerm;
                if (side <= 0) {
                    a[row + x] = (byte) 255;
                } else {
                    b[row + x] = (byte) 255;
                }
            }
        }
        out1.put(0, 0, a);
        out2.put(0, 0, b);
    }

    // ---------------- ball extraction ----------------

    private List<Ball> finishAsSingleBall(Mat mask, Viz viz, double minimumArea) {
        List<Ball> result = new ArrayList<>();
        Ball ball = ballFromMask(mask, minimumArea);
        if (ball == null) {
            return result;
        }
        Mat copy = new Mat();
        mask.copyTo(copy);
        viz.finalMasks.add(copy);
        result.add(ball);
        return result;
    }

    private Ball ballFromMask(Mat mask, double minimumArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();
        if (contours.isEmpty()) {
            return null;
        }

        MatOfPoint largest = contours.get(0);
        double largestArea = Imgproc.contourArea(largest);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestArea) {
                largestArea = area;
                largest = contour;
            }
        }
        if (largestArea <= minimumArea) {
            for (MatOfPoint contour : contours) {
                contour.release();
            }
            return null;
        }

        Point[] points = largest.toArray();
        Rect box = Imgproc.boundingRect(largest);

        double[] circle = fitCircle(points);
        double centerX;
        double centerY;
        double radius;
        if (circle == null) {
            MatOfPoint2f asFloat = new MatOfPoint2f(points);
            Point enclosingCenter = new Point();
            float[] enclosingRadius = new float[1];
            Imgproc.minEnclosingCircle(asFloat, enclosingCenter, enclosingRadius);
            asFloat.release();
            centerX = enclosingCenter.x;
            centerY = enclosingCenter.y;
            radius = enclosingRadius[0];
        } else {
            centerX = circle[0];
            centerY = circle[1];
            radius = circle[2];
        }

        for (MatOfPoint contour : contours) {
            contour.release();
        }

        Ball ball = new Ball();
        ball.contour = points;
        ball.area = largestArea;
        ball.centerX = centerX;
        ball.centerY = centerY;
        ball.radius = radius;
        ball.circleArea = Math.PI * radius * radius;
        ball.x = box.x;
        ball.y = box.y;
        ball.width = box.width;
        ball.height = box.height;
        return ball;
    }

    private static double[] fitCircle(Point[] points) {
        if (points.length < 3) {
            return null;
        }
        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxb = 0, syb = 0, sb = 0;
        for (Point p : points) {
            double b = p.x * p.x + p.y * p.y;
            sxx += p.x * p.x;
            sxy += p.x * p.y;
            syy += p.y * p.y;
            sx += p.x;
            sy += p.y;
            sxb += p.x * b;
            syb += p.y * b;
            sb += b;
        }
        double[][] a = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, points.length}};
        double[] rhs = {sxb, syb, sb};
        double[] solution = solve3(a, rhs);
        if (solution == null) {
            return null;
        }
        double cx = solution[0] / 2;
        double cy = solution[1] / 2;
        double inside = solution[2] + cx * cx + cy * cy;
        if (inside <= 0) {
            return null;
        }
        return new double[]{cx, cy, Math.sqrt(inside)};
    }

    private static double[] solve3(double[][] a, double[] b) {
        double[][] m = new double[3][4];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(a[i], 0, m[i], 0, 3);
            m[i][3] = b[i];
        }
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                    pivot = row;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                return null;
            }
            double[] swap = m[col];
            m[col] = m[pivot];
            m[pivot] = swap;
            for (int row = 0; row < 3; row++) {
                if (row == col) {
                    continue;
                }
                double factor = m[row][col] / m[col][col];
                for (int k = col; k < 4; k++) {
                    m[row][k] -= factor * m[col][k];
                }
            }
        }
        return new double[]{m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]};
    }

    // ---------------- ranging ----------------

    private void computePosition(Ball ball) {
        ball.hasPosition = false;
        if (ball.contour.length < 5) {
            return;
        }

        List<Point> kept = new ArrayList<>();
        for (Point p : ball.contour) {
            double offset = Math.hypot(p.x - ball.centerX, p.y - ball.centerY);
            if (Math.abs(offset - ball.radius) <= ball.radius * EDGE_TRIM_FRACTION) {
                kept.add(p);
            }
        }
        Point[] use = kept.size() >= 8 ? kept.toArray(new Point[0]) : ball.contour;

        MatOfPoint2f source = new MatOfPoint2f(use);
        MatOfPoint2f undistorted = new MatOfPoint2f();
        Calib3d.undistortPoints(source, undistorted, camK, camD);
        Point[] rays = undistorted.toArray();
        source.release();
        undistorted.release();

        MatOfPoint2f centerSource = new MatOfPoint2f(new Point(ball.centerX, ball.centerY));
        MatOfPoint2f centerUndistorted = new MatOfPoint2f();
        Calib3d.undistortPoints(centerSource, centerUndistorted, camK, camD);
        Point centerRay = centerUndistorted.toArray()[0];
        centerSource.release();
        centerUndistorted.release();

        double axisNorm = Math.sqrt(centerRay.x * centerRay.x + centerRay.y * centerRay.y + 1.0);
        double axisX = centerRay.x / axisNorm;
        double axisY = centerRay.y / axisNorm;
        double axisZ = 1.0 / axisNorm;

        double[] angles = new double[rays.length];
        for (int i = 0; i < rays.length; i++) {
            double norm = Math.sqrt(rays[i].x * rays[i].x + rays[i].y * rays[i].y + 1.0);
            double dot = (rays[i].x * axisX + rays[i].y * axisY + axisZ) / norm;
            angles[i] = Math.acos(Math.max(-1.0, Math.min(1.0, dot)));
        }
        Arrays.sort(angles);
        double alpha = angles.length % 2 == 1
                ? angles[angles.length / 2]
                : 0.5 * (angles[angles.length / 2 - 1] + angles[angles.length / 2]);

        if (alpha <= 1e-6 || alpha >= Math.PI / 2) {
            return;
        }

        double range = BALL_RADIUS_CM / Math.sin(alpha);
        ball.range = range;
        ball.xReal = axisX * range;
        ball.yReal = -axisY * range;
        ball.zReal = axisZ * range;

        double yaw = Math.toRadians(CAMERA_YAW);
        double pitch = Math.toRadians(CAMERA_PITCH);
        ball.xRobot = ball.xReal * Math.cos(yaw) - ball.zReal * Math.sin(yaw) + CAMERA_X_OFFSET;
        double zYaw = ball.xReal * Math.sin(yaw) + ball.zReal * Math.cos(yaw);
        ball.yRobot = ball.yReal * Math.cos(pitch) - zYaw * Math.sin(pitch) + CAMERA_Y_OFFSET;
        ball.zRobot = ball.yReal * Math.sin(pitch) + zYaw * Math.cos(pitch);
        ball.hasPosition = true;
    }

    private static double minBallArea(double radius) {
        return MIN_AREA_FRACTION * Math.PI * radius * radius;
    }

    // ---------------- overlay ----------------

    private void renderDebug(Mat frame, Mat cleanMask, Viz viz, List<Ball> balls, double scaleRadius) {
        if (debugView == DebugView.MASK) {
            Imgproc.cvtColor(cleanMask, frame, Imgproc.COLOR_GRAY2RGB);
        } else if (debugView == DebugView.DISTANCE) {
            double max = Core.minMaxLoc(viz.distance).maxVal;
            Mat normalized;
            if (max > 0) {
                normalized = new Mat();
                viz.distance.convertTo(normalized, CvType.CV_8UC1, 255.0 / max);
            } else {
                normalized = Mat.zeros(viz.distance.size(), CvType.CV_8UC1);
            }
            Imgproc.applyColorMap(normalized, frame, Imgproc.COLORMAP_JET);
            normalized.release();
        } else if (debugView == DebugView.SPLIT) {
            frame.setTo(new Scalar(0, 0, 0));
            for (int i = 0; i < viz.finalMasks.size(); i++) {
                frame.setTo(SPLIT_COLORS[i % SPLIT_COLORS.length], viz.finalMasks.get(i));
            }
        }

        Scalar green = new Scalar(0, 255, 0);
        Scalar red = new Scalar(255, 0, 0);

        for (int i = 0; i < balls.size(); i++) {
            Ball ball = balls.get(i);
            Point center = new Point(ball.centerX, ball.centerY);
            Imgproc.circle(frame, center, (int) ball.radius, green, 2);
            Imgproc.circle(frame, center, 2, red, -1);

            List<String> lines = new ArrayList<>();
            lines.add(String.format("Ball %d", i + 1));
            lines.add(String.format("px:(%d,%d) R:%.1f", (int) ball.centerX, (int) ball.centerY, ball.radius));
            lines.add(String.format("area:%.0fpx circ:%.0fpx", ball.area, ball.circleArea));
            if (ball.hasPosition) {
                lines.add(String.format("X:%.1f Y:%.1f Z:%.1fcm", ball.xReal, ball.yReal, ball.zReal));
                lines.add(String.format("range:%.1fcm", ball.range));
            } else {
                lines.add("no range");
            }

            int lineHeight = 14;
            int top = Math.max(14, ball.y - lineHeight * lines.size());
            for (int j = 0; j < lines.size(); j++) {
                Imgproc.putText(frame, lines.get(j), new Point(ball.x, top + j * lineHeight),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            }
        }

        Imgproc.putText(frame, String.format("r=%.1f balls=%d", scaleRadius, balls.size()),
                new Point(10, 24), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
    }
}