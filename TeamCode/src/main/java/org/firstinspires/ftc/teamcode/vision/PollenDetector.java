package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PollenDetector implements VisionProcessor {

    public static double REF_DISTANCE = 30.0;
    public static double REF_RADIUS_PX = 22.57;
    public static double REF_DISTANCE_FOV = 15.24;
    public static double FOV_WIDTH_CM = 40.64;

    public static Scalar LOWER_YELLOW = new Scalar(18, 65, 160);
    public static Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

    public static double CAMERA_YAW = 0;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_X_OFFSET = 0;
    public static double CAMERA_Y_OFFSET = 0;

    public static int MIN_CONTOUR_SIZE = 40;
    public static int MIN_BALL_RADIUS_PX = 5;

    public static boolean FILL_HOLES = false;

    public static int PEAK_COMPARE_RADIUS_PX = 3;
    public static double PEAK_SMOOTH_SIGMA = 1.0;
    public static double MIN_SEED_DEPTH = 5.0;
    public static double SEED_MERGE_FRACTION = 0.7;

    public static double MIN_PEAK_SEPARATION_FRACTION = 0.6;
    public static double CUT_CLEARANCE_FRACTION = 0.30;
    public static double CUT_HALF_LENGTH_FRACTION = 0.8;
    public static double CUT_THICKNESS_FRACTION = 0.20;

    public static int MAX_SPLIT_DEPTH = 8;

    public static int PROCESS_WIDTH = 320;

    private static final double K = REF_DISTANCE * REF_RADIUS_PX;

    private Mat closeKernel;
    private Mat openKernel;
    private Mat compareDisc;

    private int frameWidth;
    private int frameHeight;
    private double focalLengthPx;

    private volatile List<Ball> latestBalls = new ArrayList<>();

    public static class Ball {
        public final double x;
        public final double y;
        public final double radius;
        public final double areaPx;
        public final double xReal;
        public final double yReal;
        public final double zReal;
        public final double xAngle;
        public final double yAngle;
        public final double zAngle;

        Ball(double x, double y, double radius, double areaPx, double[] spatial) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.areaPx = areaPx;
            this.xReal = spatial[0];
            this.yReal = spatial[1];
            this.zReal = spatial[2];
            this.xAngle = spatial[3];
            this.yAngle = spatial[4];
            this.zAngle = spatial[5];
        }
    }

    private static class Seed {
        final int x;
        final int y;
        final double strength;

        Seed(int x, int y, double strength) {
            this.x = x;
            this.y = y;
            this.strength = strength;
        }
    }

    private static class Region {
        final Point center;
        final double radius;
        final double area;

        Region(Point center, double radius, double area) {
            this.center = center;
            this.radius = radius;
            this.area = area;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;
        focalLengthPx = width * (REF_DISTANCE_FOV / FOV_WIDTH_CM);

        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        int discSize = 2 * Math.max(1, PEAK_COMPARE_RADIUS_PX) + 1;
        compareDisc = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE, new Size(discSize, discSize));
    }

    public List<Ball> getBalls() {
        return latestBalls;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat working = new Mat();
        double scale = 1.0;

        if (PROCESS_WIDTH > 0 && frame.cols() > PROCESS_WIDTH) {
            scale = (double) PROCESS_WIDTH / frame.cols();
            Imgproc.resize(frame, working,
                    new Size(Math.round(frame.cols() * scale), Math.round(frame.rows() * scale)),
                    0, 0, Imgproc.INTER_AREA);
        } else {
            frame.copyTo(working);
        }

        Mat mask = preprocess(working);
        List<Region> regions = new ArrayList<>();
        for (Mat component : splitIntoComponents(mask)) {
            processMask(component, 0, regions);
            component.release();
        }

        List<Ball> balls = new ArrayList<>();
        double inverse = 1.0 / scale;
        double centerX = frameWidth / 2.0;
        double centerY = frameHeight / 2.0;
        for (Region region : regions) {
            double fx = region.center.x * inverse;
            double fy = region.center.y * inverse;
            double fr = region.radius * inverse;
            balls.add(new Ball(fx, fy, fr, region.area * inverse * inverse,
                    spatialCoords(fx, fy, fr, centerX, centerY, focalLengthPx)));
        }
        latestBalls = balls;

        mask.release();
        working.release();
        return null;
    }

    private Mat preprocess(Mat rgbFrame) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(rgbFrame, blurred, new Size(5, 5), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        blurred.release();

        Mat mask = new Mat();
        Core.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW, mask);
        hsv.release();

        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, closeKernel, new Point(-1, -1), 2);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, openKernel, new Point(-1, -1), 1);

        if (FILL_HOLES) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            hierarchy.release();
            Mat filled = Mat.zeros(mask.size(), CvType.CV_8UC1);
            Imgproc.drawContours(filled, contours, -1, new Scalar(255), -1);
            for (MatOfPoint contour : contours) contour.release();
            mask.release();
            mask = filled;
        }
        return mask;
    }

    private List<Mat> splitIntoComponents(Mat mask) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        List<Mat> components = new ArrayList<>();
        for (int label = 1; label < count; label++) {
            if (stats.get(label, Imgproc.CC_STAT_AREA)[0] <= MIN_CONTOUR_SIZE) continue;
            Mat component = new Mat();
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);
            components.add(component);
        }

        labels.release();
        stats.release();
        centroids.release();
        return components;
    }

    private List<Seed> findPeaks(Mat mask) {
        Mat distance = new Mat();
        Imgproc.distanceTransform(mask, distance, Imgproc.DIST_L2, 5);

        Mat compared = new Mat();
        if (PEAK_SMOOTH_SIGMA > 0) {
            Imgproc.GaussianBlur(distance, compared, new Size(0, 0), PEAK_SMOOTH_SIGMA);
        } else {
            distance.copyTo(compared);
        }

        Mat discMax = new Mat();
        Imgproc.dilate(compared, discMax, compareDisc);

        Mat isPeak = new Mat();
        Core.compare(compared, discMax, isPeak, Core.CMP_GE);

        Mat deepEnough = new Mat();
        Core.compare(distance, new Scalar(MIN_SEED_DEPTH), deepEnough, Core.CMP_GE);

        Mat peakMask = new Mat();
        Core.bitwise_and(isPeak, deepEnough, peakMask);

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(peakMask, labels, stats, centroids, 8, CvType.CV_32S);

        List<Seed> seeds = new ArrayList<>();
        for (int label = 1; label < count; label++) {
            int x = (int) Math.round(centroids.get(label, 0)[0]);
            int y = (int) Math.round(centroids.get(label, 1)[0]);
            if (x < 0 || x >= distance.cols() || y < 0 || y >= distance.rows()) continue;
            double value = distance.get(y, x)[0];
            if (value < MIN_SEED_DEPTH) continue;
            seeds.add(new Seed(x, y, value));
        }

        Collections.sort(seeds, new Comparator<Seed>() {
            @Override
            public int compare(Seed a, Seed b) {
                return Double.compare(b.strength, a.strength);
            }
        });

        distance.release();
        compared.release();
        discMax.release();
        isPeak.release();
        deepEnough.release();
        peakMask.release();
        labels.release();
        stats.release();
        centroids.release();

        return mergeCloseSeeds(seeds);
    }

    private List<Seed> mergeCloseSeeds(List<Seed> seeds) {
        List<Seed> kept = new ArrayList<>();
        for (Seed seed : seeds) {
            boolean tooClose = false;
            for (Seed other : kept) {
                double separation = Math.hypot(seed.x - other.x, seed.y - other.y);
                double limit = Math.max(MIN_BALL_RADIUS_PX,
                        SEED_MERGE_FRACTION * Math.min(seed.strength, other.strength));
                if (separation < limit) {
                    tooClose = true;
                    break;
                }
            }
            if (!tooClose) kept.add(seed);
        }
        return kept;
    }

    private boolean pairSeparationOk(Seed a, Seed b, double separation) {
        double smaller = Math.min(a.strength, b.strength);
        return separation >= Math.max(MIN_BALL_RADIUS_PX, MIN_PEAK_SEPARATION_FRACTION * smaller);
    }

    private double distanceToCutLine(Seed peak, Seed a, Seed b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double length = Math.hypot(dx, dy);
        if (length == 0) return Double.MAX_VALUE;
        double midX = (a.x + b.x) / 2.0;
        double midY = (a.y + b.y) / 2.0;
        return Math.abs((peak.x - midX) * dx + (peak.y - midY) * dy) / length;
    }

    private boolean cutIsClear(Seed a, Seed b, List<Seed> peaks) {
        for (Seed other : peaks) {
            if (other == a || other == b) continue;
            double clearance = Math.max(2.0, CUT_CLEARANCE_FRACTION * other.strength);
            if (distanceToCutLine(other, a, b) < clearance) return false;
        }
        return true;
    }

    private Seed[] chooseCutPair(List<Seed> peaks) {
        List<double[]> candidates = new ArrayList<>();
        for (int i = 0; i < peaks.size(); i++) {
            for (int j = i + 1; j < peaks.size(); j++) {
                double separation = Math.hypot(peaks.get(i).x - peaks.get(j).x,
                        peaks.get(i).y - peaks.get(j).y);
                if (!pairSeparationOk(peaks.get(i), peaks.get(j), separation)) continue;
                candidates.add(new double[]{separation, i, j});
            }
        }
        Collections.sort(candidates, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });
        for (double[] candidate : candidates) {
            Seed a = peaks.get((int) candidate[1]);
            Seed b = peaks.get((int) candidate[2]);
            if (cutIsClear(a, b, peaks)) return new Seed[]{a, b};
        }
        return null;
    }

    private Mat[] splitAtMidpoint(Mat mask, Seed a, Seed b, double radiusHint) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double length = Math.hypot(dx, dy);
        if (length == 0) return null;

        double midX = (a.x + b.x) / 2.0;
        double midY = (a.y + b.y) / 2.0;
        double normalX = -dy / length;
        double normalY = dx / length;
        int thickness = Math.max(3, (int) (CUT_THICKNESS_FRACTION * radiusHint));

        double boundedHalf = Math.max(10.0, CUT_HALF_LENGTH_FRACTION * radiusHint);
        double fullHalf = Math.hypot(mask.cols(), mask.rows());

        for (double half : new double[]{boundedHalf, fullHalf}) {
            Mat carved = mask.clone();
            Point p1 = new Point(midX + normalX * half, midY + normalY * half);
            Point p2 = new Point(midX - normalX * half, midY - normalY * half);
            Imgproc.line(carved, p1, p2, new Scalar(0), thickness);

            Mat labels = new Mat();
            Mat stats = new Mat();
            Mat centroids = new Mat();
            Imgproc.connectedComponentsWithStats(carved, labels, stats, centroids, 8, CvType.CV_32S);

            int label1 = (int) labels.get(a.y, a.x)[0];
            int label2 = (int) labels.get(b.y, b.x)[0];

            Mat[] result = null;
            if (label1 != 0 && label2 != 0 && label1 != label2) {
                Mat first = new Mat();
                Mat second = new Mat();
                Core.compare(labels, new Scalar(label1), first, Core.CMP_EQ);
                Core.compare(labels, new Scalar(label2), second, Core.CMP_EQ);
                result = new Mat[]{first, second};
            }

            carved.release();
            labels.release();
            stats.release();
            centroids.release();
            if (result != null) return result;
        }
        return null;
    }

    private MatOfPoint largestContour(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();
        if (contours.isEmpty()) return null;

        MatOfPoint best = contours.get(0);
        double bestArea = Imgproc.contourArea(best);
        for (int i = 1; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > bestArea) {
                best.release();
                best = contours.get(i);
                bestArea = area;
            } else {
                contours.get(i).release();
            }
        }
        return best;
    }

    private Region measureRegion(Mat mask) {
        MatOfPoint contour = largestContour(mask);
        if (contour == null) return null;

        double area = Imgproc.contourArea(contour);
        contour.release();
        if (area <= MIN_CONTOUR_SIZE) return null;

        Mat distance = new Mat();
        Imgproc.distanceTransform(mask, distance, Imgproc.DIST_L2, 5);
        Core.MinMaxLocResult extreme = Core.minMaxLoc(distance);
        double peakRadius = extreme.maxVal;
        if (peakRadius < MIN_BALL_RADIUS_PX) {
            distance.release();
            return null;
        }

        Mat atMax = new Mat();
        Core.compare(distance, new Scalar(peakRadius), atMax, Core.CMP_EQ);
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(atMax, labels, stats, centroids, 8, CvType.CV_32S);

        Point center = null;
        if (count > 1) {
            center = new Point(centroids.get(1, 0)[0], centroids.get(1, 1)[0]);
        }

        atMax.release();
        labels.release();
        stats.release();
        centroids.release();
        distance.release();
        if (center == null) return null;

        Mat houghInput = new Mat();
        Imgproc.GaussianBlur(mask, houghInput, new Size(5, 5), 1.0);
        Mat circles = new Mat();
        Imgproc.HoughCircles(houghInput, circles, Imgproc.HOUGH_GRADIENT,
                1.2, 10, 100, 15, MIN_BALL_RADIUS_PX, 100);
        houghInput.release();

        double radius = -1;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i < circles.cols(); i++) {
            double[] circle = circles.get(0, i);
            if (circle == null) continue;
            double separation = Math.hypot(circle[0] - center.x, circle[1] - center.y);
            if (separation < bestDistance) {
                bestDistance = separation;
                radius = circle[2];
            }
        }
        circles.release();

        if (radius < MIN_BALL_RADIUS_PX) return null;
        return new Region(center, radius, area);
    }

    private void finishAsSingle(Mat mask, List<Region> out) {
        Region region = measureRegion(mask);
        if (region != null) out.add(region);
    }

    private int countPeaksIn(Mat mask, List<Seed> peaks) {
        int total = 0;
        for (Seed peak : peaks) {
            if (mask.get(peak.y, peak.x)[0] > 0) total++;
        }
        return total;
    }

    private void processMask(Mat mask, int depth, List<Region> out) {
        List<Seed> peaks = findPeaks(mask);
        double rEst = peaks.isEmpty() ? 0 : peaks.get(0).strength;

        if (depth > MAX_SPLIT_DEPTH || rEst < MIN_BALL_RADIUS_PX || peaks.size() < 2) {
            finishAsSingle(mask, out);
            return;
        }

        Seed[] pair = chooseCutPair(peaks);
        if (pair == null) {
            finishAsSingle(mask, out);
            return;
        }

        Mat[] halves = splitAtMidpoint(mask, pair[0], pair[1], rEst);
        if (halves == null) {
            finishAsSingle(mask, out);
            return;
        }

        boolean usable = Core.countNonZero(halves[0]) > MIN_CONTOUR_SIZE
                && Core.countNonZero(halves[1]) > MIN_CONTOUR_SIZE
                && countPeaksIn(halves[0], peaks) > 0
                && countPeaksIn(halves[1], peaks) > 0;

        if (!usable) {
            halves[0].release();
            halves[1].release();
            finishAsSingle(mask, out);
            return;
        }

        int before = out.size();
        processMask(halves[0], depth + 1, out);
        processMask(halves[1], depth + 1, out);
        halves[0].release();
        halves[1].release();

        if (out.size() == before) finishAsSingle(mask, out);
    }

    private double[] spatialCoords(double circleX, double circleY, double radius,
                                   double centerX, double centerY, double focalLength) {
        if (radius <= 0) return new double[]{0, 0, 0, 0, 0, 0};

        double currentDistance = K / radius;
        double u = (circleX - centerX) / focalLength;
        double v = (centerY - circleY) / focalLength;
        double norm = Math.sqrt(1 + u * u + v * v);

        double zReal = currentDistance / norm;
        double xReal = (circleX - centerX) * zReal / focalLength;
        double yReal = (centerY - circleY) * zReal / focalLength;

        double yaw = Math.toRadians(CAMERA_YAW);
        double xAngle = (xReal * Math.cos(yaw) - zReal * Math.sin(yaw)) + CAMERA_X_OFFSET;
        double zYaw = xReal * Math.sin(yaw) + zReal * Math.cos(yaw);

        double pitch = Math.toRadians(CAMERA_PITCH);
        double yAngle = (yReal * Math.cos(pitch) - zYaw * Math.sin(pitch)) + CAMERA_Y_OFFSET;
        double zAngle = yReal * Math.sin(pitch) + zYaw * Math.cos(pitch);

        return new double[]{xReal, yReal, zReal, xAngle, yAngle, zAngle};
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        List<Ball> balls = latestBalls;
        if (balls.isEmpty()) return;

        Paint outline = new Paint();
        outline.setColor(Color.GREEN);
        outline.setStyle(Paint.Style.STROKE);
        outline.setStrokeWidth(scaleCanvasDensity * 3);

        Paint dot = new Paint();
        dot.setColor(Color.RED);
        dot.setStyle(Paint.Style.FILL);

        Paint label = new Paint();
        label.setColor(Color.GREEN);
        label.setTextSize(scaleCanvasDensity * 18);

        int index = 1;
        for (Ball ball : balls) {
            float cx = (float) ball.x * scaleBmpPxToCanvasPx;
            float cy = (float) ball.y * scaleBmpPxToCanvasPx;
            float r = (float) ball.radius * scaleBmpPxToCanvasPx;
            canvas.drawCircle(cx, cy, r, outline);
            canvas.drawCircle(cx, cy, scaleCanvasDensity * 3, dot);
            canvas.drawText(index + "  " + Math.round(ball.zReal) + "cm",
                    cx + r + 4, cy, label);
            index++;
        }
    }
}