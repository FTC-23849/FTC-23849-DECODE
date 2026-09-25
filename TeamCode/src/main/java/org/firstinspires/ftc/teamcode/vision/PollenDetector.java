package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class PollenDetector implements VisionProcessor, CameraStreamSource {

    private static final double[][] CAL_MATRIX = {
            {477.86395397, 0.0, 457.19465378},
            {0.0, 479.21955878, 641.11989783},
            {0.0, 0.0, 1.0}
    };
    private static final double[] CAL_DISTORTION =
            {0.00850100, -0.02061464, 0.00075404, -0.00012449, 0.0};
    private static final int CAL_WIDTH = 960;
    private static final int CAL_HEIGHT = 1280;

    public static boolean UNDISTORT = true;

    public static double BALL_RADIUS_CM = 3.55;
    public static double EDGE_TRIM_FRACTION = 0.15;

    public static double CAMERA_YAW = 0;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_X_OFFSET = 0;
    public static double CAMERA_Y_OFFSET = 0;

    public static int H_MIN = 18, H_MAX = 30;
    public static int S_MIN = 88, S_MAX = 255;
    public static int V_MIN = 184, V_MAX = 255;

    public static int H2_MIN = 18, H2_MAX = 30;
    public static int S2_MIN = 182, S2_MAX = 255;
    public static int V2_MIN = 161, V2_MAX = 193;

    public static int MIN_BLOB_AREA = 40;
    public static int RAW_OPEN_ITERATIONS = 10;

    public static int PEAK_RADIUS_PX = 6;
    public static double MIN_PEAK_DEPTH = 5.0;
    public static double MERGE_FRACTION = 1.0;

    public static int VOTE_TANGENT_STEP = 4;
    public static double VOTE_BLUR_FRACTION = 0.15;
    public static double VOTE_FLOOR_FRACTION = 0.15;

    public static double CONNECT_DISTANCE_FACTOR = 2.2;
    public static double CROSSBAR_SCALE = 1.0;
    public static int CUT_THICKNESS = 3;

    public static int PROCESS_WIDTH = 320;

    private Mat closeKernel;
    private Mat openKernel;
    private Mat erodeKernel;
    private Mat compareDisc;
    private int compareDiscRadius = -1;

    private Mat cameraMatrix;
    private Mat distortion;
    private Mat mapX;
    private Mat mapY;
    private Size mapSize;

    private int frameWidth;
    private int frameHeight;

    private volatile List<Ball> latestBalls = new ArrayList<>();

    private volatile int viewMode = 0;
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private static final String[] MODE_NAMES = {
            "0 camera", "1 mask", "2 centers", "3 connections", "4 cuts", "5 regions", "6 final"
    };
    private static final Scalar RED = new Scalar(255, 0, 0);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar CYAN = new Scalar(0, 255, 255);
    private static final Scalar MAGENTA = new Scalar(255, 0, 255);
    private static final Scalar YELLOW = new Scalar(255, 255, 0);
    private static final Scalar[] PALETTE = {
            new Scalar(255, 80, 80), new Scalar(80, 255, 80), new Scalar(80, 120, 255),
            new Scalar(255, 255, 80), new Scalar(255, 80, 255), new Scalar(80, 255, 255),
            new Scalar(255, 160, 40), new Scalar(170, 110, 255)
    };

    public static class Ball {
        public final double x;
        public final double y;
        public final double radius;
        public final double xCm;
        public final double yCm;
        public final double zCm;
        public final double rangeCm;
        public final double xRobot;
        public final double yRobot;
        public final double zRobot;
        public final boolean hasRange;

        Ball(double x, double y, double radius, double[] position) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.hasRange = position != null;
            if (position == null) {
                this.xCm = this.yCm = this.zCm = this.rangeCm = 0;
                this.xRobot = this.yRobot = this.zRobot = 0;
            } else {
                this.xCm = position[0];
                this.yCm = position[1];
                this.zCm = position[2];
                this.rangeCm = position[3];
                this.xRobot = position[4];
                this.yRobot = position[5];
                this.zRobot = position[6];
            }
        }
    }

    private static class Center {
        final int x;
        final int y;
        final double radius;
        final double votes;

        Center(int x, int y, double radius, double votes) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.votes = votes;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;

        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(3, 3));
        updateCompareDisc();
    }

    private void updateCompareDisc() {
        int radius = Math.max(1, PEAK_RADIUS_PX);
        if (compareDisc != null && radius == compareDiscRadius) return;
        if (compareDisc != null) compareDisc.release();
        compareDisc = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE, new Size(2 * radius + 1, 2 * radius + 1));
        compareDiscRadius = radius;
    }

    public List<Ball> getBalls() {
        return latestBalls;
    }

    public void setViewMode(int mode) {
        viewMode = Math.max(0, Math.min(MODE_NAMES.length - 1, mode));
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(consumer -> consumer.accept(lastFrame.get()));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        int mode = viewMode;
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

        Mat undistorted = undistortFrame(working);
        if (undistorted != working) {
            working.release();
        }

        Mat processed = new Mat();
        Mat raw = new Mat();
        buildMasks(undistorted, processed, raw);

        List<Center> centers = findCenters(processed);
        List<Point[]> links = new ArrayList<>();
        List<Point[]> cuts = new ArrayList<>();
        Mat cut = splitMaskWithLines(raw, centers, links, cuts);
        List<Ball> balls = measureRegions(cut, 1.0 / scale);
        latestBalls = balls;

        publishView(mode, frame, undistorted, processed, raw, cut, centers, links, cuts, balls, 1.0 / scale);

        processed.release();
        raw.release();
        cut.release();
        undistorted.release();
        return null;
    }

    private Mat undistortFrame(Mat frame) {
        int width = frame.cols();
        int height = frame.rows();
        Size size = new Size(width, height);

        Mat scaled = new Mat(3, 3, CvType.CV_64F);
        double scaleX = (double) width / CAL_WIDTH;
        double scaleY = (double) height / CAL_HEIGHT;
        scaled.put(0, 0, CAL_MATRIX[0][0] * scaleX, 0.0, CAL_MATRIX[0][2] * scaleX);
        scaled.put(1, 0, 0.0, CAL_MATRIX[1][1] * scaleY, CAL_MATRIX[1][2] * scaleY);
        scaled.put(2, 0, 0.0, 0.0, 1.0);

        Mat coefficients = new Mat(1, 5, CvType.CV_64F);
        coefficients.put(0, 0, CAL_DISTORTION);

        if (!UNDISTORT) {
            releaseIntrinsics();
            cameraMatrix = scaled;
            distortion = coefficients;
            return frame;
        }

        if (mapX == null || mapSize == null
                || mapSize.width != width || mapSize.height != height) {
            releaseMaps();
            Mat newMatrix = Calib3d.getOptimalNewCameraMatrix(scaled, coefficients, size, 0, size);
            mapX = new Mat();
            mapY = new Mat();
            Calib3d.initUndistortRectifyMap(scaled, coefficients, new Mat(), newMatrix,
                    size, CvType.CV_16SC2, mapX, mapY);
            mapSize = size;

            releaseIntrinsics();
            cameraMatrix = newMatrix;
            distortion = Mat.zeros(1, 5, CvType.CV_64F);
        }

        scaled.release();
        coefficients.release();

        Mat result = new Mat();
        Imgproc.remap(frame, result, mapX, mapY, Imgproc.INTER_LINEAR);
        return result;
    }

    private void releaseMaps() {
        if (mapX != null) mapX.release();
        if (mapY != null) mapY.release();
        mapX = null;
        mapY = null;
        mapSize = null;
    }

    private void releaseIntrinsics() {
        if (cameraMatrix != null) cameraMatrix.release();
        if (distortion != null) distortion.release();
        cameraMatrix = null;
        distortion = null;
    }

    private void buildMasks(Mat rgbFrame, Mat processedOut, Mat rawOut) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(rgbFrame, blurred, new Size(5, 5), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        blurred.release();

        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(H_MIN, S_MIN, V_MIN), new Scalar(H_MAX, S_MAX, V_MAX), mask);

        Mat second = new Mat();
        Core.inRange(hsv, new Scalar(H2_MIN, S2_MIN, V2_MIN), new Scalar(H2_MAX, S2_MAX, V2_MAX), second);
        Core.bitwise_or(mask, second, mask);
        second.release();
        hsv.release();

        Mat raw = new Mat();
        Imgproc.erode(mask, raw, erodeKernel, new Point(-1, -1), RAW_OPEN_ITERATIONS);
        Imgproc.dilate(raw, raw, erodeKernel, new Point(-1, -1), RAW_OPEN_ITERATIONS);

        Mat processed = new Mat();
        Imgproc.morphologyEx(mask, processed, Imgproc.MORPH_CLOSE, closeKernel,
                new Point(-1, -1), 2);
        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, openKernel,
                new Point(-1, -1), 1);
        mask.release();

        dropSmallBlobs(processed, processedOut);
        dropSmallBlobs(raw, rawOut);
        processed.release();
        raw.release();
    }

    private void dropSmallBlobs(Mat mask, Mat out) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        Mat cleaned = Mat.zeros(mask.size(), CvType.CV_8UC1);
        for (int label = 1; label < count; label++) {
            if (stats.get(label, Imgproc.CC_STAT_AREA)[0] <= MIN_BLOB_AREA) continue;
            Mat component = new Mat();
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);
            Core.bitwise_or(cleaned, component, cleaned);
            component.release();
        }
        cleaned.copyTo(out);

        cleaned.release();
        labels.release();
        stats.release();
        centroids.release();
    }

    private double estimateVoteRadius(Mat mask) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);

        Mat distance = new Mat();
        Imgproc.distanceTransform(mask, distance, Imgproc.DIST_L2, 5);

        List<Double> radii = new ArrayList<>();
        for (int label = 1; label < count; label++) {
            if (stats.get(label, Imgproc.CC_STAT_AREA)[0] <= MIN_BLOB_AREA) continue;

            Mat component = new Mat();
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(component, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            hierarchy.release();

            if (!contours.isEmpty()) {
                MatOfPoint largest = largestContour(contours);
                double area = Imgproc.contourArea(largest);
                MatOfPoint2f curve = new MatOfPoint2f(largest.toArray());
                double perimeter = Imgproc.arcLength(curve, true);
                curve.release();

                if (perimeter > 0
                        && 4 * Math.PI * area / (perimeter * perimeter) >= 0.75) {
                    Core.MinMaxLocResult peak = Core.minMaxLoc(distance, component);
                    if (peak.maxVal >= MIN_PEAK_DEPTH) radii.add(peak.maxVal);
                }
            }
            for (MatOfPoint contour : contours) contour.release();
            component.release();
        }

        labels.release();
        stats.release();
        centroids.release();
        distance.release();

        if (radii.isEmpty()) return -1;
        Collections.sort(radii);
        return radii.get(radii.size() / 2);
    }

    private Mat voteForCenters(Mat mask, double baseRadius) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        hierarchy.release();

        int width = mask.cols();
        int height = mask.rows();
        float[] buffer = new float[width * height];

        for (MatOfPoint contour : contours) {
            Point[] points = contour.toArray();
            int count = points.length;
            if (count < 2 * VOTE_TANGENT_STEP) {
                contour.release();
                continue;
            }
            for (int index = 0; index < count; index++) {
                Point before = points[Math.floorMod(index - VOTE_TANGENT_STEP, count)];
                Point after = points[(index + VOTE_TANGENT_STEP) % count];
                double tangentX = after.x - before.x;
                double tangentY = after.y - before.y;
                double length = Math.hypot(tangentX, tangentY);
                if (length < 1e-6) continue;

                double normalX = tangentY / length;
                double normalY = -tangentX / length;

                double scaleFactor = 0.6 + 0.4 * (points[index].y / height);
                double localRadius = baseRadius * scaleFactor;

                int voteX = (int) Math.round(points[index].x + normalX * localRadius);
                int voteY = (int) Math.round(points[index].y + normalY * localRadius);
                if (voteX >= 0 && voteX < width && voteY >= 0 && voteY < height) {
                    buffer[voteY * width + voteX] += 1.0f;
                }
            }
            contour.release();
        }

        Mat accumulator = new Mat(height, width, CvType.CV_32F);
        accumulator.put(0, 0, buffer);
        Imgproc.GaussianBlur(accumulator, accumulator, new Size(0, 0),
                Math.max(1.0, baseRadius * VOTE_BLUR_FRACTION));
        return accumulator;
    }

    private List<Center> findCenters(Mat mask) {
        double radius = estimateVoteRadius(mask);
        if (radius < 0) return new ArrayList<>();

        updateCompareDisc();

        Mat accumulator = voteForCenters(mask, radius);
        Core.MinMaxLocResult strongest = Core.minMaxLoc(accumulator);
        if (strongest.maxVal <= 0) {
            accumulator.release();
            return new ArrayList<>();
        }

        Mat discMax = new Mat();
        Imgproc.dilate(accumulator, discMax, compareDisc);

        Mat isPeak = new Mat();
        Core.compare(accumulator, discMax, isPeak, Core.CMP_GE);

        Mat highEnough = new Mat();
        Core.compare(accumulator, new Scalar(strongest.maxVal * VOTE_FLOOR_FRACTION),
                highEnough, Core.CMP_GE);

        Mat peakMask = new Mat();
        Core.bitwise_and(isPeak, highEnough, peakMask);

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(peakMask, labels, stats, centroids, 8, CvType.CV_32S);

        List<Center> centers = new ArrayList<>();
        for (int label = 1; label < count; label++) {
            Mat component = new Mat();
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);
            Core.MinMaxLocResult best = Core.minMaxLoc(accumulator, component);
            centers.add(new Center((int) best.maxLoc.x, (int) best.maxLoc.y, radius, best.maxVal));
            component.release();
        }

        accumulator.release();
        discMax.release();
        isPeak.release();
        highEnough.release();
        peakMask.release();
        labels.release();
        stats.release();
        centroids.release();

        Collections.sort(centers, new Comparator<Center>() {
            @Override
            public int compare(Center a, Center b) {
                return Double.compare(b.votes, a.votes);
            }
        });
        return mergeClose(centers);
    }

    private List<Center> mergeClose(List<Center> centers) {
        List<Center> kept = new ArrayList<>();
        for (Center center : centers) {
            boolean tooClose = false;
            for (Center other : kept) {
                double separation = Math.hypot(center.x - other.x, center.y - other.y);
                if (separation < MERGE_FRACTION * Math.min(center.radius, other.radius)) {
                    tooClose = true;
                    break;
                }
            }
            if (!tooClose) kept.add(center);
        }
        return kept;
    }

    private boolean isLineClear(byte[] mask, int width, int height,
                                int x1, int y1, int x2, int y2) {
        double length = Math.hypot(x2 - x1, y2 - y1);
        if (length == 0) return true;
        int samples = (int) length;
        for (int step = 0; step <= samples; step++) {
            double t = step / (double) Math.max(1, samples);
            int x = (int) Math.round(x1 + t * (x2 - x1));
            int y = (int) Math.round(y1 + t * (y2 - y1));
            if (x < 0 || x >= width || y < 0 || y >= height) return false;
            if (mask[y * width + x] == 0) return false;
        }
        return true;
    }

    private Mat splitMaskWithLines(Mat rawMask, List<Center> centers,
                                   List<Point[]> links, List<Point[]> cuts) {
        Mat split = rawMask.clone();

        int width = rawMask.cols();
        int height = rawMask.rows();
        byte[] buffer = new byte[width * height];
        rawMask.get(0, 0, buffer);

        for (int i = 0; i < centers.size(); i++) {
            for (int j = i + 1; j < centers.size(); j++) {
                Center first = centers.get(i);
                Center second = centers.get(j);

                double dx = second.x - first.x;
                double dy = second.y - first.y;
                double separation = Math.hypot(dx, dy);

                double reach = (first.radius + second.radius) / 1.8 * CONNECT_DISTANCE_FACTOR;
                double floor = Math.max(first.radius, second.radius) * 0.6;
                if (separation <= floor || separation > reach) continue;
                if (!isLineClear(buffer, width, height, first.x, first.y, second.x, second.y)) {
                    continue;
                }

                double bar = (first.radius + second.radius) / 2.0 * CROSSBAR_SCALE;
                double midX = (first.x + second.x) / 2.0;
                double midY = (first.y + second.y) / 2.0;
                double perpX = -dy / separation;
                double perpY = dx / separation;

                Point point1 = new Point(midX + bar * perpX, midY + bar * perpY);
                Point point2 = new Point(midX - bar * perpX, midY - bar * perpY);
                Imgproc.line(split, point1, point2, new Scalar(0), CUT_THICKNESS);

                links.add(new Point[]{new Point(first.x, first.y), new Point(second.x, second.y)});
                cuts.add(new Point[]{point1, point2});
            }
        }
        return split;
    }

    private List<Ball> measureRegions(Mat cutMask, double inverseScale) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int count = Imgproc.connectedComponentsWithStats(cutMask, labels, stats, centroids, 8, CvType.CV_32S);

        List<Ball> balls = new ArrayList<>();
        for (int label = 1; label < count; label++) {
            if (stats.get(label, Imgproc.CC_STAT_AREA)[0] <= MIN_BLOB_AREA) continue;

            Mat region = new Mat();
            Core.compare(labels, new Scalar(label), region, Core.CMP_EQ);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(region, contours, hierarchy,
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            hierarchy.release();
            region.release();

            if (contours.isEmpty()) continue;
            MatOfPoint largest = largestContour(contours);

            MatOfPoint2f curve = new MatOfPoint2f(largest.toArray());
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(curve, center, radius);

            double[] position = ballPosition(largest.toArray(), center, radius[0]);
            balls.add(new Ball(center.x * inverseScale, center.y * inverseScale,
                    radius[0] * inverseScale, position));

            curve.release();
            for (MatOfPoint contour : contours) contour.release();
        }

        labels.release();
        stats.release();
        centroids.release();
        return balls;
    }

    private double[] ballPosition(Point[] contour, Point center, double radius) {
        if (cameraMatrix == null || contour.length < 5) return null;

        List<Point> kept = new ArrayList<>();
        for (Point point : contour) {
            double offset = Math.hypot(point.x - center.x, point.y - center.y);
            if (Math.abs(offset - radius) <= radius * EDGE_TRIM_FRACTION) {
                kept.add(point);
            }
        }
        Point[] points = kept.size() >= 8 ? kept.toArray(new Point[0]) : contour;

        MatOfPoint2f source = new MatOfPoint2f(points);
        MatOfPoint2f normalized = new MatOfPoint2f();
        Calib3d.undistortPoints(source, normalized, cameraMatrix, distortion);
        Point[] rays = normalized.toArray();
        source.release();
        normalized.release();

        MatOfPoint2f centerSource = new MatOfPoint2f(center);
        MatOfPoint2f centerNormalized = new MatOfPoint2f();
        Calib3d.undistortPoints(centerSource, centerNormalized, cameraMatrix, distortion);
        Point centerRay = centerNormalized.toArray()[0];
        centerSource.release();
        centerNormalized.release();

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
        double alpha = angles[angles.length / 2];
        if (alpha <= 1e-6 || alpha >= Math.PI / 2) return null;

        double distance = BALL_RADIUS_CM / Math.sin(alpha);
        double xReal = axisX * distance;
        double yReal = -axisY * distance;
        double zReal = axisZ * distance;

        double yaw = Math.toRadians(CAMERA_YAW);
        double xRobot = xReal * Math.cos(yaw) - zReal * Math.sin(yaw) + CAMERA_X_OFFSET;
        double zYaw = xReal * Math.sin(yaw) + zReal * Math.cos(yaw);

        double pitch = Math.toRadians(CAMERA_PITCH);
        double yRobot = yReal * Math.cos(pitch) - zYaw * Math.sin(pitch) + CAMERA_Y_OFFSET;
        double zRobot = yReal * Math.sin(pitch) + zYaw * Math.cos(pitch);

        return new double[]{xReal, yReal, zReal, distance, xRobot, yRobot, zRobot};
    }

    private MatOfPoint largestContour(List<MatOfPoint> contours) {
        MatOfPoint best = contours.get(0);
        double bestArea = Imgproc.contourArea(best);
        for (int i = 1; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > bestArea) {
                best = contours.get(i);
                bestArea = area;
            }
        }
        return best;
    }

    private void publishView(int mode, Mat frame, Mat image, Mat processed, Mat raw, Mat cut,
                             List<Center> centers, List<Point[]> links, List<Point[]> cuts,
                             List<Ball> balls, double k) {
        Mat view = new Mat();

        if (mode <= 0) {
            frame.copyTo(view);
        } else {
            Mat small = new Mat();
            int interpolation = Imgproc.INTER_NEAREST;
            switch (mode) {
                case 1:
                    Imgproc.cvtColor(processed, small, Imgproc.COLOR_GRAY2RGB);
                    break;
                case 2:
                    Imgproc.cvtColor(processed, small, Imgproc.COLOR_GRAY2RGB);
                    Core.multiply(small, new Scalar(0.5, 0.5, 0.5), small);
                    break;
                case 3:
                case 4:
                    Imgproc.cvtColor(raw, small, Imgproc.COLOR_GRAY2RGB);
                    Core.multiply(small, new Scalar(0.5, 0.5, 0.5), small);
                    break;
                case 5:
                    colorRegions(cut, small);
                    break;
                default:
                    image.copyTo(small);
                    interpolation = Imgproc.INTER_LINEAR;
                    break;
            }
            Imgproc.resize(small, view, frame.size(), 0, 0, interpolation);
            small.release();

            int thick = Math.max(1, (int) Math.round(k));

            if (mode >= 2 && mode <= 4) {
                for (Center center : centers) {
                    Point p = new Point(center.x * k, center.y * k);
                    Imgproc.circle(view, p, (int) Math.round(center.radius * k), CYAN, thick);
                    Imgproc.circle(view, p, 3 * thick, RED, -1);
                }
            }

            if (mode == 3) {
                for (Point[] link : links) {
                    Imgproc.line(view, scaled(link[0], k), scaled(link[1], k), GREEN, 2 * thick);
                }
            }

            if (mode == 4) {
                int cutThickness = Math.max(1, (int) Math.round(CUT_THICKNESS * k));
                for (Point[] line : cuts) {
                    Imgproc.line(view, scaled(line[0], k), scaled(line[1], k), MAGENTA, cutThickness);
                }
            }

            if (mode == 6) {
                int index = 1;
                for (Ball ball : balls) {
                    Point p = new Point(ball.x, ball.y);
                    Imgproc.circle(view, p, (int) Math.round(ball.radius), GREEN, 2);
                    Imgproc.circle(view, p, 3, RED, -1);
                    String text = String.valueOf(index);
                    if (ball.hasRange) text += " " + Math.round(ball.rangeCm) + "cm";
                    Imgproc.putText(view, text, new Point(ball.x + ball.radius + 4, ball.y),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2);
                    index++;
                }
            }

            String header = MODE_NAMES[mode];
            if (mode >= 2 && mode <= 4) header += "  centers=" + centers.size();
            if (mode == 3 || mode == 4) header += "  links=" + links.size();
            if (mode >= 5) header += "  balls=" + balls.size();
            Imgproc.putText(view, header, new Point(10, 28),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2);
        }

        Bitmap bitmap = Bitmap.createBitmap(view.cols(), view.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(view, bitmap);
        lastFrame.set(bitmap);
        view.release();
    }

    private Point scaled(Point p, double k) {
        return new Point(p.x * k, p.y * k);
    }

    private void colorRegions(Mat mask, Mat out) {
        Mat labels = new Mat();
        int count = Imgproc.connectedComponents(mask, labels, 8, CvType.CV_32S);

        out.create(mask.size(), CvType.CV_8UC3);
        out.setTo(new Scalar(0, 0, 0));

        Mat component = new Mat();
        for (int label = 1; label < count; label++) {
            Core.compare(labels, new Scalar(label), component, Core.CMP_EQ);
            out.setTo(PALETTE[(label - 1) % PALETTE.length], component);
        }
        component.release();
        labels.release();
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
        label.setTextSize(scaleCanvasDensity * 16);

        int index = 1;
        for (Ball ball : balls) {
            float cx = (float) ball.x * scaleBmpPxToCanvasPx;
            float cy = (float) ball.y * scaleBmpPxToCanvasPx;
            float r = (float) ball.radius * scaleBmpPxToCanvasPx;
            canvas.drawCircle(cx, cy, r, outline);
            canvas.drawCircle(cx, cy, scaleCanvasDensity * 3, dot);

            String text = String.valueOf(index);
            if (ball.hasRange) {
                text += "  " + Math.round(ball.rangeCm) + "cm";
            }
            canvas.drawText(text, cx + r + 4, cy, label);
            index++;
        }
    }
}