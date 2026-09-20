package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
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

/**
 * undistort -> mask -> vote for centres -> cut fused balls apart -> measure.
 *
 * Range comes from angular size: contour points become rays through the
 * calibrated camera, and a ball of radius R subtends asin(R / Z). Working in
 * ray space is what makes it correct off-axis -- a sphere at 45 degrees
 * projects as an ellipse but still subtends the same angle.
 *
 * Frames arrive from VisionPortal as RGB, not BGR.
 */
public class PollenDetector implements VisionProcessor {

    // --- calibration (960 x 1280) ----------------------------------------
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

    // --- ball -------------------------------------------------------------
    public static double BALL_RADIUS_CM = 3.55;
    public static double EDGE_TRIM_FRACTION = 0.15;

    // --- camera mounting --------------------------------------------------
    public static double CAMERA_YAW = 0;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_X_OFFSET = 0;
    public static double CAMERA_Y_OFFSET = 0;

    // --- mask -------------------------------------------------------------
    public static Scalar LOWER_YELLOW = new Scalar(18, 88, 184);
    public static Scalar UPPER_YELLOW = new Scalar(30, 255, 255);
    public static Scalar LOWER_YELLOW_2 = new Scalar(18, 182, 161);
    public static Scalar UPPER_YELLOW_2 = new Scalar(30, 255, 193);

    public static int MIN_BLOB_AREA = 40;
    public static int RAW_OPEN_ITERATIONS = 10;

    // --- centres ----------------------------------------------------------
    public static int PEAK_RADIUS_PX = 6;
    public static double MIN_PEAK_DEPTH = 5.0;
    public static double MERGE_FRACTION = 1.0;

    public static int VOTE_TANGENT_STEP = 4;
    public static double VOTE_BLUR_FRACTION = 0.15;
    public static double VOTE_FLOOR_FRACTION = 0.15;

    // --- cutting ----------------------------------------------------------
    public static double CONNECT_DISTANCE_FACTOR = 2.2;
    public static double CROSSBAR_SCALE = 1.0;
    public static int CUT_THICKNESS = 3;

    /**
     * Detection runs on a downscaled copy; the contour walk in the voting stage
     * will not hold frame rate at full resolution on a Control Hub. Results are
     * scaled back to full-frame coordinates. 0 disables.
     */
    public static int PROCESS_WIDTH = 320;

    private Mat closeKernel;
    private Mat openKernel;
    private Mat erodeKernel;
    private Mat compareDisc;

    private Mat cameraMatrix;           // for the frame as processed
    private Mat distortion;
    private Mat mapX;
    private Mat mapY;
    private Size mapSize;

    private int frameWidth;
    private int frameHeight;

    private volatile List<Ball> latestBalls = new ArrayList<>();

    public static class Ball {
        public final double x;          // full-frame pixels
        public final double y;
        public final double radius;
        public final double xCm;        // camera frame
        public final double yCm;
        public final double zCm;
        public final double rangeCm;
        public final double xRobot;     // after yaw / pitch / offsets
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
        compareDisc = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE, new Size(2 * PEAK_RADIUS_PX + 1, 2 * PEAK_RADIUS_PX + 1));
    }

    /** Latest detections, in full-frame pixel coordinates. */
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

        Mat undistorted = undistortFrame(working);
        if (undistorted != working) {
            working.release();
        }

        Mat processed = new Mat();
        Mat raw = new Mat();
        buildMasks(undistorted, processed, raw);

        List<Center> centers = findCenters(processed);
        Mat cut = splitMaskWithLines(raw, centers);
        latestBalls = measureRegions(cut, 1.0 / scale);

        processed.release();
        raw.release();
        cut.release();
        undistorted.release();
        return null;
    }

    // --- step 1: undistort ------------------------------------------------

    private Mat undistortFrame(Mat frame) {
        int width = frame.cols();
        int height = frame.rows();
        Size size = new Size(width, height);

        // fx, fy, cx, cy are all in pixels, so rescale to this frame.
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
            // alpha=0 crops to valid pixels, so no black wedges reach the threshold.
            Mat newMatrix = Calib3d.getOptimalNewCameraMatrix(scaled, coefficients, size, 0, size);
            mapX = new Mat();
            mapY = new Mat();
            Calib3d.initUndistortRectifyMap(scaled, coefficients, new Mat(), newMatrix,
                    size, CvType.CV_16SC2, mapX, mapY);
            mapSize = size;

            releaseIntrinsics();
            // The frame comes out undistorted, so points from it normalise with
            // the new matrix and zero distortion -- the original D would bend
            // them a second time.
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

    // --- step 2: mask -----------------------------------------------------

    /**
     * The processed mask drives centre finding; the opened raw mask is what
     * gets cut, since its necks are already thinned.
     */
    private void buildMasks(Mat rgbFrame, Mat processedOut, Mat rawOut) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(rgbFrame, blurred, new Size(5, 5), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);
        blurred.release();

        Mat mask = new Mat();
        Core.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW, mask);

        Mat second = new Mat();
        Core.inRange(hsv, LOWER_YELLOW_2, UPPER_YELLOW_2, second);
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

    // --- step 3: find centres by voting -----------------------------------

    /**
     * Median inscribed radius of blobs round enough to be a lone ball. Fused
     * clusters are excluded, so the estimate comes from balls measuring right.
     */
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

    /**
     * Each contour point steps one radius inward along its own normal and votes
     * there. A ball's outer arc votes for that ball's centre even when its
     * inner side is fused away, which the distance transform cannot do -- it
     * reads a ring of fused balls as a thick band with maxima on the rim.
     */
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

                // Balls lower in the frame are nearer, so they are bigger.
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

    // --- step 4: cut fused balls apart ------------------------------------

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

    /**
     * Cut a crossbar between any two centres whose connecting line stays inside
     * the mask -- that is, balls actually fused to each other.
     */
    private Mat splitMaskWithLines(Mat rawMask, List<Center> centers) {
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
            }
        }
        return split;
    }

    // --- step 5: measure --------------------------------------------------

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

    /**
     * Distance from the ball's angular size. Contour points become rays through
     * the calibrated camera, and the angle each makes with the ray to the
     * ball's centre is the half-angle of the tangent cone; a ball of radius R
     * subtends asin(R / Z), so Z = R / sin(a).
     *
     * Returns {xCm, yCm, zCm, rangeCm, xRobot, yRobot, zRobot} or null.
     */
    private double[] ballPosition(Point[] contour, Point center, double radius) {
        if (cameraMatrix == null || contour.length < 5) return null;

        // Keep only points on the ball's own silhouette; the rest are cut edges
        // or a neighbour's leftover.
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

    // --- overlay ----------------------------------------------------------

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