
package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

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
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PollenDetector implements VisionProcessor {

    public static double REF_DISTANCE = 30.0;
    public static double REF_RADIUS_PX = 22.57;
    public static double REF_DISTANCE_FOV = 15.24;
    public static double FOV_WIDTH_CM = 40.64;

    public static Scalar LOWER_YELLOW = new Scalar(18, 88, 184);
    public static Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

    public static Scalar LOWER_YELLOW_2 = new Scalar(18, 182, 141);
    public static Scalar UPPER_YELLOW_2 = new Scalar(30, 255, 193);

    public static boolean USE_SECOND_MASK = true;

    public static double CAMERA_YAW = 0;
    public static double CAMERA_PITCH = 0;
    public static double CAMERA_X_OFFSET = 0;
    public static double CAMERA_Y_OFFSET = 0;

    public static int MIN_CONTOUR_SIZE = 40;
    public static int MIN_BALL_RADIUS_PX = 5;

    public static boolean FILL_HOLES = false;

    public static int PEAK_COMPARE_RADIUS_PX = 3;
    public static double PEAK_SMOOTH_SIGMA = 0.5;
    public static double MIN_SEED_DEPTH = 5.0;
    public static double SEED_MERGE_FRACTION = 0.7;

    public static double MIN_PEAK_SEPARATION_FRACTION = 0.6;
    public static double CUT_CLEARANCE_FRACTION = 0.30;
    public static double CUT_HALF_LENGTH_FRACTION = 0.8;
    public static double CUT_THICKNESS_FRACTION = 0.20;

    public static int MAX_SPLIT_DEPTH = 8;

    public static boolean SHAPE_VALIDATION = true;
    public static double MIN_CIRCULARITY = 0.55;
    public static double MIN_FILL_RATIO = 0.50;
    public static double MIN_FILL_RATIO_CUT = 0.35;

    public static double HOUGH_MIN_SCALE = 1.0;
    public static double HOUGH_MAX_SCALE = 1.8;
    public static double CENTER_TRUST_FRACTION = 0.6;

    public static int PROCESS_WIDTH = 320;

    private static final double CALIBRATION_WIDTH = 1280.0;
    private static final double CALIBRATION_HEIGHT = 960.0;

    private static final double CALIBRATION_FX = 477.86395397156593;
    private static final double CALIBRATION_FY = 479.2195587816585;
    private static final double CALIBRATION_CX = 457.1946537778554;
    private static final double CALIBRATION_CY = 641.119897833739;

    private static final double DIST_0 = 0.008500997918842705;
    private static final double DIST_1 = -0.02061464151378009;
    private static final double DIST_2 = 0.0007540416777619344;
    private static final double DIST_3 = -0.00012448636069353983;
    private static final double DIST_4 = 0.0;

    private static final double K = REF_DISTANCE * REF_RADIUS_PX;

    private Mat closeKernel;
    private Mat openKernel;
    private Mat compareDisc;

    private Mat cameraMatrix;
    private Mat distortionCoefficients;
    private Mat undistortMapX;
    private Mat undistortMapY;

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

        closeKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(5, 5)
        );

        openKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(5, 5)
        );

        int discSize = 2 * Math.max(1, PEAK_COMPARE_RADIUS_PX) + 1;

        compareDisc = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(discSize, discSize)
        );

        double scaleX = width / CALIBRATION_WIDTH;
        double scaleY = height / CALIBRATION_HEIGHT;

        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(
                0, 0,
                CALIBRATION_FX * scaleX, 0.0, CALIBRATION_CX * scaleX,
                0.0, CALIBRATION_FY * scaleY, CALIBRATION_CY * scaleY,
                0.0, 0.0, 1.0
        );

        distortionCoefficients = new Mat(1, 5, CvType.CV_64FC1);
        distortionCoefficients.put(
                0, 0,
                DIST_0,
                DIST_1,
                DIST_2,
                DIST_3,
                DIST_4
        );

        undistortMapX = new Mat();
        undistortMapY = new Mat();

        Calib3d.initUndistortRectifyMap(
                cameraMatrix,
                distortionCoefficients,
                new Mat(),
                cameraMatrix,
                new Size(width, height),
                CvType.CV_32FC1,
                undistortMapX,
                undistortMapY
        );
    }

    public List<Ball> getBalls() {
        return latestBalls;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat undistorted = new Mat();

        Imgproc.remap(
                frame,
                undistorted,
                undistortMapX,
                undistortMapY,
                Imgproc.INTER_LINEAR
        );

        Mat working = new Mat();
        double scale = 1.0;

        if (PROCESS_WIDTH > 0 && undistorted.cols() > PROCESS_WIDTH) {
            scale = (double) PROCESS_WIDTH / undistorted.cols();

            Imgproc.resize(
                    undistorted,
                    working,
                    new Size(
                            Math.round(undistorted.cols() * scale),
                            Math.round(undistorted.rows() * scale)
                    ),
                    0,
                    0,
                    Imgproc.INTER_AREA
            );
        } else {
            undistorted.copyTo(working);
        }

        Mat mask = preprocess(working);

        List<Region> regions = new ArrayList<>();

        List<Mat> components = splitIntoComponents(mask);

        for (Mat component : components) {
            processMask(component, 0, false, regions);
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

            balls.add(
                    new Ball(
                            fx,
                            fy,
                            fr,
                            region.area * inverse * inverse,
                            spatialCoords(
                                    fx,
                                    fy,
                                    fr,
                                    centerX,
                                    centerY,
                                    focalLengthPx
                            )
                    )
            );
        }

        latestBalls = balls;

        mask.release();
        working.release();
        undistorted.release();

        return null;
    }

    private Mat preprocess(Mat frame) {
        Mat blurred = new Mat();

        Imgproc.GaussianBlur(
                frame,
                blurred,
                new Size(5, 5),
                0
        );

        Mat hsv = new Mat();

        Imgproc.cvtColor(
                blurred,
                hsv,
                Imgproc.COLOR_RGB2HSV
        );

        blurred.release();

        Mat mask = new Mat();

        Core.inRange(
                hsv,
                LOWER_YELLOW,
                UPPER_YELLOW,
                mask
        );

        if (USE_SECOND_MASK) {
            Mat mask2 = new Mat();

            Core.inRange(
                    hsv,
                    LOWER_YELLOW_2,
                    UPPER_YELLOW_2,
                    mask2
            );

            Core.bitwise_or(
                    mask,
                    mask2,
                    mask
            );

            mask2.release();
        }

        hsv.release();

        Imgproc.morphologyEx(
                mask,
                mask,
                Imgproc.MORPH_CLOSE,
                closeKernel,
                new Point(-1, -1),
                2
        );

        Imgproc.morphologyEx(
                mask,
                mask,
                Imgproc.MORPH_OPEN,
                openKernel,
                new Point(-1, -1),
                1
        );

        if (FILL_HOLES) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(
                    mask,
                    contours,
                    hierarchy,
                    Imgproc.RETR_EXTERNAL,
                    Imgproc.CHAIN_APPROX_SIMPLE
            );

            Mat filled = Mat.zeros(
                    mask.size(),
                    CvType.CV_8UC1
            );

            Imgproc.drawContours(
                    filled,
                    contours,
                    -1,
                    new Scalar(255),
                    -1
            );

            for (MatOfPoint contour : contours) {
                contour.release();
            }

            hierarchy.release();
            mask.release();

            mask = filled;
        }

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        int count = Imgproc.connectedComponentsWithStats(
                mask,
                labels,
                stats,
                centroids,
                8,
                CvType.CV_32S
        );

        Mat cleaned = Mat.zeros(
                mask.size(),
                CvType.CV_8UC1
        );

        for (int label = 1; label < count; label++) {
            double area = stats.get(
                    label,
                    Imgproc.CC_STAT_AREA
            )[0];

            if (area > MIN_CONTOUR_SIZE) {
                Mat component = new Mat();

                Core.compare(
                        labels,
                        new Scalar(label),
                        component,
                        Core.CMP_EQ
                );

                Core.bitwise_or(
                        cleaned,
                        component,
                        cleaned
                );

                component.release();
            }
        }

        labels.release();
        stats.release();
        centroids.release();
        mask.release();

        return cleaned;
    }

    private List<Mat> splitIntoComponents(Mat mask) {
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        int count = Imgproc.connectedComponentsWithStats(
                mask,
                labels,
                stats,
                centroids,
                8,
                CvType.CV_32S
        );

        List<Mat> components = new ArrayList<>();

        for (int label = 1; label < count; label++) {
            double area = stats.get(
                    label,
                    Imgproc.CC_STAT_AREA
            )[0];

            if (area <= MIN_CONTOUR_SIZE) {
                continue;
            }

            Mat component = new Mat();

            Core.compare(
                    labels,
                    new Scalar(label),
                    component,
                    Core.CMP_EQ
            );

            components.add(component);
        }

        labels.release();
        stats.release();
        centroids.release();

        return components;
    }

    private List<Seed> findPeaks(Mat mask) {
        Mat distance = new Mat();

        Imgproc.distanceTransform(
                mask,
                distance,
                Imgproc.DIST_L2,
                5
        );

        Mat compared = new Mat();

        if (PEAK_SMOOTH_SIGMA > 0) {
            Imgproc.GaussianBlur(
                    distance,
                    compared,
                    new Size(0, 0),
                    PEAK_SMOOTH_SIGMA
            );
        } else {
            distance.copyTo(compared);
        }

        Mat discMax = new Mat();

        Imgproc.dilate(
                compared,
                discMax,
                compareDisc
        );

        Mat localMax = new Mat();

        Core.compare(
                compared,
                discMax,
                localMax,
                Core.CMP_GE
        );

        Mat deepEnough = new Mat();

        Core.compare(
                distance,
                new Scalar(MIN_SEED_DEPTH),
                deepEnough,
                Core.CMP_GE
        );

        Mat peakMask = new Mat();

        Core.bitwise_and(
                localMax,
                deepEnough,
                peakMask
        );

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        int count = Imgproc.connectedComponentsWithStats(
                peakMask,
                labels,
                stats,
                centroids,
                8,
                CvType.CV_32S
        );

        List<Seed> seeds = new ArrayList<>();

        for (int label = 1; label < count; label++) {
            int x = (int) Math.round(
                    centroids.get(label, 0)[0]
            );

            int y = (int) Math.round(
                    centroids.get(label, 1)[0]
            );

            if (x < 0 ||
                    x >= distance.cols() ||
                    y < 0 ||
                    y >= distance.rows()) {
                continue;
            }

            double value = distance.get(y, x)[0];

            if (value < MIN_SEED_DEPTH) {
                continue;
            }

            seeds.add(
                    new Seed(
                            x,
                            y,
                            value
                    )
            );
        }

        Collections.sort(
                seeds,
                new Comparator<Seed>() {
                    @Override
                    public int compare(Seed a, Seed b) {
                        return Double.compare(
                                b.strength,
                                a.strength
                        );
                    }
                }
        );

        distance.release();
        compared.release();
        discMax.release();
        localMax.release();
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
                double separation = Math.hypot(
                        seed.x - other.x,
                        seed.y - other.y
                );

                double limit = Math.max(
                        MIN_BALL_RADIUS_PX,
                        SEED_MERGE_FRACTION *
                                Math.min(
                                        seed.strength,
                                        other.strength
                                )
                );

                if (separation < limit) {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose) {
                kept.add(seed);
            }
        }

        return kept;
    }

    private boolean pairSeparationOk(
            Seed a,
            Seed b,
            double separation
    ) {
        double smaller = Math.min(
                a.strength,
                b.strength
        );

        double required = Math.max(
                MIN_BALL_RADIUS_PX,
                MIN_PEAK_SEPARATION_FRACTION * smaller
        );

        return separation >= required;
    }

    private double distanceToCutLine(
            Seed peak,
            Seed a,
            Seed b
    ) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;

        double length = Math.hypot(
                dx,
                dy
        );

        if (length == 0) {
            return Double.MAX_VALUE;
        }

        double midX = (a.x + b.x) / 2.0;
        double midY = (a.y + b.y) / 2.0;

        return Math.abs(
                (peak.x - midX) * dx +
                        (peak.y - midY) * dy
        ) / length;
    }

    private boolean cutIsClear(
            Seed a,
            Seed b,
            List<Seed> peaks
    ) {
        for (Seed other : peaks) {
            if (other == a || other == b) {
                continue;
            }

            double clearance = Math.max(
                    2.0,
                    CUT_CLEARANCE_FRACTION *
                            other.strength
            );

            if (distanceToCutLine(
                    other,
                    a,
                    b
            ) < clearance) {
                return false;
            }
        }

        return true;
    }

    private Seed[] chooseCutPair(List<Seed> peaks) {
        List<double[]> candidates = new ArrayList<>();

        for (int i = 0; i < peaks.size(); i++) {
            for (int j = i + 1; j < peaks.size(); j++) {
                double separation = Math.hypot(
                        peaks.get(i).x - peaks.get(j).x,
                        peaks.get(i).y - peaks.get(j).y
                );

                if (!pairSeparationOk(
                        peaks.get(i),
                        peaks.get(j),
                        separation
                )) {
                    continue;
                }

                candidates.add(
                        new double[]{
                                separation,
                                i,
                                j
                        }
                );
            }
        }

        Collections.sort(
                candidates,
                new Comparator<double[]>() {
                    @Override
                    public int compare(
                            double[] a,
                            double[] b
                    ) {
                        return Double.compare(
                                a[0],
                                b[0]
                        );
                    }
                }
        );

        for (double[] candidate : candidates) {
            Seed a = peaks.get(
                    (int) candidate[1]
            );

            Seed b = peaks.get(
                    (int) candidate[2]
            );

            if (cutIsClear(
                    a,
                    b,
                    peaks
            )) {
                return new Seed[]{a, b};
            }
        }

        return null;
    }

    private Mat[] splitAtMidpoint(
            Mat mask,
            Seed a,
            Seed b,
            double radiusHint
    ) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;

        double length = Math.hypot(
                dx,
                dy
        );

        if (length == 0) {
            return null;
        }

        double midX = (a.x + b.x) / 2.0;
        double midY = (a.y + b.y) / 2.0;

        double normalX = -dy / length;
        double normalY = dx / length;

        double half = Math.max(
                10.0,
                CUT_HALF_LENGTH_FRACTION * radiusHint
        );

        int thickness = Math.max(
                3,
                (int) (
                        CUT_THICKNESS_FRACTION *
                                radiusHint
                )
        );

        Mat carved = mask.clone();

        Point p1 = new Point(
                midX + normalX * half,
                midY + normalY * half
        );

        Point p2 = new Point(
                midX - normalX * half,
                midY - normalY * half
        );

        Imgproc.line(
                carved,
                p1,
                p2,
                new Scalar(0),
                thickness
        );

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        Imgproc.connectedComponentsWithStats(
                carved,
                labels,
                stats,
                centroids,
                8,
                CvType.CV_32S
        );

        int ax = Math.max(
                0,
                Math.min(
                        mask.cols() - 1,
                        a.x
                )
        );

        int ay = Math.max(
                0,
                Math.min(
                        mask.rows() - 1,
                        a.y
                )
        );

        int bx = Math.max(
                0,
                Math.min(
                        mask.cols() - 1,
                        b.x
                )
        );

        int by = Math.max(
                0,
                Math.min(
                        mask.rows() - 1,
                        b.y
                )
        );

        int label1 = (int) labels.get(
                ay,
                ax
        )[0];

        int label2 = (int) labels.get(
                by,
                bx
        )[0];

        Mat[] result = null;

        if (label1 != 0 &&
                label2 != 0 &&
                label1 != label2) {

            Mat first = new Mat();
            Mat second = new Mat();

            Core.compare(
                    labels,
                    new Scalar(label1),
                    first,
                    Core.CMP_EQ
            );

            Core.compare(
                    labels,
                    new Scalar(label2),
                    second,
                    Core.CMP_EQ
            );

            result = new Mat[]{
                    first,
                    second
            };
        }

        carved.release();
        labels.release();
        stats.release();
        centroids.release();

        if (result != null) {
            return result;
        }

        Mat first = Mat.zeros(
                mask.size(),
                CvType.CV_8UC1
        );

        Mat second = Mat.zeros(
                mask.size(),
                CvType.CV_8UC1
        );

        for (int y = 0; y < mask.rows(); y++) {
            for (int x = 0; x < mask.cols(); x++) {
                if (mask.get(y, x)[0] == 0) {
                    continue;
                }

                double side =
                        (x - midX) * dx +
                                (y - midY) * dy;

                if (side <= 0) {
                    first.put(
                            y,
                            x,
                            255
                    );
                } else {
                    second.put(
                            y,
                            x,
                            255
                    );
                }
            }
        }

        if (Core.countNonZero(first) <= MIN_CONTOUR_SIZE ||
                Core.countNonZero(second) <= MIN_CONTOUR_SIZE) {
            first.release();
            second.release();
            return null;
        }

        return new Mat[]{
                first,
                second
        };
    }

    private MatOfPoint largestContour(Mat mask) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(
                mask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_NONE
        );

        hierarchy.release();

        if (contours.isEmpty()) {
            return null;
        }

        MatOfPoint best = contours.get(0);
        double bestArea = Imgproc.contourArea(best);

        for (int i = 1; i < contours.size(); i++) {
            MatOfPoint current = contours.get(i);
            double area = Imgproc.contourArea(current);

            if (area > bestArea) {
                best.release();
                best = current;
                bestArea = area;
            } else {
                current.release();
            }
        }

        return best;
    }

    private Region measureRegion(
            Mat mask,
            boolean wasCut
    ) {
        MatOfPoint contour = largestContour(mask);

        if (contour == null) {
            return null;
        }

        double area = Imgproc.contourArea(
                contour
        );

        if (area <= MIN_CONTOUR_SIZE) {
            contour.release();
            return null;
        }

        Mat distance = new Mat();

        Imgproc.distanceTransform(
                mask,
                distance,
                Imgproc.DIST_L2,
                5
        );

        Core.MinMaxLocResult extreme =
                Core.minMaxLoc(distance);

        double peakRadius = extreme.maxVal;

        if (peakRadius < MIN_BALL_RADIUS_PX) {
            distance.release();
            contour.release();
            return null;
        }

        Mat maxMask = new Mat();

        Core.compare(
                distance,
                new Scalar(peakRadius),
                maxMask,
                Core.CMP_EQ
        );

        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();

        int count =
                Imgproc.connectedComponentsWithStats(
                        maxMask,
                        labels,
                        stats,
                        centroids,
                        8,
                        CvType.CV_32S
                );

        Point center = null;

        if (count > 1) {
            center = new Point(
                    centroids.get(1, 0)[0],
                    centroids.get(1, 1)[0]
            );
        }

        maxMask.release();
        labels.release();
        stats.release();
        centroids.release();
        distance.release();

        if (center == null) {
            contour.release();
            return null;
        }

        Mat houghInput = new Mat();

        Imgproc.GaussianBlur(
                mask,
                houghInput,
                new Size(5, 5),
                1.0
        );

        Mat circles = new Mat();

        int minRadius = Math.max(
                MIN_BALL_RADIUS_PX,
                (int) (
                        peakRadius *
                                HOUGH_MIN_SCALE
                )
        );

        int maxRadius = Math.max(
                minRadius + 1,
                (int) (
                        peakRadius *
                                HOUGH_MAX_SCALE
                )
        );

        Imgproc.HoughCircles(
                houghInput,
                circles,
                Imgproc.HOUGH_GRADIENT,
                1.2,
                10,
                100,
                15,
                minRadius,
                maxRadius
        );

        houghInput.release();

        double radius = peakRadius;

        if (circles.cols() > 0) {
            double bestDistance =
                    Double.MAX_VALUE;

            double[] bestCircle = null;

            for (int i = 0; i < circles.cols(); i++) {
                double[] circle =
                        circles.get(0, i);

                if (circle == null) {
                    continue;
                }

                double separation =
                        Math.hypot(
                                circle[0] - center.x,
                                circle[1] - center.y
                        );

                if (separation < bestDistance) {
                    bestDistance = separation;
                    bestCircle = circle;
                }
            }

            if (bestCircle != null) {
                radius = bestCircle[2];

                double houghDistance =
                        Math.hypot(
                                bestCircle[0] - center.x,
                                bestCircle[1] - center.y
                        );

                if (houghDistance <=
                        CENTER_TRUST_FRACTION * radius) {
                    center = new Point(
                            bestCircle[0],
                            bestCircle[1]
                    );
                }
            }
        }

        circles.release();

        if (radius < MIN_BALL_RADIUS_PX) {
            contour.release();
            return null;
        }

        MatOfPoint2f contour2f =
                new MatOfPoint2f(
                        contour.toArray()
                );

        double perimeter =
                Imgproc.arcLength(
                        contour2f,
                        true
                );

        contour2f.release();

        double circularity =
                perimeter > 0
                        ? 4.0 * Math.PI * area /
                        (perimeter * perimeter)
                        : 0.0;

        double circleArea =
                Math.PI * radius * radius;

        double fillRatio =
                circleArea > 0
                        ? area / circleArea
                        : 0.0;

        if (SHAPE_VALIDATION) {
            if (wasCut) {
                if (fillRatio < MIN_FILL_RATIO_CUT) {
                    contour.release();
                    return null;
                }
            } else {
                if (circularity < MIN_CIRCULARITY ||
                        fillRatio < MIN_FILL_RATIO) {
                    contour.release();
                    return null;
                }
            }
        }

        contour.release();

        return new Region(
                center,
                radius,
                area
        );
    }

    private int countPeaksIn(
            Mat mask,
            List<Seed> peaks
    ) {
        int total = 0;

        for (Seed peak : peaks) {
            int x = Math.max(
                    0,
                    Math.min(
                            mask.cols() - 1,
                            peak.x
                    )
            );

            int y = Math.max(
                    0,
                    Math.min(
                            mask.rows() - 1,
                            peak.y
                    )
            );

            if (mask.get(y, x)[0] > 0) {
                total++;
            }
        }

        return total;
    }

    private void finishAsSingle(
            Mat mask,
            boolean wasCut,
            List<Region> out
    ) {
        Region region =
                measureRegion(
                        mask,
                        wasCut
                );

        if (region != null) {
            out.add(region);
        }
    }

    private void processMask(
            Mat mask,
            int depth,
            boolean wasCut,
            List<Region> out
    ) {
        List<Seed> peaks =
                findPeaks(mask);

        double rEst;

        if (peaks.isEmpty()) {
            Core.MinMaxLocResult extreme =
                    Core.minMaxLoc(mask);

            rEst = extreme.maxVal;
        } else {
            rEst = peaks.get(0).strength;
        }

        if (depth > MAX_SPLIT_DEPTH ||
                rEst < MIN_BALL_RADIUS_PX) {
            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );
            return;
        }

        if (peaks.size() < 2) {
            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );
            return;
        }

        Seed[] pair =
                chooseCutPair(peaks);

        if (pair == null) {
            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );
            return;
        }

        Mat[] halves =
                splitAtMidpoint(
                        mask,
                        pair[0],
                        pair[1],
                        rEst
                );

        if (halves == null) {
            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );
            return;
        }

        boolean usable =
                Core.countNonZero(halves[0]) >
                        MIN_CONTOUR_SIZE &&
                        Core.countNonZero(halves[1]) >
                                MIN_CONTOUR_SIZE &&
                        countPeaksIn(
                                halves[0],
                                peaks
                        ) > 0 &&
                        countPeaksIn(
                                halves[1],
                                peaks
                        ) > 0;

        if (!usable) {
            halves[0].release();
            halves[1].release();

            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );

            return;
        }

        int before = out.size();

        processMask(
                halves[0],
                depth + 1,
                true,
                out
        );

        processMask(
                halves[1],
                depth + 1,
                true,
                out
        );

        halves[0].release();
        halves[1].release();

        if (out.size() == before) {
            finishAsSingle(
                    mask,
                    wasCut,
                    out
            );
        }
    }

    private double[] spatialCoords(
            double circleX,
            double circleY,
            double radius,
            double centerX,
            double centerY,
            double focalLength
    ) {
        if (radius <= 0) {
            return new double[]{
                    0,
                    0,
                    0,
                    0,
                    0,
                    0
            };
        }

        double currentDistance =
                K / radius;

        double u =
                (circleX - centerX) /
                        focalLength;

        double v =
                (centerY - circleY) /
                        focalLength;

        double norm =
                Math.sqrt(
                        1 +
                                u * u +
                                v * v
                );

        double zReal =
                currentDistance / norm;

        double xReal =
                (circleX - centerX) *
                        zReal /
                        focalLength;

        double yReal =
                (centerY - circleY) *
                        zReal /
                        focalLength;

        double yaw =
                Math.toRadians(
                        CAMERA_YAW
                );

        double xAngle =
                (
                        xReal * Math.cos(yaw) -
                                zReal * Math.sin(yaw)
                ) +
                        CAMERA_X_OFFSET;

        double zYaw =
                xReal * Math.sin(yaw) +
                        zReal * Math.cos(yaw);

        double pitch =
                Math.toRadians(
                        CAMERA_PITCH
                );

        double yAngle =
                (
                        yReal * Math.cos(pitch) -
                                zYaw * Math.sin(pitch)
                ) +
                        CAMERA_Y_OFFSET;

        double zAngle =
                yReal * Math.sin(pitch) +
                        zYaw * Math.cos(pitch);

        return new double[]{
                xReal,
                yReal,
                zReal,
                xAngle,
                yAngle,
                zAngle
        };
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext
    ) {
    }
}
