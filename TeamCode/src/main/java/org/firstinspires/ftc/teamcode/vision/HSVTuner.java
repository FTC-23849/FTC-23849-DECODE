package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

@Config
@TeleOp(name = "HSV Tuner", group = "Vision")
public class HSVTuner extends LinearOpMode {
    public static int H1Lower = 0;
    public static int S1Lower = 0;
    public static int V1Lower = 0;
    public static int H1Upper = 179;
    public static int S1Upper = 255;
    public static int V1Upper = 255;

    public static int H2Lower = 0;
    public static int S2Lower = 0;
    public static int V2Lower = 0;
    public static int H2Upper = 179;
    public static int S2Upper = 255;
    public static int V2Upper = 255;

    public static boolean USE_SECOND_MASK = true;
    public static boolean SHOW_MASK = false;
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;
    public static double STREAM_FPS = 20.0;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        HSVProcessor processor = new HSVProcessor();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(processor)
                .enableLiveView(false)
                .build();

        dashboard.startCameraStream(processor, STREAM_FPS);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Range 1", "[%d,%d,%d] - [%d,%d,%d]", H1Lower, S1Lower, V1Lower, H1Upper, S1Upper, V1Upper);
            telemetry.addData("Range 2", "[%d,%d,%d] - [%d,%d,%d]", H2Lower, S2Lower, V2Lower, H2Upper, S2Upper, V2Upper);
            telemetry.addData("Second Mask", USE_SECOND_MASK);
            telemetry.addData("Output", SHOW_MASK ? "Mask" : "Filtered");
            telemetry.update();
            sleep(50);
        }

        dashboard.stopCameraStream();
        visionPortal.close();
    }

    private static class HSVProcessor implements VisionProcessor, CameraStreamSource {
        private final Mat hsv = new Mat();
        private final Mat mask1 = new Mat();
        private final Mat mask2 = new Mat();
        private final Mat mask = new Mat();
        private final Mat output = new Mat();
        private Bitmap bitmap;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            Core.inRange(
                    hsv,
                    new Scalar(H1Lower, S1Lower, V1Lower),
                    new Scalar(H1Upper, S1Upper, V1Upper),
                    mask1
            );

            if (USE_SECOND_MASK) {
                Core.inRange(
                        hsv,
                        new Scalar(H2Lower, S2Lower, V2Lower),
                        new Scalar(H2Upper, S2Upper, V2Upper),
                        mask2
                );
                Core.bitwise_or(mask1, mask2, mask);
            } else {
                mask1.copyTo(mask);
            }

            if (SHOW_MASK) {
                Imgproc.cvtColor(mask, output, Imgproc.COLOR_GRAY2RGB);
            } else {
                Core.bitwise_and(frame, frame, output, mask);
            }

            Utils.matToBitmap(output, bitmap);
            return null;
        }

        @Override
        public void onDrawFrame(
                android.graphics.Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext) {
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            if (bitmap == null) return;
            Bitmap frame = bitmap;
            continuation.dispatch(new org.firstinspires.ftc.robotcore.external.function.ContinuationResult<Consumer<Bitmap>>() {
                @Override
                public void handle(Consumer<Bitmap> consumer) {
                    consumer.accept(frame);
                }
            });
        }
    }
}