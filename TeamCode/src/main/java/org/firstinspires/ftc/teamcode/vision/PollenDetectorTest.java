package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Pollen Detector Test", group = "Vision")
public class PollenDetectorTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final int DASHBOARD_FPS = 30;

    public static boolean LOCK_EXPOSURE = true;
    public static long EXPOSURE_MS = 6;
    public static int GAIN = 250;

    public static int A_MODE = 0;

    private PollenDetector detector;
    private VisionPortal portal;
    private FtcDashboard dashboard;

    private long appliedExposure = -1;
    private int appliedGain = -1;
    private int detailMode = 0;
    private boolean liveView = true;

    private boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastB;

    private int frames;
    private long windowStart;
    private double fps;

    @Override
    public void runOpMode() {
        detector = new PollenDetector();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(detector)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true);

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        portal = builder.build();

        dashboard = FtcDashboard.getInstance();
        detector.setViewMode(A_MODE);
        dashboard.startCameraStream(detector, DASHBOARD_FPS);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Waiting for camera...");
        telemetry.update();
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        if (!isStopRequested()) {
            syncExposure();
        }

        telemetry.addLine("Ready. Press start.");
        telemetry.addLine("Feed: http://192.168.43.1:8080/dash");
        telemetry.addLine("dpad up/down = exposure, left/right = gain");
        telemetry.addLine("A = telemetry detail, B = live view");
        telemetry.update();
        waitForStart();

        windowStart = System.nanoTime();

        while (opModeIsActive()) {
            handleControls();
            syncExposure();
            detector.setViewMode(A_MODE);
            updateFps();

            List<PollenDetector.Ball> balls = detector.getBalls();

            telemetry.addData("fps", "%.1f", fps);
            telemetry.addData("balls", balls.size());
            telemetry.addData("exposure / gain", "%d ms / %d", EXPOSURE_MS, GAIN);
            telemetry.addData("live view", liveView ? "on" : "off");
            telemetry.addData("A_MODE", A_MODE);

            PollenDetector.Ball closest = closestBall(balls);
            if (closest == null) {
                telemetry.addLine("no ranged ball");
            } else {
                telemetry.addData("closest", "%.1f cm at %.1f deg",
                        closest.rangeCm, bearingDegrees(closest));
            }

            if (detailMode >= 1 && closest != null) {
                telemetry.addData("  camera", "X %.1f  Y %.1f  Z %.1f",
                        closest.xCm, closest.yCm, closest.zCm);
                telemetry.addData("  robot", "X %.1f  Y %.1f  Z %.1f",
                        closest.xRobot, closest.yRobot, closest.zRobot);
                telemetry.addData("  pixels", "(%.0f, %.0f) r=%.1f",
                        closest.x, closest.y, closest.radius);
            }

            if (detailMode >= 2) {
                int index = 1;
                for (PollenDetector.Ball ball : balls) {
                    if (ball.hasRange) {
                        telemetry.addData(String.valueOf(index), "%.0f cm  %.0f deg  r=%.0f",
                                ball.rangeCm, bearingDegrees(ball), ball.radius);
                    } else {
                        telemetry.addData(String.valueOf(index), "no range  r=%.0f", ball.radius);
                    }
                    index++;
                    if (index > 8) break;
                }
            }

            telemetry.update();
            sleep(20);
        }

        dashboard.stopCameraStream();
        portal.close();
    }

    private double bearingDegrees(PollenDetector.Ball ball) {
        return Math.toDegrees(Math.atan2(ball.xCm, ball.zCm));
    }

    private PollenDetector.Ball closestBall(List<PollenDetector.Ball> balls) {
        PollenDetector.Ball closest = null;
        for (PollenDetector.Ball ball : balls) {
            if (!ball.hasRange) continue;
            if (closest == null || ball.rangeCm < closest.rangeCm) closest = ball;
        }
        return closest;
    }

    private void handleControls() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if (up && !lastUp) EXPOSURE_MS++;
        if (down && !lastDown && EXPOSURE_MS > 1) EXPOSURE_MS--;
        if (right && !lastRight) GAIN += 10;
        if (left && !lastLeft && GAIN >= 10) GAIN -= 10;

        if (a && !lastA) detailMode = (detailMode + 1) % 3;

        if (b && !lastB) {
            liveView = !liveView;
            if (liveView) {
                portal.resumeLiveView();
            } else {
                portal.stopLiveView();
            }
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
        lastA = a;
        lastB = b;
    }

    private void syncExposure() {
        if (!LOCK_EXPOSURE) return;
        if (EXPOSURE_MS == appliedExposure && GAIN == appliedGain) return;
        applyExposure();
        appliedExposure = EXPOSURE_MS;
        appliedGain = GAIN;
    }

    private void applyExposure() {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            }
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(GAIN);
            }
        } catch (Exception error) {
            telemetry.addLine("exposure control failed: " + error.getMessage());
        }
    }

    private void updateFps() {
        frames++;
        long now = System.nanoTime();
        double elapsed = (now - windowStart) / 1e9;
        if (elapsed >= 0.5) {
            fps = frames / elapsed;
            frames = 0;
            windowStart = now;
        }
    }
}