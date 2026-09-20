package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Test harness for PollenDetector.
 *
 * Controls, once running:
 *   dpad up / down    exposure
 *   dpad left / right gain
 *   A                 cycle telemetry detail
 *   B                 toggle the live view (off is noticeably faster)
 */
@TeleOp(name = "Pollen Detector Test", group = "Vision")
public class PollenDetectorTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    /**
     * Manual exposure keeps the HSV thresholds stable. Auto exposure under
     * arena lighting will move the mask more than any detector parameter.
     */
    private static final boolean LOCK_EXPOSURE = true;
    private static final long START_EXPOSURE_MS = 6;
    private static final int START_GAIN = 250;

    private PollenDetector detector;
    private VisionPortal portal;

    private long exposureMs = START_EXPOSURE_MS;
    private int gain = START_GAIN;
    private int detailMode = 0;             // 0 summary, 1 closest, 2 every ball
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

        telemetry.addLine("Waiting for camera...");
        telemetry.update();
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        if (LOCK_EXPOSURE && !isStopRequested()) {
            applyExposure();
        }

        telemetry.addLine("Ready. Press start.");
        telemetry.addLine("dpad up/down = exposure, left/right = gain");
        telemetry.addLine("A = telemetry detail, B = live view");
        telemetry.update();
        waitForStart();

        windowStart = System.nanoTime();

        while (opModeIsActive()) {
            handleControls();
            updateFps();

            List<PollenDetector.Ball> balls = detector.getBalls();

            telemetry.addData("fps", "%.1f", fps);
            telemetry.addData("balls", balls.size());
            telemetry.addData("exposure / gain", "%d ms / %d", exposureMs, gain);

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
                    if (index > 8) break;     // more than this is unreadable
                }
            }

            telemetry.update();
            sleep(20);
        }

        portal.close();
    }

    /** Straight-line distance is rangeCm; this is the angle to turn. */
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

        boolean exposureChanged = false;
        if (up && !lastUp) {
            exposureMs++;
            exposureChanged = true;
        }
        if (down && !lastDown && exposureMs > 1) {
            exposureMs--;
            exposureChanged = true;
        }
        if (right && !lastRight) {
            gain += 10;
            exposureChanged = true;
        }
        if (left && !lastLeft && gain >= 10) {
            gain -= 10;
            exposureChanged = true;
        }
        if (exposureChanged) applyExposure();

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

    private void applyExposure() {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            }
            GainControl gainControl = portal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(gain);
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