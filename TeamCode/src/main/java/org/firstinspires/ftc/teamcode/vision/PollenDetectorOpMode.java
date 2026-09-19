package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Pollen Detector", group = "Vision")
public class PollenDetectorOpMode extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String WEBCAM_NAME = "Webcam 1";

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    private static final boolean LOCK_EXPOSURE = true;
    private static final long EXPOSURE_MS = 6;
    private static final int GAIN = 250;

    private PollenDetector detector;
    private VisionPortal portal;

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
            lockExposure();
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<PollenDetector.Ball> balls = detector.getBalls();

            telemetry.addData("Balls", balls.size());

            PollenDetector.Ball closest = null;
            for (PollenDetector.Ball ball : balls) {
                if (closest == null || ball.zReal < closest.zReal) closest = ball;
            }

            if (closest != null) {
                telemetry.addData("Closest", "%.1f cm, bearing %.1f deg",
                        closest.zReal, Math.toDegrees(Math.atan2(closest.xReal, closest.zReal)));
                telemetry.addData("Closest px", "(%.0f, %.0f) r=%.1f",
                        closest.x, closest.y, closest.radius);
            }

            int index = 1;
            for (PollenDetector.Ball ball : balls) {
                telemetry.addData("Ball " + index, "r=%.1f  X=%.1f Y=%.1f Z=%.1f",
                        ball.radius, ball.xReal, ball.yReal, ball.zReal);
                index++;
                if (index > 8) break;
            }

            telemetry.update();
            sleep(50);
        }

        portal.close();
    }

    private void lockExposure() {
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure != null) {
                if (exposure.getMode() != ExposureControl.Mode.Manual) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                sleep(20);
            }

            GainControl gain = portal.getCameraControl(GainControl.class);
            if (gain != null) {
                gain.setGain(GAIN);
            }
        } catch (Exception e) {
            telemetry.addLine("Exposure lock failed: " + e.getMessage());
        }
    }
}