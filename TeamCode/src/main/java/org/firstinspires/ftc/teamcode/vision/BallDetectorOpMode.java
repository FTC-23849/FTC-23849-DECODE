package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Ball Detector", group = "Vision")
public class BallDetectorOpMode extends LinearOpMode {

    // must match the calibration resolution, before rotation
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 960;

    private static final long EXPOSURE_MS = 8;
    private static final int GAIN = 200;

    @Override
    public void runOpMode() {
        BallDetectorProcessor processor = new BallDetectorProcessor();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();

        telemetry.addLine("Waiting for camera...");
        telemetry.update();
        while (!isStopRequested() && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (isStopRequested()) {
            portal.close();
            return;
        }

        setManualExposure(portal, EXPOSURE_MS, GAIN);

        telemetry.addLine("Ready. A/B/X/Y switches the debug view.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                BallDetectorProcessor.debugView = BallDetectorProcessor.DebugView.FINAL;
            } else if (gamepad1.b) {
                BallDetectorProcessor.debugView = BallDetectorProcessor.DebugView.MASK;
            } else if (gamepad1.x) {
                BallDetectorProcessor.debugView = BallDetectorProcessor.DebugView.DISTANCE;
            } else if (gamepad1.y) {
                BallDetectorProcessor.debugView = BallDetectorProcessor.DebugView.SPLIT;
            }

            List<BallDetectorProcessor.Ball> balls = processor.getBalls();
            telemetry.addData("view", BallDetectorProcessor.debugView);
            telemetry.addData("scale radius", "%.1f", processor.getScaleRadius());
            telemetry.addData("balls", balls.size());

            for (int i = 0; i < balls.size(); i++) {
                BallDetectorProcessor.Ball ball = balls.get(i);
                if (ball.hasPosition) {
                    telemetry.addData("ball " + (i + 1),
                            "X %.1f  Y %.1f  Z %.1f  range %.1f cm  R %.1f px",
                            ball.xReal, ball.yReal, ball.zReal, ball.range, ball.radius);
                } else {
                    telemetry.addData("ball " + (i + 1), "no range  R %.1f px", ball.radius);
                }
            }
            telemetry.update();
            sleep(20);
        }

        portal.close();
    }

    private void setManualExposure(VisionPortal portal, long exposureMs, int gain) {
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
        }
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}