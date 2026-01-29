package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.opmode.misc.BlobDetector;

@Autonomous
public class CameraTest extends OpMode {

    private BlobDetector detector;

    @Override
    public void init() {

        detector = new BlobDetector(
                hardwareMap,
                "Webcam 1",
                640,
                480,
                135.0 / 346.0,
                26,
                "Red"
        );

        detector.start();
    }

    @Override
    public void loop() {
        telemetry.addData("offset", detector.getOffsetCm());
    }

    @Override
    public void stop() {
        detector.stop();
    }
}
