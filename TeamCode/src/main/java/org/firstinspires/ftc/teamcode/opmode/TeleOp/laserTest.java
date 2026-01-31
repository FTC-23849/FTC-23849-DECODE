package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class laserTest extends OpMode {

    DigitalChannel bottomLeftLaser;
    DigitalChannel bottomRightLaser;

    ServoImplEx zoneLight;

    boolean thirdBallPresent;

    @Override
    public void init() {

        bottomLeftLaser = hardwareMap.get(DigitalChannel.class, "bottomLeftLaser");
        bottomRightLaser = hardwareMap.get(DigitalChannel.class, "bottomRightLaser");

        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");

        bottomLeftLaser.setMode(DigitalChannel.Mode.INPUT);
        bottomRightLaser.setMode(DigitalChannel.Mode.INPUT);

    }

    public void loop() {

        thirdBallPresent = bottomLeftLaser.getState() || bottomRightLaser.getState();

        if (thirdBallPresent) {
            zoneLight.setPosition(1.0);
        } else {
            zoneLight.setPosition(0.0);
        }

        telemetry.addData("thirdball?: ", thirdBallPresent);
        telemetry.addData("bottom left:", bottomLeftLaser.getState());
        telemetry.addData("bottom right", bottomRightLaser.getState());
        telemetry.update();

    }

}
