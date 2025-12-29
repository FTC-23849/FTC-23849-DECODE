package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation. YawPitchRollAngles;

@Autonomous
public class LimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    visionTools vision = new visionTools();
    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight.setPollRateHz(20);
    }

    @Override
    public void start() {
        limelight.start();
    }



    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        //telemetry.addData("filled ramp?", vision.RampIsFull(limelight));
        telemetry.addData("null",llResult != null);
        telemetry.addData("valid",llResult.isValid());
        telemetry.addData("both",llResult != null && llResult.isValid());
        double[] pythonOutputs = llResult.getPythonOutput();
        telemetry.addData("Tx",pythonOutputs[0]);
        telemetry.addData("Ty",pythonOutputs[1]);
        telemetry.addData("Tz",pythonOutputs[2]);
        telemetry.addData("Ta",pythonOutputs[3]);
        telemetry.addData("Tb",pythonOutputs[4]);

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx",llResult.getPythonOutput()[2]);

        }
    }
}