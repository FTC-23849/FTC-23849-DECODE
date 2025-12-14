package org.firstinspires.ftc.teamcode.RoadrunnerFiles.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadrunnerFiles.Drawing;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.TankDrive;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.visionTools;

@Config
@TeleOp
public class MT2PinpointLocalizationTest extends LinearOpMode {
    visionTools vision = new visionTools();
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class,"Limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                double[] MT2Values = vision.getMT2(limelight,pinpoint);
                double[] finalMT2 =  vision.adjustMT2Values(MT2Values[0],MT2Values[1],MT2Values[2],0,-0.07650, 0.0, 0.14281);
                Pose2d MT2pose = new Pose2d(finalMT2[0],finalMT2[1],finalMT2[2]);
                telemetry.addData("mx", MT2pose.position.x);
                telemetry.addData("my", MT2pose.position.y);
                telemetry.addData("mheading (deg)", Math.toDegrees(MT2pose.heading.toDouble()));
                telemetry.update();

                packet.fieldOverlay().setStroke("#1fb547");
                Drawing.drawRobot(packet.fieldOverlay(), MT2pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                double[] MT2Values = vision.getMT2(limelight,pinpoint);
               double[] finalMT2 =  vision.adjustMT2Values(MT2Values[0],MT2Values[1],MT2Values[2],0,-0.07650, 0.0, 0.14281);
                Pose2d MT2pose = new Pose2d(finalMT2[0],finalMT2[1],finalMT2[2]);
                telemetry.addData("mx", MT2pose.position.x);
                telemetry.addData("my", MT2pose.position.y);
                telemetry.addData("mheading (deg)", Math.toDegrees(MT2pose.heading.toDouble()));
                telemetry.update();

                packet.fieldOverlay().setStroke("#1fb547");
                Drawing.drawRobot(packet.fieldOverlay(), MT2pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
