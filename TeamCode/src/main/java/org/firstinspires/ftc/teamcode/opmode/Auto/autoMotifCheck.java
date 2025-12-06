package org.firstinspires.ftc.teamcode.opmode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.vision.visionTools;


@Autonomous
public class autoMotifCheck extends LinearOpMode {

    Limelight3A limelight;

    visionTools vision = new visionTools();

    int obeliskID = -1;

    @Override
    public void runOpMode() {

        //map motors and servos
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);

        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new getObeliskID());


    }

    public class getObeliskID implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            obeliskID = vision.ObeliskID(limelight);
            telemetry.addData("Obelisk ID: ", obeliskID);
            telemetry.update();

            return true;

        }
    }

}