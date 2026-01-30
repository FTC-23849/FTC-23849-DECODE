package org.firstinspires.ftc.teamcode.opmode.Auto.AutoArchive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.visionTools;

@Disabled
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