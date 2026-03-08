package org.firstinspires.ftc.teamcode.hardwaretuning.motor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.HolonomicController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class motorSpeedTester extends OpMode {

    DcMotorEx motor;

    double velocity;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "LF");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }

    @Override
    public void loop() {

        motor.setPower(1.0);
        velocity = (motor.getVelocity() * 60) / 537.6;
        telemetry.addData("velocity", velocity);

    }
}
