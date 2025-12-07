package org.firstinspires.ftc.teamcode.opmode.misc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class distanceTest extends OpMode {
    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;
    ServoImplEx leftTurretServo;
    ServoImplEx rightTurretServo;
    ServoImplEx leftHood;
    ServoImplEx rightHood;

    public static double speed = 0;
    public static double height = 0.4;
    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        leftHood.setPosition(height);
        rightHood.setPosition(height);
        //leftTurretServo.setPosition(0.5);
        //rightTurretServo.setPosition(0.5);
        telemetry.addData("target speed", speed);
        telemetry.addData("hood height", height);
        telemetry.addData("current speed left", leftShooterMotor.getVelocity()/11200);
        telemetry.addData("current speed right", rightShooterMotor.getVelocity()/11200);
        telemetry.update();
        leftShooterMotor.setPower(speed);
        rightShooterMotor.setPower(speed);

    }
}
