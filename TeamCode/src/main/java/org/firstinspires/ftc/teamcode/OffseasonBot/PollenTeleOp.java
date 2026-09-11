package org.firstinspires.ftc.teamcode.OffseasonBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class PollenTeleOp extends OpMode {

    DcMotorEx LF, RF, LB, RB;
    DcMotorEx intakeMotor;
    ServoImplEx left4Bar, right4Bar;

    public static double intakeUpPos = 0.76;
    public static double intakeDownPos = 1.0;

    boolean intakeDown = false;

    @Override
    public void init() {

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        left4Bar = hardwareMap.get(ServoImplEx.class, "left4Bar");
        right4Bar = hardwareMap.get(ServoImplEx.class, "right4Bar");
        left4Bar.setDirection(ServoImplEx.Direction.REVERSE);
        right4Bar.setDirection(ServoImplEx.Direction.REVERSE);

        left4Bar.setPosition(intakeUpPos);
        right4Bar.setPosition(intakeUpPos);

    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        LF.setPower(frontLeftPower);
        LB.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RB.setPower(backRightPower);

        if (gamepad1.rightBumperWasPressed()) {

            left4Bar.setPosition(intakeDownPos);
            right4Bar.setPosition(intakeDownPos);
            intakeDown = true;

        } else if (gamepad1.leftBumperWasPressed()) {

            left4Bar.setPosition(intakeUpPos);
            right4Bar.setPosition(intakeUpPos);
            intakeDown = false;

        }

        if (gamepad1.right_trigger > 0.2) {

            intakeMotor.setPower(1.0);
            if (!intakeDown) {

                left4Bar.setPosition(intakeDownPos);
                right4Bar.setPosition(intakeDownPos);
                intakeDown = true;

            }

        } else {

            intakeMotor.setPower(0.0);

        }

    }

}
