package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

@TeleOp(name = "Mecanum Pinpoint Pos Lock")
public class PosLock extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private double kP = 4.0;
    private double kI = 0.0;
    private double kD = 0.5;

    private double integralX = 0;
    private double integralY = 0;
    private double lastErrorX = 0;
    private double lastErrorY = 0;

    private boolean posLockActive = false;
    private double lockX = 0;
    private double lockY = 0;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "LF");
        frontRight = hardwareMap.get(DcMotorEx.class, "RF");
        backLeft = hardwareMap.get(DcMotorEx.class, "LB");
        backRight = hardwareMap.get(DcMotorEx.class, "RB");
        pinpoint  = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.right_bumper && !posLockActive) {
            posLockActive = true;
            lockX = pinpoint.getPosX(DistanceUnit.METER);
            lockY = pinpoint.getPosY(DistanceUnit.METER);
            integralX = 0;
            integralY = 0;
            lastErrorX = 0;
            lastErrorY = 0;
        } else if (!gamepad1.right_bumper) {
            posLockActive = false;
        }

        if (posLockActive) {
            double errorX = lockX - pinpoint.getPosX(DistanceUnit.METER);
            double errorY = lockY - pinpoint.getPosY(DistanceUnit.METER);

            integralX += errorX * 0.02;
            integralY += errorY * 0.02;

            double derivativeX = (errorX - lastErrorX) / 0.02;
            double derivativeY = (errorY - lastErrorY) / 0.02;

            x = kP * errorX + kI * integralX + kD * derivativeX;
            y = kP * errorY + kI * integralY + kD * derivativeY;

            lastErrorX = errorX;
            lastErrorY = errorY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}
