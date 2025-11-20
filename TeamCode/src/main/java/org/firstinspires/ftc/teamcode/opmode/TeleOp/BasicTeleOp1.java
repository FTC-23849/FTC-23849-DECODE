package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import java.util.List;


@TeleOp
public class BasicTeleOp1 extends OpMode {

    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    CRServoImplEx leftKickerServo;
    CRServoImplEx rightKickerServo;
    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;
    ServoImplEx leftTipper;
    ServoImplEx rightTipper;
    CRServoImplEx leftBackRoller;
    CRServoImplEx rightBackRoller;
    double totalCurrent;
    boolean tipped = false;
    ElapsedTime timer = new ElapsedTime();
    @Override

    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);
        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");
        leftBackRoller = hardwareMap.get(CRServoImplEx.class, "leftBackRoller");
        rightBackRoller = hardwareMap.get(CRServoImplEx.class, "rightBackRoller");

        rightBackRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        timer.reset();
        telemetry.addData("tipped", tipped);
        telemetry.addData("lf", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rf", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lb", leftBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rb", rightBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("leftshooter", leftShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightSHooter", rightShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("frontIntake", frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
        totalCurrent =(leftFrontMotor.getCurrent(CurrentUnit.AMPS)+rightFrontMotor.getCurrent(CurrentUnit.AMPS)+leftBackMotor.getCurrent(CurrentUnit.AMPS)+rightBackMotor.getCurrent(CurrentUnit.AMPS)+leftShooterMotor.getCurrent(CurrentUnit.AMPS)+rightShooterMotor.getCurrent(CurrentUnit.AMPS)+frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("totalcurrent", totalCurrent);



        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

        if(gamepad1.right_trigger > 0.1){
            frontIntakeMotor.setPower(Globals.frontIntakeIntakeSpeed);
            backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
        }
        else if(gamepad1.a){
            frontIntakeMotor.setPower(Globals.frontIntakeReverseSpeed);
            backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
            leftBackRoller.setPower(Globals.backRollersReverse);
            rightBackRoller.setPower(Globals.backRollersReverse);

        }
        else if(gamepad1.left_trigger > 0.1){
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
            frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
            backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
        }
        else if(gamepad1.dpad_up){
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
        }
        else if(gamepad1.dpad_down){
            rightKickerServo.setPower(Globals.kickerRecycle);
            leftKickerServo.setPower(Globals.kickerRecycle);
            frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed);
            backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed);
        }
        else if(gamepad1.dpad_left){
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
        }
        else if(gamepad1.dpad_right){
            rightKickerServo.setPower(Globals.kickerRecycle);
            leftKickerServo.setPower(Globals.kickerRecycle);
        }
        else{
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            rightKickerServo.setPower(0);
            leftKickerServo.setPower(0);
            leftBackRoller.setPower(0);
            rightBackRoller.setPower(0);
        }
        //close zone shoot
        if(gamepad1.left_bumper){
            leftShooterMotor.setPower(Globals.defaultCloseZonePower);
            rightShooterMotor.setPower(Globals.defaultCloseZonePower);
        }
        else{
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }
        // tipping
        if(!gamepad1.yWasPressed()&&gamepad1.y){
            if(!tipped){
                leftTipper.setPosition(Globals.tipperExtended);
                rightTipper.setPosition(Globals.tipperExtended);
                tipped = true;
            }
            else {
                leftTipper.setPosition(Globals.tipperRetracted);
                rightTipper.setPosition(Globals.tipperRetracted);
                tipped = false;
            }
        }
        telemetry.addData("timer",timer.milliseconds());


    }

}
