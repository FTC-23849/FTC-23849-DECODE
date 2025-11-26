package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;


@TeleOp
public class BasicTeleOp extends OpMode {

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
    AnalogInput kickerEncoder;
    boolean kickerShoot = false;
    boolean kickerRecycle = false;
    double kickerLocation;
    ElapsedTime recycleIntakeTimer = new ElapsedTime();
    boolean recycleIntakeTimerStarted = false;
    int kickerAction;
    int kickerRotationsLeft;
    boolean kickerInDefaultPosition;
    int loops = 1;
    double closeZonePower;
    double farZonePower;
    boolean dpadDownPressed = false;
    ElapsedTime runTime = new ElapsedTime();
    @Override

    public void init() {
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
//        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontMotor.setDirection((DcMotorEx.Direction.REVERSE));
        //rightBackMotor.setDirection((DcMotorSimple.Direction.REVERSE));

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
        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
    }

    @Override
    public void loop() {
        closeZonePower = Globals.defaultCloseZonePower;
        farZonePower = Globals.defaultFarZonePower;
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
        telemetry.addData("KickerEncoder", kickerEncoder.getVoltage());
        telemetry.addData("kickerSHoot", kickerShoot);
        telemetry.addData("kicker power", ((kickerEncoder.getVoltage()-Globals.defaultKickerLocation)));
        telemetry.addData("kicker location", kickerLocation);
        telemetry.addData("loop time", runTime.milliseconds() / loops);
        loops = loops + 1;
        telemetry.addData("kicker rotations left", kickerRotationsLeft);
        telemetry.addData("kicker in default position", kickerInDefaultPosition);
        telemetry.addData(("dapaddownpressed"), dpadDownPressed);
        telemetry.addData("dumb dpad down pressed", gamepad1.dpadDownWasPressed());

        if(kickerEncoder.getVoltage() > 1.65){
            kickerLocation = kickerEncoder.getVoltage() - 1.65;
        }
        else{
            kickerLocation = kickerEncoder.getVoltage();
        }
        if(kickerLocation > 0.7){
            kickerLocation = kickerLocation - 0.7;
        }
        else{
            kickerLocation = 1.65 - (kickerLocation - 0.7);
        }



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
            kickerShoot = true;
            kickerRecycle = false;
        }
        else if(gamepad1.dpad_up){
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
        }
        /*else if(gamepad1.dpad_down){

            rightKickerServo.setPower(Globals.kickerRecycle);
            leftKickerServo.setPower(Globals.kickerRecycle);
            frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed);
            backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed);
            kickerRecycle = true;
            kickerShoot = false;
        }*/
        else if(gamepad1.dpad_down && !dpadDownPressed){
            dpadDownPressed = true;
            //recycleintaketimer is for turning intake after kicker reaches position so next ball is in right position
            //recycleintaketimerstarted is for only starting it once
            recycleIntakeTimerStarted = false;
            //kickerrotationsleft is how many balls to recycle, code at bottom for decreasing that number
            kickerAction = 3;
            kickerRotationsLeft = 1 + kickerRotationsLeft ;
            //checks if in default position; rotationsleft is decrased by one when kicker is in this position, dont want to double count
            if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1){
                kickerInDefaultPosition = true;
                telemetry.addLine("eewewwe");
            }
            else{
                kickerInDefaultPosition = false;
            }
            leftKickerServo.setPower(Globals.kickerRecycle);
            rightKickerServo.setPower(Globals.kickerRecycle);
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
            leftBackRoller.setPower(0);
            rightBackRoller.setPower(0);

            // going to default position
            if(kickerRotationsLeft == 0){
                if(recycleIntakeTimerStarted == false){
                    recycleIntakeTimer.reset();
                    recycleIntakeTimerStarted = true;
                }
                if(recycleIntakeTimerStarted == true && recycleIntakeTimer.milliseconds() > 1000){
                    frontIntakeMotor.setPower(0);
                    backIntakeMotor.setPower(0);
                }
                if(kickerLocation < Globals.defaultKickerLocation - 0.05){
                    leftKickerServo.setPower(0.09 /* (kickerLocation - Globals.defaultKickerLocation)*/ /* *(Globals.defaultKickerLocation - kickerEncoder.getVoltage())*/);
                    rightKickerServo.setPower(0.09);
                    telemetry.addLine("e");

                }
                else if(kickerLocation > Globals.defaultKickerLocation + 0.05){
                    leftKickerServo.setPower(-0.09);
                    rightKickerServo.setPower(-0.09);
                    telemetry.addLine("ae");
                }
                else{
                    rightKickerServo.setPower(0);
                    leftKickerServo.setPower(0);

                }
            }
            if(!gamepad1.dpad_down){
                dpadDownPressed = false;
            }
//            leftKickerServo.setPower(0.05*(kickerEncoder.getVoltage()-Globals.defaultKickerLocation));
//            rightKickerServo.setPower(0.05*(kickerEncoder.getVoltage()-Globals.defaultKickerLocation));
//            if(kickerShoot && kickerLocation < Globals.defaultKickerLocation - 0.1){
//                leftKickerServo.setPower(0.09 /* (kickerLocation - Globals.defaultKickerLocation)*/ /* *(Globals.defaultKickerLocation - kickerEncoder.getVoltage())*/);
//                rightKickerServo.setPower(0.09);
//                telemetry.addLine("e");
//
//            }
//            else if(kickerShoot && kickerLocation > Globals.defaultKickerLocation + 0.1){
//                leftKickerServo.setPower(-0.09);
//                rightKickerServo.setPower(-0.09);
//                telemetry.addLine("ae");
//            }
//            else{
//                rightKickerServo.setPower(0);
//                leftKickerServo.setPower(0);
//
//            }
        }
        //close zone shoot
        if(gamepad1.left_bumper){
            leftShooterMotor.setPower(closeZonePower);
            rightShooterMotor.setPower(closeZonePower);
        } else if (gamepad1.right_bumper) {

            leftShooterMotor.setPower(farZonePower);
            rightShooterMotor.setPower(farZonePower);
        } else {
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
//        // shoot 1 ball
//        if(gamepad2.b){
//            kickerTimer.reset();
//            kickerAction = 2;
//            leftKickerServo.setPower(-0.09);
//            rightKickerServo.setPower(-0.09);
//        }
//        //recycle 1 ball
//        if(gamepad2.a){
//            kickerTimer.reset();
//            kickerAction = 3;
//            leftKickerServo.setPower(0.09);
//            rightKickerServo.setPower(0.09);
//
//        }
//        if(kickerAction == 2 && kickerTimer.milliseconds() > 50){
//            if(kickerLocation < Globals.defaultKickerLocation) {
//
//            }
//            else{
//                if(kickerRotationsLeft != 0)
//                kickerAction = 1;
//            }
//
//        }
//        else if(kickerAction == 3 && kickerTimer.milliseconds() > 50) {
//            if(kickerLocation < Globals.defaultKickerLocation){
//
//            }
//            kickerAction = 1;
//
//        }
//        else if(kickerAction == 1){
//            leftKickerServo.setPower(0.0);
//            rightKickerServo.setPower(0.0);
//        }

        if(gamepad2.a){
            kickerAction = 3;
            kickerRotationsLeft = 2;
            if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1){
                kickerInDefaultPosition = true;
                telemetry.addLine("eewewwe");
            }
            else{
                kickerInDefaultPosition = false;
            }
            leftKickerServo.setPower(Globals.kickerRecycle);
            rightKickerServo.setPower(Globals.kickerRecycle);

        }
        if (gamepad2.b) {
            kickerAction = 2;
            kickerRotationsLeft = 2;
            if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.01){
                kickerInDefaultPosition = true;
            }
            else{
                kickerInDefaultPosition = false;
            }
            leftKickerServo.setPower(Globals.kickerShoot);
            rightKickerServo.setPower(Globals.kickerShoot);
        }
        // decreases number of rotations left
        if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1 && kickerInDefaultPosition == false && kickerRotationsLeft != 0){
            kickerRotationsLeft = kickerRotationsLeft - 1;
            kickerInDefaultPosition = true;
        }
        if(kickerLocation < Globals.defaultKickerLocation - 0.1 || kickerLocation > Globals.defaultKickerLocation + 0.1){
            kickerInDefaultPosition = false;
        }
//        if(kickerRotationsLeft == 0){
//            leftKickerServo.setPower(0);
//            rightKickerServo.setPower(0);
//        }





    }


}
