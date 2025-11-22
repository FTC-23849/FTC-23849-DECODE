package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.visionTools;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

@Config
@TeleOp
public class BasicTeleop extends OpMode {
    Limelight3A limelight;
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    CRServoImplEx leftKickerServo;
    CRServoImplEx leftTurretServo;
    CRServoImplEx rightTurretServo;
    CRServoImplEx rightKickerServo;
    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;
    ServoImplEx leftTipper;
    ServoImplEx rightTipper;
    CRServoImplEx leftBackRoller;
    CRServoImplEx rightBackRoller;
    AnalogInput kickerEncoder;
    boolean kickerShoot = false;
    boolean kickerRecycle = false;
    double kickerLocation;
    int kickerAction;
    double kickerRotationsLeft;
    boolean kickerInDefaultPosition;
    boolean dpadDownPressed = false;
    boolean recycleIntakeTimerStarted = false;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime recycleIntakeTimer = new ElapsedTime();
    AnalogInput turretEncoder;
    NormalizedColorSensor leftIntakeColorSensor;
    NormalizedColorSensor rightIntakeColorSensor;
    double totalCurrent;
    boolean tipped = false;
    visionTools vision = new visionTools();
    List currentBalls ;
    public static double Kp = 0.007;
    public static double Ki = 0.0000;
    public static double Kd = 0.00;
    boolean rightBumperTrue = false;
    boolean leftBumperTrue = false;
    @Override

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(9);
        leftIntakeColorSensor = hardwareMap.get(NormalizedColorSensor.class,"leftIntakeColorSensor");
        rightIntakeColorSensor = hardwareMap.get(NormalizedColorSensor.class,"rightIntakeColorSensor");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
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
        leftTurretServo = hardwareMap.get(CRServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServoImplEx.class, "rightTurretServo");
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
        limelight.setPollRateHz(100);
        limelight.start();
        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
    }

    @Override
    public void loop() {
        timer.reset();
        telemetry.addData("tipped", tipped);
        telemetry.addData("lf", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rf", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lb", leftBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rb", rightBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("leftshooter", leftShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightSHooter", rightShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("frontIntake", frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("# of balls: ", vision.ballsInRamp(limelight));
        telemetry.addData("flywheel",leftShooterMotor.getVelocity());
        telemetry.update();
        totalCurrent = (leftFrontMotor.getCurrent(CurrentUnit.AMPS) + rightFrontMotor.getCurrent(CurrentUnit.AMPS) + leftBackMotor.getCurrent(CurrentUnit.AMPS) + rightBackMotor.getCurrent(CurrentUnit.AMPS) + leftShooterMotor.getCurrent(CurrentUnit.AMPS) + rightShooterMotor.getCurrent(CurrentUnit.AMPS) + frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("totalcurrent", totalCurrent);
        if (kickerEncoder.getVoltage() > 1.65) {
            kickerLocation = kickerEncoder.getVoltage() - 1.65;
        } else {
            kickerLocation = kickerEncoder.getVoltage();
        }
        if (kickerLocation > 0.7) {
            kickerLocation = kickerLocation - 0.7;
        } else {
            kickerLocation = 1.65 - (kickerLocation - 0.7);
        }


        double y = -gamepad1.left_stick_y; // Remember,    Y stick value is reversed
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

        if (gamepad1.right_trigger > 0.1) {
            frontIntakeMotor.setPower(Globals.frontIntakeIntakeSpeed);
            backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
        } else if (gamepad1.a) {
            frontIntakeMotor.setPower(Globals.frontIntakeReverseSpeed);
            backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
            leftBackRoller.setPower(Globals.backRollersReverse);
            rightBackRoller.setPower(Globals.backRollersReverse);

        } else if (gamepad1.left_trigger > 0.1) {
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
            frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
            backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
        } else if (gamepad1.dpad_up) {
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
        } else if (gamepad1.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            //recycleintaketimer is for turning intake after kicker reaches position so next ball is in right position
            //recycleintaketimerstarted is for only starting it once
            recycleIntakeTimerStarted = false;
            //kickerrotationsleft is how many balls to recycle, code at bottom for decreasing that number
            kickerAction = 3;
            kickerRotationsLeft = 1 + kickerRotationsLeft;
            //checks if in default position; rotationsleft is decrased by one when kicker is in this position, dont want to double count
            if (kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1) {
                kickerInDefaultPosition = true;
                telemetry.addLine("eewewwe");
            } else {
                kickerInDefaultPosition = false;
            }
            leftKickerServo.setPower(Globals.kickerRecycle);
            rightKickerServo.setPower(Globals.kickerRecycle);
            frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed);
            backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed);
        } else if (gamepad1.dpad_left) {
            rightKickerServo.setPower(Globals.kickerShoot);
            leftKickerServo.setPower(Globals.kickerShoot);
        } else if (gamepad1.dpad_right) {
            rightKickerServo.setPower(Globals.kickerRecycle);
            leftKickerServo.setPower(Globals.kickerRecycle);
        } else {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            if (kickerRotationsLeft == 0) {
                if (recycleIntakeTimerStarted == false) {
                    recycleIntakeTimer.reset();
                    recycleIntakeTimerStarted = true;
                }
                if (recycleIntakeTimerStarted == true && recycleIntakeTimer.milliseconds() > 1000) {
                    frontIntakeMotor.setPower(0);
                    backIntakeMotor.setPower(0);
                }
                if (kickerLocation < Globals.defaultKickerLocation - 0.05) {
                    leftKickerServo.setPower(0.09 /* (kickerLocation - Globals.defaultKickerLocation)/ / (Globals.defaultKickerLocation - kickerEncoder.getVoltage())*/);
                    rightKickerServo.setPower(0.09);
                    telemetry.addLine("e");

                } else if (kickerLocation > Globals.defaultKickerLocation + 0.05) {
                    leftKickerServo.setPower(-0.09);
                    rightKickerServo.setPower(-0.09);
                    telemetry.addLine("ae");
                } else {
                    rightKickerServo.setPower(0);
                    leftKickerServo.setPower(0);

                }

            }
            if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            if (kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1 && kickerInDefaultPosition == false && kickerRotationsLeft != 0) {
                kickerRotationsLeft = kickerRotationsLeft - 1;
                kickerInDefaultPosition = true;
            }
            if (kickerLocation < Globals.defaultKickerLocation - 0.1 || kickerLocation > Globals.defaultKickerLocation + 0.1) {
                kickerInDefaultPosition = false;
            }
            LLResult results = limelight.getLatestResult();
            /*if (results.isValid() && (results != null)) {
                double errorMargin = 0.5;
                leftTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
                rightTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
            }*/
            //close zone shoot

            telemetry.addData("left",leftBumperTrue);
            telemetry.addData("right",rightBumperTrue);
            if (gamepad1.rightBumperWasReleased()) {
                if (rightBumperTrue) {
                    rightBumperTrue = false;
                } else {
                    rightBumperTrue = true;
                }
            }
            if(rightBumperTrue){
                leftShooterMotor.setPower(Globals.defaultFarZonePower);
            rightShooterMotor.setPower(Globals.defaultFarZonePower);
            telemetry.addData("flywheel",leftShooterMotor.getVelocity());
            telemetry.update();
            } else {
            telemetry.addData("slowing down far zone",0);
            telemetry.update();
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
            }
            if (rightBumperTrue) {
                double errorMargin = 0.5;
                double position = turretEncoder.getVoltage() / 3.2 * 360;

                telemetry.addData("Power", vision.TurretPower(limelight, errorMargin));
                telemetry.update();
                leftTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
                rightTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
                //leftTurretServo.setPower(0.5);
            }
            //far zone shoot
            if (gamepad1.leftBumperWasReleased()) {
                if (leftBumperTrue) {
                    leftBumperTrue = false;
                } else {
                    leftBumperTrue = true;
                }
            }
            if(leftBumperTrue) {
            leftShooterMotor.setPower(Globals.defaultCloseZonePower);
            rightShooterMotor.setPower(Globals.defaultCloseZonePower);
            telemetry.addData("flywheel",leftShooterMotor.getVelocity());
            telemetry.update();
            } else {
            telemetry.addData("slowing down far zone",0);
            telemetry.update();
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);

            }
            if (leftBumperTrue) {
                double errorMargin = 0.5;
                double position = turretEncoder.getVoltage() / 3.2 * 360;

                telemetry.addData("Power", vision.TurretPower(limelight, errorMargin));
                telemetry.update();
                leftTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
                rightTurretServo.setPower(vision.TurretPowerPID(limelight, errorMargin, Kp, Ki, Kd));
                //leftTurretServo.setPower(0.5);
            }
            //manual sort
            if (gamepad1.left_stick_button) {
                vision.recycleToColor(1, leftKickerServo, rightKickerServo, frontIntakeMotor, backIntakeMotor, leftBackRoller,
                        rightBackRoller, leftIntakeColorSensor, rightIntakeColorSensor);
                   /* String color = vision.currentColor(leftIntakeColorSensor, rightIntakeColorSensor);
                    telemetry.addData("color",color);
                    telemetry.update();
                    if( color == "Purple"){

                    }else {
                        //rotate servo
                        kickerRotationsLeft = 1;
                        if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1){
                            kickerInDefaultPosition = true;
                        }
                        else{
                            rightKickerServo.setPower(Globals.kickerRecycle);
                            leftKickerServo.setPower(Globals.kickerRecycle);
                        }
                        frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
                        leftBackRoller.setPower(Globals.backRollersMaxPower);
                        rightBackRoller.setPower(Globals.backRollersMaxPower);
                        backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
                    }*/

            }
            if (gamepad1.right_stick_button) {
                vision.recycleToColor(0, leftKickerServo, rightKickerServo, frontIntakeMotor, backIntakeMotor, leftBackRoller,
                        rightBackRoller, leftIntakeColorSensor, rightIntakeColorSensor);
                /*timer.reset();
                String color = vision.currentColor(leftIntakeColorSensor, rightIntakeColorSensor);
                telemetry.addData("color",color);
                telemetry.update();

                if( color == "Green"){

                }else {
                    if(timer.milliseconds() > 100){
                    //rotate servo
                    timer.reset();
                    kickerRotationsLeft = 1;
                    if(kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1){
                        kickerInDefaultPosition = true;
                    }
                    else{
                        rightKickerServo.setPower(Globals.kickerRecycle);
                        leftKickerServo.setPower(Globals.kickerRecycle);
                    }
                    frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
                    leftBackRoller.setPower(Globals.backRollersMaxPower);
                    rightBackRoller.setPower(Globals.backRollersMaxPower);
                    backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
                }}*/

            }
            // tipping
            if (!gamepad1.yWasPressed() && gamepad1.y) {
                if (!tipped) {
                    leftTipper.setPosition(Globals.tipperExtended);
                    rightTipper.setPosition(Globals.tipperExtended);
                    tipped = true;
                } else {
                    leftTipper.setPosition(Globals.tipperRetracted);
                    rightTipper.setPosition(Globals.tipperRetracted);
                    tipped = false;

                }
            }
            if (kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1 && kickerInDefaultPosition == false && kickerRotationsLeft != 0) {
                kickerRotationsLeft = kickerRotationsLeft - 1;
                kickerInDefaultPosition = true;
            }
            if (kickerLocation < Globals.defaultKickerLocation - 0.1 || kickerLocation > Globals.defaultKickerLocation + 0.1) {
                kickerInDefaultPosition = false;
            }
        }
    }
}