package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.visionTools;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Limelight3A limelight;
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    CRServoImplEx leftKickerServo;
    ServoImplEx leftTurretServo;
    ServoImplEx rightTurretServo;
    CRServoImplEx rightKickerServo;
    DcMotorEx leftShooterMotor;
    DcMotorEx rightShooterMotor;
    ServoImplEx leftTipper;
    ServoImplEx rightTipper;
    CRServoImplEx leftBackRoller;
    CRServoImplEx rightBackRoller;
    ServoImplEx leftHood;
    ServoImplEx rightHood;
    GoBildaPinpointDriver pinpoint;

    AnalogInput kickerEncoder;
    boolean kickerShoot = false;
    boolean kickerRecycle = false;
    double kickerLocation;
    double closezone = 1;
    int kickerAction;
    double kickerRotationsLeft;
    boolean kickerInDefaultPosition;
    boolean dpadDownPressed = false;
    boolean recycleIntakeTimerStarted = false;
    boolean shooting;
    boolean yPressed = false;
    boolean purpleSortingEnabled = false;
    boolean greenSortingEnabled = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime recycleIntakeTimer = new ElapsedTime();
    AnalogInput turretEncoder;
    NormalizedColorSensor leftIntakeColorSensor;
    NormalizedColorSensor rightIntakeColorSensor;
    double totalCurrent;
    boolean tipped = false;
    visionTools vision = new visionTools();
    List currentBalls;
    public static double Kp = 0.007;
    public static double Ki = 0.0000;
    public static double Kd = 0.00;
    double currentSpeed = 0;
    boolean rightBumperTrue = false;
    boolean leftBumperTrue = false;
    int attempts = 0;
    int status = 0;
    ElapsedTime cycleTimer = new ElapsedTime();
    ServoImplEx light;
    ServoImplEx zoneLight;
    ElapsedTime runTime = new ElapsedTime();
    double lastLoopTime;
    double loops = 1;

    @Override

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(9);
        leftIntakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftIntakeColorSensor");
        rightIntakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightIntakeColorSensor");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);
        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        light = hardwareMap.get(ServoImplEx.class, "light");
        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");
        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");
        leftBackRoller = hardwareMap.get(CRServoImplEx.class, "leftBackRoller");
        rightBackRoller = hardwareMap.get(CRServoImplEx.class, "rightBackRoller");
        rightBackRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.setPollRateHz(100);
        limelight.start();
        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
        leftTurretServo.setPosition(0.5);
        rightTurretServo.setPosition(0.5);

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

        rightHood.setPosition(0.0);
        leftHood.setPosition(0.0);
    }

    @Override
    public void loop() {
        timer.reset();
        telemetry.addData("last loop time", runTime.milliseconds() - lastLoopTime);
        telemetry.addData("average loop time", runTime.milliseconds() / loops);
        telemetry.addData("loops", loops);
        loops = loops + 1;
        lastLoopTime = runTime.milliseconds();
//        telemetry.addData("frontIntakeMotorSpeed", frontIntakeMotor.getVelocity());
//        telemetry.addData("encoder voltage: ", kickerEncoder.getVoltage());
        telemetry.addData("tipped", tipped);
//        telemetry.addData("lf", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rf", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("lb", leftBackMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rb", rightBackMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("leftshooter", leftShooterMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("rightSHooter", rightShooterMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("frontIntake", frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("Kp", Kp);
//        telemetry.addData("Ki", Ki);
//        telemetry.addData("Kd", Kd);
//        telemetry.addData("# of balls: ", vision.ballsInRamp(limelight));
//        telemetry.addData("flywheel", leftShooterMotor.getVelocity());
//        telemetry.addData("inRange? ",vision.inRange(limelight));
//        telemetry.addData("distance", vision.distance(limelight));
//        telemetry.addData("ground distance", vision.groundDistance(limelight));
//        totalCurrent = (leftFrontMotor.getCurrent(CurrentUnit.AMPS) + rightFrontMotor.getCurrent(CurrentUnit.AMPS) + leftBackMotor.getCurrent(CurrentUnit.AMPS) + rightBackMotor.getCurrent(CurrentUnit.AMPS) + leftShooterMotor.getCurrent(CurrentUnit.AMPS) + rightShooterMotor.getCurrent(CurrentUnit.AMPS) + frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("totalcurrent", totalCurrent);
        if (kickerEncoder.getVoltage() > 1.65) {
            kickerLocation = kickerEncoder.getVoltage() - 1.65;
        } else {
            kickerLocation = kickerEncoder.getVoltage();
        }
        if (kickerLocation > 0.5) {
            kickerLocation = kickerLocation - 0.5;
        } else {
            kickerLocation = 1.65 - (kickerLocation - 0.5);
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
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int tagID = 0 ;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId(); // The ID number of the Apriltag
        }
        telemetry.addData("Tag ID",tagID);
        if (gamepad1.right_trigger > 0.1) {
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontIntakeMotor.setPower(Globals.frontIntakeIntakeSpeed);
            backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
            rightKickerServo.setPower(0);
            leftKickerServo.setPower(0);
        } else if (gamepad1.a) {
            frontIntakeMotor.setPower(Globals.frontIntakeReverseSpeed);
            backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
            leftBackRoller.setPower(Globals.backRollersReverse);
            rightBackRoller.setPower(Globals.backRollersReverse);

        } else if (gamepad1.left_trigger > 0.1) {
            if(rightBumperTrue) {
                rightKickerServo.setPower(Globals.kickerShoot * 0.3);
                leftKickerServo.setPower(Globals.kickerShoot * 0.3);
            }else{
                rightKickerServo.setPower(Globals.kickerShoot);
                leftKickerServo.setPower(Globals.kickerShoot);
            }
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
            backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
            shooting = true;
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
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooting = false;
            leftBackRoller.setPower(0);
            rightBackRoller.setPower(0);
            if (kickerRotationsLeft == 0) {
                if (recycleIntakeTimerStarted == false) {
                    recycleIntakeTimer.reset();
                    recycleIntakeTimerStarted = true;
                }
                if (recycleIntakeTimerStarted == true && recycleIntakeTimer.milliseconds() > 1000 && gamepad1.left_trigger < 0.1) {
                    frontIntakeMotor.setPower(0);
                    backIntakeMotor.setPower(0);
                }
                if (kickerLocation < Globals.defaultKickerLocation - 0.03) {
                    leftKickerServo.setPower(0.2 /* (kickerLocation - Globals.defaultKickerLocation)/ / (Globals.defaultKickerLocation - kickerEncoder.getVoltage())*/);
                    rightKickerServo.setPower(0.2);
                    telemetry.addLine("e");

                } else if (kickerLocation > Globals.defaultKickerLocation + 0.03) {
                    leftKickerServo.setPower(-0.08);
                    rightKickerServo.setPower(-0.08);
                    telemetry.addLine("ae");
                } else {
                    rightKickerServo.setPower(0);
                    leftKickerServo.setPower(0);
                }

            }
        }
        if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }
        if (gamepad1.b){
            if(closezone == 1){
                closezone = 3;
            }else{
                closezone = 1;
            }
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

        telemetry.addData("left", leftBumperTrue);
        telemetry.addData("right", rightBumperTrue);
        if (gamepad1.rightBumperWasReleased()) {
            if (rightBumperTrue) {
                rightBumperTrue = false;
            } else {
                rightBumperTrue = true;
            }
        }
        if (rightBumperTrue && !leftBumperTrue) {
            leftShooterMotor.setPower(Globals.defaultFarZonePower);
            rightShooterMotor.setPower(Globals.defaultFarZonePower);

            leftHood.setPosition(0.4);
            rightHood.setPosition(0.4);
            telemetry.addData("flywheel", leftShooterMotor.getVelocity());

        }
        if (rightBumperTrue && !leftBumperTrue) {
            double errorMargin = 0.5;
            double position = leftTurretServo.getPosition();

            telemetry.addData("Power", vision.TurretPower(limelight, errorMargin));

            leftTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight,2));
            rightTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight,2));
            //leftTurretServo.setPower(0.5);
        }
        //far zone shoot
        if (gamepad1.leftBumperWasReleased()) {
            if (leftBumperTrue) {
                leftBumperTrue = false;
                //currentSpeed = 0.67;
            } else {
                leftBumperTrue = true;
            }
        }
        if (leftBumperTrue && !rightBumperTrue) {
            String allianceColor = "Red";
            leftShooterMotor.setPower(vision.closeZoneFlywheelSpeed(limelight,currentSpeed,pinpoint,allianceColor));
            rightShooterMotor.setPower(vision.closeZoneFlywheelSpeed(limelight,currentSpeed,pinpoint,allianceColor));
            currentSpeed = vision.closeZoneFlywheelSpeed(limelight,currentSpeed,pinpoint,allianceColor);
            leftHood.setPosition(vision.closeZonehood(limelight,leftHood.getPosition(),pinpoint,allianceColor));
            rightHood.setPosition(vision.closeZonehood(limelight,leftHood.getPosition(),pinpoint,allianceColor));
            telemetry.addData("flywheel", leftShooterMotor.getVelocity());

        }
        if (!rightBumperTrue && !leftBumperTrue) {
            telemetry.addData("slowing down flywheel", 0);
            leftTurretServo.setPosition(0.5);
            rightTurretServo.setPosition(0.5);

            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);

        }
        if (leftBumperTrue && !rightBumperTrue) {
            double errorMargin = 0.5;
            double position = leftTurretServo.getPosition();
            telemetry.addData("Power", vision.TurretPower(limelight, errorMargin));

            leftTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight,closezone));
            rightTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight,closezone));
            //leftTurretServo.setPower(0.5);
        }
        //manual sort
        if (gamepad1.left_stick_button) {
            purpleSortingEnabled = true;
        }

        if (purpleSortingEnabled) {
            if(gamepad1.x){
                purpleSortingEnabled = false;
            }
            purpleSortingEnabled = vision.recycleToColor("Purple",leftIntakeColorSensor,rightIntakeColorSensor,frontIntakeMotor,backIntakeMotor,leftKickerServo,rightKickerServo,leftBackRoller,rightBackRoller,cycleTimer,kickerLocation, Globals.defaultKickerLocation,3,gamepad1.x);
        }

        if (gamepad1.right_stick_button) {
            greenSortingEnabled = true;
        }

        if (greenSortingEnabled) {
            if(gamepad1.x){
                greenSortingEnabled = false;
            }
            greenSortingEnabled = vision.recycleToColor("Green",leftIntakeColorSensor,rightIntakeColorSensor,frontIntakeMotor,backIntakeMotor,leftKickerServo,rightKickerServo,leftBackRoller,rightBackRoller,cycleTimer,kickerLocation, Globals.defaultKickerLocation,3,gamepad1.x);
        }

        // tipping
        if (!yPressed && gamepad1.y) {
            yPressed = true;
            if (!tipped) {
                leftTipper.setPosition(Globals.tipperExtended);
                rightTipper.setPosition(Globals.tipperExtended);
                tipped = true;
            } else {
                leftTipper.setPosition(Globals.tipperRetracted);
                rightTipper.setPosition(Globals.tipperRetracted);
                tipped = false;

            }
        } else {
            yPressed = false;
        }
        if (kickerLocation > Globals.defaultKickerLocation - 0.1 && kickerLocation < Globals.defaultKickerLocation + 0.1 && kickerInDefaultPosition == false && kickerRotationsLeft != 0) {
            kickerRotationsLeft = kickerRotationsLeft - 1;
            kickerInDefaultPosition = true;
//                if(shooting) {
//                    frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
//                    backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
//                }
        }
        if (kickerLocation < Globals.defaultKickerLocation - 0.1 || kickerLocation > Globals.defaultKickerLocation + 0.1) {
            kickerInDefaultPosition = false;
//                if(shooting) {
//                    frontIntakeMotor.setPower(0);
//                    backIntakeMotor.setPower(0);
//                }

//            if(kickerLocation > Globals.defaultKickerLocation - 0.3 && kickerLocation < Globals.defaultKickerLocation + 0.3 && gamepad1.left_trigger > 0.1){
//                frontIntakeMotor.setPower(0);
//                backIntakeMotor.setPower(0);
//                telemetry.addLine("whjtoewrewm");
//            }
//            else if(gamepad1.left_trigger > 0.1){
////                frontIntakeMotor.setPower(0);
////                backIntakeMotor.setPower(0);
//                frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
//                backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
//                telemetry.addLine("whjtoewrewm");
//            }
            telemetry.addData("shooting", shooting);
//            if (kickerLocation > Globals.defaultKickerLocation - 0.5 && kickerLocation < Globals.defaultKickerLocation + 0.1) {
//                if(shooting == true) {
//                    frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
//                    backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
//                }
//            }
//            if (kickerLocation < Globals.defaultKickerLocation - 0.1 || kickerLocation > Globals.defaultKickerLocation + 0.1) {
//                kickerInDefaultPosition = false;
////                if(shooting) {
////                    frontIntakeMotor.setPower(0);
////                    backIntakeMotor.setPower(0);
////                }
//            }
//            if((vision.inRange(limelight))||((vision.correctPos ==1)&&(rightBumperTrue))){
//                light.setPosition(0.333);
//            }
//            else if((vision.currentColor(leftIntakeColorSensor,rightIntakeColorSensor).equals("Green"))){
//                light.setPosition(0.5);
//            }
//            else if((vision.currentColor(leftIntakeColorSensor,rightIntakeColorSensor).equals("Purple"))){
//                light.setPosition(0.722);
//            }
//            else{
//                light.setPosition(0);
//            }
            if(tipped){
                zoneLight.setPosition(0.3);
            }

            else if(rightBumperTrue){
                zoneLight.setPosition(0.388);
            }
            else if(leftBumperTrue){
                zoneLight.setPosition(0.666);
            }else{
                zoneLight.setPosition(0);
            }
        }
        telemetry.addData("looptime", timer.milliseconds());
    }
}
