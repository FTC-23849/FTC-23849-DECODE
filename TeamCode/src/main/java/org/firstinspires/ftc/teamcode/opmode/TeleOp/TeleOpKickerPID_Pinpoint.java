package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.graphics.Color;

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
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpKickerPID_Pinpoint extends OpMode {
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
    private NormalizedColorSensor colorLeft;
    private NormalizedColorSensor colorRight;
    private Servo rgbLight;

    double closezone = 1;

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

    double encoderVoltage;
    double processedEncoderValue;
    boolean kickerPIDEnabled;

    double kickerTarget = Globals.KICKER_IDLE;

    double plainKickerPower = 0.0;

    // recycling variables

    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;

    // Total time to run the recycle sequence (ms)
    double totalMs = recyclingDelay;
    // Delay between commanding KICKER_RECYCLE and starting intake (ms)
    double intakeDelayMs = recyclingIntakeDelay;
    // Delay between intake starting and kicker going back up (ms)
    double kickerUpDelayMs = recyclingKickerUpDelay;

    ElapsedTime recyclerTimer = new ElapsedTime();
    boolean started = false;
    boolean intakeStarted = false;
    boolean recyclerIsRunning = false;

    // Stall

    boolean wasShootingOpenLoop = false;
    double openLoopStartTimeMs = 0.0;
    double stallGraceMs = 300; // how long after starting open-loop before we detect stall


    // Stall detection state
    ElapsedTime stallTimer = new ElapsedTime();
    boolean stallSampleValid = false;
    double stallSampleTimeMs = 0.0;
    double stallSamplePos = 0.0;

    boolean recoveringFromStall = false;
    double lastShootPower = 0.0; // remember what power we were shooting with
    double recoveryStartTimeMs = 0.0;

    double stallWindowMs   = 200;   // how long we wait to see movement
    double stallMinDelta   = 0.1;  // minimum encoder change to consider "moving"
    double stallRecoveryMs = 1000;   // how long to hold in IDLE before resuming shot

    // Kicker

    public static double kickerKP = Globals.KICKER_kP;
    public static double kickerKD = Globals.KICKER_kD;

    // Color Detection

    // ---------- TUNABLES ----------
    public static float MIN_SATURATION   = 0.02f;   // only reject if BOTH sat & val are below these
    public static float MIN_VALUE        = 0.005f;

    public static float GREEN_HUE_CENTER  = 150f;   // adjust if your green reads ~100–140
    public static float PURPLE_HUE_CENTER = 225f;

    public static float COLOR_SENSOR_GAIN = 1f;     // try 2–8 if readings are tiny

    // goBILDA RGB light servo positions
    public static double LED_OFF_POS    = 0.00;
    public static double LED_GREEN_POS  = 0.50;
    public static double LED_PURPLE_POS = 0.70;
    // ------------------------------

    private enum ArtifactColor { GREEN, PURPLE, UNKNOWN }

    private static class Reading {
        ArtifactColor color;
        float hue, sat, val;
        float conf;   // confidence = s * v
        Reading(ArtifactColor c, float h, float s, float v) {
            color = c; hue = h; sat = s; val = v; conf = s * v;
        }
    }


    CRAxonPDController kickerPID = new CRAxonPDController();
    DroidForceMethods DFM = new DroidForceMethods();

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
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        rgbLight = hardwareMap.get(ServoImplEx.class, "light");
        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");
        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");
        leftBackRoller = hardwareMap.get(CRServoImplEx.class, "leftBackRoller");
        rightBackRoller = hardwareMap.get(CRServoImplEx.class, "rightBackRoller");
        rightBackRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.setPollRateHz(100);
        limelight.start();
        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        leftTurretServo.setPosition(0.5);
        rightTurretServo.setPosition(0.5);



        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        colorLeft  = hardwareMap.get(NormalizedColorSensor.class, "leftIntakeColorSensor");
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "rightIntakeColorSensor");

        kickerTarget = Globals.KICKER_IDLE;

        kickerPIDEnabled = true;

        rightHood.setPosition(0.0);
        leftHood.setPosition(0.0);

        pinpoint.resetPosAndIMU();

        initSensor(colorLeft);
        initSensor(colorRight);

        stallTimer.reset();


    }

    @Override
    public void init_loop() {

        kickerTarget = Globals.KICKER_IDLE;

        if (kickerPIDEnabled) {
            double encoderVoltage = kickerEncoder.getVoltage();
            double processedEncoderValue = DFM.zeroAndNormalizeAxonEncoder(encoderVoltage, Globals.KICKER_ZERO);

            double power = kickerPID.Output(Globals.KICKER_kP, Globals.KICKER_kD, kickerTarget, processedEncoderValue);

            leftKickerServo.setPower(power);
            rightKickerServo.setPower(power);
        } else {
            leftKickerServo.setPower(plainKickerPower);
            rightKickerServo.setPower(plainKickerPower);
        }

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

        //Drive

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
            frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
            backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);

        } else if (gamepad1.a) {
            frontIntakeMotor.setPower(-Globals.frontIntakeReverseSpeed);
            backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);

        } else if (gamepad1.left_trigger > 0.1 && !recyclerIsRunning) {

            kickerPIDEnabled = false;

            if(rightBumperTrue) {

                plainKickerPower = Globals.kickerShoot * 0.3;

            } else {

                plainKickerPower = Globals.kickerShoot;

            }

            backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed);
            shooting = true;

        } else if (gamepad1.dpad_up && !recyclerIsRunning) {

            kickerPIDEnabled = false;
            plainKickerPower = Globals.kickerShoot;

        } else if (gamepad1.dpadDownWasReleased() && !recyclerIsRunning) {

            recyclerIsRunning = true;

        } else {

            kickerPIDEnabled = true;
            plainKickerPower = 0.0;

            // Only mess with intake when we are NOT recycling.
            if (!recyclerIsRunning) {
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }

        }

        updateRecycle();

        if (gamepad1.b){
            if(closezone == 1){
                closezone = 3;
            }else{
                closezone = 1;
            }
        }

        LLResult results = limelight.getLatestResult();

        //close zone shoot

        telemetry.addData("left", leftBumperTrue);
        telemetry.addData("right", rightBumperTrue);
        if (gamepad1.rightBumperWasReleased()) {
            if (rightBumperTrue) {
                rightBumperTrue = false;
            } else {
                rightBumperTrue = false;
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
            kickerKP = 3;
            leftShooterMotor.setPower(vision.closeZoneflywheelspeed(limelight,currentSpeed,pinpoint));
            rightShooterMotor.setPower(vision.closeZoneflywheelspeed(limelight,currentSpeed,pinpoint));
            currentSpeed = vision.closeZoneflywheelspeed(limelight,currentSpeed,pinpoint);
            leftHood.setPosition(vision.closeZonehood(limelight,leftHood.getPosition(),pinpoint));
            rightHood.setPosition(vision.closeZonehood(limelight,leftHood.getPosition(),pinpoint));
            telemetry.addData("flywheel", leftShooterMotor.getVelocity());

        }


        if (!rightBumperTrue && !leftBumperTrue) {
            kickerKP = Globals.KICKER_kP;
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
            pinpoint.update();
            Pose2D pose2d = pinpoint.getPosition();

            double px = pose2d.getX(DistanceUnit.METER);
            double py = pose2d.getY(DistanceUnit.METER);
//            telemetry.addData("pinpoint x", px);
//            telemetry.addData("pinpoint y", py);
//            telemetry.addData("odometery", pose2d.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("pinpoint turret", vision.pinpointTurret(pinpoint,leftTurretServo.getPosition())*(13.0/33)*1800);
//            telemetry.addData("turret location", rightTurretServo.getPosition()*(13.0/33)*1800);
            leftTurretServo.setPosition(vision.pinpointTurret(pinpoint,leftTurretServo.getPosition())/*vision.adjustedTurretAngle(position, limelight,closezone)*/);
            rightTurretServo.setPosition(vision.pinpointTurret(pinpoint,leftTurretServo.getPosition())/*vision.adjustedTurretAngle(position, limelight,closezone)*/);
            //leftTurretServo.setPower(0.5);
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

        telemetry.addData("looptime", timer.milliseconds());

        // --- color sensing ---
        Reading L = readAndClassify(colorLeft);
        Reading R = readAndClassify(colorRight);

        ArtifactColor overall = combineByConfidence(L, R);
        showOnRgbLight(overall);

        //Kicker

        updateKickerPIDWithStall();

    }

    public void updateRecycle() {
        // Only run if we are in a recycle cycle
        if (!recyclerIsRunning) {
            return;
        }

        // First call for this run
        if (!started) {
            started = true;
            intakeStarted = false;
            recyclerTimer.reset();

            // --- RESET STALL LOGIC SO RECYCLE HAS FULL CONTROL ---
            recoveringFromStall = false;
            stallSampleValid = false;
            kickerPIDEnabled = true;
            plainKickerPower = 0.0;

            kickerTarget = Globals.KICKER_RECYCLE; // kicker down
            frontIntakeMotor.setPower(0.0);        // don't start intake yet
        }

        double t = recyclerTimer.milliseconds();

        // --- KICKER TIMING ---
        double kickerUpTime = intakeDelayMs + kickerUpDelayMs;
        if (t < kickerUpTime) {
            kickerTarget = 0.27; //Globals.KICKER_RECYCLE
        } else {
            kickerTarget = Globals.KICKER_IDLE;
        }

        // --- INTAKE TIMING ---
        if (!intakeStarted && t >= intakeDelayMs) {
            frontIntakeMotor.setPower(-1.0);
            intakeStarted = true;
        }

        // After totalMs, stop everything and finish the cycle
        if (t >= totalMs) {
            kickerTarget = Globals.KICKER_IDLE;
            frontIntakeMotor.setPower(0.0);

            // reset for next time
            recyclerIsRunning = false;
            started = false;
            intakeStarted = false;
        }
    }


    public void updateKickerPIDWithStall() {

        double nowMs = stallTimer.milliseconds();
        double encoderVoltage = kickerEncoder.getVoltage();
        double processedEncoderValue =
                DFM.zeroAndNormalizeAxonEncoder(encoderVoltage, Globals.KICKER_ZERO);

        boolean shootingOpenLoop =
                !kickerPIDEnabled && Math.abs(plainKickerPower) > 0.01 && !recyclerIsRunning;

        // Detect the moment we ENTER open-loop shooting
        if (shootingOpenLoop && !wasShootingOpenLoop) {
            openLoopStartTimeMs = nowMs;
            stallSampleValid = false;
        }

        // ---------- STALL RECOVERY STATE MACHINE ----------

        if (recoveringFromStall) {

            kickerPIDEnabled = true;
            kickerTarget = Globals.KICKER_IDLE;
            plainKickerPower = 0.0;

            if (nowMs - recoveryStartTimeMs >= stallRecoveryMs) {
                recoveringFromStall = false;
                stallSampleValid = false;

                kickerPIDEnabled = false;
                plainKickerPower = lastShootPower;
            }

        } else if (shootingOpenLoop && (nowMs - openLoopStartTimeMs) >= stallGraceMs) {
            // Only do stall detection AFTER the grace period

            if (!stallSampleValid) {
                stallSampleValid = true;
                stallSampleTimeMs = nowMs;
                stallSamplePos = processedEncoderValue;
            } else {
                double dt = nowMs - stallSampleTimeMs;
                if (dt >= stallWindowMs) {
                    double dPos = Math.abs(processedEncoderValue - stallSamplePos);
                    if (dPos < stallMinDelta) {
                        // STALL DETECTED
                        recoveringFromStall = true;
                        recoveryStartTimeMs = nowMs;

                        lastShootPower = plainKickerPower;

                        kickerPIDEnabled = true;
                        kickerTarget = Globals.KICKER_IDLE;
                        plainKickerPower = 0.0;

                        stallSampleValid = false;
                    } else {
                        stallSampleTimeMs = nowMs;
                        stallSamplePos = processedEncoderValue;
                    }
                }
            }

        } else if (!shootingOpenLoop && !recoveringFromStall) {
            // Not in open-loop → clear sample
            stallSampleValid = false;
        }

        wasShootingOpenLoop = shootingOpenLoop;

        // ---------- DRIVE THE SERVOS ----------

        if (kickerPIDEnabled) {
            double power = kickerPID.Output(
                    kickerKP,
                    kickerKD,
                    kickerTarget,
                    processedEncoderValue
            );

            leftKickerServo.setPower(power);
            rightKickerServo.setPower(power);
        } else {
            leftKickerServo.setPower(plainKickerPower);
            rightKickerServo.setPower(plainKickerPower);
        }
    }

    private void initSensor(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
        sensor.setGain(COLOR_SENSOR_GAIN);
    }

    private Reading readAndClassify(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);

        float h = hsv[0], s = hsv[1], v = hsv[2];

        // Only UNKNOWN if it's BOTH very dark AND very unsaturated
        if (v < MIN_VALUE && s < MIN_SATURATION) {
            return new Reading(ArtifactColor.UNKNOWN, h, s, v);
        }

        // Otherwise, closest hue wins
        float dGreen  = hueDistance(h, GREEN_HUE_CENTER);
        float dPurple = hueDistance(h, PURPLE_HUE_CENTER);
        ArtifactColor c = (dGreen <= dPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        return new Reading(c, h, s, v);
    }

    private float hueDistance(float a, float b) {
        float d = Math.abs(a - b);
        return Math.min(d, 360f - d);
    }

    // If either sensor reports a color, choose the one with higher confidence (s * v).
    // If both UNKNOWN, return UNKNOWN.
    private ArtifactColor combineByConfidence(Reading left, Reading right) {
        boolean leftColor  = left.color  != ArtifactColor.UNKNOWN;
        boolean rightColor = right.color != ArtifactColor.UNKNOWN;

        if (!leftColor && !rightColor) return ArtifactColor.UNKNOWN;
        if (leftColor && !rightColor)  return left.color;
        if (!leftColor && rightColor)  return right.color;

        // both have colors; pick by confidence
        if (left.conf > right.conf) return left.color;
        if (right.conf > left.conf) return right.color;

        // equal confidence tie-breaker: pick the one with higher V (brighter)
        if (left.val > right.val) return left.color;
        if (right.val > left.val) return right.color;

        // final tie-breaker: prefer PURPLE (arbitrary; change if you prefer GREEN)
        return (left.color == ArtifactColor.PURPLE || right.color == ArtifactColor.PURPLE)
                ? ArtifactColor.PURPLE : ArtifactColor.GREEN;
    }

    private void showOnRgbLight(ArtifactColor color) {
        switch (color) {
            case GREEN:  rgbLight.setPosition(LED_GREEN_POS);  break;
            case PURPLE: rgbLight.setPosition(LED_PURPLE_POS); break;
            default:     rgbLight.setPosition(LED_OFF_POS);     break;
        }
    }

}
