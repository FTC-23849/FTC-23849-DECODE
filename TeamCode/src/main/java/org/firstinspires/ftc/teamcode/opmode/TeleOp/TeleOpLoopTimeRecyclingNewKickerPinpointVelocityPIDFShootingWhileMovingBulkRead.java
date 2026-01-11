package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController2;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.List;

@TeleOp
@Config
public class TeleOpLoopTimeRecyclingNewKickerPinpointVelocityPIDFShootingWhileMovingBulkRead extends OpMode {
    List<LynxModule> hubs;
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
    ServoImplEx leftHood;
    ServoImplEx rightHood;
    ServoImplEx leftTongueServo;
    ServoImplEx rightTongueServo;
    GoBildaPinpointDriver pinpoint;
    AnalogInput kickerEncoder;
    private NormalizedColorSensor colorLeft;
    private NormalizedColorSensor colorRight;
    private Servo rgbLight;
    private PIDVelocityController2 velocityPID;
    public static double currentVelocity;
    public static double TargetVelocity = 900;
    public static double VKp = 0.002;
    public static double VKi = 0.003;
    public static double VKd = 0;
    public static double VkS = 0;
    public static double VkV = 0.00042;
    public static double sec = 0.2;
    double closezone = 1;
    String allianceColor = "Red";
    boolean recycleIntakeTimerStarted = false;
    double turretCorrection = 0;
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
    public static double Kp = 0.001;
    public static double Ki = 0.0048;
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

    double plainKickerPower = 0.0;

    // recycling variables
    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;
    //throttling
    double lastPinpointUpdate = 0;
    double pinpointThrottleMS = 50;
    double telemeteryThrottleMS = 100;
    double lastTelemetryUpdate = 0;
    double flywheelCurrentVelocity = 0;
    double targetVelocity = 0;
    double power = 0;
    // Total time to run the recycle sequence (ms)
    double totalMs = recyclingDelay;
    // Delay between commanding KICKER_RECYCLE and starting intake (ms)
    double intakeDelayMs = recyclingIntakeDelay;
    // Delay between intake starting and kicker going back up (ms)
    double kickerUpDelayMs = recyclingKickerUpDelay;

    // --- TELEOP RECYCLE TIMINGS (matches your updated auto default: 400 down, 600 intake) ---
    public static double RECYCLE_TONGUE_DOWN_MS = 400;
    public static double RECYCLE_INTAKE_RUN_MS  = 600;

    ElapsedTime recyclerTimer = new ElapsedTime();
    ElapsedTime kickerStartDelayTimer = new ElapsedTime();
    boolean started = false;
    boolean intakeStarted = false;
    boolean recyclerIsRunning = false;

    // lock helper
    boolean cancelHeld = false;
    boolean lockIntakeKickerTongue = false;

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

    // track B state for edge detection
    boolean lastBPressed = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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

        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");

        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rgbLight = hardwareMap.get(ServoImplEx.class, "light");
        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");

        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");

        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        leftTurretServo.setPosition(0.5);
        rightTurretServo.setPosition(0.5);

        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        colorLeft  = hardwareMap.get(NormalizedColorSensor.class, "leftIntakeColorSensor");
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "rightIntakeColorSensor");

        rightHood.setPosition(0.0);
        leftHood.setPosition(0.0);

        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(19.970472542,DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        initSensor(colorLeft);
        initSensor(colorRight);

        //leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        velocityPID = new PIDVelocityController2(
                VKp, VKi, VKd,
                VkS, VkV,
                TargetVelocity
        );

        limelight.setPollRateHz(30);
        limelight.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void init_loop() {
        leftTipper.setPosition(Globals.tipperRetracted);
        rightTipper.setPosition(Globals.tipperRetracted);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        double now = runTime.milliseconds();
        pinpoint.update();
        if (now - lastPinpointUpdate >= pinpointThrottleMS) {
            //pinpoint.update();
            lastPinpointUpdate = now;
        }
        timer.reset();

        telemetry.addData("last loop time", runTime.milliseconds() - lastLoopTime);
        telemetry.addData("average loop time", runTime.milliseconds() / loops);
        telemetry.addData("loops", loops);
        loops = loops + 1;
        lastLoopTime = runTime.milliseconds();
        telemetry.addData("tipped", tipped);

        // -------------------- DRIVE (always allowed) --------------------
        double y = -gamepad1.left_stick_y; // Y is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

        // ----------------- RECYCLE TRIGGER + SAFE LOCKOUT -----------------
        if (gamepad1.dpadDownWasReleased() && !recyclerIsRunning) {
            recyclerIsRunning = true;
            started = false;
        }

        cancelHeld = gamepad1.right_stick_button;
        if (cancelHeld && recyclerIsRunning) {
            cancelRecycleTeleOp();
        }

        if (recyclerIsRunning) {
            updateRecycleTeleOp();
        }

        // Lock out ONLY commands that fight recycle (and while B is held)
        lockIntakeKickerTongue = recyclerIsRunning || cancelHeld;
        // -----------------------------------------------------------------

        // -------------------- TAG TELEMETRY (allowed) --------------------
//        LLResult result = limelight.getLatestResult();
//        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//        int tagID = 0;
//        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//            tagID = fiducial.getFiducialId();
//        }
//        telemetry.addData("Tag ID", tagID);

        // -------------------- INTAKE / KICKERS / TONGUE -------------------
        // Disabled ONLY when recycle owns these actuators
        if (!lockIntakeKickerTongue) {

            if (gamepad1.left_trigger < 0.1) {
                kickerStartDelayTimer.reset();
            }

            if (gamepad1.right_trigger > 0.1) {
                frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
                backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);


            }
            else if (gamepad1.b) {
                leftKickerServo.setPower(1.0);
                rightKickerServo.setPower(1.0);
            }
            else if (gamepad1.a) {
                frontIntakeMotor.setPower(-Globals.frontIntakeReverseSpeed);
                backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);

            } else if (gamepad1.left_trigger > 0.1) {

                leftTongueServo.setPosition(Globals.tongueShoot);
                rightTongueServo.setPosition(Globals.tongueShoot);

                if (vision.groundDistancePinpoint(pinpoint,allianceColor)> 2.88) {
                    if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay) {
                        leftKickerServo.setPower(Globals.rollerKickerShoot * 0.5);
                        rightKickerServo.setPower(Globals.rollerKickerShoot * 0.5);
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(-0.5 *Globals.frontIntakeShootSpeed);
                    } else {
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(0.7);
                    }
                } else {
                    if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay) {
                        leftKickerServo.setPower(Globals.rollerKickerShoot);
                        rightKickerServo.setPower(Globals.rollerKickerShoot);
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed);
                    } else {
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(0.5);
                    }
                }

                backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
                shooting = true;

            } else if (gamepad1.dpad_up) {

                leftTongueServo.setPosition(Globals.tongueShoot);
                rightTongueServo.setPosition(Globals.tongueShoot);
                leftKickerServo.setPower(Globals.rollerKickerShoot);
                rightKickerServo.setPower(Globals.rollerKickerShoot);

            } else {

                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);
                leftKickerServo.setPower(0.0);
                rightKickerServo.setPower(0.0);
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }
        }

        // -------------------- OTHER CONTROLS (allowed) --------------------
        // closezone toggle moved to X
        if (gamepad1.x) {
            if (closezone == 1) {
                closezone = 3;
            } else {
                closezone = 1;
            }
        }
        if (gamepad1.back) {
            if (leftTurretServo.getPosition() < 0.84) {
                turretCorrection += 0.0025;
            }
        }
        if (gamepad1.start) {
            if (leftTurretServo.getPosition() > 0.34) {
                turretCorrection -= 0.0025;
            }
        }

        // close zone shoot
        //telemetry.addData("left", leftBumperTrue);
        //telemetry.addData("right", rightBumperTrue);

//        if (gamepad1.rightBumperWasReleased()) {
//            rightBumperTrue = !rightBumperTrue;
//        }

        if (gamepad1.dpad_left) {
            allianceColor = "Blue";
        }
        if (gamepad1.dpad_right) {
            allianceColor = "Red";
        }

//        if (rightBumperTrue && !leftBumperTrue) {
//            leftShooterMotor.setPower(Globals.defaultFarZonePower);
//            rightShooterMotor.setPower(Globals.defaultFarZonePower);
//
//            leftHood.setPosition(0.4);
//            rightHood.setPosition(0.4);
//            telemetry.addData("flywheel", leftShooterMotor.getVelocity());
//        }
//        if (rightBumperTrue && !leftBumperTrue) {
//            double errorMargin = 0.5;
//            double position = leftTurretServo.getPosition();
//
//            telemetry.addData("Power", vision.TurretPower(limelight, errorMargin));
//
//            leftTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight, 2) + turretCorrection);
//            rightTurretServo.setPosition(vision.adjustedTurretAngle(position, limelight, 2) + turretCorrection);
//        }
        if (gamepad1.x) {
            turretCorrection = 0;
            vision.mt1pinpoint(pinpoint, limelight);
        }
        if (gamepad2.a){
            if(allianceColor.equals("Blue")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-65, AngleUnit.DEGREES,0));
            }else if(allianceColor.equals("Red")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,65, AngleUnit.DEGREES,0));
            }
        }


        // far zone shoot
        if (gamepad1.leftBumperWasReleased()) {
            leftBumperTrue = !leftBumperTrue;
        }

        if (leftBumperTrue && !rightBumperTrue) {

            flywheelCurrentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2;

            targetVelocity = vision.FlywheelSpeedRegressor(sec, flywheelCurrentVelocity, pinpoint, allianceColor);

            velocityPID.setTargetVelocity(targetVelocity);
            velocityPID.setPID(VKp, VKi, VKd);
            velocityPID.setFeedforward(VkS, VkV);

            power = velocityPID.update(flywheelCurrentVelocity);

            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);

            currentSpeed = targetVelocity;

            leftHood.setPosition(vision.hoodHeightRegressor(limelight, leftHood.getPosition(), pinpoint, allianceColor));
            rightHood.setPosition(vision.hoodHeightRegressor(limelight, leftHood.getPosition(), pinpoint, allianceColor));


        }

        if (!rightBumperTrue && !leftBumperTrue) {
            //telemetry.addData("slowing down flywheel", 0);
            leftTurretServo.setPosition(0.5);
            rightTurretServo.setPosition(0.5);

            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

        if (leftBumperTrue && !rightBumperTrue) {
            //telemetry.addData("Power", vision.TurretPower(limelight, 0.5));

            leftTurretServo.setPosition(
                    vision.pinpointTurretMoving(sec, pinpoint, leftTurretServo.getPosition(), allianceColor) + turretCorrection
            );
            rightTurretServo.setPosition(
                    vision.pinpointTurretMoving(sec, pinpoint, leftTurretServo.getPosition(), allianceColor) + turretCorrection
            );
        }

        // tipping
        if (gamepad1.y) {
            leftTipper.setPosition(Globals.tipperExtended);
            rightTipper.setPosition(Globals.tipperExtended);
        }
        telemetry.addData("Error", flywheelCurrentVelocity-targetVelocity );
        telemetry.update();
        if (now - lastTelemetryUpdate >= telemeteryThrottleMS) {
            telemetry.addData("leftservo",leftTurretServo.getPosition());
            telemetry.addData("rightservo",rightTurretServo.getPosition());
            telemetry.addData("pinpoint turret",vision.pinpointTurretMoving(sec, pinpoint, leftTurretServo.getPosition(), allianceColor));
            telemetry.addData("flywheel", flywheelCurrentVelocity);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("Error", flywheelCurrentVelocity - targetVelocity);
            telemetry.addData("PIDF Power", power);
            telemetry.addData("looptime", timer.milliseconds());
            telemetry.addData("left kicker speed", leftKickerServo.getPower());
            telemetry.addData("right kicker speed", rightKickerServo.getPower());
            telemetry.addData("distance", vision.groundDistancePinpoint(pinpoint,allianceColor));

            telemetry.update();
            lastTelemetryUpdate = now;
        }

        // --- color sensing ---
        Reading L = readAndClassify(colorLeft);
        Reading R = readAndClassify(colorRight);

        ArtifactColor overall = combineByConfidence(L, R);
        showOnRgbLight(overall);


    }

    // --- TeleOp recycle state machine (ONLY owns tongue/kickers/intake while active) ---
    private void updateRecycleTeleOp() {
        if (!started) {
            started = true;
            recyclerTimer.reset();
        }

        double t = recyclerTimer.milliseconds();

        // PHASE 1: tongue DOWN + rollers ON, intake OFF
        if (t < RECYCLE_TONGUE_DOWN_MS) {
            leftTongueServo.setPosition(Globals.tongueRecycle);
            rightTongueServo.setPosition(Globals.tongueRecycle);

            leftKickerServo.setPower(Globals.rollerKickerRecycle);
            rightKickerServo.setPower(Globals.rollerKickerRecycle);

            frontIntakeMotor.setPower(0.0);
            backIntakeMotor.setPower(0.0);
            return;
        }

        // PHASE 2: tongue UP + rollers OFF, intake ON
        if (t < RECYCLE_TONGUE_DOWN_MS + RECYCLE_INTAKE_RUN_MS) {
            leftTongueServo.setPosition(Globals.tongueIntake);
            rightTongueServo.setPosition(Globals.tongueIntake);

            leftKickerServo.setPower(0.0);
            rightKickerServo.setPower(0.0);

            frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontIntakeMotor.setPower(-1.0);
            backIntakeMotor.setPower(0.0);
            return;
        }

        // PHASE 3: stop, finish
        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);

        leftKickerServo.setPower(0.0);
        rightKickerServo.setPower(0.0);

        frontIntakeMotor.setPower(0.0);
        backIntakeMotor.setPower(0.0);

        recyclerIsRunning = false;
        started = false;
    }

    private void cancelRecycleTeleOp() {
        recyclerIsRunning = false;
        started = false;

        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);

        leftKickerServo.setPower(0.0);
        rightKickerServo.setPower(0.0);

        frontIntakeMotor.setPower(0.0);
        backIntakeMotor.setPower(0.0);
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

        float dGreen  = hueDistance(h, GREEN_HUE_CENTER);
        float dPurple = hueDistance(h, PURPLE_HUE_CENTER);
        ArtifactColor c = (dGreen <= dPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        return new Reading(c, h, s, v);
    }

    private float hueDistance(float a, float b) {
        float d = Math.abs(a - b);
        return Math.min(d, 360f - d);
    }

    private ArtifactColor combineByConfidence(Reading left, Reading right) {
        boolean leftColor  = left.color  != ArtifactColor.UNKNOWN;
        boolean rightColor = right.color != ArtifactColor.UNKNOWN;

        if (!leftColor && !rightColor) return ArtifactColor.UNKNOWN;
        if (leftColor && !rightColor)  return left.color;
        if (!leftColor && rightColor)  return right.color;

        if (left.conf > right.conf) return left.color;
        if (right.conf > left.conf) return right.color;

        if (left.val > right.val) return left.color;
        if (right.val > left.val) return right.color;

        return (left.color == ArtifactColor.PURPLE || right.color == ArtifactColor.PURPLE)
                ? ArtifactColor.PURPLE : ArtifactColor.GREEN;
    }

    private void showOnRgbLight(ArtifactColor color) {
        switch (color) {
            case GREEN:  rgbLight.setPosition(LED_GREEN_POS);  break;
            case PURPLE: rgbLight.setPosition(LED_PURPLE_POS); break;
            default:     rgbLight.setPosition(LED_OFF_POS);    break;
        }
    }
}
