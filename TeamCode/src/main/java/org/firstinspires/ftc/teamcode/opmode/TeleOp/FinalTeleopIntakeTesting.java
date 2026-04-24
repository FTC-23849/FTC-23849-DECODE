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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CustomGoBildaPrismRgbLedDriver;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.opmode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController3;
import org.firstinspires.ftc.teamcode.vision.visionToolsClean;

import java.util.List;

@TeleOp
@Config
public class FinalTeleopIntakeTesting extends OpMode {
    List<LynxModule> hubs;
    private VoltageSensor myControlHubVoltageSensor;
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
    ServoImplEx frontTurretServo;
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
    private DigitalChannel bottomLeftLaser;
    private DigitalChannel bottomRightLaser;


    private PIDVelocityController3 velocityPID;
    public Pose2D robotPos;
    public Pose2D predictedPos;
    public double predDistance;
    public static double currentVelocity;
    public double velX;
    public double velY;
    public double velH;
    public static double moveAwayFar = 1.2;
    public static double moveAwayTurretFar = 1.4;
    public static double secFar = 0.8;
    public static double turretSecFar = 1.1;

    public static double moveAwayClose = 2;
    public static double moveAwayTurretClose = 0.6;
    public static double secClose = 0.6;
    public static double turretSecClose = 0.8;

    public static double rotationalSec = 0.1;
    public static double lockedFlywheelVelocity = -920;
    public static double lockedHoodHeight = 0.15;
    public static double TargetVelocity = -1200;
    public static double red = 0;
    public static double blue = 0;
    //more for left, less for right
    public static double turretZeroCorrection = -0.006;
    public static double turretZeroCorrection2 = -0.013;
    //    public static double VKp = 0.02;
//    public static double VKi = 0.003;
//    public static double VKd = 0;
//    public static double VkS = 0;
//    public static double VkV = 0.00042;
    public static double shotSpeed = 0.9;
    public static double defaultShotSpeed = 0.9;
    public static double lowBatteryShotSpeed = 0.7;
    public static double lockedPosShotSpeed = 0.7;
    public static double moveTowardsGoalshotSpeed = 0.85;
    public static double moveAwayFromGoalshotSpeed = 0.75;
    public static double KpMaintain = 0.003;
    public static double KiMaintain = 0.002;

    public static double KpDriveRecovery = 0.03;

    public static double KpRecovery = 0;
    public static double KiRecovery = 0.001;
    public static double KsRecovery = 1;
    public static double KvFF = 0.00042;
    public static double KsFF = 0.055 ;

    public static double recoveryThreshold = 60;
    public static double maintainThreshold = 40;
    public static double defaultVoltage = 13.15;
    public static double gear = 12.5;
    double currentVoltage;
    double closezone = 1;
    boolean firstLoop = true;
    String allianceColor = "Red";
    boolean recycleIntakeTimerStarted = false;
    public static double turretCorrection = 0.01;
    boolean shooting;
    boolean yPressed = false;
    boolean purpleSortingEnabled = false;
    boolean greenSortingEnabled = false;
    boolean lowVoltage = false;
    boolean LockedModeEnabled = false;
    boolean rightBumperFirstTime = false;
    boolean rightBumperHeld = false;
    boolean ParkModeEnabled = false;
    Pose2D AnchorPos = null;
    double AnchorxPos;
    double AnchoryPos;
    double AnchorYaw;
    double lastHeadingErrorLocked = 0;
    double lastXErrorLocked = 0;
    double lastYErrorLocked = 0;

    public static double kP_Lock = 0.16;
    public static double kP_Lock_Small = 0.07;
    public static double kD_Lock = 0.0007;
    public static double PIDDeadband = 8;
    public static double kPRot_Lock = 0.08;
    public static double kDRot_Lock = 0.00028;
    double lastTime = 0;
    ElapsedTime lockedPostimer = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime recycleIntakeTimer = new ElapsedTime();
    AnalogInput turretEncoder;
    private NormalizedColorSensor colorLeft1;
    private NormalizedColorSensor colorRight1;

    private NormalizedColorSensor colorLeft2;
    private NormalizedColorSensor colorRight2;

    private NormalizedColorSensor colorLeft3;
    private NormalizedColorSensor colorRight3;

    CustomGoBildaPrismRgbLedDriver prism;
    double totalCurrent;
    double groundDistance = 0;
    double turretPos = 0.5+turretZeroCorrection;
    double hoodHeight = 0;
    double flywheelSpeed = 0;
    boolean tipped = false;
    visionToolsClean vision = new visionToolsClean();
    List currentBalls;
    //    public static double Kp = 0.001;
//    public static double Ki = 0.0048;
//    public static double Kd = 0.00;
    double currentSpeed = 0;
    boolean rightBumperTrue = false;
    boolean backButtonTrue = false;
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
    double lastPIDUpdate = 0;
    double lastPinpointUpdate = 0;
    public static double pinpointThrottleMS =50;
    public static double PIDthrottleMS =60;
    public static double telemeteryThrottleMS = 40000000;
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
    public static double xcoeff = 1;
    public static double ycoeff = 1;
    ElapsedTime recyclerTimer = new ElapsedTime();
    ElapsedTime kickerStartDelayTimer = new ElapsedTime();
    boolean started = false;
    boolean intakeStarted = false;
    boolean recyclerIsRunning = false;

    // lock helper
    boolean cancelHeld = false;
    boolean lockIntakeKickerTongue = false;
    public static double flywheelCorrection = 0;
    boolean useTurret = true;
    boolean usePower = true;
    private final float[] hsvBuf = new float[3];
    // Colstaic or Detection
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

    // LED colors
    private static final int[] RGB_GREEN  = {0, 255, 0};
    private static final int[] RGB_PURPLE = {180, 0, 255};
    private static final int[] RGB_OFF    = {0, 0, 0};

    private static final int BRIGHT_ON = 100;
    private static final int BRIGHT_OFF = 0;

    private ArtifactColor last1 = null;
    private ArtifactColor last2 = null;
    private ArtifactColor last3 = null;

    // --- Color sensing + Prism updates (throttled round-robin) ---
    public static double colorTickMs = 80;   // 50ms tick = 20Hz. Each pair updates every 150ms (~6.7Hz)
    private double lastColorTick = 0;
    private int colorPhase = 0;             // 0->pair1, 1->pair2, 2->pair3
    private boolean prismSenseEnabled = false;   // optional toggle
    private boolean lastEnableCombo = false;
    private boolean lastDisableCombo = false;
    public static double kalmanQ = 0.1;
    public static double kalmanR = 0.01;
    private enum InitStage {
        RESET,
        WAIT_AFTER_RESET,
        RECAL_IMU,
        WAIT_AFTER_RECAL,
        SET_POSE,
        WAIT_AFTER_SET,
        UPDATE_AND_PRINT,
        DONE
    }

    private InitStage initStage = InitStage.RESET;

    // track B state for edge detection
    boolean lastBPressed = false;
    
    boolean thirdBallPresent;

    @Override
    public void init() {
        lockedPostimer.reset();
        lastTime = lockedPostimer.seconds();
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(9);

        prism = hardwareMap.get(CustomGoBildaPrismRgbLedDriver.class, "prism");

        // Configure 3 layers as 3 segments: (0-1), (2-3), (4-5)
        // Index 0 is the LED closest to the Prism driver. :contentReference[oaicite:10]{index=10}
        prism.configureSolidLayer(0, 0, 1, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, BRIGHT_ON, 0, 0, 0);

        colorLeft1 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight1 = hardwareMap.get(NormalizedColorSensor.class, "colorRight1");

        colorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft2");
        colorRight2 = hardwareMap.get(NormalizedColorSensor.class, "colorRight2");

        colorLeft3 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft3");
        colorRight3 = hardwareMap.get(NormalizedColorSensor.class, "colorRight3");

        initSensor(colorLeft1);
        initSensor(colorRight1);

        initSensor(colorLeft2);
        initSensor(colorRight2);

        initSensor(colorLeft3);
        initSensor(colorRight3);

        last1 = last2 = last3 = null;

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
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        frontTurretServo = hardwareMap.get(ServoImplEx.class, "frontTurretServo");

        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rgbLight = hardwareMap.get(ServoImplEx.class, "light");
        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");

        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");

        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        frontTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
//        leftTurretServo.setPosition(0.5);
//        rightTurretServo.setPosition(0.5);
//
//        leftTongueServo.setPosition(Globals.tongueIntake);
//        rightTongueServo.setPosition(Globals.tongueIntake);

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood.setDirection(ServoImplEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        colorLeft  = hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "colorRight1");

        bottomLeftLaser = hardwareMap.get(DigitalChannel.class, "bottomLeftLaser");
        bottomRightLaser = hardwareMap.get(DigitalChannel.class, "bottomRightLaser");

        bottomLeftLaser.setMode(DigitalChannel.Mode.INPUT);
        bottomRightLaser.setMode(DigitalChannel.Mode.INPUT);

//        rightHood.setPosition(0.0);
//        leftHood.setPosition(0.0);

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

        velocityPID = new PIDVelocityController3(
                targetVelocity,
                KpMaintain,KiMaintain,
                KpRecovery,KiRecovery,KsRecovery
                , KsFF, KvFF
        );

        limelight.setPollRateHz(15);
        limelight.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(120);
        robotPos = pinpoint.getPosition();
        velX = pinpoint.getVelX(DistanceUnit.MM);
        velY = pinpoint.getVelY(DistanceUnit.MM);
        velH = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        predictedPos = vision.predictPos(allianceColor,robotPos,velX,velY,velH,secFar, rotationalSec,moveAwayFar);
        hoodHeight = vision.hoodHeightRegressor(predictedPos, leftHood.getPosition(), allianceColor);
        turretPos = vision.CalculateTurretAngle360NEW(xcoeff,ycoeff,predictedPos,gear,leftTurretServo.getPosition(),turretZeroCorrection,turretZeroCorrection2, allianceColor) + turretCorrection;
        groundDistance = vision.groundDistancePinpoint(pinpoint,allianceColor);
        flywheelSpeed = Math.min(0,vision.CalculatedFlywheelSpeed(predictedPos, allianceColor));

        prism.configureSolidLayer(0, 0, 1, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, BRIGHT_ON, 0, 0, 0);

        leftTipper.setPosition(Globals.tipperRetracted);
        rightTipper.setPosition(Globals.tipperRetracted);

        lastColorTick = runTime.milliseconds();
        colorPhase = 0;
        prismSenseEnabled = false; // start OFF (or true if you want always on)

    }

    @Override
    public void init_loop() {
        switch (initStage) {

            case RESET:
                // Run ONCE
                pinpoint.resetPosAndIMU();
                timer.reset();
                initStage = InitStage.WAIT_AFTER_RESET;
                break;

            case WAIT_AFTER_RESET:
                // Wait 500ms
                if (timer.milliseconds() >= 500) {
                    initStage = InitStage.RECAL_IMU;
                }
                break;

            case RECAL_IMU:
                // Run ONCE
                pinpoint.recalibrateIMU();
                timer.reset();
                initStage = InitStage.WAIT_AFTER_RECAL;
                break;

            case WAIT_AFTER_RECAL:
                // Wait 1000ms
                if (timer.milliseconds() >= 1000) {
                    initStage = InitStage.SET_POSE;
                }
                break;

            case SET_POSE:
                // Run ONCE (IMPORTANT: convert if your pinpoint expects Pose2D units)
                pinpoint.setPosition(PoseStorage.currentPose);
                timer.reset();
                initStage = InitStage.WAIT_AFTER_SET;
                break;

            case WAIT_AFTER_SET:
                // Optional small settle time (you had 1000ms; keep it if you want)
                if (timer.milliseconds() >= 1000) {
                    initStage = InitStage.UPDATE_AND_PRINT;
                }
                break;

            case UPDATE_AND_PRINT:
                // Run ONCE
                pinpoint.update();
                telemetry.addData("file current pose", PoseStorage.currentPose);
                telemetry.addData("pinpoint current pose", pinpoint.getPosition().toString());
                initStage = InitStage.DONE;
                break;

            case DONE:
                // Keep showing telemetry every loop if you want
                telemetry.addData("Init", "DONE");
                telemetry.addData("file current pose", PoseStorage.currentPose);
                telemetry.addData("pinpoint current pose", pinpoint.getPosition().toString());
                break;
        }

        telemetry.addData("InitStage", initStage);
        telemetry.addData("t(ms)", (int) timer.milliseconds());
        telemetry.update();
    }

    @Override
    public void loop() {
        double currentTime = lockedPostimer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        if(dt <= 0) dt = 0.001;

        boolean isFarZone = robotPos.getX(DistanceUnit.MM) <= 0;
        double moveAway       = isFarZone ? moveAwayFar       : moveAwayClose;
        double moveAwayTurret = isFarZone ? moveAwayTurretFar : moveAwayTurretClose;
        double sec            = isFarZone ? secFar            : secClose;
        double turretSec      = isFarZone ? turretSecFar      : turretSecClose;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        
        thirdBallPresent = bottomLeftLaser.getState() || bottomRightLaser.getState();

        if (thirdBallPresent) {
            zoneLight.setPosition(1.0);
            //telemetry.addData("thirdball?: ", thirdBallPresent);
        } else {
            zoneLight.setPosition(0.0);
        }
        
        if(firstLoop){
            leftTurretServo.setPosition(0.5+turretZeroCorrection);
            rightTurretServo.setPosition(0.5+turretZeroCorrection);
            frontTurretServo.setPosition(0.5+turretZeroCorrection);

            leftTongueServo.setPosition(Globals.tongueIntake);
            rightTongueServo.setPosition(Globals.tongueIntake);
            rightHood.setPosition(0.0);
            leftHood.setPosition(0.0);
            firstLoop = false;
        }
        currentVoltage = myControlHubVoltageSensor.getVoltage();
        double now = runTime.milliseconds();
        if(gamepad1.left_bumper){
            moveAwayFar = 1.2;
            moveAwayTurretFar = 1.4;
            secFar = 0.8;
            turretSecFar = 1.1;

            rotationalSec = 0.1;

            moveAwayClose = 2;
             moveAwayTurretClose = 0.6;
            secClose = 0.6;
            turretSecClose = 0.8;
        }else{
            moveAwayFar = 0;
            moveAwayTurretFar = 0;
            secFar = 0;
            turretSecFar = 0;

            rotationalSec = 0;

            moveAwayClose = 0;
            moveAwayTurretClose = 0;
            secClose = 0;
            turretSecClose = 0;
        }
        if (now - lastPinpointUpdate >= pinpointThrottleMS) {
            robotPos = pinpoint.getPosition();
            if(LockedModeEnabled ^ ParkModeEnabled){
                runLockedPos(robotPos, dt);
            }
            velX = pinpoint.getVelX(DistanceUnit.MM);
            velY = pinpoint.getVelY(DistanceUnit.MM);
            velH = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            predictedPos = vision.predictPos(allianceColor,robotPos,velX,velY,velH,sec, rotationalSec,moveAway);
            predDistance = Math.sqrt(
                    Math.pow(predictedPos.getX(DistanceUnit.CM) - robotPos.getX(DistanceUnit.CM), 2) +
                            Math.pow(predictedPos.getY(DistanceUnit.CM) - robotPos.getY(DistanceUnit.CM), 2)
            );
            if(backButtonTrue){
                hoodHeight = vision.hoodHeightRegressor(predictedPos, leftHood.getPosition(), allianceColor);
                turretPos = vision.CalculateTurretAngle360NEW(xcoeff,ycoeff,vision.predictPos(allianceColor,robotPos,velX,velY,velH,turretSec, rotationalSec,moveAwayTurret),gear,leftTurretServo.getPosition(),turretZeroCorrection,turretZeroCorrection2, allianceColor) + turretCorrection;
                groundDistance = vision.groundDistancePinpoint(robotPos.getX(DistanceUnit.METER),robotPos.getY(DistanceUnit.METER),allianceColor);
                flywheelSpeed = Math.min(0,vision.CalculatedFlywheelSpeed(predictedPos, allianceColor));

            }
            pinpoint.update();
            lastPinpointUpdate = now;
        }
        timer.reset();

        // --- Toggle PrismSense ON/OFF (optional) ---
        boolean enableCombo  = gamepad2.right_stick_button;
        boolean disableCombo = gamepad2.left_stick_button;

        if (enableCombo && !lastEnableCombo) {
            prismSenseEnabled = true;
            last1 = last2 = last3 = null;   // force refresh
            colorPhase = 0;
        }

        if (disableCombo && !lastDisableCombo) {
            prismSenseEnabled = false;
            applyToLayer(prism, 0, ArtifactColor.UNKNOWN);
            applyToLayer(prism, 1, ArtifactColor.UNKNOWN);
            applyToLayer(prism, 2, ArtifactColor.UNKNOWN);
            showOnRgbLight(ArtifactColor.UNKNOWN);
        }

        lastEnableCombo = enableCombo;
        lastDisableCombo = disableCombo;

        telemetry.addData("last loop time", runTime.milliseconds() - lastLoopTime);
        telemetry.addData("average loop time", runTime.milliseconds() / loops);
        telemetry.addData("loops", loops);
        loops = loops + 1;
        lastLoopTime = runTime.milliseconds();
        telemetry.addData("tipped", tipped);

        telemetry.addData("Front Intake Velocity", (frontIntakeMotor.getVelocity() * 60) / 103.8);
        telemetry.addData("Back Intake Velocity", (backIntakeMotor.getVelocity() * 60) / 103.8);

        // -------------------- DRIVE --------------------
            double y = -gamepad1.left_stick_y; // Y is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
        if(!LockedModeEnabled && !ParkModeEnabled)  {
            leftFrontMotor.setPower(frontLeftPower);
            leftBackMotor.setPower(backLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            rightBackMotor.setPower(backRightPower);
        }


        // ----------------- RECYCLE TRIGGER + SAFE LOCKOUT -----------------
        if ((gamepad1.dpadDownWasReleased() || gamepad2.dpadRightWasReleased()) && !recyclerIsRunning) {
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
                backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
                backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);
                leftKickerServo.setPower(0.0);
                rightKickerServo.setPower(0.0);



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

                //frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed*shotSpeed);
                leftKickerServo.setPower(Globals.rollerKickerShoot);
                rightKickerServo.setPower(Globals.rollerKickerShoot);

//                if (groundDistance> 2.88) {
//                    if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay) {
//                        leftKickerServo.setPower(Globals.rollerKickerShoot * 0.5);
//                        rightKickerServo.setPower(Globals.rollerKickerShoot * 0.5);
//                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        frontIntakeMotor.setPower(-1*Globals.frontIntakeShootSpeed);
//                    } else {
//                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        frontIntakeMotor.setPower(0.7);
//                    }
//                } else {
//                    if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay) {
//                        leftKickerServo.setPower(Globals.rollerKickerShoot);
//                        rightKickerServo.setPower(Globals.rollerKickerShoot);
//                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed);
//                    } else {
//                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        frontIntakeMotor.setPower(0.5);
//                    }
//                }

                //backIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backIntakeMotor.setPower(-Globals.backIntakeShootSpeed*shotSpeed);
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
        if (gamepad2.backWasReleased()) {
            if (leftTurretServo.getPosition() < 0.764) {
                turretCorrection += 0.001;
            }
        }
        if (gamepad2.startWasReleased()) {
            if (leftTurretServo.getPosition() > 0.246) {
                turretCorrection -= 0.001;
            }
        }
        if (gamepad2.dpad_up) {
            flywheelCorrection -= 2;
        }
        if (gamepad2.dpad_down) {
            flywheelCorrection += 2;
        }
        if (gamepad2.y){
            flywheelCorrection = 0;
        }
        if(gamepad2.x){
            pinpoint.recalibrateIMU();
        }
        if(gamepad2.bWasReleased()){
            lowVoltage=!lowVoltage;
        }
        if (predDistance > 10) {
            double goalX = allianceColor.equals("Red") ? -165 : 165;
            double goalY = 0;

            double actualDist = Math.sqrt(
                    Math.pow(robotPos.getX(DistanceUnit.CM) - goalX, 2) +
                            Math.pow(robotPos.getY(DistanceUnit.CM) - goalY, 2)
            );
            double predictedDist = Math.sqrt(
                    Math.pow(predictedPos.getX(DistanceUnit.CM) - goalX, 2) +
                            Math.pow(predictedPos.getY(DistanceUnit.CM) - goalY, 2)
            );

            shotSpeed = predictedDist > actualDist ? moveAwayFromGoalshotSpeed : moveTowardsGoalshotSpeed;
        }else if(lowVoltage){
            shotSpeed = lowBatteryShotSpeed;
        }else if(LockedModeEnabled){
            shotSpeed = lockedPosShotSpeed;
        }else{
            shotSpeed = defaultShotSpeed;

        }if(gamepad1.start){
            pinpoint.recalibrateIMU();
        }
        if(gamepad1.right_bumper){
            if(!rightBumperHeld){
                rightBumperFirstTime = true;
            }
            else{
                rightBumperFirstTime = false;
            }
            rightBumperHeld = true;
        }
        else{
            rightBumperHeld = false;
            rightBumperFirstTime = false;
            LockedModeEnabled = false;
        }

        //Locked
        if(rightBumperFirstTime){
            LockedModeEnabled = !LockedModeEnabled;
            AnchorPos = null;
            if(LockedModeEnabled){
                AnchorPos = pinpoint.getPosition();

                AnchorxPos = -1 * AnchorPos.getY(DistanceUnit.METER);
                AnchoryPos = AnchorPos.getX(DistanceUnit.METER);

                double heading = robotPos.getHeading(AngleUnit.DEGREES);
                double adjustedHeading = heading + 90;
                adjustedHeading = ((adjustedHeading % 360) + 360) % 360;

                AnchorYaw = adjustedHeading;
            }else{
                lastXErrorLocked = 0;
                lastYErrorLocked = 0;
                lastHeadingErrorLocked = 0;
            }
        }



        if (gamepad2.leftBumperWasReleased()){
            useTurret = !useTurret;
            if(!useTurret) {
                leftTurretServo.setPosition(0.5+turretZeroCorrection);
                rightTurretServo.setPosition(0.5+turretZeroCorrection);
                frontTurretServo.setPosition(0.5+turretZeroCorrection);
            }
        }
        if (gamepad2.rightBumperWasReleased()){
            usePower= !usePower;
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
//        if (gamepad2.bWasReleased()) {
//            ParkModeEnabled = !ParkModeEnabled;
//            AnchorPos = null;
//            if(ParkModeEnabled) {
//                AnchorYaw = 90;
//                AnchoryPos = -0.716;
//                if(allianceColor.equals("Red")) {
//                    //Red
//                    AnchorxPos = -0.8832;
//                }else {
//                    //Blue
//                    AnchorxPos = 0.8832;
//                }
//            }
//
//        }


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
            //vision.mt2pinpoint(pinpoint, limelight);
            vision.mt1pinpoint(red,blue,pinpoint,allianceColor, limelight);
        }
        if (gamepad2.a){
            if(allianceColor.equals("Blue")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-65, AngleUnit.DEGREES,0));
            }else if(allianceColor.equals("Red")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,65, AngleUnit.DEGREES,0));
            }
        }


        // far zone shoot
        if (gamepad1.backWasReleased()) {
            backButtonTrue = !backButtonTrue;
        }

        if (backButtonTrue && !rightBumperTrue) {
            //flywheelCurrentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2;
            flywheelCurrentVelocity = leftShooterMotor.getVelocity();
            if (!usePower) {
                targetVelocity = lockedFlywheelVelocity + flywheelCorrection;
                leftHood.setPosition(lockedHoodHeight);
                rightHood.setPosition(lockedHoodHeight);
            } else {
                targetVelocity = vision.CalculatedFlywheelSpeed(predictedPos, allianceColor)
                        + flywheelCorrection;
                leftHood.setPosition(hoodHeight);
                rightHood.setPosition(hoodHeight);
            }

            velocityPID.setTargetVelocity(targetVelocity);
            velocityPID.setMaintainGains(KpMaintain,KiMaintain,KpDriveRecovery);
            velocityPID.setRecoveryGains(KpRecovery,KiRecovery,KsRecovery);
            velocityPID.setFeedforward(KsFF, KvFF);
            velocityPID.setRecoveryThreshold(recoveryThreshold);
            velocityPID.setMaintainThreshold(maintainThreshold);
            velocityPID.setVoltageStatus(lowVoltage);
            power = velocityPID.update(flywheelCurrentVelocity, currentVoltage, defaultVoltage);
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);

            double error = flywheelCurrentVelocity - targetVelocity;

        }

        if (!rightBumperTrue && !backButtonTrue) {
            //telemetry.addData("slowing down flywheel", 0);
            leftTurretServo.setPosition(0.5+turretZeroCorrection);
            rightTurretServo.setPosition(0.5+turretZeroCorrection);
            frontTurretServo.setPosition(0.5+turretZeroCorrection);

            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

        if (backButtonTrue && !rightBumperTrue) {
            //telemetry.addData("Power", vision.TurretPower(limelight, 0.5));
            if (useTurret) {


                leftTurretServo.setPosition(
                        turretPos
                );
                rightTurretServo.setPosition(
                        turretPos
                );
                frontTurretServo.setPosition(
                        turretPos
                );
            }
        }

        // tipping
        if (gamepad1.y) {
            leftTipper.setPosition(Globals.tipperExtended);
            rightTipper.setPosition(Globals.tipperExtended);
        }
        telemetry.addData("Error", flywheelCurrentVelocity-targetVelocity );
        telemetry.addData("flywheel Correction",flywheelCorrection);
        telemetry.addData("turret Correction",turretCorrection);
        telemetry.addData("distance", groundDistance);
        telemetry.addData("low voltage?", lowVoltage);
        if (now - lastTelemetryUpdate >= telemeteryThrottleMS) {
            telemetry.addData("Predicted Pos",predictedPos);
            telemetry.addData("Robot Pos",robotPos);
            telemetry.addData("VelX", velX);
            telemetry.addData("VelY",velY);
            telemetry.addData("AnchorX", AnchorxPos);
            telemetry.addData("AnchorY", AnchoryPos);
            telemetry.addData("robotPosX", -1 * robotPos.getY(DistanceUnit.METER));
            telemetry.addData("robotPosY", robotPos.getX(DistanceUnit.METER));
            telemetry.addData("Park? ", ParkModeEnabled);
            telemetry.addData("xError", (AnchorxPos - (-1 * robotPos.getY(DistanceUnit.METER))) * 100);
            telemetry.addData("yError", (AnchoryPos - robotPos.getX(DistanceUnit.METER)) * 100);
            telemetry.addData("current voltage, ", currentVoltage);
            telemetry.addData("leftturretservo",leftTurretServo.getPosition());
            telemetry.addData("rightturretservo",rightTurretServo.getPosition());
            telemetry.addData("frontturretservo",frontTurretServo.getPosition());
            telemetry.addData("pinpoint turret",turretPos);
            telemetry.addData("flywheel", flywheelCurrentVelocity);
            telemetry.addData("flywheell", leftShooterMotor.getVelocity() );
            telemetry.addData("flywheelr", rightShooterMotor.getVelocity());

            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("PIDF Power", power);
            telemetry.addData("looptime", timer.milliseconds());
            telemetry.addData("left kicker speed", leftKickerServo.getPower());
            telemetry.addData("right kicker speed", rightKickerServo.getPower());
            telemetry.addData("encoderposTurret", (turretEncoder.getVoltage()/ 3.2 * 360 ) % 360);
            telemetry.addData("rightBumperHeld", rightBumperHeld);
            telemetry.addData("rightBumperFirstTime", rightBumperFirstTime);

            telemetry.update();
            lastTelemetryUpdate = now;
        }

        // --- Color sensing + Prism updates (THROTTLED ROUND-ROBIN) ---
        if (prismSenseEnabled && (now - lastColorTick >= colorTickMs)) {
            lastColorTick = now;

            switch (colorPhase) {
                case 0: {
                    Reading L1 = readAndClassify(colorLeft1);
                    Reading R1 = readAndClassify(colorRight1);
                    ArtifactColor overall1 = combineByConfidence(L1, R1);

                    if (last1 == null || overall1 != last1) applyToLayer(prism, 2, overall1);
                    last1 = overall1;

                    showOnRgbLight(overall1); // servo light shows pair1
                    break;
                }
                case 1: {
                    Reading L2 = readAndClassify(colorLeft2);
                    Reading R2 = readAndClassify(colorRight2);
                    ArtifactColor overall2 = combineByConfidence(L2, R2);

                    if (last2 == null || overall2 != last2) applyToLayer(prism, 1, overall2);
                    last2 = overall2;
                    break;
                }
                case 2: {
                    Reading L3 = readAndClassify(colorLeft3);
                    Reading R3 = readAndClassify(colorRight3);
                    ArtifactColor overall3 = combineByConfidence(L3, R3);

                    if (last3 == null || overall3 != last3) applyToLayer(prism, 0, overall3);
                    last3 = overall3;
                    break;
                }
            }

            colorPhase = (colorPhase + 1) % 3;
        }


    }

    @Override
    public void stop() {
        applyToLayer(prism, 0, ArtifactColor.UNKNOWN);
        applyToLayer(prism, 1, ArtifactColor.UNKNOWN);
        applyToLayer(prism, 2, ArtifactColor.UNKNOWN);
    }

    private void applyToLayer(CustomGoBildaPrismRgbLedDriver prism, int layer, ArtifactColor c) {
        int[] rgb;
        int bright;

        switch (c) {
            case GREEN:
                rgb = RGB_GREEN; bright = BRIGHT_ON; break;
            case PURPLE:
                rgb = RGB_PURPLE; bright = BRIGHT_ON; break;
            default:
                rgb = RGB_OFF; bright = BRIGHT_OFF; break;
        }

        //prism.setLayerBrightness(layer, bright);
        prism.setLayerColor(layer, rgb[0], rgb[1], rgb[2]);
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

            //frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        Color.colorToHSV(rgba.toColor(), hsvBuf);

        float h = hsvBuf[0];
        float s = hsvBuf[1];
        float v = hsvBuf[2];

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
//    public void runLockedMode(
//            GoBildaPinpointDriver pinpoint,
//            DcMotorEx lf,
//            DcMotorEx rf,
//            DcMotorEx lb,
//            DcMotorEx rb,
//            double dt
//    ){
//        if(LockedModeEnabled&&AnchorPos == null){
//            AnchorPos = pinpoint.getPosition();
//
//            AnchorxPos = -1 * AnchorPos.getY(DistanceUnit.METER);
//            AnchoryPos = AnchorPos.getX(DistanceUnit.METER);
//
//            Pose2D robotPos = pinpoint.getPosition();
//
//            double heading = robotPos.getHeading(AngleUnit.DEGREES);
//            double adjustedHeading = heading + 90;
//
//            if(adjustedHeading < 0) adjustedHeading += 360;
//
//            AnchorYaw = adjustedHeading;
//        }
//
//        double now = runTime.milliseconds();
//        if(LockedModeEnabled && now - lastPIDUpdate >= PIDthrottleMS){
//
//            Pose2D robotPos = pinpoint.getPosition();
//
//            double heading = robotPos.getHeading(AngleUnit.DEGREES);
//            double adjustedHeading = heading + 90;
//
//            if(adjustedHeading < 0) adjustedHeading += 360;
//
//            double headingError = AnchorYaw - adjustedHeading;
//            headingError = ((headingError + 180) % 360) - 180;
//
//            double headingDerivative = (headingError - lastHeadingErrorLocked) / dt;
//
//            double rotPower = kPRot_Lock * headingError + kDRot_Lock * headingDerivative;
//
//            lastHeadingErrorLocked = headingError;
//
//            double xPos = -1 * robotPos.getY(DistanceUnit.METER);
//            double yPos = robotPos.getX(DistanceUnit.METER);
//
//            double xError = (AnchorxPos - xPos) * 100;
//            double yError = (AnchoryPos - yPos) * 100;
//
//            double headingRad = Math.toRadians(adjustedHeading);
//
//            double robotXError = xError * Math.cos(headingRad) + yError * Math.sin(headingRad);
//            double robotYError = -xError * Math.sin(headingRad) + yError * Math.cos(headingRad);
//
//            double xDerivative = (robotXError - lastXErrorLocked) / dt;
//            double yDerivative = (robotYError - lastYErrorLocked) / dt;
//
//            double xPower = kP_Lock * robotXError + kD_Lock * xDerivative;
//            double yPower = kP_Lock * robotYError + kD_Lock * yDerivative;
//
//            lastXErrorLocked = robotXError;
//            lastYErrorLocked = robotYError;
//
//            lastXErrorLocked = xError;
//            lastYErrorLocked = yError;
//
//            double lfPower = yPower + xPower - rotPower;
//            double rfPower = yPower - xPower + rotPower;
//            double lbPower = yPower - xPower - rotPower;
//            double rbPower = yPower + xPower + rotPower;
//
//            lf.setPower(Math.max(-1, Math.min(1, lfPower)));
//            rf.setPower(Math.max(-1, Math.min(1, rfPower)));
//            lb.setPower(Math.max(-1, Math.min(1, lbPower)));
//            rb.setPower(Math.max(-1, Math.min(1, rbPower)));
//        }
//    }


    public void runLockedPos(Pose2D robotPos, double dt){

        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = heading + 90;
        adjustedHeading = ((adjustedHeading % 360) + 360) % 360;


        double headingError = AnchorYaw - adjustedHeading;
        if(LockedModeEnabled){
        if (headingError > 180) {
            headingError -= 360;
        } else if (headingError < -180) {
            headingError += 360;
        }
        }else{
            headingError = ((headingError + 180) % 360) - 180;
        }


        double headingDerivative = (headingError - lastHeadingErrorLocked) / dt;
        double rotPower = kPRot_Lock * Math.signum(headingError) * Math.sqrt(Math.abs(headingError)) + kDRot_Lock * headingDerivative;

        lastHeadingErrorLocked = headingError;

        double xPos = -1 * robotPos.getY(DistanceUnit.METER);
        double yPos = robotPos.getX(DistanceUnit.METER);

        double xError = (AnchorxPos - xPos) * 100;
        double yError = (AnchoryPos - yPos) * 100;

//        double headingRad = Math.toRadians(adjustedHeading);
//        double xError =  xError * Math.cos(headingRad) - yError * Math.sin(headingRad);
//        double yError = xError * Math.sin(headingRad) + yError * Math.cos(headingRad);
        double distanceFromAnchor = Math.sqrt(xError * xError + yError * yError);

        //conveer error to robot space
        double anchorAngle = Math.atan2(yError, xError);
        double headingRad = Math.toRadians(adjustedHeading);
        double relativeAngle = anchorAngle - headingRad;

        xError = distanceFromAnchor* -Math.sin(relativeAngle);
        yError = distanceFromAnchor * Math.cos(relativeAngle);
        double xDerivative = (xError - lastXErrorLocked) / dt;
        double yDerivative = (yError - lastYErrorLocked) / dt;
        double xPower ;
        double yPower;

        if(distanceFromAnchor< PIDDeadband){
            xPower = kP_Lock_Small * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xDerivative;
            yPower = kP_Lock_Small * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yDerivative;

        }else{
            xPower = kP_Lock * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xDerivative;
            yPower = kP_Lock * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yDerivative;

        }

        lastXErrorLocked = xError;
        lastYErrorLocked = yError;

        double lfPower = yPower + xPower - rotPower;
        double rfPower = yPower - xPower + rotPower;
        double lbPower = yPower - xPower - rotPower;
        double rbPower = yPower + xPower + rotPower;

        leftFrontMotor.setPower(Math.max(-1, Math.min(1, lfPower)));
        rightFrontMotor.setPower(Math.max(-1, Math.min(1, rfPower)));
        leftBackMotor.setPower(Math.max(-1, Math.min(1, lbPower)));
        rightBackMotor.setPower(Math.max(-1, Math.min(1, rbPower)));
    }

}