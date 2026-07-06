package org.firstinspires.ftc.teamcode.opmode.TeleOp.TeleOpArchive;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CustomGoBildaPrismRgbLedDriver;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.opmode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController3;
import org.firstinspires.ftc.teamcode.vision.visionToolsClean;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;

@TeleOp
@Config
public class TeleopCurrentLogging extends OpMode {
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
    AnalogInput switchCurrent;  // GoBilda Floodgate: 0-3.3V = 0-80A
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
    public static double blue = 6;

    public static double turretZeroCorrection = -0.007;
    public static double turretZeroCorrection2 = -0.012;

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
    public static double KsRecovery = 0.8;
    public static double KvFF = 0.00042;
    public static double KsFF = 0.055;

    public static double recoveryThreshold = 60;
    public static double maintainThreshold = 40;
    public static double defaultVoltage = 13.15;
    public static double gear = 12.5;
    double currentVoltage;
    double closezone = 1;
    boolean firstLoop = true;
    String allianceColor = "Red";
    boolean recycleIntakeTimerStarted = false;
    public static double turretCorrection = 0.008;
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

    CustomGoBildaPrismRgbLedDriver prism;
    double totalCurrent;
    double groundDistance = 0;
    double turretPos = 0.5 + turretZeroCorrection;
    double hoodHeight = 0;
    double flywheelSpeed = 0;
    boolean tipped = false;
    visionToolsClean vision = new visionToolsClean();
    List currentBalls;
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

    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;

    double lastPIDUpdate = 0;
    double lastPinpointUpdate = 0;
    public static double pinpointThrottleMS = 50;
    public static double PIDthrottleMS = 60;
    public static double telemeteryThrottleMS = 40000000;
    double lastTelemetryUpdate = 0;
    double flywheelCurrentVelocity = 0;
    double targetVelocity = 0;
    double power = 0;
    double targetIntakePower = 0;

    double totalMs = recyclingDelay;
    double intakeDelayMs = recyclingIntakeDelay;
    double kickerUpDelayMs = recyclingKickerUpDelay;

    public static double RECYCLE_TONGUE_DOWN_MS = 400;
    public static double THIRD_BALL_CONFIRM_MS = 400;
    public static double RECYCLE_INTAKE_MAX_MS = 2000;

    public static double MANUAL_INTAKE_TRIGGER_THRESHOLD = 0.1;
    public static double MANUAL_INTAKE_THIRD_BALL_CONFIRM_MS = 200;
    public static double MANUAL_INTAKE_ALREADY_FULL_RUN_MS = 800;

    public static double xcoeff = 1;
    public static double ycoeff = 1;
    ElapsedTime recyclerTimer = new ElapsedTime();
    ElapsedTime kickerStartDelayTimer = new ElapsedTime();

    ElapsedTime thirdBallConfirmTimer = new ElapsedTime();
    boolean thirdBallConfirmTimerStarted = false;

    ElapsedTime manualIntakeThirdBallConfirmTimer = new ElapsedTime();
    ElapsedTime manualIntakeAlreadyFullTimer = new ElapsedTime();
    boolean manualIntakeWasPressed = false;
    boolean manualIntakeLatchedOff = false;
    boolean manualIntakeThirdBallConfirmTimerStarted = false;
    boolean manualIntakeAlreadyFullMode = false;

    boolean started = false;
    boolean intakeStarted = false;
    boolean recyclerIsRunning = false;

    boolean cancelHeld = false;
    boolean lockIntakeKickerTongue = false;
    public static double flywheelCorrection = 0;
    boolean useTurret = true;
    boolean usePower = true;
    private final float[] hsvBuf = new float[3];

    public static float MIN_SATURATION = 0.02f;
    public static float MIN_VALUE = 0.005f;

    public static float GREEN_HUE_CENTER = 150f;
    public static float PURPLE_HUE_CENTER = 225f;

    public static float COLOR_SENSOR_GAIN = 1f;

    public static double LED_OFF_POS = 0.00;
    public static double LED_GREEN_POS = 0.50;
    public static double LED_PURPLE_POS = 0.70;

    public static boolean SOTM = false;
    public static boolean SOTMTesting = false;
    private enum ArtifactColor { GREEN, PURPLE, RED, UNKNOWN }

    private static class Reading {
        ArtifactColor color;
        float hue, sat, val;
        float conf;

        Reading(ArtifactColor c, float h, float s, float v) {
            color = c;
            hue = h;
            sat = s;
            val = v;
            conf = s * v;
        }
    }

    private static final int[] RGB_GREEN  = {0, 255, 0};
    private static final int[] RGB_PURPLE = {180, 0, 255};
    private static final int[] RGB_RED    = {255, 0, 0};
    private static final int[] RGB_OFF    = {0, 0, 0};

    private static final int BRIGHT_ON  = 100;
    private static final int BRIGHT_OFF = 0;

    private ArtifactColor last1 = null;
    private ArtifactColor last2 = null;
    private ArtifactColor last3 = null;

    boolean intakeStallFront = false;
    boolean intakeStallBack  = false;

    public static double colorTickMs = 80;
    private double lastColorTick = 0;
    private int colorPhase = 0;
    private boolean prismSenseEnabled = false;
    private boolean lastEnableCombo  = false;
    private boolean lastDisableCombo = false;
    public static double kalmanQ = 0.1;
    public static double kalmanR = 0.01;
    public static double frontIntakeAmpLimit = 8.5;
    public static double backIntakeAmpLimit  = 8.5;

    private enum InitStage {
        RESET, WAIT_AFTER_RESET, RECAL_IMU, WAIT_AFTER_RECAL,
        SET_POSE, WAIT_AFTER_SET, UPDATE_AND_PRINT, DONE
    }
    private InitStage initStage = InitStage.RESET;

    boolean lastBPressed = false;
    boolean thirdBallPresent;
    boolean firstStallLoop = false;
    double stallStartTime = -1;
    boolean intakeIsStalled = false;

    private FtcDashboard dashboard;

    // --- Current Logger ---
    // Shared pool: total of 5 log files across ALL teleop opmodes combined
    public static double currentLogIntervalMs = 20; // 50 Hz — reduces RS-485 bus load on servo hub
    private static final int MAX_TOTAL_LOG_FILES = 5;
    private BufferedWriter currentLogger;
    private double lastCurrentLogTime = 0;
    private String currentLogPath;
    private long lastLoggedTimeMs;

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
        prism.configureSolidLayer(0, 0, 1, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, BRIGHT_ON, 0, 0, 0);

        colorLeft1 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight1 = hardwareMap.get(NormalizedColorSensor.class, "colorRight1");
        colorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft2");
        colorRight2 = hardwareMap.get(NormalizedColorSensor.class, "colorRight2");

        initSensor(colorLeft1);
        initSensor(colorRight1);
        initSensor(colorLeft2);
        initSensor(colorRight2);

        last1 = last2 = last3 = null;

        turretEncoder    = hardwareMap.get(AnalogInput.class, "turretEncoder");
        leftFrontMotor   = hardwareMap.get(DcMotorEx.class, "LF");
        rightFrontMotor  = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor    = hardwareMap.get(DcMotorEx.class, "LB");
        rightBackMotor   = hardwareMap.get(DcMotorEx.class, "RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntakeMotor  = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftKickerServo  = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTurretServo  = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        frontTurretServo = hardwareMap.get(ServoImplEx.class, "frontTurretServo");

        leftTongueServo  = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

        leftShooterMotor  = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rgbLight  = hardwareMap.get(ServoImplEx.class, "light");
        zoneLight = hardwareMap.get(ServoImplEx.class, "zoneLight");
        leftTipper  = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");
        kickerEncoder  = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");
        switchCurrent  = hardwareMap.get(AnalogInput.class, "switchCurrent");

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        frontTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood.setDirection(ServoImplEx.Direction.REVERSE);

        pinpoint   = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        colorLeft  = hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "colorRight1");

        bottomLeftLaser  = hardwareMap.get(DigitalChannel.class, "bottomLeftLaser");
        bottomRightLaser = hardwareMap.get(DigitalChannel.class, "bottomRightLaser");
        bottomLeftLaser.setMode(DigitalChannel.Mode.INPUT);
        bottomRightLaser.setMode(DigitalChannel.Mode.INPUT);

        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(19.970472542, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();

        initSensor(colorLeft);
        initSensor(colorRight);

        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        velocityPID = new PIDVelocityController3(
                targetVelocity,
                KpMaintain, KiMaintain,
                KpRecovery, KiRecovery, KsRecovery,
                KsFF, KvFF);

        limelight.setPollRateHz(15);
        limelight.start();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(120);

        robotPos = pinpoint.getPosition();
        velX = pinpoint.getVelX(DistanceUnit.MM);
        velY = pinpoint.getVelY(DistanceUnit.MM);
        velH = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        predictedPos = vision.predictPos(allianceColor, robotPos, velX, velY, velH, secFar, rotationalSec, moveAwayFar);
        hoodHeight   = vision.hoodHeightRegressor(predictedPos, leftHood.getPosition(), allianceColor);
        turretPos    = vision.CalculateTurretAngle360NEW(xcoeff, ycoeff, predictedPos, gear, leftTurretServo.getPosition(), turretZeroCorrection, turretZeroCorrection2, allianceColor) + turretCorrection;
        groundDistance = vision.groundDistancePinpoint(robotPos.getX(DistanceUnit.METER), robotPos.getY(DistanceUnit.METER), allianceColor);
        flywheelSpeed  = Math.min(0, vision.CalculatedFlywheelSpeed(predictedPos, allianceColor));

        prism.configureSolidLayer(0, 0, 1, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, BRIGHT_ON, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, BRIGHT_ON, 0, 0, 0);

        leftTipper.setPosition(Globals.tipperRetracted);
        rightTipper.setPosition(Globals.tipperRetracted);

        lastColorTick = runTime.milliseconds();
        colorPhase = 0;
        prismSenseEnabled = false;

        // --- Open current log file ---
        // File persists across power cycles in /sdcard/FIRST/
        // Filename: TeleopCurrentLogging_YYYYMMDD_HHmmss.csv
        // On stop() it is renamed to include end time: ..._to_HHmmss.csv
        // If robot dies (no stop), file keeps start-only name (INTERRUPTED)
        long nowMs = System.currentTimeMillis();
        String opmodeName = getClass().getSimpleName();
        String startStr = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date(nowMs));
        currentLogPath = "/sdcard/FIRST/" + opmodeName + "_" + startStr + ".csv";
        lastLoggedTimeMs = nowMs;
        try {
            currentLogger = new BufferedWriter(new FileWriter(currentLogPath, false));
            currentLogger.write("wall_clock_ms,lf_A,rf_A,lb_A,rb_A,frontIntake_A,backIntake_A,lShooter_A,rShooter_A,motor_total_A,floodgate_A");
            currentLogger.newLine();
            currentLogger.flush();
        } catch (IOException e) {
            telemetry.addData("Logger Error", e.getMessage());
        }
        // Shared 5-file limit across ALL teleop opmodes in /sdcard/FIRST/
        File logDir = new File("/sdcard/FIRST/");
        File[] logFiles = logDir.listFiles((dir, name) -> name.endsWith(".csv"));
        if (logFiles != null && logFiles.length > MAX_TOTAL_LOG_FILES) {
            Arrays.sort(logFiles, (a, b) -> a.getName().compareTo(b.getName()));
            for (int i = 0; i < logFiles.length - MAX_TOTAL_LOG_FILES; i++) {
                logFiles[i].delete();
            }
        }
    }

    @Override
    public void init_loop() {
        switch (initStage) {
            case RESET:
                pinpoint.resetPosAndIMU();
                timer.reset();
                initStage = InitStage.WAIT_AFTER_RESET;
                break;
            case WAIT_AFTER_RESET:
                if (timer.milliseconds() >= 500) initStage = InitStage.RECAL_IMU;
                break;
            case RECAL_IMU:
                pinpoint.recalibrateIMU();
                timer.reset();
                initStage = InitStage.WAIT_AFTER_RECAL;
                break;
            case WAIT_AFTER_RECAL:
                if (timer.milliseconds() >= 1000) initStage = InitStage.SET_POSE;
                break;
            case SET_POSE:
                pinpoint.setPosition(PoseStorage.currentPose);
                timer.reset();
                initStage = InitStage.WAIT_AFTER_SET;
                break;
            case WAIT_AFTER_SET:
                if (timer.milliseconds() >= 1000) initStage = InitStage.UPDATE_AND_PRINT;
                break;
            case UPDATE_AND_PRINT:
                pinpoint.update();
                telemetry.addData("file current pose", PoseStorage.currentPose);
                telemetry.addData("pinpoint current pose", pinpoint.getPosition().toString());
                initStage = InitStage.DONE;
                break;
            case DONE:
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
        if (gamepad1.right_stick_button) {
            THIRD_BALL_CONFIRM_MS = 2000000000;
        }
        double dtMotorDraw = leftFrontMotor.getCurrent(CurrentUnit.AMPS) + rightFrontMotor.getCurrent(CurrentUnit.AMPS)
                + leftBackMotor.getCurrent(CurrentUnit.AMPS) + rightBackMotor.getCurrent(CurrentUnit.AMPS);
        double currentTime = lockedPostimer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        if (dt <= 0) dt = 0.001;

        frontIntakeMotor.setCurrentAlert(frontIntakeAmpLimit, CurrentUnit.AMPS);
        backIntakeMotor.setCurrentAlert(backIntakeAmpLimit, CurrentUnit.AMPS);
        boolean intakeStallFront = frontIntakeMotor.isOverCurrent();
        boolean intakeStallBack  = backIntakeMotor.isOverCurrent();

        if (intakeStallFront || intakeStallBack || dtMotorDraw > 30) {
            if (!firstStallLoop) {
                stallStartTime = currentTime;
                firstStallLoop = true;
            }
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            intakeIsStalled = true;
        } else if (intakeIsStalled) {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            if (currentTime - stallStartTime >= 3.0) {
                intakeIsStalled = false;
                firstStallLoop  = false;
                stallStartTime  = -1;
            }
        }

        telemetry.addData("LF", leftFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RF", rightFrontMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LB", leftBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RB", rightBackMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("frontIntakeMotor", frontIntakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("backIntakeMotor",  backIntakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("leftShooterMotor",  leftShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rightShooterMotor", rightShooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("total DT", dtMotorDraw);
        telemetry.addData("Third ball present", thirdBallPresent);

        boolean isFarZone    = robotPos.getX(DistanceUnit.MM) <= 0;
        double moveAway      = isFarZone ? moveAwayFar      : moveAwayClose;
        double moveAwayTurret = isFarZone ? moveAwayTurretFar : moveAwayTurretClose;
        double sec           = isFarZone ? secFar           : secClose;
        double turretSec     = isFarZone ? turretSecFar     : turretSecClose;

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        thirdBallPresent = bottomLeftLaser.getState() && bottomRightLaser.getState();
        if (intakeIsStalled) {
            zoneLight.setPosition(0.277);
        } else if (thirdBallPresent) {
            zoneLight.setPosition(1.0);
        } else {
            zoneLight.setPosition(0.0);
        }

        if (robotPos.getX(DistanceUnit.METER) > 0.6) {
            vision.GoalY      = 1.6488;
            vision.GoalXRed   = 1.8288;
            vision.GoalXBlue  = -1.8288;
        }

        if (firstLoop) {
            leftTurretServo.setPosition(0.5 + turretZeroCorrection);
            rightTurretServo.setPosition(0.5 + turretZeroCorrection);
            frontTurretServo.setPosition(0.5 + turretZeroCorrection);
            leftTongueServo.setPosition(Globals.tongueIntake);
            rightTongueServo.setPosition(Globals.tongueIntake);
            rightHood.setPosition(0.0);
            leftHood.setPosition(0.0);
            firstLoop = false;
        }

        currentVoltage = myControlHubVoltageSensor.getVoltage();
        double now = runTime.milliseconds();

        if (gamepad1.left_bumper) {
            SOTM = true;
        } else {
            if (!SOTMTesting) SOTM = false;
        }

        double frontIntakePower    = frontIntakeMotor.getVelocity() * 0.58;
        double intakeMotorDifference = frontIntakePower - targetIntakePower * 1620;

        if (((intakeMotorDifference < 500 && frontIntakePower > 100)))
        if (gamepad1.left_bumper) {
            moveAwayFar       = 1.2;  moveAwayTurretFar   = 1.4;
            secFar            = 0.8;  turretSecFar         = 1.1;
            rotationalSec     = 0.1;
            moveAwayClose     = 2;    moveAwayTurretClose  = 0.6;
            secClose          = 0.6;  turretSecClose       = 0.8;
        } else {
            moveAwayFar       = 0;    moveAwayTurretFar    = -0.2;
            secFar            = 0;    turretSecFar          = -0.2;
            rotationalSec     = 0;
            moveAwayClose     = 0;    moveAwayTurretClose   = -0.2;
            secClose          = 0;    turretSecClose         = -0.2;
        }

        if (now - lastPinpointUpdate >= pinpointThrottleMS) {
            robotPos = pinpoint.getPosition();
            if (LockedModeEnabled ^ ParkModeEnabled) runLockedPos(robotPos, dt);

            velX = pinpoint.getVelX(DistanceUnit.MM);
            velY = pinpoint.getVelY(DistanceUnit.MM);
            velH = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            predictedPos = vision.predictPos(allianceColor, robotPos, velX, velY, velH, sec, rotationalSec, moveAway);
            predDistance = Math.sqrt(
                    Math.pow(predictedPos.getX(DistanceUnit.CM) - robotPos.getX(DistanceUnit.CM), 2) +
                    Math.pow(predictedPos.getY(DistanceUnit.CM) - robotPos.getY(DistanceUnit.CM), 2));

            if (backButtonTrue) {
                hoodHeight     = vision.hoodHeightRegressor(predictedPos, leftHood.getPosition(), allianceColor);
                turretPos      = vision.CalculateTurretAngle360NEW(xcoeff, ycoeff, vision.predictPos(allianceColor, robotPos, velX, velY, velH, turretSec, rotationalSec, moveAwayTurret), gear, leftTurretServo.getPosition(), turretZeroCorrection, turretZeroCorrection2, allianceColor) + turretCorrection;
                groundDistance = vision.groundDistancePinpoint(robotPos.getX(DistanceUnit.METER), robotPos.getY(DistanceUnit.METER), allianceColor);
                flywheelSpeed  = Math.min(0, vision.CalculatedFlywheelSpeed(predictedPos, allianceColor));
            }
            pinpoint.update();
            lastPinpointUpdate = now;
        }

        // --- Current logger: log every loop (or throttled via currentLogIntervalMs) ---
        if (now - lastCurrentLogTime >= currentLogIntervalMs) {
            logMotorCurrents();
            lastCurrentLogTime = now;
        }

        timer.reset();

        boolean enableCombo  = gamepad2.right_stick_button;
        boolean disableCombo = gamepad2.left_stick_button;

        if (enableCombo && !lastEnableCombo) {
            prismSenseEnabled = true;
            last1 = last2 = last3 = null;
            colorPhase = 0;
        }
        if (disableCombo && !lastDisableCombo) {
            prismSenseEnabled = false;
            applyToLayer(prism, 0, ArtifactColor.UNKNOWN);
            applyToLayer(prism, 1, ArtifactColor.UNKNOWN);
            applyToLayer(prism, 2, ArtifactColor.UNKNOWN);
            showOnRgbLight(ArtifactColor.UNKNOWN);
        }
        lastEnableCombo  = enableCombo;
        lastDisableCombo = disableCombo;

        telemetry.addData("last loop time", runTime.milliseconds() - lastLoopTime);
        telemetry.addData("average loop time", runTime.milliseconds() / loops);
        telemetry.addData("loops", loops);
        loops = loops + 1;
        lastLoopTime = runTime.milliseconds();

        // -------------------- DRIVE --------------------
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.1;
        double rx =  gamepad1.right_stick_x;

        double denominator    = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        if (!LockedModeEnabled && !ParkModeEnabled) {
            leftFrontMotor.setPower(frontLeftPower);
            leftBackMotor.setPower(backLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            rightBackMotor.setPower(backRightPower);
        }

        // ----------------- RECYCLE TRIGGER -----------------
        if ((gamepad1.dpadDownWasReleased() || gamepad2.dpadRightWasReleased()) && !recyclerIsRunning) {
            recyclerIsRunning = true;
            started = false;
            intakeStarted = false;
            thirdBallConfirmTimerStarted = false;
            thirdBallConfirmTimer.reset();
        }
        cancelHeld = gamepad1.right_stick_button;
        if (cancelHeld && recyclerIsRunning) cancelRecycleTeleOp();
        if (recyclerIsRunning) updateRecycleTeleOp();
        lockIntakeKickerTongue = recyclerIsRunning || cancelHeld;
        if (lockIntakeKickerTongue) lockManualIntakeUntilTriggerRelease();

        // -------------------- INTAKE / KICKERS / TONGUE -------------------
        if (!lockIntakeKickerTongue) {
            if (gamepad1.left_trigger < 0.1) kickerStartDelayTimer.reset();

            boolean manualIntakePressed = gamepad1.right_trigger > MANUAL_INTAKE_TRIGGER_THRESHOLD;
            if (!manualIntakePressed) resetManualIntakeState();

            if (manualIntakePressed) {
                boolean runManualIntake = shouldRunManualIntake(manualIntakePressed);
                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);
                leftKickerServo.setPower(0.0);
                rightKickerServo.setPower(0.0);
                if (runManualIntake && !intakeIsStalled) {
                    frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
                    backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
                    targetIntakePower = Globals.frontIntakeIntakeSpeed;
                } else {
                    frontIntakeMotor.setPower(0.0);
                    backIntakeMotor.setPower(0.0);
                    targetIntakePower = 0;
                }
            } else if (gamepad1.b) {
                leftKickerServo.setPower(1.0);
                rightKickerServo.setPower(1.0);
            } else if (gamepad1.a && !intakeIsStalled) {
                frontIntakeMotor.setPower(-Globals.frontIntakeReverseSpeed);
                backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
                targetIntakePower = -Globals.frontIntakeReverseSpeed;
            } else if (gamepad1.left_trigger > 0.1) {
                leftTongueServo.setPosition(Globals.tongueShoot);
                rightTongueServo.setPosition(Globals.tongueShoot);
                if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay && !intakeIsStalled) {
                    leftKickerServo.setPower(Globals.rollerKickerShoot);
                    rightKickerServo.setPower(Globals.rollerKickerShoot);
                    frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed * shotSpeed);
                    backIntakeMotor.setPower(-Globals.backIntakeShootSpeed * shotSpeed);
                    targetIntakePower = -Globals.frontIntakeShootSpeed * shotSpeed;
                } else {
                    if (!intakeIsStalled) frontIntakeMotor.setPower(0.7);
                    leftKickerServo.setPower(Globals.rollerKickerShoot);
                    rightKickerServo.setPower(Globals.rollerKickerShoot);
                    targetIntakePower = 0.7;
                }
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
                targetIntakePower = 0;
                backIntakeMotor.setPower(0.0);
            }
        }

        // -------------------- OTHER CONTROLS --------------------
        if (gamepad1.x) closezone = (closezone == 1) ? 3 : 1;

        if (gamepad2.backWasReleased())  { if (leftTurretServo.getPosition() < 0.764) turretCorrection += 0.001; }
        if (gamepad2.startWasReleased()) { if (leftTurretServo.getPosition() > 0.246) turretCorrection -= 0.001; }
        if (gamepad2.dpad_up)   flywheelCorrection -= 2;
        if (gamepad2.dpad_down) flywheelCorrection += 2;
        if (gamepad2.y) flywheelCorrection = 0;
        if (gamepad2.x) pinpoint.recalibrateIMU();
        if (gamepad2.bWasReleased()) lowVoltage = !lowVoltage;

        if (predDistance > 10) {
            double goalX = allianceColor.equals("Red") ? -165 : 165;
            double actualDist = Math.sqrt(
                    Math.pow(robotPos.getX(DistanceUnit.CM) - goalX, 2) +
                    Math.pow(robotPos.getY(DistanceUnit.CM), 2));
            double predictedDist = Math.sqrt(
                    Math.pow(predictedPos.getX(DistanceUnit.CM) - goalX, 2) +
                    Math.pow(predictedPos.getY(DistanceUnit.CM), 2));
            shotSpeed = predictedDist > actualDist ? moveAwayFromGoalshotSpeed : moveTowardsGoalshotSpeed;
        } else if (lowVoltage) {
            shotSpeed = lowBatteryShotSpeed;
        } else if (LockedModeEnabled) {
            shotSpeed = lockedPosShotSpeed;
        } else {
            shotSpeed = defaultShotSpeed;
        }

        if (gamepad1.start) pinpoint.recalibrateIMU();

        if (gamepad1.right_bumper) {
            if (!rightBumperHeld) rightBumperFirstTime = true;
            else rightBumperFirstTime = false;
            rightBumperHeld = true;
        } else {
            rightBumperHeld      = false;
            rightBumperFirstTime = false;
            LockedModeEnabled    = false;
        }

        if (rightBumperFirstTime) {
            LockedModeEnabled = !LockedModeEnabled;
            AnchorPos = null;
            if (LockedModeEnabled) {
                AnchorPos  = pinpoint.getPosition();
                AnchorxPos = -1 * AnchorPos.getY(DistanceUnit.METER);
                AnchoryPos = AnchorPos.getX(DistanceUnit.METER);
                double heading = robotPos.getHeading(AngleUnit.DEGREES);
                double adjustedHeading = ((heading + 90) % 360 + 360) % 360;
                AnchorYaw = adjustedHeading;
            } else {
                lastXErrorLocked = 0; lastYErrorLocked = 0; lastHeadingErrorLocked = 0;
            }
        }

        if (gamepad2.leftBumperWasReleased()) {
            useTurret = !useTurret;
            if (!useTurret) {
                leftTurretServo.setPosition(0.5 + turretZeroCorrection);
                rightTurretServo.setPosition(0.5 + turretZeroCorrection);
                frontTurretServo.setPosition(0.5 + turretZeroCorrection);
            }
        }
        if (gamepad2.rightBumperWasReleased()) usePower = !usePower;

        if (gamepad1.dpad_left)  allianceColor = "Blue";
        if (gamepad1.dpad_right) allianceColor = "Red";

        if (gamepad1.x) vision.mt1pinpoint(red, blue, pinpoint, allianceColor, limelight);

        if (gamepad2.a) {
            if (allianceColor.equals("Blue"))
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -63, -65, AngleUnit.DEGREES, 0));
            else if (allianceColor.equals("Red"))
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -63, 65, AngleUnit.DEGREES, 0));
        }

        if (gamepad1.backWasReleased()) backButtonTrue = !backButtonTrue;

        if (backButtonTrue && !rightBumperTrue) {
            if ((leftShooterMotor.getVelocity() > 100) && (rightShooterMotor.getVelocity() == 0))
                flywheelCurrentVelocity = leftShooterMotor.getVelocity();
            else if ((rightShooterMotor.getVelocity() > 100) && (leftShooterMotor.getVelocity() == 0))
                flywheelCurrentVelocity = rightShooterMotor.getVelocity();
            else
                flywheelCurrentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2;

            if (!usePower) {
                targetVelocity = lockedFlywheelVelocity + flywheelCorrection;
                leftHood.setPosition(lockedHoodHeight);
                rightHood.setPosition(lockedHoodHeight);
            } else {
                targetVelocity = vision.CalculatedFlywheelSpeed(predictedPos, allianceColor) + flywheelCorrection;
                leftHood.setPosition(hoodHeight);
                rightHood.setPosition(hoodHeight);
            }

            velocityPID.setTargetVelocity(targetVelocity);
            velocityPID.setMaintainGains(KpMaintain, KiMaintain, KpDriveRecovery);
            velocityPID.setRecoveryGains(KpRecovery, KiRecovery, KsRecovery);
            velocityPID.setFeedforward(KsFF, KvFF);
            velocityPID.setRecoveryThreshold(recoveryThreshold);
            velocityPID.setMaintainThreshold(maintainThreshold);
            velocityPID.setVoltageStatus(lowVoltage);
            power = velocityPID.update(flywheelCurrentVelocity, currentVoltage, defaultVoltage);
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
        }

        if (!rightBumperTrue && !backButtonTrue) {
            leftTurretServo.setPosition(0.5 + turretZeroCorrection);
            rightTurretServo.setPosition(0.5 + turretZeroCorrection);
            frontTurretServo.setPosition(0.5 + turretZeroCorrection);
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }

        if (backButtonTrue && !rightBumperTrue) {
            if (useTurret) {
                leftTurretServo.setPosition(turretPos);
                rightTurretServo.setPosition(turretPos);
                frontTurretServo.setPosition(turretPos);
            }
        }

        if (gamepad1.y) {
            leftTipper.setPosition(Globals.tipperExtended);
            rightTipper.setPosition(Globals.tipperExtended);
        }

        telemetry.addData("Error", flywheelCurrentVelocity - targetVelocity);
        telemetry.addData("flywheel Correction", flywheelCorrection);
        telemetry.addData("turret Correction", turretCorrection);
        telemetry.addData("distance", groundDistance);
        telemetry.addData("low voltage?", lowVoltage);
        telemetry.addData("thirdBallPresent", thirdBallPresent);
        telemetry.addData("Prism Ball 1", last1);
        telemetry.addData("Prism Ball 2", last2);
        telemetry.addData("Prism Predicted Ball 3", last3);
        telemetry.addData("thirdBallConfirmStarted", thirdBallConfirmTimerStarted);
        telemetry.addData("thirdBallConfirmMs", thirdBallConfirmTimer.milliseconds());
        telemetry.addData("recycleIntakeMaxMs", RECYCLE_INTAKE_MAX_MS);

        if (now - lastTelemetryUpdate >= telemeteryThrottleMS) {
            telemetry.addData("Predicted Pos", predictedPos);
            telemetry.addData("Robot Pos", robotPos);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("PIDF Power", power);
            telemetry.addData("looptime", timer.milliseconds());
            telemetry.update();
            lastTelemetryUpdate = now;
        }

        if (prismSenseEnabled && (now - lastColorTick >= colorTickMs)) {
            lastColorTick = now;
            switch (colorPhase) {
                case 0: {
                    Reading L1 = readAndClassify(colorLeft1);
                    Reading R1 = readAndClassify(colorRight1);
                    ArtifactColor overall1 = combineByConfidence(L1, R1);
                    if (last1 == null || overall1 != last1) applyToLayer(prism, 0, overall1);
                    last1 = overall1;
                    showOnRgbLight(overall1);
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
                    ArtifactColor overall3 = predictThirdBallColor(last1, last2, thirdBallPresent);
                    if (last3 == null || overall3 != last3) applyToLayer(prism, 2, overall3);
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
        try {
            if (currentLogger != null) {
                currentLogger.flush();
                currentLogger.close();
                // Rename to include end time: OpMode_start_to_end.csv
                String endStr   = new SimpleDateFormat("HHmmss", Locale.US).format(new Date(lastLoggedTimeMs));
                String finalPath = currentLogPath.replace(".csv", "_to_" + endStr + ".csv");
                new File(currentLogPath).renameTo(new File(finalPath));
            }
        } catch (IOException ignored) { }
    }

    // --- Logging ---
    private void logMotorCurrents() {
        if (currentLogger == null) return;
        try {
            double lf = leftFrontMotor.getCurrent(CurrentUnit.AMPS);
            double rf = rightFrontMotor.getCurrent(CurrentUnit.AMPS);
            double lb = leftBackMotor.getCurrent(CurrentUnit.AMPS);
            double rb = rightBackMotor.getCurrent(CurrentUnit.AMPS);
            double fi = frontIntakeMotor.getCurrent(CurrentUnit.AMPS);
            double bi = backIntakeMotor.getCurrent(CurrentUnit.AMPS);
            double ls = leftShooterMotor.getCurrent(CurrentUnit.AMPS);
            double rs = rightShooterMotor.getCurrent(CurrentUnit.AMPS);
            double motorTotal = lf + rf + lb + rb + fi + bi + ls + rs;

            // GoBilda Floodgate: 0-3.3V analog = 0-80A (true total system current)
            double floodgate = (switchCurrent.getVoltage() / 3.3) * 80.0;

            lastLoggedTimeMs = System.currentTimeMillis();
            currentLogger.write(String.format(Locale.US,
                    "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                    lastLoggedTimeMs, lf, rf, lb, rb, fi, bi, ls, rs, motorTotal, floodgate));
            currentLogger.newLine();

            // Send all channels to FTC Dashboard as live graphs
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current/01_lf_A",          lf);
            packet.put("current/02_rf_A",          rf);
            packet.put("current/03_lb_A",          lb);
            packet.put("current/04_rb_A",          rb);
            packet.put("current/05_frontIntake_A", fi);
            packet.put("current/06_backIntake_A",  bi);
            packet.put("current/07_lShooter_A",    ls);
            packet.put("current/08_rShooter_A",    rs);
            packet.put("current/09_motor_total_A", motorTotal);
            packet.put("current/10_floodgate_A",   floodgate);
            dashboard.sendTelemetryPacket(packet);
        } catch (IOException ignored) { }
    }

    private double safeGetCurrent(LynxModule hub) {
        if (hub == null) return 0;
        try { return hub.getCurrent(CurrentUnit.AMPS); } catch (Exception e) { return 0; }
    }

    // --- Helpers ---
    private void applyToLayer(CustomGoBildaPrismRgbLedDriver prism, int layer, ArtifactColor c) {
        int[] rgb;
        switch (c) {
            case GREEN:   rgb = RGB_GREEN;  break;
            case PURPLE:  rgb = RGB_PURPLE; break;
            case RED:     rgb = RGB_RED;    break;
            default:      rgb = RGB_OFF;    break;
        }
        prism.setLayerColor(layer, rgb[0], rgb[1], rgb[2]);
    }

    private ArtifactColor predictThirdBallColor(ArtifactColor first, ArtifactColor second, boolean thirdPresent) {
        if (!thirdPresent) return ArtifactColor.UNKNOWN;
        if (first == null || second == null || first == ArtifactColor.UNKNOWN || second == ArtifactColor.UNKNOWN
                || first == ArtifactColor.RED || second == ArtifactColor.RED) return ArtifactColor.UNKNOWN;
        int greenCount = 0, purpleCount = 0;
        if (first  == ArtifactColor.GREEN)  greenCount++;
        if (second == ArtifactColor.GREEN)  greenCount++;
        if (first  == ArtifactColor.PURPLE) purpleCount++;
        if (second == ArtifactColor.PURPLE) purpleCount++;
        if (purpleCount == 2) return ArtifactColor.GREEN;
        if (purpleCount == 1 && greenCount == 1) return ArtifactColor.PURPLE;
        if (greenCount == 2) return ArtifactColor.RED;
        return ArtifactColor.UNKNOWN;
    }

    private boolean shouldRunManualIntake(boolean intakePressed) {
        if (!intakePressed) { resetManualIntakeState(); return false; }
        if (!manualIntakeWasPressed) {
            manualIntakeWasPressed = true;
            manualIntakeLatchedOff = false;
            manualIntakeThirdBallConfirmTimerStarted = false;
            manualIntakeThirdBallConfirmTimer.reset();
            if (thirdBallPresent) { manualIntakeAlreadyFullMode = true; manualIntakeAlreadyFullTimer.reset(); }
            else { manualIntakeAlreadyFullMode = false; manualIntakeAlreadyFullTimer.reset(); }
        }
        if (manualIntakeLatchedOff) return false;
        if (manualIntakeAlreadyFullMode) {
            if (manualIntakeAlreadyFullTimer.milliseconds() >= MANUAL_INTAKE_ALREADY_FULL_RUN_MS) {
                manualIntakeLatchedOff = true; manualIntakeAlreadyFullMode = false;
                manualIntakeThirdBallConfirmTimerStarted = false; manualIntakeThirdBallConfirmTimer.reset();
                return false;
            }
            return true;
        }
        if (thirdBallPresent) {
            if (!manualIntakeThirdBallConfirmTimerStarted) { manualIntakeThirdBallConfirmTimerStarted = true; manualIntakeThirdBallConfirmTimer.reset(); }
            if (manualIntakeThirdBallConfirmTimer.milliseconds() >= MANUAL_INTAKE_THIRD_BALL_CONFIRM_MS) { manualIntakeLatchedOff = true; return false; }
        } else {
            manualIntakeThirdBallConfirmTimerStarted = false; manualIntakeThirdBallConfirmTimer.reset();
        }
        return true;
    }

    private void resetManualIntakeState() {
        manualIntakeWasPressed = false; manualIntakeLatchedOff = false;
        manualIntakeThirdBallConfirmTimerStarted = false; manualIntakeAlreadyFullMode = false;
        manualIntakeThirdBallConfirmTimer.reset(); manualIntakeAlreadyFullTimer.reset();
    }

    private void lockManualIntakeUntilTriggerRelease() {
        boolean intakePressed = gamepad1.right_trigger > MANUAL_INTAKE_TRIGGER_THRESHOLD;
        manualIntakeWasPressed = intakePressed; manualIntakeLatchedOff = intakePressed;
        manualIntakeThirdBallConfirmTimerStarted = false; manualIntakeAlreadyFullMode = false;
        manualIntakeThirdBallConfirmTimer.reset(); manualIntakeAlreadyFullTimer.reset();
    }

    private void updateRecycleTeleOp() {
        if (!started) { started = true; intakeStarted = false; thirdBallConfirmTimerStarted = false; recyclerTimer.reset(); thirdBallConfirmTimer.reset(); }
        double t = recyclerTimer.milliseconds();
        if (t < RECYCLE_TONGUE_DOWN_MS) {
            leftTongueServo.setPosition(Globals.tongueRecycle); rightTongueServo.setPosition(Globals.tongueRecycle);
            leftKickerServo.setPower(Globals.rollerKickerRecycle); rightKickerServo.setPower(Globals.rollerKickerRecycle);
            frontIntakeMotor.setPower(0.0); targetIntakePower = 0; backIntakeMotor.setPower(0.0);
            return;
        }
        leftTongueServo.setPosition(Globals.tongueIntake); rightTongueServo.setPosition(Globals.tongueIntake);
        leftKickerServo.setPower(0.0); rightKickerServo.setPower(0.0);
        if (!intakeIsStalled) { frontIntakeMotor.setPower(-1.0); targetIntakePower = -1; backIntakeMotor.setPower(-1.0); }
        if (!intakeStarted) { intakeStarted = true; thirdBallConfirmTimerStarted = false; thirdBallConfirmTimer.reset(); }
        double intakeRunTime = t - RECYCLE_TONGUE_DOWN_MS;
        if (intakeRunTime >= RECYCLE_INTAKE_MAX_MS) { finishRecycleTeleOp(); return; }
        if (thirdBallPresent) {
            if (!thirdBallConfirmTimerStarted) { thirdBallConfirmTimerStarted = true; thirdBallConfirmTimer.reset(); }
            if (thirdBallConfirmTimer.milliseconds() >= THIRD_BALL_CONFIRM_MS) finishRecycleTeleOp();
        } else { thirdBallConfirmTimerStarted = false; thirdBallConfirmTimer.reset(); }
    }

    private void finishRecycleTeleOp() {
        leftTongueServo.setPosition(Globals.tongueIntake); rightTongueServo.setPosition(Globals.tongueIntake);
        leftKickerServo.setPower(0.0); rightKickerServo.setPower(0.0);
        frontIntakeMotor.setPower(0.0); targetIntakePower = 0; backIntakeMotor.setPower(0.0);
        recyclerIsRunning = false; started = false; intakeStarted = false;
        thirdBallConfirmTimerStarted = false; thirdBallConfirmTimer.reset();
    }

    private void cancelRecycleTeleOp() {
        recyclerIsRunning = false; started = false; intakeStarted = false;
        thirdBallConfirmTimerStarted = false; thirdBallConfirmTimer.reset();
        leftTongueServo.setPosition(Globals.tongueIntake); rightTongueServo.setPosition(Globals.tongueIntake);
        leftKickerServo.setPower(0.0); rightKickerServo.setPower(0.0);
        frontIntakeMotor.setPower(0.0); targetIntakePower = 0; backIntakeMotor.setPower(0.0);
    }

    private void initSensor(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(true);
        sensor.setGain(COLOR_SENSOR_GAIN);
    }

    private Reading readAndClassify(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBuf);
        float h = hsvBuf[0], s = hsvBuf[1], v = hsvBuf[2];
        if (v < MIN_VALUE && s < MIN_SATURATION) return new Reading(ArtifactColor.UNKNOWN, h, s, v);
        float dGreen = hueDistance(h, GREEN_HUE_CENTER), dPurple = hueDistance(h, PURPLE_HUE_CENTER);
        return new Reading((dGreen <= dPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE, h, s, v);
    }

    private float hueDistance(float a, float b) {
        float d = Math.abs(a - b);
        return Math.min(d, 360f - d);
    }

    private ArtifactColor combineByConfidence(Reading left, Reading right) {
        boolean lc = left.color != ArtifactColor.UNKNOWN, rc = right.color != ArtifactColor.UNKNOWN;
        if (!lc && !rc) return ArtifactColor.UNKNOWN;
        if (lc && !rc)  return left.color;
        if (!lc && rc)  return right.color;
        if (left.conf > right.conf) return left.color;
        if (right.conf > left.conf) return right.color;
        if (left.val > right.val)   return left.color;
        if (right.val > left.val)   return right.color;
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

    public void runLockedPos(Pose2D robotPos, double dt) {
        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = ((heading + 90) % 360 + 360) % 360;
        double headingError = AnchorYaw - adjustedHeading;
        if (LockedModeEnabled) {
            if (headingError > 180) headingError -= 360;
            else if (headingError < -180) headingError += 360;
        } else {
            headingError = ((headingError + 180) % 360) - 180;
        }
        double headingDerivative = (headingError - lastHeadingErrorLocked) / dt;
        double rotPower = kPRot_Lock * Math.signum(headingError) * Math.sqrt(Math.abs(headingError)) + kDRot_Lock * headingDerivative;
        lastHeadingErrorLocked = headingError;

        double xPos = -1 * robotPos.getY(DistanceUnit.METER);
        double yPos = robotPos.getX(DistanceUnit.METER);
        double xError = (AnchorxPos - xPos) * 100;
        double yError = (AnchoryPos - yPos) * 100;
        double distanceFromAnchor = Math.sqrt(xError * xError + yError * yError);
        double relativeAngle = Math.atan2(yError, xError) - Math.toRadians(adjustedHeading);
        xError = distanceFromAnchor * -Math.sin(relativeAngle);
        yError = distanceFromAnchor *  Math.cos(relativeAngle);

        double xDerivative = (xError - lastXErrorLocked) / dt;
        double yDerivative = (yError - lastYErrorLocked) / dt;
        double xPower, yPower;
        if (distanceFromAnchor < PIDDeadband) {
            xPower = kP_Lock_Small * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xDerivative;
            yPower = kP_Lock_Small * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yDerivative;
        } else {
            xPower = kP_Lock * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xDerivative;
            yPower = kP_Lock * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yDerivative;
        }
        lastXErrorLocked = xError;
        lastYErrorLocked = yError;

        leftFrontMotor.setPower(Math.max(-1, Math.min(1, yPower + xPower - rotPower)));
        rightFrontMotor.setPower(Math.max(-1, Math.min(1, yPower - xPower + rotPower)));
        leftBackMotor.setPower(Math.max(-1, Math.min(1, yPower - xPower - rotPower)));
        rightBackMotor.setPower(Math.max(-1, Math.min(1, yPower + xPower + rotPower)));
    }
}
