package org.firstinspires.ftc.teamcode.opmode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.function.Function;

//@Disabled
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BlueFarAutoCycling extends LinearOpMode {

    // Initialize all hardware
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
    AnalogInput kickerEncoder;
    AnalogInput turretEncoder;
    NormalizedColorSensor leftIntakeColorSensor;
    NormalizedColorSensor rightIntakeColorSensor;

    ServoImplEx leftHood;
    ServoImplEx rightHood;

    // Initialize all parameters
    public static double minVelIntaking = 40;
    public static double minAccelIntaking = -40;
    public static double maxAccelIntaking = 40;

    public static double minVelDrive = 80;
    public static double minAccelDrive = -70;
    public static double maxAccelDrive = 70;

    public static double shooterStartDelay = 0.3;
    public static double shootingDelay = 3.2;

    public static double intakeStopDelay = 0.4;

    public static double turretStartPos = 0.385;
    public static double turretShootPos = 0.422;

    public static double plainKickerPower = 0.0;

    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;

    public static boolean kickerPIDEnabled = true;

    public static double kickerTarget = Globals.KICKER_IDLE;

    // Stall detection
    public static double stallWindowMs   = 200;   // how long we wait to see movement
    public static double stallMinDelta   = 0.1;  // minimum encoder change to consider "moving"
    public static double stallRecoveryMs = 1000;   // how long to hold in IDLE before resuming shot

    public static double stallGraceMs    = 400;  // ms


    int obeliskID = -1;

    // Initialize any instances of classes
    ElapsedTime recyclerTimer = new ElapsedTime();

    visionTools vision = new visionTools();

    CRAxonPDController kickerPID = new CRAxonPDController();
    DroidForceMethods DFM = new DroidForceMethods();


    @Override
    public void runOpMode() {

        // Instantiate MecanumDrive
        Pose2d startPose = new Pose2d(63, -14.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Map motors and servos
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
        frontIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");

        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

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

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

        //Limelight
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);

        limelight.setPollRateHz(100);
        limelight.start();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pre-Auto robot initlization. MUST BE LAST
        leftHood.setPosition(0.4);
        rightHood.setPosition(0.4);

        leftTurretServo.setPosition(turretStartPos);
        rightTurretServo.setPosition(turretStartPos);

        plainKickerPower = 0.0;
        kickerPIDEnabled = true;

        // Set kickers during init
        while(!opModeIsActive() && !isStopRequested()) {

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

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new ParallelAction(

                // Thread 1: Pathing + General Robot
                new SequentialAction(

                        new setShooter(leftShooterMotor, rightShooterMotor, Globals.defaultFarZonePowerAuto),
                        new SleepAction(2),
                        new kickerShoot(),

                        new SleepAction(shootingDelay),

                        new ParallelAction(
                                // Go To Intake Last Spike Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(36, -28), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false),
                                new setIntake(frontIntakeMotor, backIntakeMotor, -1.0)
                        ),

                        // Intake balls path

                        new PathFromCurrentPose(drive, pose ->
                                drive.actionBuilder(pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(36, -60), Math.toRadians(270),
                                                new TranslationalVelConstraint(minVelDrive),
                                                new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                        )
                                        .build()
                        ),

                        // Go to Shooting Position

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new setIntake(frontIntakeMotor, backIntakeMotor, 0.0)
                                )
                        ),

                        new SleepAction(shooterStartDelay),

                        new kickerShoot(),

                        new SleepAction(shootingDelay),


                        // Intake Cycle 1
                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -50), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false)
                        ),

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new setIntake(frontIntakeMotor, backIntakeMotor, 0.0)
                                )
                        ),

                        new SleepAction(shooterStartDelay),

                        new kickerShoot(),

                        new SleepAction(shootingDelay),

                        //Intake Cycle 2

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -50), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false)
                        ),

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new setIntake(frontIntakeMotor, backIntakeMotor, 0.0)
                                )
                        ),

                        new SleepAction(shooterStartDelay),

                        new kickerShoot(),

                        new SleepAction(shootingDelay),

                        //Intake Cycle 3

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -50), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -62), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false)
                        ),

                        new ParallelAction(
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(62, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(1),
                                        new setIntake(frontIntakeMotor, backIntakeMotor, 0.0)
                                )
                        ),

                        new SleepAction(shooterStartDelay),

                        new kickerShoot(),

                        new SleepAction(shootingDelay)

                ),

                // Thread 2: Kicker PID
                new SequentialAction(
                        new startKickerPID(leftKickerServo, rightKickerServo)
                ),

                // Thread 3: Turret Tracking
                new SequentialAction(
                        // new startTurretTracking(leftTurretServo, rightTurretServo, limelight, vision)
                )

        ));

        sleep(1000);

    }

    /**
     * Lazy action that, on first run, builds a real RR Action
     * from the *current* pose using the provided factory.
     *
     * This avoids any type issues with specific builder classes.
     */
    public class PathFromCurrentPose implements Action {
        private final MecanumDrive drive;
        private final Function<Pose2d, Action> actionFactory;
        private Action inner = null;

        public PathFromCurrentPose(MecanumDrive drive, Function<Pose2d, Action> actionFactory) {
            this.drive = drive;
            this.actionFactory = actionFactory;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (inner == null) {
                Pose2d now = drive.localizer.getPose();
                inner = actionFactory.apply(now);
            }
            return inner.run(telemetryPacket);
        }
    }

    public class startKickerPID implements Action {

        private final CRServoImplEx leftKickerServo;
        private final CRServoImplEx rightKickerServo;

        // Stall detection state
        private final ElapsedTime stallTimer = new ElapsedTime();
        private boolean stallSampleValid = false;
        private double stallSampleTimeMs = 0.0;
        private double stallSamplePos = 0.0;

        private boolean recoveringFromStall = false;
        private double lastShootPower = 0.0; // remember what power we were shooting with
        private double recoveryStartTimeMs = 0.0;

        // Grace-period tracking
        private boolean wasShootingOpenLoop = false;
        private double openLoopStartTimeMs = 0.0;

        public startKickerPID(CRServoImplEx leftKickerServo, CRServoImplEx rightKickerServo){
            this.leftKickerServo = leftKickerServo;
            this.rightKickerServo = rightKickerServo;
            stallTimer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double nowMs = stallTimer.milliseconds();
            double encoderVoltage = kickerEncoder.getVoltage();
            double processedEncoderValue =
                    DFM.zeroAndNormalizeAxonEncoder(encoderVoltage, Globals.KICKER_ZERO);

            // Are we currently shooting open-loop?
            boolean shootingOpenLoop = !kickerPIDEnabled && Math.abs(plainKickerPower) > 0.01;

            // Just entered open-loop: start grace timer and clear sample
            if (shootingOpenLoop && !wasShootingOpenLoop) {
                openLoopStartTimeMs = nowMs;
                stallSampleValid = false;
            }

            // ---------- STALL RECOVERY STATE MACHINE ----------

            if (recoveringFromStall) {
                // Hold at IDLE with PID for stallRecoveryMs
                kickerPIDEnabled = true;
                kickerTarget = Globals.KICKER_IDLE;
                plainKickerPower = 0.0;

                if (nowMs - recoveryStartTimeMs >= stallRecoveryMs) {
                    // Done recovering: restart open-loop shooting
                    recoveringFromStall = false;
                    stallSampleValid = false; // new window for next stall detection

                    kickerPIDEnabled = false;
                    plainKickerPower = lastShootPower;
                }

            } else {
                // Only detect stall while shooting open-loop *after* grace period
                if (shootingOpenLoop && (nowMs - openLoopStartTimeMs) >= stallGraceMs) {

                    if (!stallSampleValid) {
                        // Take first sample
                        stallSampleValid = true;
                        stallSampleTimeMs = nowMs;
                        stallSamplePos = processedEncoderValue;
                    } else {
                        double dt = nowMs - stallSampleTimeMs;
                        if (dt >= stallWindowMs) {
                            double dPos = Math.abs(processedEncoderValue - stallSamplePos);
                            if (dPos < stallMinDelta) {
                                // ----- STALL DETECTED -----
                                recoveringFromStall = true;
                                recoveryStartTimeMs = nowMs;

                                lastShootPower = plainKickerPower; // remember shoot power

                                kickerPIDEnabled = true;
                                kickerTarget = Globals.KICKER_IDLE;
                                plainKickerPower = 0.0;

                                stallSampleValid = false;
                            } else {
                                // Still moving: refresh sample window
                                stallSampleTimeMs = nowMs;
                                stallSamplePos = processedEncoderValue;
                            }
                        }
                    }
                } else if (!shootingOpenLoop) {
                    // Not in open-loop shooting mode, don't track stall
                    stallSampleValid = false;
                }
            }

            // Remember last open-loop state
            wasShootingOpenLoop = shootingOpenLoop;

            // ---------- DRIVE THE SERVOS ----------

            if (kickerPIDEnabled) {
                double power = kickerPID.Output(
                        Globals.KICKER_kP,
                        Globals.KICKER_kD,
                        kickerTarget,
                        processedEncoderValue
                );

                leftKickerServo.setPower(power);
                rightKickerServo.setPower(power);
            } else {
                // Open-loop mode: use plainKickerPower
                leftKickerServo.setPower(plainKickerPower);
                rightKickerServo.setPower(plainKickerPower);
            }

            return true;  // keep this action running for the entire auto
        }
    }



    public class updatePose implements Action {

        private MecanumDrive drive = null;

        public updatePose(MecanumDrive drive){

            this.drive = drive;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            drive.updatePoseEstimate();
            drive.localizer.update();

            return false;

        }
    }

    public class getObeliskID implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            obeliskID = vision.ObeliskID(limelight);

            return false;

        }
    }

    // DO NOT USE FOR REGULAR USE. THIS IS A TEST METHOD
    public class debugTelemetry implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            telemetry.addData("Obelisk ID: ", obeliskID);
            telemetry.update();

            return true;

        }
    }

    public class kickerPIDEnableDisable implements Action {

        private String status = null;

        public kickerPIDEnableDisable(String status){

            this.status = status;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if ("Enabled".equals(status)) {
                kickerPIDEnabled = true;
            } else if ("Disabled".equals(status)) {
                kickerPIDEnabled = false;
            }

            return false;

        }
    }

    public class recycle implements Action {

        private final DcMotorEx frontIntakeMotor;

        // How many recycle cycles to run based on obeliskID
        private int cyclesToRun = 0;
        private int cyclesCompleted = 0;

        private String currentPattern = null;

        // The current recycleArtifact action we’re driving
        private Action currentRecycle = null;

        private boolean initialized = false;

        public recycle(String currentPattern, DcMotorEx frontIntakeMotor) {
            this.currentPattern = currentPattern;
            this.frontIntakeMotor = frontIntakeMotor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // One-time init: decide how many cycles based on obeliskID
            if (!initialized) {
                initialized = true;

                if (currentPattern.equals("PPG")) {
                    if (obeliskID == 23) {
                        cyclesToRun = 0;
                    } else if (obeliskID == 22) {
                        cyclesToRun = 1;
                    } else if (obeliskID == 21) {
                        cyclesToRun = 2;
                    } else {
                        cyclesToRun = 0;
                    }
                } else if (currentPattern.equals("PGP")) {
                    if (obeliskID == 23) {
                        cyclesToRun = 2;
                    } else if (obeliskID == 22) {
                        cyclesToRun = 0;
                    } else if (obeliskID == 21) {
                        cyclesToRun = 1;
                    } else {
                        cyclesToRun = 0;
                    }
                } else if (currentPattern.equals("GPP")) {
                    if (obeliskID == 23) {
                        cyclesToRun = 1;
                    } else if (obeliskID == 22) {
                        cyclesToRun = 2;
                    } else if (obeliskID == 21) {
                        cyclesToRun = 0;
                    } else {
                        cyclesToRun = 0;
                    }
                }

                // If nothing to do, finish immediately
                if (cyclesToRun == 0) {
                    return false;
                }
            }

            // If we don't have an active recycleArtifact, start one
            if (currentRecycle == null) {
                currentRecycle = new recycleArtifact(frontIntakeMotor);
            }

            // Drive the current recycleArtifact
            boolean stillRunning = currentRecycle.run(telemetryPacket);

            if (stillRunning) {
                // Let RR call us again next loop
                return true;
            }

            // This recycleArtifact just finished
            cyclesCompleted++;

            if (cyclesCompleted >= cyclesToRun) {
                // All cycles done
                currentRecycle = null;
                return false; // Action finished
            } else {
                // Start another recycle cycle
                currentRecycle = new recycleArtifact(frontIntakeMotor);
                return true; // Still more work to do
            }
        }
    }

    public class recycleArtifact implements Action {

        private final DcMotorEx frontIntakeMotor;

        // Total time to run the recycle sequence (ms)
        private final double totalMs;
        // Delay between commanding KICKER_RECYCLE and starting intake (ms)
        private final double intakeDelayMs;
        // Delay between intake starting and kicker going back up (ms)
        private final double kickerUpDelayMs;

        private final ElapsedTime recyclerTimer = new ElapsedTime();
        private boolean started = false;
        private boolean intakeStarted = false;

        // Default timings constructor
        public recycleArtifact(DcMotorEx frontIntakeMotor) {
            this.frontIntakeMotor = frontIntakeMotor;

            this.totalMs = recyclingDelay;
            this.intakeDelayMs = recyclingIntakeDelay;
            this.kickerUpDelayMs = recyclingKickerUpDelay;
        }

        // Optional: custom timings constructor
        public recycleArtifact(DcMotorEx frontIntakeMotor, double totalMs, double intakeDelayMs) {
            this.frontIntakeMotor = frontIntakeMotor;
            this.totalMs = totalMs;
            this.intakeDelayMs = intakeDelayMs;
            this.kickerUpDelayMs = recyclingKickerUpDelay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // First call: move kicker, start timer, keep intake OFF
            if (!started) {
                started = true;
                recyclerTimer.reset();

                kickerTarget = Globals.KICKER_RECYCLE; // kicker down
                frontIntakeMotor.setPower(0.0);        // don't start intake yet
            }

            double t = recyclerTimer.milliseconds();

            // --- KICKER TIMING ---
            double kickerUpTime = intakeDelayMs + kickerUpDelayMs;
            if (t < kickerUpTime) {
                kickerTarget = Globals.KICKER_RECYCLE;
            } else {
                kickerTarget = Globals.KICKER_IDLE;
            }

            // --- INTAKE TIMING ---
            if (!intakeStarted && t >= intakeDelayMs) {
                frontIntakeMotor.setPower(-1.0);
                intakeStarted = true;
            }

            // After totalMs, stop everything and finish the action
            if (t >= totalMs) {
                kickerTarget = Globals.KICKER_IDLE;
                frontIntakeMotor.setPower(0.0);

                return false;
            }

            return true;
        }
    }

    public class setTurret implements Action {

        private final double pos;

        public setTurret(double pos){
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftTurretServo.setPosition(pos);
            rightTurretServo.setPosition(pos);

            return false;
        }
    }

    public class startTurretTracking implements Action {

        private final ServoImplEx leftTurretServo;
        private final ServoImplEx rightTurretServo;
        private final Limelight3A limelight;

        private final visionTools visionTools;

        public startTurretTracking(ServoImplEx leftTurretServo, ServoImplEx rightTurretServo, Limelight3A limelight, visionTools visionTools){
            this.leftTurretServo = leftTurretServo;
            this.rightTurretServo = rightTurretServo;
            this.limelight = limelight;
            this.visionTools = visionTools;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double position = leftTurretServo.getPosition();

            leftTurretServo.setPosition(visionTools.adjustedTurretAngle(position, limelight,1));
            rightTurretServo.setPosition(visionTools.adjustedTurretAngle(position, limelight,1));

            return true;
        }
    }

    public class setShooter implements Action {

        private final DcMotorEx topShooterMotor;
        private final DcMotorEx bottomShooterMotor;

        double power;

        public setShooter(DcMotorEx topShooterMotor, DcMotorEx bottomShooterMotor, double power){
            this.topShooterMotor = topShooterMotor;
            this.bottomShooterMotor = bottomShooterMotor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            topShooterMotor.setPower(power);
            bottomShooterMotor.setPower(power);

            return false;
        }
    }

    public class kickerShoot implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            kickerPIDEnabled = false;
            plainKickerPower = Globals.kickerShoot * 0.3;
            frontIntakeMotor.setPower(-0.7); //-1.0
            backIntakeMotor.setPower(-0.7); //-1.0

            return false;
        }
    }

    public class kickerIdle implements Action {

        private final boolean stopIntake;

        public kickerIdle(boolean stopIntake){
            this.stopIntake = stopIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            kickerTarget = Globals.KICKER_IDLE;
            kickerPIDEnabled = true;

            if (stopIntake) {
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }

            return false;
        }
    }

    public class setIntake implements Action {

        private final DcMotorEx frontIntakeMotor;
        private final DcMotorEx backIntakeMotor;

        double power;

        public setIntake(DcMotorEx frontIntakeMotor, DcMotorEx backIntakeMotor, double power){
            this.frontIntakeMotor = frontIntakeMotor;
            this.backIntakeMotor = backIntakeMotor;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (power != 0.0) {
                frontIntakeMotor.setPower(power);
                backIntakeMotor.setPower(1);
            } else {
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }
            return false;
        }
    }

}
