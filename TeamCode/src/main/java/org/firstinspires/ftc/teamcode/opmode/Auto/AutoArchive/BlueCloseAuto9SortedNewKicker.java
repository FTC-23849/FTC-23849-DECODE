package org.firstinspires.ftc.teamcode.opmode.Auto.AutoArchive;

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

import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.function.Function;

@Disabled
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class BlueCloseAuto9SortedNewKicker extends LinearOpMode {

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
    AnalogInput kickerEncoder;
    AnalogInput turretEncoder;
    NormalizedColorSensor leftIntakeColorSensor;
    NormalizedColorSensor rightIntakeColorSensor;

    ServoImplEx leftHood;
    ServoImplEx rightHood;

    ServoImplEx leftTongueServo;
    ServoImplEx rightTongueServo;

    // Initialize all parameters
    public static double minVelIntaking = 40;
    public static double minAccelIntaking = -40;
    public static double maxAccelIntaking = 40;

    public static double minVelDrive = 70;
    public static double minAccelDrive = -60;
    public static double maxAccelDrive = 60;

    public static double shooterStartDelay = 0.3;
    public static double shootingDelay = 3.5;

    public static double intakeStopDelay = 0.4;

    public static double turretStartPos = 0.34;
    public static double turretShootPos = 0.422;

    public static double plainKickerPower = 0.0;

    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;

    public static boolean kickersStarted = false;

    public static double shootingSpeed = -0.67;

    int obeliskID = -1;

    // Initialize any instances of classes
    ElapsedTime recyclerTimer = new ElapsedTime();

    ElapsedTime shooterTimer = new ElapsedTime();

    visionTools vision = new visionTools();

    @Override
    public void runOpMode() {

        // Instantiate MecanumDrive
        Pose2d startPose = new Pose2d(-54.5, -45, Math.toRadians(225));
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

        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

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

        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

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
        leftHood.setPosition(0.25);
        rightHood.setPosition(0.25);

        leftTurretServo.setPosition(turretStartPos);
        rightTurretServo.setPosition(turretStartPos);

        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);


        plainKickerPower = 0.0;

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new ParallelAction(

                // Thread 1: Pathing + General Robot
                new SequentialAction(

                        // Preloads
                        new ParallelAction(
                                // Preload Path from known start pose
                                drive.actionBuilder(startPose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-12, -15), Math.toRadians(270),
                                                new TranslationalVelConstraint(minVelDrive),
                                                new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                        )
                                        .build(),

                                new setShooter(leftShooterMotor, rightShooterMotor, shootingSpeed)
                        ),
                        new getObeliskID(),
                        new SleepAction(0.3),
                        new setTurret(turretShootPos),
                        new recycle("PPG", frontIntakeMotor),
                        new SleepAction(0.3),
                        new kickerShoot(),
                        new SleepAction(shootingDelay),

                        // Spike 1
                        new ParallelAction(
                                // Intake Spike 1 Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(-11, -60), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelIntaking),
                                                        new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false)
                                // new setIntake(frontIntakeMotor, backIntakeMotor, 0.75)
                        ),

                        new ParallelAction(
                                // Score Spike 1 Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(-12, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(intakeStopDelay),
                                        new recycle("PPG", frontIntakeMotor)
                                )
                        ),
                        new SleepAction(shooterStartDelay),
                        new kickerShoot(),
                        new SleepAction(shootingDelay),

                        // Spike 2
                        new ParallelAction(
                                // Go to intake Spike 2 Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(12.5, -25), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new kickerIdle(false)
                                // new setIntake(frontIntakeMotor, backIntakeMotor, 0.75)
                        ),

                        // Intake Spike 2 Path (from *current* pose)
                        new PathFromCurrentPose(drive, pose ->
                                drive.actionBuilder(pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(12.5, -66), Math.toRadians(270),
                                                new TranslationalVelConstraint(minVelIntaking),
                                                new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking)
                                        )
                                        .build()
                        ),

                        new ParallelAction(
                                // Score Spike 2 Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(11.5, -52), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .strafeToLinearHeading(
                                                        new Vector2d(-12, -15), Math.toRadians(270),
                                                        new TranslationalVelConstraint(minVelDrive),
                                                        new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                )
                                                .build()
                                ),
                                new SequentialAction(
                                        new SleepAction(intakeStopDelay),
                                        new recycle("PGP", frontIntakeMotor)
                                )
                        ),
                        new SleepAction(shooterStartDelay = 0.2),
                        new kickerShoot(),
                        new SleepAction(shootingDelay),

                        // (You can add Spike 3 here if needed)

                        // Park
                        new ParallelAction(
                                // Park Path (from *current* pose)
                                new PathFromCurrentPose(drive, pose ->
                                        drive.actionBuilder(pose)
                                                .strafeToLinearHeading(
                                                        new Vector2d(-22, -58), Math.toRadians(270),
                                                        new TranslationalVelConstraint(60),
                                                        new ProfileAccelConstraint(-60, 60)
                                                )
                                                .build()
                                ),
                                new setShooter(leftShooterMotor, rightShooterMotor, 0.0),
                                new kickerIdle(true)
                        )

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

        // Phase durations (ms)
        private final double tongueDownMs;
        private final double intakeRunMs;

        private final ElapsedTime recyclerTimer = new ElapsedTime();
        private boolean started = false;

        // Default timings constructor: 400 ms tongue down, 600 ms intake
        public recycleArtifact(DcMotorEx frontIntakeMotor) {
            this(frontIntakeMotor, 400, 1000);
        }

        // Optional: custom timings constructor
        public recycleArtifact(DcMotorEx frontIntakeMotor,
                               double tongueDownMs,
                               double intakeRunMs) {
            this.frontIntakeMotor = frontIntakeMotor;
            this.tongueDownMs = tongueDownMs;
            this.intakeRunMs = intakeRunMs;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (!started) {
                started = true;
                recyclerTimer.reset();
            }

            double t = recyclerTimer.milliseconds();

            // -------- PHASE 1: tongue DOWN + rollers ON, intake OFF --------
            if (t < tongueDownMs) {
                leftTongueServo.setPosition(Globals.tongueRecycle);
                rightTongueServo.setPosition(Globals.tongueRecycle);

                leftKickerServo.setPower(Globals.rollerKickerRecycle);
                rightKickerServo.setPower(Globals.rollerKickerRecycle);

                frontIntakeMotor.setPower(0.0);

                return true;
            }

            // -------- PHASE 2: tongue UP + rollers OFF, intake ON --------
            if (t < tongueDownMs + intakeRunMs) {
                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);

                leftKickerServo.setPower(0.0);
                rightKickerServo.setPower(0.0);

                frontIntakeMotor.setPower(-1.0);

                return true;
            }

            // -------- PHASE 3: stop intake, finish --------
            leftTongueServo.setPosition(Globals.tongueIntake);
            rightTongueServo.setPosition(Globals.tongueIntake);

            leftKickerServo.setPower(0.0);
            rightKickerServo.setPower(0.0);

            frontIntakeMotor.setPower(0.0);

            return false;
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

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // one-time init
            if (!initialized) {
                kickersStarted = false;

                leftTongueServo.setPosition(Globals.tongueShoot);
                rightTongueServo.setPosition(Globals.tongueShoot);

                shooterTimer.reset();   // start the 200ms delay timer
                initialized = true;
            }

            // after 200ms, run the kickers
            if (!kickersStarted && shooterTimer.milliseconds() >= 200) {
                leftKickerServo.setPower(Globals.rollerKickerShoot);
                rightKickerServo.setPower(Globals.rollerKickerShoot);
                frontIntakeMotor.setPower(-1.0);
                backIntakeMotor.setPower(-1.0);

                kickersStarted = true;
            }

            // keep running until kickersStarted becomes true; then action completes
            return !kickersStarted;
        }
    }


    public class kickerIdle implements Action {

        private final boolean stopIntake;

        public kickerIdle(boolean stopIntake){
            this.stopIntake = stopIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftTongueServo.setPosition(Globals.tongueIntake);
            rightTongueServo.setPosition(Globals.tongueIntake);
            leftKickerServo.setPower(0.0);
            rightKickerServo.setPower(0.0);

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
