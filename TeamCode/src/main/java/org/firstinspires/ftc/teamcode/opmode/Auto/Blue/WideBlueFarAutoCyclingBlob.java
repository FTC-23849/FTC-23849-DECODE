package org.firstinspires.ftc.teamcode.opmode.Auto.Blue;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.opmode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.opmode.misc.BlobDetector;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController3;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.function.Function;

//@Disabled
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class WideBlueFarAutoCyclingBlob extends LinearOpMode {

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

    GoBildaPinpointDriver pinpoint;

    // Initialize all parameters
//    public static double minVelIntaking = 40;
//    public static double minAccelIntaking = -40;
//    public static double maxAccelIntaking = 40;

    public static double minVelDrive = 90;
    public static double minAccelDrive = -90;
    public static double maxAccelDrive = 90;

    public static double shooterStartDelay = 0.1;
    public static double shootingDelay = 1;

    public static double intakeStopDelay = 0.4;

    public static double turretStartPos = 0.5; /*0.39*/
    //public static double turretShootPos = 0.422;

    public static double plainKickerPower = 0.0;

    public static boolean allowExternalUpdating = true;

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

    public static double intakeShootingSpeed = -0.9;

    public static double kickerSpeed = 1.0;

    public static double shootingSpeed = -1.0;

    public static boolean kickersStarted;


    public static boolean intakeLastSpike = true;
    public static boolean intakeSecondSpike = false;

    public static double preloadShootingSpeed = -1250;
    public static double cyclingShootingSpeed = -1200;
    public static double shootingSpeedPID = preloadShootingSpeed;
    public static double turretOffset = -0.0065;

    public static String allianceColor = "Blue";

    public static boolean enableTurretTracking = true;
    public static boolean enableVelPID = true;

    public static double secondCycleOffset = 0.0;
    public static double thirdCycleOffset = 0.0;
    public static double fourthCycleOffset = 0.0;

    public static double widenCycleOffset = 0.0;

    public static boolean secondCycleOffsetEnabled = false;
    public static boolean thirdCycleOffsetEnabled = false;
    public static boolean fourthCycleOffsetEnabled = false;
    public static boolean widenCycleOffsetEnabled = false;

    public static double cyclingShootingY = -25;

    // vPID
    double flywheelCurrentVelocity;
    double currentSpeed;
    double variableFlywheelSpeed;
    double targetVelocity;
    double power;

    public PIDVelocityController3 velocityPID;
    public static double currentVelocity;
    public static double TargetVelocity;
    public static double VKp = Globals.VKp;
    public static double VKi = Globals.VKi;
    public static double VKd = Globals.VKd;
    public static double VkS = Globals.VkS;
    public static double VkV = Globals.VkV;
    public static double sec = Globals.sec;

    public static double KpMaintain = Globals.KpMaintain;
    public static double KiMaintain = Globals.KiMaintain;

    public static double KpDriveRecovery = Globals.KpDriveRecovery;

    public static double KpRecovery = Globals.KpRecovery;
    public static double KiRecovery = Globals.KiRecovery;
    public static double KsRecovery = Globals.KsRecovery;
    public static double KvFF = Globals.KvFF;
    public static double KsFF = Globals.KsFF;


    public static double recoveryThreshold = Globals.recoveryThreshold;
    public static double maintainThreshold = Globals.maintainThreshold;
    public static double defaultVoltage = Globals.defaultVoltage;

    int obeliskID = -1;

    double blobOffset = 0;

    // Initialize any instances of classes
    ElapsedTime recyclerTimer = new ElapsedTime();

    ElapsedTime shooterTimer = new ElapsedTime();

    private BlobDetector detector;

    visionTools vision = new visionTools();

    CRAxonPDController kickerPID = new CRAxonPDController();
    DroidForceMethods DFM = new DroidForceMethods();


    @Override
    public void runOpMode() {

        // Instantiate MecanumDrive
        Pose2d startPose = new Pose2d(61.5, -14.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Map motors and servos
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
        leftKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

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

        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood.setDirection(ServoImplEx.Direction.REVERSE);

        //Limelight
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);

        limelight.setPollRateHz(100);
        limelight.start();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Pre-Auto robot initlization. MUST BE LAST
        leftHood.setPosition(0.28);
        rightHood.setPosition(0.28);

        leftTurretServo.setPosition(turretStartPos);
        rightTurretServo.setPosition(turretStartPos);

        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);

        plainKickerPower = 0.0;


        velocityPID = new PIDVelocityController3(
                targetVelocity,
                KpMaintain,KiMaintain,
                KpRecovery,KiRecovery,KsRecovery
                , KsFF, KvFF
        );
        sleep(1000);

        pinpoint.recalibrateIMU();

        sleep(500);
        telemetry.addData("FINISHED",true);
        telemetry.update();

        detector = new BlobDetector(
                hardwareMap,
                "Webcam 1",
                640,
                480,
                135.0 / 346.0,
                26,
                "Blue"
        );

        detector.start();

        waitForStart();

        if (isStopRequested()) return;

        try {

            sleep(4);

            Actions.runBlocking(new ParallelAction(

                    // Thread 1: Pathing + General Robot
                    new SequentialAction(

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
                                    new setIntake(frontIntakeMotor, backIntakeMotor, -1.0),
                                    new setPIDToCycling()
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
                                                            new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
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

                            // repeat for correction
                            new PathFromCurrentPose(drive, pose ->
                                    drive.actionBuilder(pose)
                                            .strafeToLinearHeading(
                                                    new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
                                                    new TranslationalVelConstraint(minVelDrive),
                                                    new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                            )
                                            .build()
                            ),

                            new SleepAction(shooterStartDelay),

                            new kickerShoot(),

                            new SleepAction(shootingDelay),


                            // Intake Cycle 1
                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, -50), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .build()
                                    ),
                                    new kickerIdle(false),
                                    new setIntake(frontIntakeMotor, backIntakeMotor, -1.0)
                            ),

                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
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

                            // repeat for correction
                            new PathFromCurrentPose(drive, pose ->
                                    drive.actionBuilder(pose)
                                            .strafeToLinearHeading(
                                                    new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
                                                    new TranslationalVelConstraint(minVelDrive),
                                                    new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                            )
                                            .build()
                            ),

                            new SleepAction(shooterStartDelay),

                            new kickerShoot(),

                            new SleepAction(shootingDelay),

                            new getBlobOffset(),

                            new SleepAction(0.1),


                            //Intake Cycle 2

                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -50), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .build()
                                    ),
                                    new kickerIdle(false),
                                    new setIntake(frontIntakeMotor, backIntakeMotor, -1.0)
                            ),

                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
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

                            // repeat for correction
                            new PathFromCurrentPose(drive, pose ->
                                    drive.actionBuilder(pose)
                                            .strafeToLinearHeading(
                                                    new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
                                                    new TranslationalVelConstraint(minVelDrive),
                                                    new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                            )
                                            .build()
                            ),

                            new SleepAction(shooterStartDelay),

                            new kickerShoot(),

                            new SleepAction(shootingDelay),

                            new getBlobOffset(),

                            new SleepAction(0.1),

                            //Intake Cycle 3

                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -50), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5 - blobOffset, -66), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .build()
                                    ),
                                    new kickerIdle(false),
                                    new setIntake(frontIntakeMotor, backIntakeMotor, -1.0)
                            ),

                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
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

                            // repeat for correction
                            new PathFromCurrentPose(drive, pose ->
                                    drive.actionBuilder(pose)
                                            .strafeToLinearHeading(
                                                    new Vector2d(60.5, cyclingShootingY), Math.toRadians(270),
                                                    new TranslationalVelConstraint(minVelDrive),
                                                    new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                            )
                                            .build()
                            ),

                            new SleepAction(shooterStartDelay),

                            new kickerShoot(),

                            new SleepAction(shootingDelay),

                            //Park
                            new ParallelAction(
                                    new PathFromCurrentPose(drive, pose ->
                                            drive.actionBuilder(pose)
                                                    .strafeToLinearHeading(
                                                            new Vector2d(60.5, -40), Math.toRadians(270),
                                                            new TranslationalVelConstraint(minVelDrive),
                                                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive)
                                                    )
                                                    .build()
                                    ),
                                    new kickerIdle(true),
                                    new setIntake(frontIntakeMotor, backIntakeMotor, 0.0),
                                    new setShooter(leftShooterMotor, rightShooterMotor, 0.0)
                            )

                    ),

                    new SequentialAction(
                            new startVelPIDPlain(shootingSpeedPID)
                    ),

                    new SequentialAction(
                            new startTurretTracking(drive)
                    )

            ));

        } finally {

            sleep(200);

            drive.updatePoseEstimate();
            drive.localizer.update();

            PoseStorage.currentPose = vision.RRtoPinpoint(drive);

            detector.stop();

            sleep(1000);

        }

        //sleep(1000);

    }

    public class startTurretTracking implements Action {

        private final MecanumDrive drive;

        public startTurretTracking(MecanumDrive drive){
            this.drive = drive;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (enableTurretTracking) {
                // Always update first
                if (allowExternalUpdating) {
                    drive.updatePoseEstimate();
                    drive.localizer.update();
                }

                //pinpoint.update();

                Pose2D pose = vision.RRtoPinpoint(drive);

                telemetry.addData("pos",pose.toString());
                telemetry.addData("heading",pose.getHeading(AngleUnit.DEGREES));
                telemetry.update();

                // Normal tracking
                double leftCmd  = vision.pinpointTurretMovingAUTO(pose,11.9, sec, leftTurretServo.getPosition(), allianceColor);
                double rightCmd = vision.pinpointTurretMovingAUTO(pose,11.9, sec, rightTurretServo.getPosition(), allianceColor);

                leftTurretServo.setPosition(leftCmd - turretOffset);
                rightTurretServo.setPosition(rightCmd - turretOffset);
            }

            return true; // keep running for whole auto
        }
    }

    public class setPIDToCycling implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            shootingSpeedPID = cyclingShootingSpeed;

            return false;

        }
    }


    public class startVelPIDPlain implements Action {

        private final double speed;

        public startVelPIDPlain(double speed){
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (enableVelPID) {
                flywheelCurrentVelocity = /*(*/leftShooterMotor.getVelocity() /*+ rightShooterMotor.getVelocity()) / 2*/;

                targetVelocity = speed;

                velocityPID.setTargetVelocity(targetVelocity);
                velocityPID.setMaintainGains(KpMaintain,KiMaintain,KpDriveRecovery);
                velocityPID.setRecoveryGains(KpRecovery,KiRecovery,KsRecovery);
                velocityPID.setFeedforward(KsFF, KvFF);
                velocityPID.setRecoveryThreshold(recoveryThreshold);
                velocityPID.setMaintainThreshold(maintainThreshold);
                power = velocityPID.update(flywheelCurrentVelocity,13.5, defaultVoltage);

                leftShooterMotor.setPower(power);
                rightShooterMotor.setPower(power);

                currentSpeed = targetVelocity;
            }

            return true;
        }
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
        private boolean started = false;

        public PathFromCurrentPose(MecanumDrive drive, Function<Pose2d, Action> actionFactory) {
            this.drive = drive;
            this.actionFactory = actionFactory;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (inner == null) {
                // Before the path starts
                allowExternalUpdating = false;
                started = true;

                Pose2d now = drive.localizer.getPose();
                inner = actionFactory.apply(now);

                // Fail-safe: if factory returns null, don't leave updates disabled
                if (inner == null) {
                    allowExternalUpdating = true;
                    started = false;
                    return false;
                }
            }

            boolean keepRunning = inner.run(telemetryPacket);

            if (!keepRunning && started) {
                // Once the path completes
                allowExternalUpdating = true;
                started = false;
            }

            return keepRunning;
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

    public class getBlobOffset implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            blobOffset = detector.getOffsetCm() / 2.54;

            if (blobOffset < 5) {
                blobOffset = 0;
            } else {
                blobOffset -= 5;
            }

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

        // Phase durations (ms)
        private final double tongueDownMs;
        private final double intakeRunMs;

        private final ElapsedTime recyclerTimer = new ElapsedTime();
        private boolean started = false;

        // Default timings constructor: 400 ms tongue down, 600 ms intake
        public recycleArtifact(DcMotorEx frontIntakeMotor) {
            this(frontIntakeMotor, 400, 600);
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
                leftKickerServo.setPower(kickerSpeed);
                rightKickerServo.setPower(kickerSpeed);
                frontIntakeMotor.setPower(intakeShootingSpeed);
                backIntakeMotor.setPower(intakeShootingSpeed);

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
                backIntakeMotor.setPower(power);
            } else {
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }
            return false;
        }
    }

}