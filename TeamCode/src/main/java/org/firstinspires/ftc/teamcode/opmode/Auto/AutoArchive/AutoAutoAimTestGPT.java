package org.firstinspires.ftc.teamcode.opmode.Auto.AutoArchive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController2;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoAutoAimTestGPT extends LinearOpMode {

    // Initialize all hardware
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

    Limelight3A limelight;

    ServoImplEx leftHood;
    ServoImplEx rightHood;

    // SINGLE Pinpoint instance (only driver)
    private GoBildaPinpointDriver pinpoint;

    // ---- Pinpoint start pose (pick ONE frame/units and stick to it) ----
    // I’m keeping your intent: 54.5in, 45in, 45deg, but in INCH so you don’t mix meters/inches.
    private static final Pose2D START_PINPOINT_POSE =
            new Pose2D(DistanceUnit.INCH, 54.5, 45.0, AngleUnit.DEGREES, 45.0);

    // Initialize all parameters
    public static double minVelIntaking = 40;
    public static double minAccelIntaking = -40;
    public static double maxAccelIntaking = 40;

    public static double minVelDrive = 80;
    public static double minAccelDrive = -70;
    public static double maxAccelDrive = 70;

    public static double shooterStartDelay = 0.3;
    public static double shootingDelay = 3;

    public static double intakeStopDelay = 0.4;

    public static double turretStartPos = 0.422;
    public static double turretShootPos = 0.422;

    public static double plainKickerPower = 0.0;

    public static double recyclingDelay = 2000;
    public static double recyclingIntakeDelay = 500;
    public static double recyclingKickerUpDelay = 500;

    public static boolean kickerPIDEnabled = true;

    public static double kickerTarget = Globals.KICKER_IDLE;

    public static String allianceColor = "Blue";

    double flywheelCurrentVelocity;
    double currentSpeed;
    double variableFlywheelSpeed;
    double targetVelocity;
    double power;

    // vPID
    private PIDVelocityController2 velocityPID;
    public static double currentVelocity;
    public static double TargetVelocity = 900;
    public static double VKp = 0.002;
    public static double VKi = 0.003;
    public static double VKd = 0;
    public static double VkS = 0;
    public static double VkV = 0.00042;
    public static double sec = 0.2;

    // Stall detection
    public static double stallWindowMs   = 200;
    public static double stallMinDelta   = 0.1;
    public static double stallRecoveryMs = 1000;
    public static double stallGraceMs    = 400;

    int obeliskID = -1;

    // Initialize any instances of classes
    ElapsedTime recyclerTimer = new ElapsedTime();
    visionTools vision = new visionTools();
    CRAxonPDController kickerPID = new CRAxonPDController();
    DroidForceMethods DFM = new DroidForceMethods();

    @Override
    public void runOpMode() {

        // NOTE: Do NOT instantiate MecanumDrive here if it internally creates/uses Pinpoint,
        // otherwise you get "multiple drivers" again.

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

        // ---- Single Pinpoint driver instance ----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        leftTurretServo.setPosition(0.5);
        rightTurretServo.setPosition(0.5);

        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(19.970472542, DistanceUnit.MM);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Reset, then WAIT for READY, then set pose ONCE (prevents “half-initialized” weirdness)
        pinpoint.resetPosAndIMU();

        while (!isStarted() && !isStopRequested()
                && pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpoint.update();
            telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
            telemetry.addData("Pinpoint pose", pinpoint.getPosition());
            telemetry.update();
        }

        // Apply start pose once after READY
        pinpoint.setPosition(START_PINPOINT_POSE);
        pinpoint.update();

        waitForStart();
        if (isStopRequested()) return;

        // Run ONE updater action + your tracking actions in parallel
        Actions.runBlocking(new ParallelAction(
                // Thread 1: Pinpoint update loop (ONLY place update() is called)
                new SequentialAction(new PinpointUpdater()),

                // Thread 2: Turret Tracking (no pinpoint.update() inside)
                new SequentialAction(new startTurretTracking())

                // Thread 3 / 4 left commented like you had:
                // , new SequentialAction(new startVelPID())
                // , new SequentialAction(new startHoodTracking())
        ));
    }

    /**
     * Calls pinpoint.update() exactly once per scheduler tick.
     * Everyone else only reads pinpoint.getPosition().
     */
    public class PinpointUpdater implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pinpoint.update();
            return true;
        }
    }

    public class startTurretTracking implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // NOTE: NO pinpoint.update() here anymore

            telemetry.addData("pos", pinpoint.getPosition().toString());
            telemetry.update();

            double leftCmd  = vision.pinpointTurretMoving(
                    11.9, sec, pinpoint, leftTurretServo.getPosition(), allianceColor
            );
            double rightCmd = vision.pinpointTurretMoving(
                    11.9, sec, pinpoint, rightTurretServo.getPosition(), allianceColor
            );

            leftTurretServo.setPosition(leftCmd);
            rightTurretServo.setPosition(rightCmd);

            return true;
        }
    }

    public class startHoodTracking implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // NOTE: NO pinpoint.update() here anymore
            double leftHoodCurr = leftHood.getPosition();
            leftHood.setPosition(vision.hoodHeightRegressor(limelight, leftHoodCurr, pinpoint, allianceColor));
            rightHood.setPosition(vision.hoodHeightRegressor(limelight, leftHoodCurr, pinpoint, allianceColor));

            return true;
        }
    }

    public class startVelPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // NOTE: NO pinpoint.update() here anymore
            flywheelCurrentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2;

            targetVelocity = vision.FlywheelSpeedRegressor(sec, flywheelCurrentVelocity, pinpoint, allianceColor);

            velocityPID.setTargetVelocity(targetVelocity);
            velocityPID.setPID(VKp, VKi, VKd);
            velocityPID.setFeedforward(VkS, VkV);

            power = velocityPID.update(flywheelCurrentVelocity);

            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);

            currentSpeed = targetVelocity;

            return true;
        }
    }

    // ---- Everything below is unchanged from your file (kicker PID / recycle / etc.) ----
    // I’m leaving it as-is; none of it affects the “multiple Pinpoint driver” issue.

    public class startKickerPID implements Action {

        private final CRServoImplEx leftKickerServo;
        private final CRServoImplEx rightKickerServo;

        // Stall detection state
        private final ElapsedTime stallTimer = new ElapsedTime();
        private boolean stallSampleValid = false;
        private double stallSampleTimeMs = 0.0;
        private double stallSamplePos = 0.0;

        private boolean recoveringFromStall = false;
        private double lastShootPower = 0.0;
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

            boolean shootingOpenLoop = !kickerPIDEnabled && Math.abs(plainKickerPower) > 0.01;

            if (shootingOpenLoop && !wasShootingOpenLoop) {
                openLoopStartTimeMs = nowMs;
                stallSampleValid = false;
            }

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

            } else {
                if (shootingOpenLoop && (nowMs - openLoopStartTimeMs) >= stallGraceMs) {

                    if (!stallSampleValid) {
                        stallSampleValid = true;
                        stallSampleTimeMs = nowMs;
                        stallSamplePos = processedEncoderValue;
                    } else {
                        double dt = nowMs - stallSampleTimeMs;
                        if (dt >= stallWindowMs) {
                            double dPos = Math.abs(processedEncoderValue - stallSamplePos);
                            if (dPos < stallMinDelta) {
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
                } else if (!shootingOpenLoop) {
                    stallSampleValid = false;
                }
            }

            wasShootingOpenLoop = shootingOpenLoop;

            if (kickerPIDEnabled) {
                double pwr = kickerPID.Output(
                        Globals.KICKER_kP,
                        Globals.KICKER_kD,
                        kickerTarget,
                        processedEncoderValue
                );

                leftKickerServo.setPower(pwr);
                rightKickerServo.setPower(pwr);
            } else {
                leftKickerServo.setPower(plainKickerPower);
                rightKickerServo.setPower(plainKickerPower);
            }

            return true;
        }
    }

    // ... (rest of your unchanged classes: debugTelemetry, recycle, recycleArtifact, setTurret,
    // setShooter, kickerShoot, kickerIdle, setIntake, etc.)
}
