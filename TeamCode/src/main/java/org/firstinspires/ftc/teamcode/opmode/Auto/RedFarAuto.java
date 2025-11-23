package org.firstinspires.ftc.teamcode.opmode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@Autonomous
public class RedFarAuto extends LinearOpMode {

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

    ElapsedTime timer = new ElapsedTime();

    double minVelIntaking = 30;
    double minAccelIntaking = -30;
    double maxAccelIntaking = 30;

    double minVelDrive = 60;
    double minAccelDrive = -60;
    double maxAccelDrive = 60;

    double shooterStartDelay = 1.5;
    double shootingDelay = 3;

    double intakeStopDelay = 0.1;

    double shootingSpeed = -0.98;

    double initialDelay = 0.0;
    boolean pickupHP = false;
    boolean pickupLastSpike = false;

    @Override
    public void runOpMode() {

        // Create Roadrunner Trajectories

        Pose2d startPose = new Pose2d(63, 14.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //map motors and servos
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(8);
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
        leftTipper = hardwareMap.get(ServoImplEx.class, "leftTipper");
        rightTipper = hardwareMap.get(ServoImplEx.class, "rightTipper");
        leftBackRoller = hardwareMap.get(CRServoImplEx.class, "leftBackRoller");
        rightBackRoller = hardwareMap.get(CRServoImplEx.class, "rightBackRoller");
        rightBackRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.setPollRateHz(100);
        limelight.start();
        kickerEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);

        leftHood.setPosition(0.4);
        rightHood.setPosition(0.4);

        leftTurretServo.setPosition(0.61);
        rightTurretServo.setPosition(0.61);

        while(!opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) {
                initialDelay = 20;
            } else if (gamepad1.dpad_down) {
                initialDelay = 5;
            } else if (gamepad1.dpad_left) {
                initialDelay = 10;
            } else if (gamepad1.dpad_right) {
                initialDelay = 15;
            }

//            if (gamepad1.a) {
//                pickupLastSpike = true;
//            } else if (gamepad1.b) {
//                pickupLastSpike = false;
//            }

            if (gamepad1.x) {
                pickupHP = true;
            } else if (gamepad1.y) {
                pickupHP = false;
            }

            telemetry.addData("Initial Delay (Seconds): ", initialDelay);
            //telemetry.addData("Picking up last spike mark? (a/b): ", pickupLastSpike);
            telemetry.addData("Picking up HP? (x/y): ", pickupHP);

            telemetry.update();

        }

        waitForStart();

        if (isStopRequested()) return;

        sleep(4);

        Actions.runBlocking(new SequentialAction(
                new setShooter(leftShooterMotor, rightShooterMotor, -0.90),
                new SleepAction(shooterStartDelay),
                new SleepAction(initialDelay),
                new startKicker(leftKickerServo, rightKickerServo, frontIntakeMotor),
                new SleepAction(shootingDelay)
        ));

        if (pickupLastSpike) {

        } else if (pickupHP) {

            drive.updatePoseEstimate();
            drive.localizer.update();

            TrajectoryActionBuilder goToIntakeHP = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(44.5, 64), Math.toRadians(0),
                            new TranslationalVelConstraint(minVelIntaking),
                            new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking));

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            goToIntakeHP.build(),
                            new stopKickerPlain(leftKickerServo, rightKickerServo, frontIntakeMotor, false)
                    )
            ));

            drive.updatePoseEstimate();
            drive.localizer.update();

            TrajectoryActionBuilder intakeHP = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(62, 64), Math.toRadians(0),
                            new TranslationalVelConstraint(minVelIntaking),
                            new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking));

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            intakeHP.build(),
                            new setIntake(frontIntakeMotor, backIntakeMotor, 0.7)
                    )
            ));

            drive.updatePoseEstimate();
            drive.localizer.update();

            TrajectoryActionBuilder scoreHP = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(52, 15), Math.toRadians(90),
                            new TranslationalVelConstraint(minVelDrive),
                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            scoreHP.build()
                    ),
                    new SleepAction(shooterStartDelay),
                    new startKicker(leftKickerServo, rightKickerServo, frontIntakeMotor),
                    new SleepAction(shootingDelay)
            ));

            drive.updatePoseEstimate();
            drive.localizer.update();

            TrajectoryActionBuilder plainPark = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(63, 35), Math.toRadians(90),
                            new TranslationalVelConstraint(minVelDrive),
                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            plainPark.build(),
                            new stopKickerPlain(leftKickerServo, rightKickerServo, frontIntakeMotor, true)
                    )
            ));

        } else {

            drive.updatePoseEstimate();
            drive.localizer.update();

            TrajectoryActionBuilder plainPark = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(63, 35), Math.toRadians(90),
                            new TranslationalVelConstraint(minVelDrive),
                            new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));

            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            plainPark.build(),
                            new stopKickerPlain(leftKickerServo, rightKickerServo, frontIntakeMotor, true)
                    )
            ));

        }

        sleep(1000);

    }

    public class setShooter implements Action {

        DcMotorEx topShooterMotor;
        DcMotorEx bottomShooterMotor;

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

    public class startKicker implements Action {

        CRServo leftKickerServo;
        CRServo rightKickerServo;
        DcMotorEx frontIntakeMotor;

        public startKicker(CRServo leftKickerServo, CRServo rightKickerServo, DcMotorEx frontIntakeMotor){
            this.leftKickerServo = leftKickerServo;
            this.rightKickerServo = rightKickerServo;
            this.frontIntakeMotor = frontIntakeMotor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftKickerServo.setPower(-0.3);
            rightKickerServo.setPower(-0.3);
            frontIntakeMotor.setPower(1);

            return false;
        }
    }

    public class stopKickerPlain implements Action {

        CRServo leftKickerServo;
        CRServo rightKickerServo;
        DcMotorEx frontIntakeMotor;
        boolean stopIntake;

        public stopKickerPlain(CRServo leftKickerServo, CRServo rightKickerServo, DcMotorEx frontIntakeMotor, boolean stopIntake){
            this.leftKickerServo = leftKickerServo;
            this.rightKickerServo = rightKickerServo;
            this.frontIntakeMotor = frontIntakeMotor;
            this.stopIntake = stopIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftKickerServo.setPower(0.0);
            rightKickerServo.setPower(0.0);

            if (stopIntake) {
                frontIntakeMotor.setPower(0.0);
            }

            return false;
        }
    }

    public class stopKicker implements Action {

        CRServo leftKickerServo;
        CRServo rightKickerServo;
        AnalogInput kickerEncoder;
        DcMotorEx frontIntakeMotor;
        boolean stopIntake;

        public stopKicker(CRServo leftKickerServo, CRServo rightKickerServo, AnalogInput kickerEncoder, DcMotorEx frontIntakeMotor, boolean stopIntake) {
            this.leftKickerServo = leftKickerServo;
            this.rightKickerServo = rightKickerServo;
            this.kickerEncoder = kickerEncoder;
            this.frontIntakeMotor = frontIntakeMotor;
            this.stopIntake = stopIntake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double kickerLocation;
            double kickerRotationsLeft = 0;
            ElapsedTime recycleIntakeTimer = new ElapsedTime(0);
            boolean recycleIntakeTimerStarted = false;
            boolean kickerInDefaultPosition = false;
            int loops = 0;
            loops ++;
            telemetry.addData("loops", loops );
            while(kickerInDefaultPosition == false){
                if (stopIntake) {
                frontIntakeMotor.setPower(0.0);
                }

                if (kickerEncoder.getVoltage() > 1.65) {
                    kickerLocation = kickerEncoder.getVoltage() - 1.65;
                } else {
                    kickerLocation = kickerEncoder.getVoltage();
                }
                if (kickerLocation > 0.6) {
                    kickerLocation = kickerLocation - 0.6;
                } else {
                    kickerLocation = 1.65 - (kickerLocation - 0.6);
                }

                if (kickerRotationsLeft == 0) {
                    if (recycleIntakeTimerStarted == false) {
                        recycleIntakeTimer.reset();
                        recycleIntakeTimerStarted = true;
                    }
                    if (recycleIntakeTimerStarted == true && recycleIntakeTimer.milliseconds() > 1000) {
//                        frontIntakeMotor.setPower(0);
//                        backIntakeMotor.setPower(0);
                    }
                    telemetry.addData("kicker locaiton", kickerLocation);
                    if (kickerLocation < Globals.defaultKickerLocationAuto - 0.01) {
                        leftKickerServo.setPower(0.075 /* (kickerLocation - Globals.defaultKickerLocation)/ / (Globals.defaultKickerLocation - kickerEncoder.getVoltage())*/);
                        rightKickerServo.setPower(0.075);
                        telemetry.addLine("e");
                    } else if (kickerLocation > Globals.defaultKickerLocationAuto + 0.01) {
                        leftKickerServo.setPower(-0.075);
                        rightKickerServo.setPower(-0.075);
                        telemetry.addLine("ae");
                    } else {
//                        rightKickerServo.setPower(0);
//                        leftKickerServo.setPower(0);
                        kickerInDefaultPosition = true;
                        telemetry.addLine("6767676767667 VICTORY IT WORKS");
                    }
    //                if (kickerLocation > Globals.defaultKickerLocationAuto - 0.1 && kickerLocation < Globals.defaultKickerLocationAuto + 0.1 && kickerInDefaultPosition == false) {
    //                    kickerRotationsLeft = kickerRotationsLeft - 1;
    //                    kickerInDefaultPosition = true;
    //                }
    //                if (kickerLocation < Globals.defaultKickerLocationAuto - 0.1 || kickerLocation > Globals.defaultKickerLocationAuto + 0.1) {
    //                    kickerInDefaultPosition = false;
    //                }
                    telemetry.addData("kickerindefault positon", kickerInDefaultPosition);
                    telemetry.update();
                }
                rightKickerServo.setPower(0);
                leftKickerServo.setPower(0);
                frontIntakeMotor.setPower(0);
                backIntakeMotor.setPower(0);

            }

            return !kickerInDefaultPosition;

        }

    }

    public class setIntake implements Action {

        DcMotorEx frontIntakeMotor;
        DcMotorEx backIntakeMotor;

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
                backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
            } else if (power == 0.0) {
                frontIntakeMotor.setPower(0.0);
                backIntakeMotor.setPower(0.0);
            }
            return false;
        }
    }

}