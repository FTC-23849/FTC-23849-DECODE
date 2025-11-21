//package org.firstinspires.ftc.teamcode.opmode.Auto;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ProfileAccelConstraint;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
//import org.firstinspires.ftc.teamcode.hardware.Globals;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous
//public class BlueCloseAuto extends LinearOpMode {
//
//    DcMotorEx leftFrontMotor;
//    DcMotorEx rightFrontMotor;
//    DcMotorEx leftBackMotor;
//    DcMotorEx rightBackMotor;
//    DcMotorEx intakeMotor;
//    DcMotorEx transferMotor;
//    DcMotorEx topShooterMotor;
//    DcMotorEx bottomShooterMotor;
//    CRServo transferServoLeft;
//    CRServo transferServoRight;
//    //GoBildaPinpointDriver pinpoint;
//    Servo light;
//    Servo gate;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    double minVelIntaking = 60;
//    double minAccelIntaking = -60;
//    double maxAccelIntaking = 60;
//
//    double minVelDrive = 70;
//    double minAccelDrive = -70;
//    double maxAccelDrive = 70;
//
//    double shooterDelay = 0.3;
//
//    @Override
//    public void runOpMode() {
//
//        // Create Roadrunner Trajectories
//
//        Pose2d startPose = new Pose2d(-53.5, -45.5, Math.toRadians(45));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
//
//        //map motors and servos
//        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
//        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
//        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
//        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
//        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
//
//        topShooterMotor = hardwareMap.get(DcMotorEx.class,"topShooterMotor");
//        bottomShooterMotor = hardwareMap.get(DcMotorEx.class,"bottomShooterMotor");
//        topShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
//
//        topShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
//
//        transferServoLeft = hardwareMap.get(CRServo.class, "transferServoLeft");
//        transferServoRight = hardwareMap.get(CRServo.class, "transferServoRight");
//
//        light = hardwareMap.get(Servo.class,"light");
//        gate = hardwareMap.get(Servo.class,"gate");
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        sleep(4);
//
//        //score preload
//        TrajectoryActionBuilder scorePreloads = drive.actionBuilder(startPose)
//                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(50), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        scorePreloads.build(),
//                        new setShooter(topShooterMotor, bottomShooterMotor, Globals.defaultCloseZonePower)
//                ),
//                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
//                new SleepAction(1.8)
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//        //Collect 1st spike mark
//        TrajectoryActionBuilder goToIntakeSpike1 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(-11, -25), Math.toRadians(270), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new ParallelAction(
//                goToIntakeSpike1.build(),
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true),
//                new setIntake(intakeMotor, transferMotor, 1.0),
//                new setShooter(topShooterMotor, bottomShooterMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        TrajectoryActionBuilder intakeSpike1 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(-11, -60), Math.toRadians(270), new TranslationalVelConstraint(minVelIntaking), new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking));
//
//        Actions.runBlocking(new SequentialAction(
//                intakeSpike1.build()
//                //new setIntake(intakeMotor, transferMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        //shoot 1st spike mark
//        TrajectoryActionBuilder scoreSpike1 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(50), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        scoreSpike1.build(),
//                        new setShooter(topShooterMotor, bottomShooterMotor, Globals.defaultCloseZonePower),
//                        new SequentialAction(
//                                new SleepAction(0.8),
//                                new ParallelAction(
//                                        new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//                                )
//                        )
//                ),
//                new SleepAction(shooterDelay),
//                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
//                new SleepAction(1.5)
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        //Collect 2nd spike mark
//        TrajectoryActionBuilder goToIntakeSpike2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(12.5, -25), Math.toRadians(270), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new ParallelAction(
//                goToIntakeSpike2.build(),
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true),
//                new setIntake(intakeMotor, transferMotor, 1.0),
//                new setShooter(topShooterMotor, bottomShooterMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        TrajectoryActionBuilder intakeSpike2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(12.5, -66), Math.toRadians(270), new TranslationalVelConstraint(minVelIntaking), new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking));
//
//        Actions.runBlocking(new SequentialAction(
//                intakeSpike2.build()
//                //new setIntake(intakeMotor, transferMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        //shoot 2nd spike mark
//        TrajectoryActionBuilder scoreSpike2 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(11.5, -52), Math.toRadians(270), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive))
//                .strafeToLinearHeading(new Vector2d(-23, -15), Math.toRadians(50), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        scoreSpike2.build(),
//                        new setShooter(topShooterMotor, bottomShooterMotor, Globals.defaultCloseZonePower),
//                        new SequentialAction(
//                                new SleepAction(0.8),
//                                new ParallelAction(
//                                        new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//                                )
//                        )
//                ),
//                new SleepAction(shooterDelay),
//                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
//                new SleepAction(1.5)
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//
//        //Collect 3rd spike mark
//        TrajectoryActionBuilder goToIntakeSpike3 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(36, -25), Math.toRadians(270), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new ParallelAction(
//                goToIntakeSpike3.build(),
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true),
//                new setIntake(intakeMotor, transferMotor, 1.0),
//                new setShooter(topShooterMotor, bottomShooterMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        TrajectoryActionBuilder intakeSpike3 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(36, -66), Math.toRadians(270), new TranslationalVelConstraint(minVelIntaking), new ProfileAccelConstraint(minAccelIntaking, maxAccelIntaking));
//
//        Actions.runBlocking(new SequentialAction(
//                intakeSpike3.build()
//                //new setIntake(intakeMotor, transferMotor, 0.0)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//
//
//        //shoot 3rd spike mark
//        TrajectoryActionBuilder scoreSpike3 = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(-28, -15), Math.toRadians(48), new TranslationalVelConstraint(minVelDrive), new ProfileAccelConstraint(minAccelDrive, maxAccelDrive));
//
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        scoreSpike3.build(),
//                        new setShooter(topShooterMotor, bottomShooterMotor, Globals.defaultCloseZonePower),
//                        new SequentialAction(
//                                new SleepAction(0.8),
//                                new ParallelAction(
//                                        new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//                                )
//                        )
//                ),
//                new SleepAction(shooterDelay),
//                new feedBalls(transferMotor, intakeMotor, transferServoLeft, transferServoRight),
//                new SleepAction(1.5)
//                //new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//        //park
//        TrajectoryActionBuilder park = drive.actionBuilder(drive.localizer.getPose())
//                .strafeToLinearHeading(new Vector2d(-22, -58), Math.toRadians(90), new TranslationalVelConstraint(200), new ProfileAccelConstraint(-200, 200));
//
//        Actions.runBlocking(new SequentialAction(
//                new ParallelAction(
//                        park.build(),
//                        new setShooter(topShooterMotor, bottomShooterMotor, 0.0),
//                        new stopFeed(transferMotor, intakeMotor, transferServoLeft, transferServoRight, true)
//                )
//        ));
//
//        drive.updatePoseEstimate();
//        drive.localizer.update();
//
//        sleep(1000);
//
//    }
//
//    public class setShooter implements Action {
//
//        DcMotorEx topShooterMotor;
//        DcMotorEx bottomShooterMotor;
//
//        double power;
//
//        public setShooter(DcMotorEx topShooterMotor, DcMotorEx bottomShooterMotor, double power){
//            this.topShooterMotor = topShooterMotor;
//            this.bottomShooterMotor = bottomShooterMotor;
//            this.power = power;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            topShooterMotor.setPower(power);
//            bottomShooterMotor.setPower(power);
//
//            return false;
//        }
//    }
//
//    public class feedBalls implements Action {
//
//        DcMotorEx transferMotor;
//        DcMotorEx intakeMotor;
//        CRServo transferServoLeft;
//        CRServo transferServoRight;
//
//        public feedBalls(DcMotorEx transferMotor, DcMotorEx intakeMotor, CRServo transferServoLeft, CRServo transferServoRight){
//            this.transferMotor = transferMotor;
//            this.intakeMotor = intakeMotor;
//            this.transferServoLeft = transferServoLeft;
//            this.transferServoRight = transferServoRight;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            transferMotor.setPower(v1.transferMotorShoot);
//            intakeMotor.setPower(v1.intakeShoot);
//            transferServoLeft.setPower(-v1.transferServoShoot);
//            transferServoRight.setPower(v1.transferServoShoot);
//
//            return false;
//        }
//    }
//
//    public class stopFeed implements Action {
//
//        DcMotorEx transferMotor;
//        DcMotorEx intakeMotor;
//        CRServo transferServoLeft;
//        CRServo transferServoRight;
//        boolean stopIntakeMotor;
//
//        public stopFeed(DcMotorEx transferMotor, DcMotorEx intakeMotor, CRServo transferServoLeft, CRServo transferServoRight, boolean stopIntakeMotor){
//            this.transferMotor = transferMotor;
//            this.intakeMotor = intakeMotor;
//            this.transferServoLeft = transferServoLeft;
//            this.transferServoRight = transferServoRight;
//            this.stopIntakeMotor = stopIntakeMotor;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            transferMotor.setPower(0.0);
//            transferServoLeft.setPower(0.0);
//            transferServoRight.setPower(0.0);
//
//            if (stopIntakeMotor == true) {
//                intakeMotor.setPower(0.0);
//            } else {
//
//            }
//
//            return false;
//        }
//    }
//
//    public class setIntake implements Action {
//
//        DcMotorEx intakeMotor;
//        DcMotorEx transferMotor;
//
//        double power;
//
//        public setIntake(DcMotorEx intakeMotor, DcMotorEx transferMotor, double power){
//            this.intakeMotor = intakeMotor;
//            this.transferMotor = transferMotor;
//            this.power = power;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//
//            intakeMotor.setPower(power);
//            if (power != 0.0) {
//                transferMotor.setPower(v1.transferIntake);
//            } else {
//                transferMotor.setPower(0.0);
//            }
//
//            return false;
//        }
//    }
//
//}