package org.firstinspires.ftc.teamcode.PedroAuto; // make sure this aligns with class location

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.PedroAuto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.PedroAuto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class BlueAuto21 extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    private Follower follower;
    private Intake intake;
    private Shooter shooter;

    public static double shootingSpeedPID = -800;

    public static double turretOffset = 0.002;

    public static double turretStartPos = 0.5;

    public static int transferStartPause = 0;
    public static int transferPause = 400;
    public static int gateIntakePause = 1500;

    private final Pose startPose = new Pose(27, 128, Math.toRadians(-45)); // Start Pose of our robot.
    private final Pose scorePreloads = new Pose(51, 84, Math.toRadians(252));
    private final Pose spike2 = new Pose(12, 58);
    private final Pose scorePose = new Pose(51, 84);
    private final Pose gateIntake = new Pose(12, 59, Math.toRadians(150));
    private final Pose scoreGatePos = new Pose(51, 84, Math.toRadians(222));

    //defining our PathChains
    private PathChain scorePreload, intakeSpike2, scoreSpike2, intakeGate, scoreGate;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePreloads))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloads.getHeading())
                .build();

        intakeSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloads, new Pose(43, 58), spike2))
                .setTangentHeadingInterpolation()
                .build();

        scoreSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(spike2, new Pose(22, 58), scorePose))
                .setTangentHeadingInterpolation()
                .build();

        intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(34, 59), gateIntake))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateIntake.getHeading())
                .build();

        scoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(gateIntake, new Pose(34, 59), scoreGatePos))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), scorePreloads.getHeading())
                .build();

    }

    public Command autoRoutine() {
        return sequential(

                follow(follower, scorePreload, true),
                waitMs(transferStartPause),
                intake.transferOn(),
                waitMs(transferPause),

                parallel(
                        sequential(
                                intake.transferOff(),
                                intake.intakeOn()
                        ),
                        follow(follower, intakeSpike2, true)
                ),
                follow(follower, scoreSpike2, true),
                waitMs(transferStartPause),
                intake.transferOn(),
                waitMs(transferPause),

                parallel(
                        sequential(
                                intake.transferOff(),
                                intake.intakeOn()
                        ),
                        follow(follower, intakeGate, true)
                ),
                waitMs(gateIntakePause),
                follow(follower, scoreGate, true),
                waitMs(transferStartPause),
                intake.transferOn(),
                waitMs(transferPause)

        );
    }

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, turretOffset, turretStartPos);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        pinpoint.recalibrateIMU();
        sleep(1000);

        intake.init();
        shooter.init();

        waitForStart();

        schedule(autoRoutine());
        schedule(shooter.startVelPID(shootingSpeedPID));
        schedule(shooter.startTurretTracking(follower));

        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
