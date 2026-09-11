package org.firstinspires.ftc.teamcode.PedroAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PathGen.Utilities.PathPlanner;
import org.firstinspires.ftc.teamcode.PathGen.Utilities.PathResult;
import org.firstinspires.ftc.teamcode.PathGen.core.Field;
import org.firstinspires.ftc.teamcode.PathGen.core.RectObstacle;
import org.firstinspires.ftc.teamcode.PathGen.core.TriObstacle;
import org.firstinspires.ftc.teamcode.PedroAuto.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.PedroAuto.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * PURE TEST for the path generator -- forever, until stopped.
 *   - starts bottom-right facing UP; shooter (velPID + turret tracking) runs the whole time
 *   - each PATH is ONE continuous route: current -> 3 random ball points -> the fixed scoring spot
 *     END. The robot flows through all 3 balls without stopping (front leads = intake), and the
 *     final leg into END auto-picks forward vs reverse (whichever is faster).
 *   - AT END: transfer ON to fire, hold 1s, then transfer OFF + intake back ON -- then a brand-new
 *     path with 3 fresh random balls begins.
 *
 * Robustness is all in the core (PathPlanner never throws, SafeDrive can't hang, blocked targets are
 * nudged clear at any heading).
 *
 * Pedro coords: 144x144 in, x right, y up. Heading 0 = +x (right), 90deg = +y (up).
 */
@Configurable
@Autonomous(name = "Random Point Auto")
public class RandomPointAuto extends LinearOpMode {

    // ---- set these to your real robot ----
    private static final double ROBOT_WIDTH  = 15.5;   // inches
    private static final double ROBOT_LENGTH = 17.5;   // inches
    private static final int    FIRE_MS      = 1000;   // firing dwell at the end point
    private static final double BALL_SPACING = 20;     // min spacing between generated ball points

    // The fixed scoring spot every path returns to. Orientation there doesn't matter (turret aims).
    private static final double[] END = { 48, 96 };

    // ---- shooter config (tunable in Panels, same as BlueAuto21) ----
    public static double shootingSpeedPID = -800;
    public static double turretOffset     = 0.002;
    public static double turretStartPos   = 0.5;

    // Start: robot JAMMED in the bottom-right corner, facing UP. The center is computed from the
    // robot's size so this stays correct if you change ROBOT_WIDTH/LENGTH: facing up, WIDTH is the
    // x extent (against the right wall at x=144) and LENGTH is the y extent (against the bottom wall
    // at y=0). Set this to wherever you physically place the robot -- it MUST match, or the whole
    // run is offset by the difference.
    private final Pose startPose =
            new Pose(144 - ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, Math.toRadians(90));

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private GoBildaPinpointDriver pinpoint;
    private Field field;
    private PathPlanner planner;
    private final Random rng = new Random();

    private boolean busy = false;
    private int pathCount = 0;
    private double[] b1, b2, b3;
    private boolean returnReversed = false;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, turretOffset, turretStartPos);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pinpoint.recalibrateIMU();
        sleep(1000);

        intake.init();
        shooter.init();

        // ===== obstacles (edit freely). Field is 144x144, x right, y up. =====
        field = new Field(ROBOT_WIDTH, ROBOT_LENGTH)
                .add(new RectObstacle(0, 67, 0, 144))
                .add(new TriObstacle(0, 144, 25, 144, 0, 110))
                .add(new TriObstacle(72, 96, 48, 72, 96, 72))   // diamond top half
                .add(new TriObstacle(72, 48, 48, 72, 96, 72));  // diamond bottom half
        field.minClearance(3);
        planner = new PathPlanner(field);

        telemetry.addLine("BUILD: wall-intake  (2026-09-08b)");   // change this each deploy to confirm the new code is running
        telemetry.addLine("Random Point Auto -- one continuous path through 3 balls to " + fmt(END) + ", forever.");
        telemetry.addLine("Intake collects along the way; fires at the goal, then a fresh path.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        schedule(shooter.startVelPID(shootingSpeedPID));
        schedule(shooter.startTurretTracking(follower));
        schedule(intake.intakeOn());   // intaking for the first run through the balls

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            if (!busy && scheduleNewPath()) busy = true;

            telemetry.addData("BUILD", "wall-intake 2026-09-08b");   // confirms which code is deployed
            telemetry.addData("path #", pathCount);
            telemetry.addData("state", busy ? "driving" : "planning next path");
            telemetry.addData("balls", fmt(b1) + " " + fmt(b2) + " " + fmt(b3));
            telemetry.addData("return", returnReversed ? "REVERSE" : "forward");
            telemetry.addData("flywheel target", shootingSpeedPID);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    /** ONE continuous path: current -> b1 -> b2 -> b3 -> END, then fire and resume intaking. */
    private boolean scheduleNewPath() {
        double[] cur = { follower.getPose().getX(), follower.getPose().getY() };
        b1 = randomValidPoint(cur, BALL_SPACING);
        b2 = randomValidPoint(b1,  BALL_SPACING);
        b3 = randomValidPoint(b2,  BALL_SPACING);

        List<double[]> waypoints = new ArrayList<>();
        waypoints.add(cur);
        waypoints.add(b1);
        waypoints.add(b2);
        waypoints.add(b3);
        waypoints.add(END);

        // ONE continuous path; the final leg into END auto-picks forward vs reverse (whichever faster).
        PathResult path = planner.planWithReturn(follower, waypoints);
        returnReversed = planner.lastReturnReversed;
        if (!path.usable()) return false;   // couldn't plan this run -> retry next loop with new balls

        List<Command> steps = new ArrayList<>();
        steps.add(follow(follower, path.path, true));   // Pedro's follow; end-tolerance set in Constants
        // score at END: fire for FIRE_MS, then stop firing and resume intaking as the next path starts
        steps.add(intake.transferOn());
        steps.add(waitMs(FIRE_MS));
        steps.add(intake.transferOff());
        steps.add(intake.intakeOn());
        // allow the next path to be generated
        steps.add(Command.build().setStart(() -> busy = false).setDone(() -> true));

        schedule(sequential(steps.toArray(new Command[0])));
        pathCount++;
        return true;
    }

    /**
     * Random ball point ANYWHERE on the field, including the wall strip (balls can be against walls).
     * Only points inside a real obstacle are rejected -- wall-strip balls are fine, the planner grabs
     * them with a head-on or wall-ride intake. Must be at least minAway inches from `from`.
     */
    private double[] randomValidPoint(double[] from, double minAway) {
        double lo = 2, hi = 142;
        double[] best = { 72, 72 };
        for (int i = 0; i < 300; i++) {
            double x = lo + rng.nextDouble() * (hi - lo);
            double y = lo + rng.nextDouble() * (hi - lo);
            if (field.blockedByObstacle(x, y)) continue;   // reject only real obstacles; walls are OK
            best = new double[]{ x, y };
            if (dist(best, from) >= minAway) return best;
        }
        return best;
    }

    private static double dist(double[] a, double[] b) { return Math.hypot(a[0] - b[0], a[1] - b[1]); }
    private static String fmt(double[] p) { return p == null ? "-" : String.format("(%.0f,%.0f)", p[0], p[1]); }
}
