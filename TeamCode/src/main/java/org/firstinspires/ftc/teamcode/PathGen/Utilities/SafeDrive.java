package org.firstinspires.ftc.teamcode.PathGen.Utilities;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

/**
 * Safe path following that completes on ARRIVAL, not on Pedro's strict end-tolerance.
 *
 * The problem this fixes: Pedro's follower only reports done when it's inside a tight end tolerance,
 * and the robot often settles a hair outside it -> the follow command never finishes, so whatever is
 * scheduled after it (e.g. firing) never runs, until you physically nudge the robot into tolerance.
 *
 * SafeDrive.follow completes when ANY of these is true:
 *   - the robot is within ARRIVE_TOL of the target point, AFTER having first left it by LEAVE_DIST
 *     (the "left first" guard stops it from instantly completing at the start of a cycle when it's
 *      already sitting on the target from the previous cycle),
 *   - Pedro's own isBusy() clears (the normal case, when the follower does settle in tolerance),
 *   - or it stalls (no movement/turn for STALL_MS) -- a backstop so it can never hang.
 *
 * This is general: it only needs the path and the target point, so it works for any path/target,
 * random test point or live camera point alike. The turret handles aiming, so being within a few
 * inches of the target is fine for scoring.
 */
public class SafeDrive {

    public static double ARRIVE_TOL = 4.0;    // inches from target that counts as "arrived"
    public static double LEAVE_DIST = 12.0;   // must get this far from target before arrival can trigger
    public static long   STALL_MS   = 2500;   // give up if no progress for this long (backstop)
    public static double POS_EPS    = 0.5;    // inches of motion that counts as progress
    public static double HEAD_EPS   = Math.toRadians(2);

    public static Command follow(Follower follower, PathChain chain, double targetX, double targetY) {
        return follow(follower, chain, true, targetX, targetY);
    }

    public static Command follow(Follower follower, PathChain chain, boolean holdEnd,
                                 double targetX, double targetY) {
        final boolean[] hasLeft = { false };
        final double[] mark = new double[3];      // last-progress x, y, heading
        final long[]   markTime = new long[1];

        return Command.build()
                .setStart(() -> {
                    follower.followPath(chain, holdEnd);
                    mark[0] = follower.getPose().getX();
                    mark[1] = follower.getPose().getY();
                    mark[2] = follower.getPose().getHeading();
                    markTime[0] = System.currentTimeMillis();
                    hasLeft[0] = false;
                })
                .setDone(() -> {
                    double x = follower.getPose().getX();
                    double y = follower.getPose().getY();
                    double h = follower.getPose().getHeading();
                    double d = Math.hypot(x - targetX, y - targetY);

                    if (d > LEAVE_DIST) hasLeft[0] = true;

                    // arrived: back near the target after leaving, OR the follower itself finished
                    if (hasLeft[0] && (d < ARRIVE_TOL || !follower.isBusy())) return true;

                    // stall backstop: no movement/turn for STALL_MS
                    double moved  = Math.hypot(x - mark[0], y - mark[1]);
                    double turned = Math.abs(wrap(h - mark[2]));
                    if (moved > POS_EPS || turned > HEAD_EPS) {
                        mark[0] = x; mark[1] = y; mark[2] = h;
                        markTime[0] = System.currentTimeMillis();
                        return false;
                    }
                    return System.currentTimeMillis() - markTime[0] > STALL_MS;
                });
    }

    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
