package org.firstinspires.ftc.teamcode.PathGen.Utilities;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.PathGen.core.Field;
import org.firstinspires.ftc.teamcode.PathGen.core.GeoResult;
import org.firstinspires.ftc.teamcode.PathGen.core.PlanEngine;
import org.firstinspires.ftc.teamcode.PathGen.core.Seg;

import java.util.List;

/**
 * Thin Pedro adapter over the pure PlanEngine (in :PathCore). All path logic lives in PlanEngine;
 * this only turns its plain-data segments into a Pedro PathChain.
 *
 *  - plan(follower, waypoints): forward pass, front leads through every point.
 *  - planWithReturn(follower, waypoints): ONE continuous path -- front leads through every point
 *    except the last, and the final leg into the end point auto-picks forward vs reverse (whichever
 *    faster). lastReturnReversed reports the choice.
 *  - planReturn(follower, from, to, incomingHeading): a single reversible leg on its own.
 *
 * Blocked targets are nudged clear (valid at any heading). Never throws.
 */
public class PathPlanner {

    // These SEED from PlanEngine's single-source defaults, so the robot, the engine, and the desktop
    // visualizer all shape paths the same way. Edit the DEFAULT_* constants in PlanEngine to change all
    // three at once; set these fields directly only if you want the robot to differ from that default.
    public double Vx = PlanEngine.DEFAULT_VX;
    public double Vy = PlanEngine.DEFAULT_VY;
    public double catmullTension = PlanEngine.DEFAULT_CATMULL_TENSION;
    public double turnRate = PlanEngine.DEFAULT_TURN_RATE;   // rad/s used in the forward-vs-reverse cost estimate

    /** How far along a path (0-1) counts as "reached", set per-path on every segment we build. Lower =
     *  the follower calls the path done sooner, so it won't hang just short of the end (Pedro's default
     *  0.99 is so tight the robot often settles at ~0.98 and never completes). holdEnd still finishes
     *  the last bit onto the exact point. Tune up toward 0.99 for tighter, down for looser/snappier. */
    public double endTValue = 0.94;

    /** After planWithReturn / planReturn: true if the return leg was driven in reverse. */
    public boolean lastReturnReversed = false;

    private final Field field;

    public PathPlanner(Field field) { this.field = field; }

    private PlanEngine engine() {
        PlanEngine e = new PlanEngine(field);
        e.Vx = Vx; e.Vy = Vy; e.catmullTension = catmullTension; e.turnRate = turnRate;
        return e;
    }

    /** Forward plan through waypoints (each {x,y}). Never throws. */
    public PathResult plan(Follower follower, List<double[]> waypoints) {
        try {
            return toResult(follower, engine().plan(waypoints));
        } catch (Exception e) {
            return PathResult.failed("planner error: " + e.getMessage());
        }
    }

    /** ONE continuous path through all waypoints; the last leg (into the end) auto-forward/reverses. */
    public PathResult planWithReturn(Follower follower, List<double[]> waypoints) {
        try {
            PlanEngine e = engine();
            GeoResult g = e.planWithReturn(waypoints);
            lastReturnReversed = e.lastReturnReversed;
            return toResult(follower, g);
        } catch (Exception ex) {
            return PathResult.failed("planner error: " + ex.getMessage());
        }
    }

    /** A single non-intake leg that auto-picks forward vs reverse. incomingHeading = current heading. */
    public PathResult planReturn(Follower follower, double[] from, double[] to, double incomingHeading) {
        try {
            PlanEngine e = engine();
            GeoResult g = e.planReturn(from, to, incomingHeading);
            lastReturnReversed = e.lastReturnReversed;
            return toResult(follower, g);
        } catch (Exception ex) {
            return PathResult.failed("planner error: " + ex.getMessage());
        }
    }

    /** Convert the pure segments into a Pedro PathChain (the only Pedro-specific step). */
    private PathResult toResult(Follower follower, GeoResult g) {
        if (!g.usable())
            return PathResult.failed(g.notes.isEmpty() ? "no usable path" : g.notes.get(0));

        PathBuilder builder = follower.pathBuilder();
        for (Seg s : g.segs) {
            Pose p1 = new Pose(s.x1, s.y1, s.h1);
            Pose p2 = new Pose(s.x2, s.y2, s.h2);
            if (s.curve) {
                builder.addPath(new BezierCurve(p1, new Pose(s.c1x, s.c1y), new Pose(s.c2x, s.c2y), p2));
            } else {
                builder.addPath(new BezierLine(p1, p2));
            }
            builder.setLinearHeadingInterpolation(s.h1, s.h2);
            builder.setTValueConstraint(endTValue);   // per-path: how close to the end counts as reached
        }
        PathChain chain = builder.build();

        return g.status == GeoResult.Status.ADJUSTED
                ? PathResult.adjusted(chain, g.notes)
                : PathResult.ok(chain);
    }
}
