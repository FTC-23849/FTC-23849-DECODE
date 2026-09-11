package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.ArrayList;
import java.util.List;

/**
 * The PURE result of planning: a list of drivable segments, the routed polyline, and notes.
 * No Pedro types, so the desktop visualizer can use it directly. On the robot, PathPlanner
 * converts the segments into a Pedro PathChain and wraps this in a PathResult.
 * Like PathResult, it is NEVER thrown -- always returned -- so auto can't crash.
 *   OK       - planned exactly what was asked
 *   ADJUSTED - planned, but had to fix something (see notes); still usable
 *   FAILED   - no usable path (see notes); segs is empty, use a fallback
 */
public class GeoResult {
    public enum Status { OK, ADJUSTED, FAILED }

    public final Status status;
    public final List<Seg> segs;         // in order; empty only when FAILED
    public final List<double[]> route;   // the raw routed points {x, y} (handy for drawing nodes)
    public final List<String> notes;     // adjustments made, or the failure reason

    private GeoResult(Status s, List<Seg> segs, List<double[]> route, List<String> notes) {
        this.status = s; this.segs = segs; this.route = route; this.notes = notes;
    }

    public static GeoResult ok(List<Seg> segs, List<double[]> route) {
        return new GeoResult(Status.OK, segs, route, new ArrayList<String>());
    }
    public static GeoResult adjusted(List<Seg> segs, List<double[]> route, List<String> notes) {
        return new GeoResult(Status.ADJUSTED, segs, route, notes);
    }
    public static GeoResult failed(String why) {
        List<String> n = new ArrayList<String>(); n.add(why);
        return new GeoResult(Status.FAILED, new ArrayList<Seg>(), new ArrayList<double[]>(), n);
    }

    /** OK or ADJUSTED - safe to drive/draw. */
    public boolean usable() { return status != Status.FAILED; }
}
