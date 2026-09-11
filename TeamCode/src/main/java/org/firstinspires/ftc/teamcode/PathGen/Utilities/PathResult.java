package org.firstinspires.ftc.teamcode.PathGen.Utilities;

import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * What the planner returns. It NEVER throws -- it always hands back one of these so auto can't crash.
 *   OK       - planned exactly what was asked
 *   ADJUSTED - planned, but had to fix something (see notes); path is still usable
 *   FAILED   - couldn't make a usable path (see notes); path is null, use a fallback
 */
public class PathResult {
    public enum Status { OK, ADJUSTED, FAILED }

    public final Status status;
    public final PathChain path;       // null only when FAILED
    public final List<String> notes;   // adjustments made, or the failure reason

    private PathResult(Status s, PathChain p, List<String> n) { status = s; path = p; notes = n; }

    public static PathResult ok(PathChain p) {
        return new PathResult(Status.OK, p, new ArrayList<>());
    }
    public static PathResult adjusted(PathChain p, List<String> notes) {
        return new PathResult(Status.ADJUSTED, p, notes);
    }
    public static PathResult failed(String why) {
        List<String> n = new ArrayList<>(); n.add(why);
        return new PathResult(Status.FAILED, null, n);
    }

    /** OK or ADJUSTED - safe to drive. */
    public boolean usable() { return path != null; }
}
