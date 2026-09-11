package org.firstinspires.ftc.teamcode.pathgen;

import java.util.ArrayList;
import java.util.List;

/**
 * The field: holds any number of obstacles and answers collision questions for the planner.
 * Clearance is a bounding circle from the robot's width & length, so the robot stays clear at
 * ANY heading (conservative but safe for a rotating holonomic robot).
 */
public class Field {
    private final List<Obstacle> obstacles = new ArrayList<>();
    public final double robotRadius;

    public Field(double robotWidth, double robotLength) {
        // half the diagonal = the robot's worst-case radius no matter which way it's facing
        this.robotRadius = 0.5 * Math.hypot(robotWidth, robotLength);
    }

    public Field add(Obstacle o) { obstacles.add(o); return this; }

    /** Would the robot's body touch any obstacle if its center were here? */
    public boolean blocked(double x, double y) {
        for (Obstacle o : obstacles) if (o.contains(x, y, robotRadius)) return true;
        return false;
    }

    /** Can the robot drive a straight line from (x1,y1) to (x2,y2) without its body clipping anything? */
    public boolean isSegmentClear(double x1, double y1, double x2, double y2) {
        double dist = Math.hypot(x2 - x1, y2 - y1);
        int samples = Math.max(2, (int) dist);   // ~1 inch spacing
        for (int i = 0; i <= samples; i++) {
            double t = i / (double) samples;
            if (blocked(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)) return false;
        }
        return true;
    }

    /** All obstacle corner nav-points, dropping any that land inside another obstacle. */
    public List<double[]> navPoints() {
        List<double[]> all = new ArrayList<>();
        for (Obstacle o : obstacles)
            for (double[] p : o.navPoints(robotRadius))
                if (!blocked(p[0], p[1])) all.add(p);
        return all;
    }
}
