package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.ArrayList;
import java.util.List;

/**
 * Holds any number of obstacles and answers collision questions for the planner.
 * Keep-out margin = robot bounding-circle radius (from width & length, so it clears at ANY
 * heading) + an adjustable minimum clearance you set with minClearance(...).
 */
public class Field {
    private final List<Obstacle> obstacles = new ArrayList<>();
    public final double robotRadius;
    public final double robotWidth;    // side-to-side extent (x when facing +y)
    public final double robotLength;   // front-to-back extent (used for the front/intake reach)
    private double minClearance = 0;   // extra breathing room beyond the robot's body
    private double size = 144;         // field is size x size inches (FTC = 144). 0 disables walls.

    public Field() { this.robotRadius = 0; this.robotWidth = 0; this.robotLength = 0; }   // original behavior

    public Field(double robotWidth, double robotLength) {
        // half the diagonal = worst-case radius no matter which way the robot faces
        this.robotRadius = 0.5 * Math.hypot(robotWidth, robotLength);
        this.robotWidth = robotWidth;
        this.robotLength = robotLength;
    }

    /** Extra inches of clearance kept around every obstacle, on top of the robot's body. Chainable. */
    public Field minClearance(double inches) { this.minClearance = inches; return this; }
    public double getMinClearance() { return minClearance; }

    /** Field side length in inches (default 144). Set 0 to disable the wall boundary. Chainable. */
    public Field size(double inches) { this.size = inches; return this; }
    public double getSize() { return size; }

    /** Total keep-out margin the planner actually uses. */
    public double clearance() { return robotRadius + minClearance; }

    public Field add(Obstacle o) { obstacles.add(o); return this; }

    /**
     * Would the robot's body (plus min clearance) touch any obstacle -- OR a WALL -- if its center
     * were here? The wall check keeps the center at least its own radius from every edge, because the
     * center physically can't get closer than that (its body would be in the wall). So the planner
     * never targets or routes through the unreachable strip around the field edge.
     */
    public boolean blocked(double x, double y) {
        // walls: the robot center can't get within its radius of an edge
        if (size > 0 && robotRadius > 0 &&
                (x < robotRadius || x > size - robotRadius ||
                 y < robotRadius || y > size - robotRadius)) return true;
        return blockedByObstacle(x, y);
    }

    /** Obstacle-only block (ignores walls). Used for the deliberate intake dip into the wall strip,
     *  where the robot is approaching the wall on purpose at a heading where its body fits. */
    public boolean blockedByObstacle(double x, double y) {
        double m = clearance();
        for (Obstacle o : obstacles) if (o.contains(x, y, m)) return true;
        return false;
    }

    /** Is this point on the field but inside the wall strip (center can't reach it, only the front can)? */
    public boolean inWallStrip(double x, double y) {
        if (size <= 0 || robotRadius <= 0) return false;
        boolean onField = x >= 0 && x <= size && y >= 0 && y <= size;
        boolean nearWall = x < robotRadius || x > size - robotRadius
                        || y < robotRadius || y > size - robotRadius;
        return onField && nearWall && !blockedByObstacle(x, y);
    }

    /** Can the robot drive a straight line from (x1,y1) to (x2,y2) without clipping anything? */
    public boolean isSegmentClear(double x1, double y1, double x2, double y2) {
        return segmentClear(x1, y1, x2, y2, true);
    }

    /** Straight-line clearance; wallAware=false ignores walls (for the intake dip). */
    public boolean segmentClear(double x1, double y1, double x2, double y2, boolean wallAware) {
        double dist = Math.hypot(x2 - x1, y2 - y1);
        int samples = Math.max(2, (int) dist);   // ~1 inch spacing
        for (int i = 0; i <= samples; i++) {
            double t = i / (double) samples;
            double px = x1 + (x2 - x1) * t, py = y1 + (y2 - y1) * t;
            if (wallAware ? blocked(px, py) : blockedByObstacle(px, py)) return false;
        }
        return true;
    }

    /** All obstacle corner nav-points, dropping any that land inside another obstacle. */
    public List<double[]> navPoints() {
        double m = clearance();
        List<double[]> all = new ArrayList<>();
        for (Obstacle o : obstacles)
            for (double[] p : o.navPoints(m))
                if (!blocked(p[0], p[1])) all.add(p);
        return all;
    }

    public List<Obstacle> getObstacles() { return obstacles; }
}
