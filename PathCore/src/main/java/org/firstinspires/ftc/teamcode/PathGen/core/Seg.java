package org.firstinspires.ftc.teamcode.PathGen.core;

/**
 * One drawable/drivable piece of a plan: either a cubic Bezier ("curve") or a straight line,
 * with the heading at each end. This is PURE DATA -- no Pedro, no Android -- so the robot's
 * PathPlanner turns it into a Pedro path, and the desktop visualizer draws the very same numbers.
 * That shared representation is what keeps the two from ever disagreeing.
 */
public class Seg {
    public final double x1, y1, h1;          // start point + heading (radians)
    public final double x2, y2, h2;          // end point + heading (radians)
    public final boolean curve;              // true = cubic Bezier, false = straight line
    public final double c1x, c1y, c2x, c2y;  // Bezier control points (only meaningful when curve)

    public Seg(double x1, double y1, double h1,
               double x2, double y2, double h2,
               boolean curve,
               double c1x, double c1y, double c2x, double c2y) {
        this.x1 = x1; this.y1 = y1; this.h1 = h1;
        this.x2 = x2; this.y2 = y2; this.h2 = h2;
        this.curve = curve;
        this.c1x = c1x; this.c1y = c1y; this.c2x = c2x; this.c2y = c2y;
    }

    public static Seg straight(double x1, double y1, double h1, double x2, double y2, double h2) {
        return new Seg(x1, y1, h1, x2, y2, h2, false, 0, 0, 0, 0);
    }

    public static Seg curved(double x1, double y1, double h1, double x2, double y2, double h2,
                             double c1x, double c1y, double c2x, double c2y) {
        return new Seg(x1, y1, h1, x2, y2, h2, true, c1x, c1y, c2x, c2y);
    }
}
