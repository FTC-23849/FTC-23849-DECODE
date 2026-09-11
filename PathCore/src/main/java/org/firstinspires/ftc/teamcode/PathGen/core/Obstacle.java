package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.List;

/** Anything the robot must drive around. */
public interface Obstacle {

    /** Inside the shape? */
    boolean contains(double x, double y);

    /** Inside the shape expanded outward by `margin` (pass the robot radius for clearance)? */
    boolean contains(double x, double y, double margin);

    /** Candidate route nodes sitting `margin` outside my corners/edges. Each is {x, y}. */
    List<double[]> navPoints(double margin);
}
