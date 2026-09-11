package org.firstinspires.ftc.teamcode.pathgen;

import java.util.List;

/**
 * Anything the robot must drive around. Each shape answers two questions the planner needs:
 * "is this point inside me (with clearance)?" and "what corner points should the route consider?"
 */
public interface Obstacle {

    /** Is (x,y) inside this shape, expanded outward by `margin` (pass the robot radius for clearance)? */
    boolean contains(double x, double y, double margin);

    /** Candidate route nodes sitting `margin` outside my corners/edges. Each is {x, y}. */
    List<double[]> navPoints(double margin);
}
