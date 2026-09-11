package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.ArrayList;
import java.util.List;

public class CircleObstacle implements Obstacle {
    private final double cx, cy, r;

    public CircleObstacle(double cx, double cy, double r) {
        this.cx = cx; this.cy = cy; this.r = r;
    }

    @Override
    public boolean contains(double x, double y) {
        return contains(x, y, 0);
    }

    @Override
    public boolean contains(double x, double y, double margin) {
        // distance from the CIRCLE'S CENTER (was measuring from origin (0,0) before -- bug fix)
        return Math.hypot(x - cx, y - cy) <= r + margin;
    }

    @Override
    public List<double[]> navPoints(double margin) {
        List<double[]> pts = new ArrayList<>();
        double rr = r + margin + 3;   // ring just outside the inflated circle
        // 12 points (not 8): with too few, the straight chord between adjacent nav points dips back
        // inside the keep-out for a big robot, so the router can't connect them and routing fails.
        int n = 12;
        for (int i = 0; i < n; i++) {
            double a = 2 * Math.PI * i / n;
            pts.add(new double[]{ cx + rr * Math.cos(a), cy + rr * Math.sin(a) });
        }
        return pts;
    }
}
