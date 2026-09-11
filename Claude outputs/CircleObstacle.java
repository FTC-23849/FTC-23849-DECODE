package org.firstinspires.ftc.teamcode.pathgen;

import java.util.ArrayList;
import java.util.List;

public class CircleObstacle implements Obstacle {
    final double cx, cy, r;

    public CircleObstacle(double cx, double cy, double r) {
        this.cx = cx; this.cy = cy; this.r = r;
    }

    @Override
    public boolean contains(double x, double y, double margin) {
        return Math.hypot(x - cx, y - cy) <= r + margin;
    }

    @Override
    public List<double[]> navPoints(double margin) {
        List<double[]> pts = new ArrayList<>();
        double rr = r + margin + 2;          // a ring just outside the inflated circle
        int n = 8;                           // 8 nodes around it
        for (int i = 0; i < n; i++) {
            double a = 2 * Math.PI * i / n;
            pts.add(new double[]{ cx + rr * Math.cos(a), cy + rr * Math.sin(a) });
        }
        return pts;
    }
}
