package org.firstinspires.ftc.teamcode.pathgen;

import java.util.Arrays;
import java.util.List;

public class TriangleObstacle implements Obstacle {
    final double ax, ay, bx, by, cx, cy;

    public TriangleObstacle(double ax, double ay, double bx, double by, double cx, double cy) {
        this.ax = ax; this.ay = ay; this.bx = bx; this.by = by; this.cx = cx; this.cy = cy;
    }

    @Override
    public boolean contains(double x, double y, double m) {
        // inflate by pushing each corner out from the centroid by `m`, then point-in-triangle
        double gx = (ax + bx + cx) / 3, gy = (ay + by + cy) / 3;
        double[] A = push(ax, ay, gx, gy, m), B = push(bx, by, gx, gy, m), C = push(cx, cy, gx, gy, m);
        double d1 = sign(x, y, A[0], A[1], B[0], B[1]);
        double d2 = sign(x, y, B[0], B[1], C[0], C[1]);
        double d3 = sign(x, y, C[0], C[1], A[0], A[1]);
        boolean neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(neg && pos);
    }

    @Override
    public List<double[]> navPoints(double m) {
        double gx = (ax + bx + cx) / 3, gy = (ay + by + cy) / 3, o = m + 2;
        return Arrays.asList(
            push(ax, ay, gx, gy, o),
            push(bx, by, gx, gy, o),
            push(cx, cy, gx, gy, o));
    }

    // push (px,py) away from center (gx,gy) by distance d
    private static double[] push(double px, double py, double gx, double gy, double d) {
        double dx = px - gx, dy = py - gy, len = Math.hypot(dx, dy);
        if (len < 1e-9) return new double[]{ px, py };
        return new double[]{ px + dx / len * d, py + dy / len * d };
    }

    private static double sign(double px, double py, double ax, double ay, double bx, double by) {
        return (px - bx) * (ay - by) - (ax - bx) * (py - by);
    }
}
