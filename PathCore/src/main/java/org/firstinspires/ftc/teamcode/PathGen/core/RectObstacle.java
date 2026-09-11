package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.Arrays;
import java.util.List;

public class RectObstacle implements Obstacle {
    private final double lcx, lcy, ucx, ucy;   // lower-left corner to upper-right corner

    public RectObstacle(double lcx, double lcy, double ucx, double ucy) {
        this.lcx = lcx; this.lcy = lcy; this.ucx = ucx; this.ucy = ucy;
    }

    @Override
    public boolean contains(double x, double y) {
        return contains(x, y, 0);
    }

    @Override
    public boolean contains(double x, double y, double m) {
        return x >= lcx - m && x <= ucx + m && y >= lcy - m && y <= ucy + m;
    }

    @Override
    public List<double[]> navPoints(double m) {
        double o = m + 2;   // corners pushed out past the inflated box
        return Arrays.asList(
            new double[]{ lcx - o, lcy - o },
            new double[]{ ucx + o, lcy - o },
            new double[]{ ucx + o, ucy + o },
            new double[]{ lcx - o, ucy + o });
    }
}
