package org.firstinspires.ftc.teamcode.pathgen;

import java.util.Arrays;
import java.util.List;

public class RectObstacle implements Obstacle {
    final double minX, minY, maxX, maxY;

    public RectObstacle(double minX, double minY, double maxX, double maxY) {
        this.minX = minX; this.minY = minY; this.maxX = maxX; this.maxY = maxY;
    }

    @Override
    public boolean contains(double x, double y, double m) {
        return x >= minX - m && x <= maxX + m && y >= minY - m && y <= maxY + m;
    }

    @Override
    public List<double[]> navPoints(double m) {
        double o = m + 2;   // corners pushed out past the inflated box
        return Arrays.asList(
            new double[]{ minX - o, minY - o },
            new double[]{ maxX + o, minY - o },
            new double[]{ maxX + o, maxY + o },
            new double[]{ minX - o, maxY + o });
    }
}
