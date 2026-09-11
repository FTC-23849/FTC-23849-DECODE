package org.firstinspires.ftc.teamcode.pathgen;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

/**
 * Builds the fastest sensible path through a list of field points while routing AROUND obstacles.
 *
 * How it meets the goals:
 *  - FRONT through the points: every point gets a tangent heading (faces the next point), so the
 *    robot drives forward through each waypoint -- which is what lets a front intake grab the balls.
 *  - Fast / accounts for strafe: because the robot drives FORWARD (tangent heading) it uses its fast
 *    axis (Vx) the whole time and never pays the slow strafe speed (Vy). The route itself is the
 *    SHORTEST forward-drivable path around the obstacles (visibility graph + Dijkstra), and corners
 *    are smoothed so it doesn't have to slow to turn. Shortest forward distance = least time here.
 *  - Doesn't hug obstacles: obstacles are inflated by the robot's bounding-circle radius and the
 *    route only travels between nav-points that sit OUTSIDE that inflation, so it keeps real clearance
 *    instead of scraping the edge.
 *  - Never crashes auto: everything is wrapped so it always returns a PathResult, never throws.
 */
public class PathPlanner {

    // Tunables (defaults ~ your robot; Vy is the slow strafe axis, kept for reference/robustness)
    public double Vx = 76;            // forward speed in/s (fast axis)
    public double Vy = 52;            // strafe speed in/s (slow axis)
    public double catmullTension = 6.0;   // higher = tighter/straighter smoothing

    private final Field field;

    public PathPlanner(Field field) { this.field = field; }

    /**
     * Plan a path through `waypoints` (each {x, y}), start first and end last, any number of middles.
     * Returns a PathResult you check before driving. Never throws.
     */
    public PathResult plan(Follower follower, List<double[]> waypoints) {
        try {
            List<String> notes = new ArrayList<>();
            if (waypoints == null || waypoints.size() < 2)
                return PathResult.failed("need at least a start and an end point");

            // 1) sanitize: nudge any waypoint sitting inside an obstacle out to the nearest free spot
            List<double[]> pts = new ArrayList<>();
            for (int i = 0; i < waypoints.size(); i++) {
                double[] w = waypoints.get(i);
                if (field.blocked(w[0], w[1])) {
                    double[] freed = nudgeOut(w[0], w[1]);
                    if (freed == null)
                        return PathResult.failed("waypoint " + i + " " + fmt(w) + " is stuck inside an obstacle");
                    notes.add("waypoint " + i + " was inside an obstacle, moved " + fmt(w) + " -> " + fmt(freed));
                    pts.add(freed);
                } else {
                    pts.add(w);
                }
            }

            // 2) route each consecutive pair around obstacles into one long polyline of points
            List<double[]> route = new ArrayList<>();
            route.add(pts.get(0));
            for (int i = 0; i < pts.size() - 1; i++) {
                double[] a = pts.get(i), b = pts.get(i + 1);
                List<double[]> leg;
                if (field.isSegmentClear(a[0], a[1], b[0], b[1])) {
                    leg = new ArrayList<>(); leg.add(a); leg.add(b);         // straight shot is fine
                } else {
                    leg = routeAround(a, b);                                 // go around via nav-points
                    if (leg == null)
                        return PathResult.failed("no clear route from waypoint " + i + " to " + (i + 1));
                    notes.add("routed around an obstacle between waypoint " + i + " and " + (i + 1));
                }
                for (int k = 1; k < leg.size(); k++) route.add(leg.get(k));  // skip the duplicated start
            }

            // 3) tangent headings so the FRONT leads through every point (needed for intake)
            List<Pose> poses = withTangentHeadings(route);

            // 4) build the Pedro PathChain: smooth curve where it stays clear, straight where it wouldn't
            PathChain chain = buildChain(follower, poses);

            return notes.isEmpty() ? PathResult.ok(chain) : PathResult.adjusted(chain, notes);

        } catch (Exception e) {
            return PathResult.failed("planner error: " + e.getMessage());   // never crash auto
        }
    }

    // ---- shortest route around obstacles: visibility graph + Dijkstra ----
    private List<double[]> routeAround(double[] start, double[] goal) {
        List<double[]> nodes = new ArrayList<>();
        nodes.add(start);                    // index 0
        nodes.add(goal);                     // index 1
        nodes.addAll(field.navPoints());     // 2..n-1 (obstacle corners, already clearance-inflated)
        int n = nodes.size();

        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        for (int i = 0; i < n; i++) { dist[i] = Double.MAX_VALUE; prev[i] = -1; }
        dist[0] = 0;

        for (int it = 0; it < n; it++) {
            int u = -1; double best = Double.MAX_VALUE;
            for (int i = 0; i < n; i++) if (!done[i] && dist[i] < best) { best = dist[i]; u = i; }
            if (u == -1) break;
            done[u] = true;
            if (u == 1) break;               // reached the goal
            double[] a = nodes.get(u);
            for (int v = 0; v < n; v++) {
                if (done[v]) continue;
                double[] b = nodes.get(v);
                if (!field.isSegmentClear(a[0], a[1], b[0], b[1])) continue;   // can't connect through an obstacle
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]) / Vx; // travel time (forward)
                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
            }
        }
        if (prev[1] == -1) return null;      // goal unreachable
        List<double[]> path = new ArrayList<>();
        for (int at = 1; at != -1; at = prev[at]) path.add(0, nodes.get(at));
        return path;
    }

    // ---- move a blocked point out to the nearest free spot (spiral search) ----
    private double[] nudgeOut(double x, double y) {
        for (double rad = 2; rad <= 48; rad += 2) {
            for (int a = 0; a < 16; a++) {
                double ang = 2 * Math.PI * a / 16;
                double nx = x + rad * Math.cos(ang), ny = y + rad * Math.sin(ang);
                if (!field.blocked(nx, ny)) return new double[]{ nx, ny };
            }
        }
        return null;   // couldn't free it within 48"
    }

    // ---- each point faces the next point, so the front leads (drives forward through the balls) ----
    private List<Pose> withTangentHeadings(List<double[]> route) {
        List<Pose> poses = new ArrayList<>();
        for (int i = 0; i < route.size(); i++) {
            double[] p = route.get(i);
            double heading;
            if (i < route.size() - 1) {
                double[] nxt = route.get(i + 1);
                heading = Math.atan2(nxt[1] - p[1], nxt[0] - p[0]);   // face where we're going
            } else {
                heading = poses.get(i - 1).getHeading();              // last point keeps arrival heading
            }
            poses.add(new Pose(p[0], p[1], heading));
        }
        return poses;
    }

    // ---- build the chain: cubic bezier where the smoothed curve stays clear, straight otherwise ----
    private PathChain buildChain(Follower follower, List<Pose> poses) {
        var builder = follower.pathBuilder();
        int n = poses.size();
        for (int i = 0; i < n - 1; i++) {
            Pose p1 = poses.get(i), p2 = poses.get(i + 1);
            Pose p0 = poses.get(Math.max(i - 1, 0));
            Pose p3 = poses.get(Math.min(i + 2, n - 1));
            double k = catmullTension;
            double c1x = p1.getX() + (p2.getX() - p0.getX()) / k, c1y = p1.getY() + (p2.getY() - p0.getY()) / k;
            double c2x = p2.getX() - (p3.getX() - p1.getX()) / k, c2y = p2.getY() - (p3.getY() - p1.getY()) / k;

            if (curveIsClear(p1.getX(), p1.getY(), c1x, c1y, c2x, c2y, p2.getX(), p2.getY())) {
                builder.addPath(new BezierCurve(p1, new Pose(c1x, c1y), new Pose(c2x, c2y), p2));  // smooth
            } else {
                builder.addPath(new BezierLine(p1, p2));   // smoothing would clip -> safe straight segment
            }
            builder.setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading());
        }
        return builder.build();
    }

    // sample a cubic bezier and confirm none of it is blocked
    private boolean curveIsClear(double x0, double y0, double x1, double y1,
                                 double x2, double y2, double x3, double y3) {
        int samples = 20;
        for (int i = 0; i <= samples; i++) {
            double t = i / (double) samples, u = 1 - t;
            double x = u*u*u*x0 + 3*u*u*t*x1 + 3*u*t*t*x2 + t*t*t*x3;
            double y = u*u*u*y0 + 3*u*u*t*y1 + 3*u*t*t*y2 + t*t*t*y3;
            if (field.blocked(x, y)) return false;
        }
        return true;
    }

    private static String fmt(double[] p) { return String.format("(%.1f, %.1f)", p[0], p[1]); }
}
