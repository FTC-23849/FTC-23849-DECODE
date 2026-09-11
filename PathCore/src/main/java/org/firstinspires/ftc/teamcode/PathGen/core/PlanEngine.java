package org.firstinspires.ftc.teamcode.PathGen.core;

import java.util.ArrayList;
import java.util.List;

/**
 * The PURE path-planning brain -- plain Java + Math only, NO Pedro/Android. The robot's PathPlanner
 * runs this then converts the result to a Pedro path; the desktop visualizer runs the same class.
 *
 *  - plan(waypoints): forward pass, the FRONT leads through every point (for a front intake).
 *  - planReturn(from, to, incomingHeading): a non-intake leg (e.g. returning to a fixed scoring
 *    spot) that automatically picks FORWARD or REVERSE -- whichever is faster -- so a straight
 *    there-and-back reverses (no 180 spin) while a long loop around an obstacle keeps driving
 *    forward. Endpoints that land inside an obstacle are nudged to the nearest clear spot; because
 *    clearance is a bounding circle, "clear" means clear at ANY heading.
 *  - never throws: always returns a GeoResult.
 */
public class PlanEngine {

    // ---- SINGLE SOURCE OF TRUTH for the path-shaping constants ----
    // Edit these four numbers and EVERYTHING that shapes a path changes together: this engine, the
    // robot's PathPlanner (it reads these), and the desktop visualizer (it reads these too). There is
    // no second copy to keep in sync. This is what makes "adjust the code, rerun, it updates" work.
    public static final double DEFAULT_VX             = 76;    // forward/reverse speed in/s (fast axis)
    public static final double DEFAULT_VY             = 52;    // strafe speed in/s (slow lateral axis)
    public static final double DEFAULT_CATMULL_TENSION = 6.0;  // path rounding: smaller = rounder curves
    public static final double DEFAULT_TURN_RATE       = 4.0;  // rad/s, rough turn rate for fwd-vs-reverse

    public double Vx = DEFAULT_VX;              // forward/reverse speed in/s (fast longitudinal axis)
    public double Vy = DEFAULT_VY;             // strafe speed in/s (slow lateral axis)
    public double catmullTension = DEFAULT_CATMULL_TENSION;   // "path rounding constant"
    public double turnRate = DEFAULT_TURN_RATE;      // rad/s, rough in-place turn rate for the fwd-vs-reverse cost

    /** set by planReturn: true if the last return leg was chosen to be driven in reverse. */
    public boolean lastReturnReversed = false;

    private final Field field;

    public PlanEngine(Field field) { this.field = field; }

    /** Plan through waypoints (each {x, y}), start first and end last, all forward. Never throws. */
    public GeoResult plan(List<double[]> waypoints) {
        try {
            List<String> notes = new ArrayList<String>();
            if (waypoints == null || waypoints.size() < 2)
                return GeoResult.failed("need at least a start and an end point");

            List<double[]> pts = new ArrayList<double[]>();
            for (int i = 0; i < waypoints.size(); i++) {
                double[] w = waypoints.get(i);
                if (field.blocked(w[0], w[1])) {
                    double[] freed = nudgeOut(w[0], w[1]);
                    if (freed == null)
                        return GeoResult.failed("waypoint " + i + " " + fmt(w) + " is stuck inside an obstacle");
                    notes.add("waypoint " + i + " was inside an obstacle, moved " + fmt(w) + " -> " + fmt(freed));
                    pts.add(freed);
                } else {
                    pts.add(w);
                }
            }

            List<double[]> route = new ArrayList<double[]>();
            route.add(pts.get(0));
            for (int i = 0; i < pts.size() - 1; i++) {
                double[] a = pts.get(i), b = pts.get(i + 1);
                List<double[]> leg;
                if (field.isSegmentClear(a[0], a[1], b[0], b[1])) {
                    leg = new ArrayList<double[]>(); leg.add(a); leg.add(b);
                } else {
                    leg = routeAround(a, b);
                    if (leg == null)
                        return GeoResult.failed("no clear route from waypoint " + i + " to " + (i + 1));
                    notes.add("routed around an obstacle between waypoint " + i + " and " + (i + 1));
                }
                for (int k = 1; k < leg.size(); k++) route.add(leg.get(k));
            }

            double[] head = tangentHeadings(route);
            List<Seg> segs = buildSegs(route, head);
            return notes.isEmpty() ? GeoResult.ok(segs, route) : GeoResult.adjusted(segs, route, notes);
        } catch (Exception e) {
            return GeoResult.failed("planner error: " + e.getMessage());
        }
    }

    /**
     * Plan ONE non-intake leg from `from` to `to` that may be driven forward OR in reverse, choosing
     * whichever is faster. `incomingHeading` is the heading the robot already has (e.g. the direction
     * it faced arriving at the last ball) -- reversing holds that heading and skips the turn.
     */
    public GeoResult planReturn(double[] from, double[] to, double incomingHeading) {
        try {
            lastReturnReversed = false;
            if (from == null || to == null) return GeoResult.failed("null endpoint on return leg");

            List<String> notes = new ArrayList<String>();
            // nudge endpoints out of obstacles (works for all headings -- clearance is a circle)
            if (field.blocked(from[0], from[1])) {
                double[] f = nudgeOut(from[0], from[1]);
                if (f == null) return GeoResult.failed("return start stuck inside an obstacle");
                notes.add("return start moved out of an obstacle " + fmt(from) + " -> " + fmt(f));
                from = f;
            }
            if (field.blocked(to[0], to[1])) {
                double[] t = nudgeOut(to[0], to[1]);
                if (t == null) return GeoResult.failed("return target stuck inside an obstacle");
                notes.add("return target moved out of an obstacle " + fmt(to) + " -> " + fmt(t));
                to = t;
            }

            List<double[]> route;
            if (field.isSegmentClear(from[0], from[1], to[0], to[1])) {
                route = new ArrayList<double[]>(); route.add(from); route.add(to);
            } else {
                route = routeAround(from, to);
                if (route == null) return GeoResult.failed("no clear route on the return leg");
                notes.add("return routed around an obstacle");
            }

            // --- cost of FORWARD: turn to face the path, then drive the fast axis the whole way ---
            double firstTangent = Math.atan2(route.get(1)[1] - route.get(0)[1],
                                             route.get(1)[0] - route.get(0)[0]);
            double turn = Math.abs(wrap(firstTangent - incomingHeading));
            double len = polylineLength(route);
            double fwdCost = turn / turnRate + len / Vx;

            // --- cost of REVERSE: hold incomingHeading; speed drops where travel isn't along that axis ---
            double revCost = 0;
            for (int i = 0; i < route.size() - 1; i++) {
                double dx = route.get(i + 1)[0] - route.get(i)[0];
                double dy = route.get(i + 1)[1] - route.get(i)[1];
                double seg = Math.hypot(dx, dy);
                double theta = axisAngle(Math.atan2(dy, dx) - incomingHeading); // 0=along axis, pi/2=strafe
                revCost += seg / effSpeed(theta);
            }

            boolean reverse = revCost < fwdCost;
            lastReturnReversed = reverse;
            notes.add(reverse
                    ? "return REVERSED (est " + t1(revCost) + "s vs forward " + t1(fwdCost) + "s)"
                    : "return FORWARD (est " + t1(fwdCost) + "s vs reverse " + t1(revCost) + "s)");

            double[] head = new double[route.size()];
            if (reverse) {
                for (int i = 0; i < route.size(); i++) head[i] = incomingHeading;   // hold heading -> reverse
            } else {
                head = tangentHeadings(route);                                        // face travel -> forward
            }

            List<Seg> segs = buildSegs(route, head);
            return GeoResult.adjusted(segs, route, notes);   // note always explains the choice
        } catch (Exception e) {
            return GeoResult.failed("return planner error: " + e.getMessage());
        }
    }

    /**
     * Plan ONE continuous path through all waypoints (each {x,y}), start first and END last. Every
     * point except the last is treated as an INTAKE point (front leads through it, forward). The
     * LAST leg (into the end point) auto-picks forward vs reverse -- whichever is faster -- just like
     * planReturn, but it's all one path so the robot flows through the balls without stopping.
     * lastReturnReversed is set to the choice. Never throws.
     */
    public GeoResult planWithReturn(List<double[]> waypoints) {
        try {
            lastReturnReversed = false;
            List<String> notes = new ArrayList<String>();
            if (waypoints == null || waypoints.size() < 2)
                return GeoResult.failed("need at least a start and an end point");

            // ---- build STOPS. A normal point is one stop. A ball sitting in the wall strip becomes
            // an intake approach of three stops: staging (outside strip) -> intake center (front on the
            // ball, forced heading) -> staging (back out). Dip legs (into/out of the strip) are marked
            // so they route as a straight obstacle-only line instead of avoiding the wall. ----
            List<double[]> sp = new ArrayList<double[]>();
            List<Double> sh = new ArrayList<Double>();      // forced heading, NaN = tangent
            List<Boolean> sd = new ArrayList<Boolean>();    // dip leg (straight, obstacle-only) into this stop
            int last = waypoints.size() - 1;
            for (int i = 0; i <= last; i++) {
                double[] w = waypoints.get(i);
                double[] prev = sp.isEmpty() ? w : sp.get(sp.size() - 1);
                if (i > 0 && i < last && field.inWallStrip(w[0], w[1])) {
                    double[] ia = chooseIntake(w, prev);   // {stX,stY,cX,cY,heading,strat}
                    if (ia != null) {
                        sp.add(new double[]{ ia[0], ia[1] }); sh.add(Double.NaN); sd.add(false); // staging
                        sp.add(new double[]{ ia[2], ia[3] }); sh.add(ia[4]);      sd.add(true);  // dip in to center
                        sp.add(new double[]{ ia[0], ia[1] }); sh.add(Double.NaN); sd.add(true);  // dip back out
                        notes.add("wall ball " + fmt(w) + (ia[5] == 0 ? " -> head-on intake" : " -> wall-ride intake"));
                        continue;
                    }
                }
                double[] p = w;
                if (field.blocked(w[0], w[1])) {
                    double[] freed = nudgeOut(w[0], w[1]);
                    if (freed == null) return GeoResult.failed("waypoint " + i + " " + fmt(w) + " is stuck");
                    if (i != 0) notes.add("waypoint " + i + " moved " + fmt(w) + " -> " + fmt(freed));
                    p = freed;
                }
                sp.add(new double[]{ p[0], p[1] }); sh.add(Double.NaN); sd.add(false);
            }

            // ---- route between stops into one flat polyline; note where the final leg (to END) begins ----
            List<double[]> route = new ArrayList<double[]>();
            List<Double> hf = new ArrayList<Double>();   // parallel forced heading, NaN = tangent
            route.add(sp.get(0)); hf.add(sh.get(0));
            int finalLegStart = 0;
            for (int j = 1; j < sp.size(); j++) {
                if (j == sp.size() - 1) finalLegStart = route.size() - 1;
                double[] a = route.get(route.size() - 1), b = sp.get(j);
                if (sd.get(j) || field.isSegmentClear(a[0], a[1], b[0], b[1])) {
                    route.add(b); hf.add(sh.get(j));
                } else {
                    List<double[]> leg = routeAround(a, b);
                    if (leg == null) return GeoResult.failed("no clear route to stop " + j);
                    notes.add("routed around an obstacle");
                    for (int k = 1; k < leg.size(); k++) {
                        route.add(leg.get(k));
                        hf.add(k == leg.size() - 1 ? sh.get(j) : Double.NaN);
                    }
                }
            }

            int n = route.size();
            double[] head = tangentHeadings(route);
            for (int i = 0; i < n; i++) if (!Double.isNaN(hf.get(i))) head[i] = hf.get(i);

            // ---- reverse-return on the final leg (only when it's a straight, un-routed shot home) ----
            if (finalLegStart + 1 < n) {
                double hHold = (finalLegStart > 0)
                        ? Math.atan2(route.get(finalLegStart)[1] - route.get(finalLegStart - 1)[1],
                                     route.get(finalLegStart)[0] - route.get(finalLegStart - 1)[0])
                        : head[0];
                double firstTangent = Math.atan2(route.get(finalLegStart + 1)[1] - route.get(finalLegStart)[1],
                                                 route.get(finalLegStart + 1)[0] - route.get(finalLegStart)[0]);
                double turn = Math.abs(wrap(firstTangent - hHold));
                double len = 0, revCost = 0;
                for (int i = finalLegStart; i < n - 1; i++) {
                    double dx = route.get(i + 1)[0] - route.get(i)[0], dy = route.get(i + 1)[1] - route.get(i)[1];
                    double s = Math.hypot(dx, dy); len += s;
                    revCost += s / effSpeed(axisAngle(Math.atan2(dy, dx) - hHold));
                }
                double fwdCost = turn / turnRate + len / Vx;
                boolean directReturn = (n - 1 - finalLegStart) == 1;
                boolean reverse = directReturn && revCost < fwdCost;
                lastReturnReversed = reverse;
                notes.add(reverse ? "return REVERSED (est " + t1(revCost) + "s vs fwd " + t1(fwdCost) + "s)"
                                  : "return FORWARD");
                if (reverse) for (int i = finalLegStart; i < n; i++)
                    if (Double.isNaN(hf.get(i))) head[i] = hHold;
            }

            List<Seg> segs = buildSegs(route, head);
            return notes.isEmpty() ? GeoResult.ok(segs, route) : GeoResult.adjusted(segs, route, notes);
        } catch (Exception e) {
            return GeoResult.failed("planWithReturn error: " + e.getMessage());
        }
    }

    /** For a ball in the wall strip, pick head-on vs wall-ride (whichever is cheaper from `from`) and
     *  return {stageX, stageY, centerX, centerY, heading, strat(0=head-on,1=wall-ride)}, or null if
     *  neither works (caller then nudges the ball inward instead). */
    private double[] chooseIntake(double[] ball, double[] from) {
        double S = field.getSize(), R = field.robotRadius, W = field.robotWidth, L = field.robotLength;
        if (S <= 0 || R <= 0 || W <= 0 || L <= 0) return null;
        double bx = ball[0], by = ball[1];
        double dL = bx, dR = S - bx, dB = by, dT = S - by;
        double m = Math.min(Math.min(dL, dR), Math.min(dB, dT));
        double nx, ny, ax, ay;                       // inward normal, along-wall axis
        if (m == dB) { nx = 0; ny = 1; ax = 1; ay = 0; }
        else if (m == dT) { nx = 0; ny = -1; ax = 1; ay = 0; }
        else if (m == dL) { nx = 1; ny = 0; ax = 0; ay = 1; }
        else { nx = -1; ny = 0; ax = 0; ay = 1; }
        double stageOff = R + 3;
        // head-on: face the wall, front on the ball (center L/2 in from the ball)
        double hoCx = bx + nx * (L / 2), hoCy = by + ny * (L / 2);
        double hoHd = Math.atan2(-ny, -nx);
        double hoSx = bx + nx * stageOff, hoSy = by + ny * stageOff;
        // wall-ride: a side against the wall, front driving along the wall into the ball
        double along = (from[0] - bx) * ax + (from[1] - by) * ay;
        double sgn = along >= 0 ? 1 : -1;
        double wrCx = bx + nx * (W / 2) - ax * sgn * (L / 2);
        double wrCy = by + ny * (W / 2) - ay * sgn * (L / 2);
        double wrHd = Math.atan2(ay * sgn, ax * sgn);
        double wrSx = wrCx + nx * stageOff, wrSy = wrCy + ny * stageOff;
        boolean hoOk = intakeValid(hoSx, hoSy, hoCx, hoCy);
        boolean wrOk = intakeValid(wrSx, wrSy, wrCx, wrCy);
        double hoCost = Math.hypot(from[0] - hoSx, from[1] - hoSy) + 2 * Math.hypot(hoSx - hoCx, hoSy - hoCy);
        double wrCost = Math.hypot(from[0] - wrSx, from[1] - wrSy) + 2 * Math.hypot(wrSx - wrCx, wrSy - wrCy);
        boolean useHo;
        if (hoOk && wrOk) useHo = hoCost <= wrCost;
        else if (hoOk) useHo = true;
        else if (wrOk) useHo = false;
        else return null;
        return useHo ? new double[]{ hoSx, hoSy, hoCx, hoCy, hoHd, 0 }
                     : new double[]{ wrSx, wrSy, wrCx, wrCy, wrHd, 1 };
    }

    private boolean intakeValid(double sx, double sy, double cx, double cy) {
        return !field.blocked(sx, sy)
            && field.segmentClear(sx, sy, cx, cy, false)
            && cx >= 0 && cx <= field.getSize() && cy >= 0 && cy <= field.getSize()
            && !field.blockedByObstacle(cx, cy);
    }

    // ---- shortest route around obstacles: visibility graph + Dijkstra ----
    private List<double[]> routeAround(double[] start, double[] goal) {
        List<double[]> nodes = new ArrayList<double[]>();
        nodes.add(start);
        nodes.add(goal);
        nodes.addAll(field.navPoints());

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
            if (u == 1) break;
            double[] a = nodes.get(u);
            for (int v = 0; v < n; v++) {
                if (done[v]) continue;
                double[] b = nodes.get(v);
                if (!field.isSegmentClear(a[0], a[1], b[0], b[1])) continue;
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]) / Vx;
                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
            }
        }
        if (prev[1] == -1) return null;
        List<double[]> path = new ArrayList<double[]>();
        for (int at = 1; at != -1; at = prev[at]) path.add(0, nodes.get(at));
        return path;
    }

    private double[] nudgeOut(double x, double y) {
        for (double rad = 2; rad <= 48; rad += 2) {
            for (int a = 0; a < 16; a++) {
                double ang = 2 * Math.PI * a / 16;
                double nx = x + rad * Math.cos(ang), ny = y + rad * Math.sin(ang);
                if (!field.blocked(nx, ny)) return new double[]{ nx, ny };
            }
        }
        return null;
    }

    /** each point faces the next; last keeps the arrival heading (front leads = forward driving). */
    private double[] tangentHeadings(List<double[]> route) {
        int n = route.size();
        double[] h = new double[n];
        for (int i = 0; i < n; i++) {
            if (i < n - 1) {
                double[] p = route.get(i), nx = route.get(i + 1);
                h[i] = Math.atan2(nx[1] - p[1], nx[0] - p[0]);
            } else {
                h[i] = h[i - 1];
            }
        }
        return h;
    }

    /** build smoothed segments (cubic where it stays clear, straight otherwise) with given headings. */
    private List<Seg> buildSegs(List<double[]> route, double[] head) {
        List<Seg> segs = new ArrayList<Seg>();
        int n = route.size();
        for (int i = 0; i < n - 1; i++) {
            double[] p1 = route.get(i), p2 = route.get(i + 1);
            double[] p0 = route.get(Math.max(i - 1, 0));
            double[] p3 = route.get(Math.min(i + 2, n - 1));
            double[] cp = clearControlPoints(p0, p1, p2, p3);
            if (cp != null)
                segs.add(Seg.curved(p1[0], p1[1], head[i], p2[0], p2[1], head[i + 1],
                                    cp[0], cp[1], cp[2], cp[3]));
            else
                segs.add(Seg.straight(p1[0], p1[1], head[i], p2[0], p2[1], head[i + 1]));
        }
        return segs;
    }

    private double[] clearControlPoints(double[] p0, double[] p1, double[] p2, double[] p3) {
        double[] tensions = { catmullTension, 10, 18, 30, 60 };
        for (double k : tensions) {
            double c1x = p1[0] + (p2[0] - p0[0]) / k, c1y = p1[1] + (p2[1] - p0[1]) / k;
            double c2x = p2[0] - (p3[0] - p1[0]) / k, c2y = p2[1] - (p3[1] - p1[1]) / k;
            if (curveIsClear(p1[0], p1[1], c1x, c1y, c2x, c2y, p2[0], p2[1]))
                return new double[]{ c1x, c1y, c2x, c2y };
        }
        return null;
    }

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

    // ---- small math helpers ----

    /** elliptical speed limit: full Vx along the robot's axis, down to Vy at 90deg (strafe). */
    private double effSpeed(double theta) {
        double c = Math.cos(theta) / Vx, s = Math.sin(theta) / Vy;
        return 1.0 / Math.sqrt(c * c + s * s);
    }

    /** reduce an angle to [0, pi/2]: forward and reverse along the axis are equally fast. */
    private static double axisAngle(double a) {
        double t = Math.abs(wrap(a));
        if (t > Math.PI / 2) t = Math.PI - t;
        return t;
    }

    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static double polylineLength(List<double[]> r) {
        double s = 0;
        for (int i = 0; i < r.size() - 1; i++)
            s += Math.hypot(r.get(i + 1)[0] - r.get(i)[0], r.get(i + 1)[1] - r.get(i)[1]);
        return s;
    }

    private static String fmt(double[] p) { return String.format("(%.1f, %.1f)", p[0], p[1]); }
    private static String t1(double s) { return String.format("%.1f", s); }
}
