package pathviz;

import org.firstinspires.ftc.teamcode.PathGen.core.CircleObstacle;
import org.firstinspires.ftc.teamcode.PathGen.core.Field;
import org.firstinspires.ftc.teamcode.PathGen.core.GeoResult;
import org.firstinspires.ftc.teamcode.PathGen.core.Obstacle;
import org.firstinspires.ftc.teamcode.PathGen.core.PlanEngine;
import org.firstinspires.ftc.teamcode.PathGen.core.RectObstacle;
import org.firstinspires.ftc.teamcode.PathGen.core.Seg;
import org.firstinspires.ftc.teamcode.PathGen.core.TriObstacle;

import javax.swing.*;
import javax.swing.Timer;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Pathgen Field Lab (desktop). Runs the REAL planner: it builds a real
 * org...PathGen.core.Field, adds real obstacles, and calls the real PlanEngine -- the same
 * classes the robot compiles. Whatever changes you make to that planner code show up here on the
 * next build, because there is no separate copy of the logic.
 *
 * Run:  ./gradlew :PathViz:run      (or the green Run arrow on this file in Android Studio)
 */
public class PathVizApp {

    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            public void run() { buildAndShow(); }
        });
    }

    private static void buildAndShow() {
        JFrame f = new JFrame("Pathgen Field Lab  -  FTC 23849  (runs the real PlanEngine)");
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        FieldPanel panel = new FieldPanel();
        f.setLayout(new BorderLayout());
        f.add(panel, BorderLayout.CENTER);
        f.add(panel.buildSidebar(), BorderLayout.EAST);
        f.add(panel.buildStatusBar(), BorderLayout.SOUTH);
        f.setSize(1120, 820);
        f.setLocationRelativeTo(null);
        f.setVisible(true);
    }

    // ================= editable obstacle shapes (UI-side; each builds a REAL Obstacle) =============

    interface Shape {
        Obstacle toObstacle();
        double[][] handles();                 // draggable points, in inches
        void setHandle(int i, double x, double y);
        void translate(double dx, double dy);
        void draw(Graphics2D g, FieldPanel tf, boolean inflate, double clr);
        char kind();
    }

    static class CircleShape implements Shape {
        double cx, cy, r;
        CircleShape(double cx, double cy, double r) { this.cx = cx; this.cy = cy; this.r = r; }
        public Obstacle toObstacle() { return new CircleObstacle(cx, cy, r); }
        public double[][] handles() { return new double[][]{ {cx, cy}, {cx + r, cy} }; }
        public void setHandle(int i, double x, double y) {
            if (i == 0) { cx = x; cy = y; }
            else { r = Math.max(2, Math.hypot(x - cx, y - cy)); }
        }
        public void translate(double dx, double dy) { cx += dx; cy += dy; }
        public char kind() { return 'C'; }
        public void draw(Graphics2D g, FieldPanel p, boolean inflate, double clr) {
            p.fillCircleIn(g, cx, cy, r, FieldPanel.OBST_FILL);
            p.strokeCircleIn(g, cx, cy, r, FieldPanel.OBST_EDGE, 1.5f, false);
            if (inflate) p.strokeCircleIn(g, cx, cy, r + clr, FieldPanel.INFLATE, 1f, true);
        }
    }

    static class RectShape implements Shape {
        double x0, y0, x1, y1;   // two opposite corners
        RectShape(double x0, double y0, double x1, double y1) { this.x0 = x0; this.y0 = y0; this.x1 = x1; this.y1 = y1; }
        double lx() { return Math.min(x0, x1); }
        double ly() { return Math.min(y0, y1); }
        double ux() { return Math.max(x0, x1); }
        double uy() { return Math.max(y0, y1); }
        public Obstacle toObstacle() { return new RectObstacle(lx(), ly(), ux(), uy()); }
        public double[][] handles() { return new double[][]{ {x0, y0}, {x1, y1} }; }
        public void setHandle(int i, double x, double y) {
            if (i == 0) { x0 = x; y0 = y; } else { x1 = x; y1 = y; }
        }
        public void translate(double dx, double dy) { x0 += dx; y0 += dy; x1 += dx; y1 += dy; }
        public char kind() { return 'R'; }
        public void draw(Graphics2D g, FieldPanel p, boolean inflate, double clr) {
            p.fillRectIn(g, lx(), ly(), ux(), uy(), FieldPanel.OBST_FILL);
            p.strokeRectIn(g, lx(), ly(), ux(), uy(), FieldPanel.OBST_EDGE, 1.5f, false);
            if (inflate) p.strokeRectIn(g, lx() - clr, ly() - clr, ux() + clr, uy() + clr, FieldPanel.INFLATE, 1f, true);
        }
    }

    static class TriShape implements Shape {
        double ax, ay, bx, by, cx, cy;
        TriShape(double ax, double ay, double bx, double by, double cx, double cy) {
            this.ax = ax; this.ay = ay; this.bx = bx; this.by = by; this.cx = cx; this.cy = cy;
        }
        public Obstacle toObstacle() { return new TriObstacle(ax, ay, bx, by, cx, cy); }
        public double[][] handles() { return new double[][]{ {ax, ay}, {bx, by}, {cx, cy} }; }
        public void setHandle(int i, double x, double y) {
            if (i == 0) { ax = x; ay = y; } else if (i == 1) { bx = x; by = y; } else { cx = x; cy = y; }
        }
        public void translate(double dx, double dy) { ax += dx; ay += dy; bx += dx; by += dy; cx += dx; cy += dy; }
        public char kind() { return 'T'; }
        public void draw(Graphics2D g, FieldPanel p, boolean inflate, double clr) {
            p.fillTriIn(g, ax, ay, bx, by, cx, cy, FieldPanel.OBST_FILL);
            p.strokeTriIn(g, ax, ay, bx, by, cx, cy, FieldPanel.OBST_EDGE, 1.5f, false);
            if (inflate) {
                double gx = (ax + bx + cx) / 3, gy = (ay + by + cy) / 3;
                double[] A = push(ax, ay, gx, gy, clr), B = push(bx, by, gx, gy, clr), C = push(cx, cy, gx, gy, clr);
                p.strokeTriIn(g, A[0], A[1], B[0], B[1], C[0], C[1], FieldPanel.INFLATE, 1f, true);
            }
        }
        private static double[] push(double px, double py, double gx, double gy, double d) {
            double dx = px - gx, dy = py - gy, len = Math.hypot(dx, dy);
            if (len < 1e-9) return new double[]{ px, py };
            return new double[]{ px + dx / len * d, py + dy / len * d };
        }
    }

    // ============================== the field canvas + all state ==================================

    static class FieldPanel extends JPanel {
        static final double FIELD = 144.0;

        // palette
        static final Color BG        = new Color(0x14, 0x17, 0x1c);
        static final Color FIELD_BG  = new Color(0x1b, 0x20, 0x27);
        static final Color GRID      = new Color(0x2b, 0x33, 0x3e);
        static final Color OBST_FILL = new Color(0xE0, 0x53, 0x53, 60);
        static final Color OBST_EDGE = new Color(0xE0, 0x53, 0x53);
        static final Color INFLATE   = new Color(0xE8, 0x8A, 0x3A);
        static final Color NAV       = new Color(0xF2, 0xC3, 0x4E);
        static final Color GRAPH     = new Color(0x3a, 0x45, 0x54);
        static final Color ROUTE     = new Color(0x8a, 0x93, 0xa0);
        static final Color CURVE     = new Color(0x3D, 0xD6, 0x8C);
        static final Color STRAIGHT  = new Color(0xE8, 0x8A, 0x3A);
        static final Color WP_MID    = new Color(0x4E, 0x9B, 0xF2);
        static final Color WP_START  = new Color(0x3D, 0xD6, 0x8C);
        static final Color WP_END    = new Color(0xE0, 0x53, 0x53);
        static final Color ROBOT     = new Color(0x4E, 0x9B, 0xF2, 90);
        static final Color WALL      = new Color(0xE0, 0x53, 0x53, 26);   // wall keep-out band

        // model -- robot size & clearance get loaded from whichever field you pick.
        double robotW = 15.5, robotL = 17.5, minClear = 3;
        // PATH CONSTANTS, seeded straight from PlanEngine's single-source defaults (so editing the code
        // and rerunning updates them). All four are also adjustable live in the sidebar.
        double Vx = PlanEngine.DEFAULT_VX;
        double Vy = PlanEngine.DEFAULT_VY;
        double catmullTension = PlanEngine.DEFAULT_CATMULL_TENSION;   // the "path rounding" constant
        double turnRate = PlanEngine.DEFAULT_TURN_RATE;
        String planMode = "withReturn";   // withReturn = exactly what the robot runs | forward = simple pass
        boolean lastReversed = false;

        static final double BALL_SPACING = 20;                 // matches RandomPointAuto's min ball spacing
        final java.util.Random rng = new java.util.Random();   // for the "new random path" button
        boolean loop = false;                                  // replay the path on a loop

        final List<Shape> shapes = new ArrayList<>();
        final List<double[]> waypoints = new ArrayList<>();

        // fields discovered in the repo source
        List<FieldScanner.FieldSpec> fields = new ArrayList<>();
        FieldScanner.FieldSpec currentField;
        JComboBox<FieldScanner.FieldSpec> fieldCombo;
        final Map<String, JTextField> boxes = new HashMap<>();   // sidebar inputs, for syncing on load

        // overlay toggles
        boolean showRoute = false, showInflate = true, showNav = false, showGraph = false, showWall = true;

        // editing mode
        String mode = "select";   // select | waypoint | circle | rect | tri | delete

        // drag state
        int dragType = -1, dragObj = -1, dragHandle = -1;   // dragType 0=shape,1=waypoint

        // cached plan
        Field lastField;
        GeoResult lastResult;
        List<double[]> trace = new ArrayList<>();   // {x,y,heading} samples
        double[] cumDist = new double[0];
        double totalDist = 0;

        // playback
        Timer timer;
        double playDist = 0;
        boolean playing = false;

        JTextArea status;

        FieldPanel() {
            setBackground(BG);
            refreshFields();
            MouseAdapter ma = new MouseAdapter() {
                public void mousePressed(MouseEvent e) { onPress(e); }
                public void mouseDragged(MouseEvent e) { onDrag(e); }
                public void mouseReleased(MouseEvent e) { dragType = -1; }
            };
            addMouseListener(ma);
            addMouseMotionListener(ma);
            timer = new Timer(33, new ActionListener() {
                public void actionPerformed(ActionEvent e) { tick(); }
            });
            recompute();
        }

        /** Scan the repo for fields and load the best match (or a built-in fallback). */
        void refreshFields() {
            fields = FieldScanner.scan();
            if (fields.isEmpty()) fields.add(FieldScanner.builtinDecode());
            FieldScanner.FieldSpec pick = fields.get(0);
            for (FieldScanner.FieldSpec f : fields)               // prefer the auto's field if present
                if (f.source.toLowerCase().contains("randompoint")) { pick = f; break; }
            loadField(pick);
        }

        /** Replace the current obstacles/robot with a scanned field, then lay down a fresh random path
         *  (start in the corner -> 3 random balls -> END) exactly like RandomPointAuto does. No drawing. */
        void loadField(FieldScanner.FieldSpec fs) {
            currentField = fs;
            shapes.clear();
            for (double[] r : fs.rects)   shapes.add(new RectShape(r[0], r[1], r[2], r[3]));
            for (double[] t : fs.tris)    shapes.add(new TriShape(t[0], t[1], t[2], t[3], t[4], t[5]));
            for (double[] c : fs.circles) shapes.add(new CircleShape(c[0], c[1], c[2]));
            robotW = fs.robotW; robotL = fs.robotL; minClear = fs.minClear;
            // start = bottom-right corner (computed from robot size, like the auto); END from the code.
            double sx = round(FIELD - robotW / 2), sy = round(robotL / 2);
            double ex = fs.hasEnd ? fs.endX : 48, ey = fs.hasEnd ? fs.endY : 96;
            newRandomPath(new double[]{ sx, sy }, new double[]{ ex, ey });
        }

        /** Build a Field from the current obstacles/robot (same one recompute and the point generator use). */
        Field buildField() {
            Field f = new Field(robotW, robotL).minClearance(minClear);
            for (Shape s : shapes) f.add(s.toObstacle());
            return f;
        }

        /** Lay down start -> 3 random valid balls -> end, the SAME way RandomPointAuto generates a path. */
        void newRandomPath(double[] start, double[] end) {
            Field f = buildField();
            waypoints.clear();
            waypoints.add(start);
            double[] b1 = randomValidPoint(f, start, BALL_SPACING);
            double[] b2 = randomValidPoint(f, b1,    BALL_SPACING);
            double[] b3 = randomValidPoint(f, b2,    BALL_SPACING);
            waypoints.add(b1); waypoints.add(b2); waypoints.add(b3);
            waypoints.add(end);
        }

        /** New random path reusing the existing start & END (button action). */
        void newRandomPath() {
            double[] start = waypoints.size() >= 1 ? waypoints.get(0)
                    : new double[]{ round(FIELD - robotW / 2), round(robotL / 2) };
            double[] end   = waypoints.size() >= 2 ? waypoints.get(waypoints.size() - 1)
                    : new double[]{ 48, 96 };
            newRandomPath(start, end);
            recompute();
        }

        /** Add one more random intermediate ball just before END. */
        void addRandomBall() {
            Field f = buildField();
            double[] from = waypoints.size() >= 2 ? waypoints.get(waypoints.size() - 2)
                          : (waypoints.isEmpty() ? new double[]{ 72, 72 } : waypoints.get(0));
            double[] b = randomValidPoint(f, from, BALL_SPACING);
            int insertAt = Math.max(1, waypoints.size() - 1);   // before END
            waypoints.add(insertAt, b);
            recompute();
        }

        /** A random point that isn't inside a real obstacle (walls allowed), like RandomPointAuto. */
        double[] randomValidPoint(Field f, double[] from, double minAway) {
            double lo = 2, hi = 142;
            double[] best = { 72, 72 };
            for (int i = 0; i < 300; i++) {
                double x = lo + rng.nextDouble() * (hi - lo);
                double y = lo + rng.nextDouble() * (hi - lo);
                if (f.blockedByObstacle(x, y)) continue;   // reject only real obstacles; walls are OK
                best = new double[]{ round(x), round(y) };
                if (dist(best, from) >= minAway) return best;
            }
            return best;
        }

        static double dist(double[] a, double[] b) { return Math.hypot(a[0] - b[0], a[1] - b[1]); }

        // ---------- transform (inches <-> pixels), field drawn as a centered square ----------
        double scale() { return (Math.min(getWidth(), getHeight()) - 48) / FIELD; }
        double side()  { return FIELD * scale(); }
        double ox()    { return (getWidth()  - side()) / 2.0; }
        double oy()    { return (getHeight() - side()) / 2.0; }
        double px(double x) { return ox() + x * scale(); }
        double py(double y) { return oy() + side() - y * scale(); }     // y up
        double ix(double px) { return (px - ox()) / scale(); }
        double iy(double py) { return (side() - (py - oy())) / scale(); }

        // ---------- planning ----------
        void recompute() {
            Field f = buildField();
            PlanEngine eng = new PlanEngine(f);
            eng.Vx = Vx; eng.Vy = Vy; eng.catmullTension = catmullTension; eng.turnRate = turnRate;
            lastField = f;
            List<double[]> wp = new ArrayList<>();
            for (double[] w : waypoints) wp.add(new double[]{ w[0], w[1] });
            // run the SAME engine call the robot runs, so what you see is what it drives
            lastResult = planMode.equals("withReturn") ? eng.planWithReturn(wp) : eng.plan(wp);
            lastReversed = eng.lastReturnReversed;
            buildTrace();
            playDist = 0;
            updateStatus();
            repaint();
        }

        void buildTrace() {
            trace = new ArrayList<>();
            if (lastResult == null || !lastResult.usable()) { cumDist = new double[0]; totalDist = 0; return; }
            for (Seg s : lastResult.segs) {
                int N = 36;
                for (int i = 0; i <= N; i++) {
                    double t = i / (double) N, u = 1 - t, x, y;
                    if (s.curve) {
                        x = u*u*u*s.x1 + 3*u*u*t*s.c1x + 3*u*t*t*s.c2x + t*t*t*s.x2;
                        y = u*u*u*s.y1 + 3*u*u*t*s.c1y + 3*u*t*t*s.c2y + t*t*t*s.y2;
                    } else {
                        x = s.x1 + (s.x2 - s.x1) * t;
                        y = s.y1 + (s.y2 - s.y1) * t;
                    }
                    double h = s.h1 + (s.h2 - s.h1) * t;
                    if (!trace.isEmpty()) {
                        double[] last = trace.get(trace.size() - 1);
                        if (Math.hypot(x - last[0], y - last[1]) < 1e-6) continue;
                    }
                    trace.add(new double[]{ x, y, h });
                }
            }
            cumDist = new double[trace.size()];
            totalDist = 0;
            for (int i = 1; i < trace.size(); i++) {
                totalDist += Math.hypot(trace.get(i)[0] - trace.get(i - 1)[0], trace.get(i)[1] - trace.get(i - 1)[1]);
                cumDist[i] = totalDist;
            }
        }

        double[] poseAt(double d) {
            if (trace.isEmpty()) return null;
            if (d <= 0) return trace.get(0);
            if (d >= totalDist) return trace.get(trace.size() - 1);
            int i = 1;
            while (i < cumDist.length && cumDist[i] < d) i++;
            double seg = cumDist[i] - cumDist[i - 1];
            double t = seg < 1e-9 ? 0 : (d - cumDist[i - 1]) / seg;
            double[] a = trace.get(i - 1), b = trace.get(i);
            return new double[]{ a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t };
        }

        void tick() {
            if (!playing || totalDist <= 0) return;
            playDist += Vx * 0.033;   // travel forward at Vx in/s
            if (playDist >= totalDist) {
                if (loop) { playDist = 0; }                       // start over
                else { playDist = totalDist; playing = false; timer.stop(); }
            }
            repaint();
        }

        // ---------- interaction ----------
        void onPress(MouseEvent e) {
            double x = ix(e.getX()), y = iy(e.getY());
            if (mode.equals("waypoint")) {
                // insert as an INTERMEDIATE point (before END), so clicks add balls, not new endpoints
                int at = waypoints.size() >= 2 ? waypoints.size() - 1 : waypoints.size();
                waypoints.add(at, new double[]{ round(x), round(y) });
                recompute(); return;
            }
            if (mode.equals("circle"))   { shapes.add(new CircleShape(x, y, 10)); recompute(); return; }
            if (mode.equals("rect"))     { shapes.add(new RectShape(x - 10, y - 8, x + 10, y + 8)); recompute(); return; }
            if (mode.equals("tri"))      { shapes.add(new TriShape(x - 12, y - 8, x + 12, y - 8, x, y + 12)); recompute(); return; }
            if (mode.equals("delete"))   { deleteNear(e.getX(), e.getY()); recompute(); return; }
            // select: grab nearest handle
            grabNear(e.getX(), e.getY());
        }

        void onDrag(MouseEvent e) {
            if (!mode.equals("select") || dragType < 0) return;
            double x = ix(e.getX()), y = iy(e.getY());
            if (dragType == 0) shapes.get(dragObj).setHandle(dragHandle, x, y);
            else waypoints.set(dragObj, new double[]{ x, y });
            recompute();
        }

        void grabNear(double sx, double sy) {
            double best = 12 * 12; dragType = -1;
            for (int i = 0; i < shapes.size(); i++) {
                double[][] hs = shapes.get(i).handles();
                for (int h = 0; h < hs.length; h++) {
                    double d = dist2(sx, sy, px(hs[h][0]), py(hs[h][1]));
                    if (d < best) { best = d; dragType = 0; dragObj = i; dragHandle = h; }
                }
            }
            for (int i = 0; i < waypoints.size(); i++) {
                double d = dist2(sx, sy, px(waypoints.get(i)[0]), py(waypoints.get(i)[1]));
                if (d < best) { best = d; dragType = 1; dragObj = i; dragHandle = 0; }
            }
        }

        void deleteNear(double sx, double sy) {
            double best = 14 * 14; int type = -1, obj = -1;
            for (int i = 0; i < shapes.size(); i++)
                for (double[] h : shapes.get(i).handles()) {
                    double d = dist2(sx, sy, px(h[0]), py(h[1]));
                    if (d < best) { best = d; type = 0; obj = i; }
                }
            for (int i = 0; i < waypoints.size(); i++) {
                double d = dist2(sx, sy, px(waypoints.get(i)[0]), py(waypoints.get(i)[1]));
                if (d < best) { best = d; type = 1; obj = i; }
            }
            if (type == 0) shapes.remove(obj);
            else if (type == 1) waypoints.remove(obj);
        }

        static double dist2(double ax, double ay, double bx, double by) {
            double dx = ax - bx, dy = ay - by; return dx * dx + dy * dy;
        }
        static double round(double v) { return Math.round(v * 10) / 10.0; }

        // ---------- rendering ----------
        protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // field + grid
            g.setColor(FIELD_BG);
            g.fillRect((int) ox(), (int) oy(), (int) side(), (int) side());
            g.setColor(GRID);
            for (int i = 0; i <= 6; i++) {           // 24" tiles
                double t = i * 24;
                g.drawLine((int) px(t), (int) py(0), (int) px(t), (int) py(FIELD));
                g.drawLine((int) px(0), (int) py(t), (int) px(FIELD), (int) py(t));
            }

            double clr = lastField != null ? lastField.clearance() : 0;

            // wall keep-out: the strip around the edge the robot CENTER can't reach (its body would be
            // in the wall). A ball here is grabbed with the head-on / wall-ride intake instead.
            if (showWall && lastField != null && lastField.robotRadius > 0) {
                double rr = lastField.robotRadius;
                fillRectIn(g, 0, 0, FIELD, rr, WALL);              // bottom
                fillRectIn(g, 0, FIELD - rr, FIELD, FIELD, WALL);  // top
                fillRectIn(g, 0, 0, rr, FIELD, WALL);              // left
                fillRectIn(g, FIELD - rr, 0, FIELD, FIELD, WALL);  // right
            }

            // obstacles
            for (Shape s : shapes) s.draw(g, this, showInflate, clr);

            // nav points
            if (showNav && lastField != null) {
                g.setColor(NAV);
                for (double[] p : lastField.navPoints())
                    g.fillOval((int) px(p[0]) - 3, (int) py(p[1]) - 3, 6, 6);
            }

            // visibility graph (drawn from the real Field.isSegmentClear)
            if (showGraph && lastField != null) {
                g.setColor(GRAPH);
                g.setStroke(new BasicStroke(0.6f));
                List<double[]> nodes = new ArrayList<>();
                for (double[] w : waypoints) nodes.add(w);
                nodes.addAll(lastField.navPoints());
                for (int i = 0; i < nodes.size(); i++)
                    for (int j = i + 1; j < nodes.size(); j++) {
                        double[] a = nodes.get(i), b = nodes.get(j);
                        if (lastField.isSegmentClear(a[0], a[1], b[0], b[1]))
                            g.drawLine((int) px(a[0]), (int) py(a[1]), (int) px(b[0]), (int) py(b[1]));
                    }
            }

            // routed polyline
            if (showRoute && lastResult != null && lastResult.usable() && !lastResult.route.isEmpty()) {
                g.setColor(ROUTE);
                g.setStroke(new BasicStroke(1.2f));
                List<double[]> r = lastResult.route;
                for (int i = 0; i < r.size() - 1; i++)
                    g.drawLine((int) px(r.get(i)[0]), (int) py(r.get(i)[1]),
                               (int) px(r.get(i + 1)[0]), (int) py(r.get(i + 1)[1]));
            }

            // smoothed path
            if (lastResult != null && lastResult.usable()) {
                g.setStroke(new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                for (Seg s : lastResult.segs) {
                    g.setColor(s.curve ? CURVE : STRAIGHT);
                    Path2D.Double path = new Path2D.Double();
                    path.moveTo(px(s.x1), py(s.y1));
                    if (s.curve)
                        path.curveTo(px(s.c1x), py(s.c1y), px(s.c2x), py(s.c2y), px(s.x2), py(s.y2));
                    else
                        path.lineTo(px(s.x2), py(s.y2));
                    g.draw(path);
                }
            }

            // waypoints
            for (int i = 0; i < waypoints.size(); i++) {
                double[] w = waypoints.get(i);
                Color c = i == 0 ? WP_START : (i == waypoints.size() - 1 ? WP_END : WP_MID);
                g.setColor(c);
                g.fillOval((int) px(w[0]) - 6, (int) py(w[1]) - 6, 12, 12);
                g.setColor(Color.WHITE);
                g.setFont(getFont().deriveFont(Font.BOLD, 11f));
                g.drawString(String.valueOf(i), (int) px(w[0]) - 3, (int) py(w[1]) - 9);
            }

            // playback robot
            double[] pose = poseAt(playDist);
            if (pose != null && lastResult != null && lastResult.usable()) drawRobot(g, pose[0], pose[1], pose[2]);

            // failure banner
            if (lastResult != null && !lastResult.usable()) {
                g.setColor(WP_END);
                g.setFont(getFont().deriveFont(Font.BOLD, 15f));
                g.drawString("PATH FAILED: " + lastResult.notes.get(0), (int) ox() + 12, (int) oy() + 24);
            }
        }

        void drawRobot(Graphics2D g, double cx, double cy, double h) {
            double halfW = robotW / 2, halfL = robotL / 2;
            double[][] corners = { {halfL, halfW}, {halfL, -halfW}, {-halfL, -halfW}, {-halfL, halfW} };
            Path2D.Double p = new Path2D.Double();
            for (int i = 0; i < 4; i++) {
                double rx = corners[i][0] * Math.cos(h) - corners[i][1] * Math.sin(h);
                double ry = corners[i][0] * Math.sin(h) + corners[i][1] * Math.cos(h);
                double X = px(cx + rx), Y = py(cy + ry);
                if (i == 0) p.moveTo(X, Y); else p.lineTo(X, Y);
            }
            p.closePath();
            g.setColor(ROBOT);
            g.fill(p);
            g.setColor(WP_MID);
            g.setStroke(new BasicStroke(2f));
            g.draw(p);
            // front heading tick
            double fx = cx + Math.cos(h) * halfL, fy = cy + Math.sin(h) * halfL;
            g.drawLine((int) px(cx), (int) py(cy), (int) px(fx), (int) py(fy));
        }

        // ---------- drawing helpers used by the shapes ----------
        void fillCircleIn(Graphics2D g, double cx, double cy, double r, Color c) {
            g.setColor(c);
            g.fillOval((int) px(cx - r), (int) py(cy + r), (int) (2 * r * scale()), (int) (2 * r * scale()));
        }
        void strokeCircleIn(Graphics2D g, double cx, double cy, double r, Color c, float w, boolean dash) {
            g.setColor(c); g.setStroke(stroke(w, dash));
            g.drawOval((int) px(cx - r), (int) py(cy + r), (int) (2 * r * scale()), (int) (2 * r * scale()));
        }
        void fillRectIn(Graphics2D g, double lx, double ly, double ux, double uy, Color c) {
            g.setColor(c);
            g.fillRect((int) px(lx), (int) py(uy), (int) ((ux - lx) * scale()), (int) ((uy - ly) * scale()));
        }
        void strokeRectIn(Graphics2D g, double lx, double ly, double ux, double uy, Color c, float w, boolean dash) {
            g.setColor(c); g.setStroke(stroke(w, dash));
            g.drawRect((int) px(lx), (int) py(uy), (int) ((ux - lx) * scale()), (int) ((uy - ly) * scale()));
        }
        void fillTriIn(Graphics2D g, double ax, double ay, double bx, double by, double cx, double cy, Color c) {
            g.setColor(c); g.fill(tri(ax, ay, bx, by, cx, cy));
        }
        void strokeTriIn(Graphics2D g, double ax, double ay, double bx, double by, double cx, double cy, Color c, float w, boolean dash) {
            g.setColor(c); g.setStroke(stroke(w, dash)); g.draw(tri(ax, ay, bx, by, cx, cy));
        }
        Path2D.Double tri(double ax, double ay, double bx, double by, double cx, double cy) {
            Path2D.Double p = new Path2D.Double();
            p.moveTo(px(ax), py(ay)); p.lineTo(px(bx), py(by)); p.lineTo(px(cx), py(cy)); p.closePath();
            return p;
        }
        Stroke stroke(float w, boolean dash) {
            if (dash) return new BasicStroke(w, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10f, new float[]{ 6f, 6f }, 0f);
            return new BasicStroke(w);
        }

        // ---------- sidebar / status ----------
        JComponent buildSidebar() {
            JPanel side = new JPanel();
            side.setLayout(new BoxLayout(side, BoxLayout.Y_AXIS));
            side.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

            // ---- field picker: loaded straight from the repo's source ----
            side.add(header("Field (loaded from your code)"));
            fieldCombo = new JComboBox<>(fields.toArray(new FieldScanner.FieldSpec[0]));
            fieldCombo.setSelectedItem(currentField);
            fieldCombo.setMaximumSize(new Dimension(252, 28));
            fieldCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
            fieldCombo.addActionListener(e -> {
                Object sel = fieldCombo.getSelectedItem();
                if (sel instanceof FieldScanner.FieldSpec) {
                    loadField((FieldScanner.FieldSpec) sel); syncInputs(); recompute();
                }
            });
            side.add(fieldCombo);
            side.add(Box.createVerticalStrut(4));
            side.add(leftButton("Rescan repo", e -> {
                refreshFields();
                fieldCombo.setModel(new DefaultComboBoxModel<>(fields.toArray(new FieldScanner.FieldSpec[0])));
                fieldCombo.setSelectedItem(currentField);
                syncInputs(); recompute();
            }));

            // ---- path actions ----
            side.add(Box.createVerticalStrut(10));
            side.add(header("Path"));
            JPanel pa = new JPanel(new GridLayout(1, 2, 6, 0));
            pa.setMaximumSize(new Dimension(252, 30));
            pa.setAlignmentX(Component.LEFT_ALIGNMENT);
            JButton newPath = new JButton("New random path");
            JButton addBall = new JButton("Add ball");
            newPath.setMargin(new Insets(2, 2, 2, 2));
            addBall.setMargin(new Insets(2, 2, 2, 2));
            newPath.addActionListener(e -> newRandomPath());
            addBall.addActionListener(e -> addRandomBall());
            pa.add(newPath); pa.add(addBall);
            side.add(pa);
            side.add(Box.createVerticalStrut(4));
            side.add(leftButton("Clear intermediate points", e -> {
                if (waypoints.size() > 2) {
                    double[] s = waypoints.get(0), en = waypoints.get(waypoints.size() - 1);
                    waypoints.clear(); waypoints.add(s); waypoints.add(en);
                }
                recompute();
            }));

            // ---- playback (drive the robot along the path) ----
            side.add(Box.createVerticalStrut(10));
            side.add(header("Playback (drive the robot)"));
            JPanel pb = new JPanel(new GridLayout(1, 3, 6, 0));
            pb.setMaximumSize(new Dimension(252, 30));
            pb.setAlignmentX(Component.LEFT_ALIGNMENT);
            JButton play = new JButton("Play"), pause = new JButton("Pause"), reset2 = new JButton("Reset");
            play.setMargin(new Insets(2, 2, 2, 2)); pause.setMargin(new Insets(2, 2, 2, 2)); reset2.setMargin(new Insets(2, 2, 2, 2));
            play.addActionListener(e -> { if (totalDist > 0) { if (playDist >= totalDist) playDist = 0; playing = true; timer.start(); } });
            pause.addActionListener(e -> { playing = false; timer.stop(); });
            reset2.addActionListener(e -> { playing = false; timer.stop(); playDist = 0; repaint(); });
            pb.add(play); pb.add(pause); pb.add(reset2);
            side.add(pb);
            side.add(checkRow("Loop", loop, b -> { loop = b; }));

            side.add(Box.createVerticalStrut(10));
            side.add(header("Planner mode"));
            side.add(planRow(new ButtonGroup()));

            side.add(Box.createVerticalStrut(10));
            side.add(header("Draw / edit  (click canvas)"));
            ButtonGroup bg = new ButtonGroup();
            side.add(modeRow(bg, new String[]{ "select", "waypoint", "delete" }));
            side.add(modeRow(bg, new String[]{ "circle", "rect", "tri" }));

            side.add(Box.createVerticalStrut(10));
            side.add(header("Robot & clearance (in)"));
            side.add(numRow("Width", robotW, v -> { robotW = v; recompute(); }));
            side.add(numRow("Length", robotL, v -> { robotL = v; recompute(); }));
            side.add(numRow("Min clearance", minClear, v -> { minClear = v; recompute(); }));

            side.add(Box.createVerticalStrut(10));
            side.add(header("Path constants (from PlanEngine)"));
            side.add(numRow("Vx (in/s)", Vx, v -> { Vx = v; recompute(); }));
            side.add(numRow("Vy (in/s)", Vy, v -> { Vy = v; recompute(); }));
            side.add(numRow("Path rounding", catmullTension, v -> { catmullTension = v; recompute(); }));
            side.add(numRow("Turn rate", turnRate, v -> { turnRate = v; recompute(); }));
            side.add(Box.createVerticalStrut(4));
            side.add(leftButton("Reset constants to code", e -> {
                Vx = PlanEngine.DEFAULT_VX; Vy = PlanEngine.DEFAULT_VY;
                catmullTension = PlanEngine.DEFAULT_CATMULL_TENSION; turnRate = PlanEngine.DEFAULT_TURN_RATE;
                syncInputs(); recompute();
            }));

            side.add(Box.createVerticalStrut(10));
            side.add(header("Overlays"));
            side.add(checkRow("Wall keep-out", showWall, b -> { showWall = b; repaint(); }));
            side.add(checkRow("Inflated keep-out", showInflate, b -> { showInflate = b; repaint(); }));
            side.add(checkRow("Routed polyline", showRoute, b -> { showRoute = b; repaint(); }));
            side.add(checkRow("Nav points", showNav, b -> { showNav = b; repaint(); }));
            side.add(checkRow("Visibility graph", showGraph, b -> { showGraph = b; repaint(); }));

            side.add(Box.createVerticalStrut(10));
            JLabel legend = new JLabel("<html><small>green=smooth curve&nbsp; orange=straight fallback<br>"
                    + "blue dot=start&nbsp; red dot=END&nbsp; middle dots=balls<br>"
                    + "waypoint mode inserts balls before END; drag dots in Select</small></html>");
            legend.setForeground(new Color(0x9a, 0xa3, 0xb0));
            legend.setAlignmentX(Component.LEFT_ALIGNMENT);
            side.add(legend);

            // scroll fix: let the panel report its true content height, and always allow vertical scroll
            JScrollPane sc = new JScrollPane(side,
                    JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
            sc.setPreferredSize(new Dimension(272, 200));
            sc.getVerticalScrollBar().setUnitIncrement(16);
            sc.setBorder(null);
            return sc;
        }

        /** A full-width left-aligned button wired to an action (keeps buildSidebar tidy). */
        JButton leftButton(String text, ActionListener a) {
            JButton b = new JButton(text);
            b.setAlignmentX(Component.LEFT_ALIGNMENT);
            b.setMaximumSize(new Dimension(252, 28));
            b.addActionListener(a);
            return b;
        }

        /** Planner-mode toggle: withReturn (what the robot runs) vs a plain forward pass. */
        JComponent planRow(ButtonGroup g) {
            JPanel row = new JPanel(new GridLayout(1, 2, 4, 0));
            row.setMaximumSize(new Dimension(252, 28));
            row.setAlignmentX(Component.LEFT_ALIGNMENT);
            String[] opts = { "withReturn", "forward" };
            String[] lbls = { "withReturn (robot)", "forward" };
            for (int i = 0; i < opts.length; i++) {
                final String m = opts[i];
                JToggleButton b = new JToggleButton(lbls[i]);
                b.setSelected(m.equals(planMode));
                b.setMargin(new Insets(2, 2, 2, 2));
                b.addActionListener(e -> { planMode = m; recompute(); });
                g.add(b); row.add(b);
            }
            return row;
        }

        /** Push the current model values back into the sidebar text boxes (after loading a field). */
        void syncInputs() {
            setBox("Width", robotW); setBox("Length", robotL); setBox("Min clearance", minClear);
            setBox("Vx (in/s)", Vx); setBox("Vy (in/s)", Vy);
            setBox("Path rounding", catmullTension); setBox("Turn rate", turnRate);
        }
        void setBox(String key, double v) { JTextField f = boxes.get(key); if (f != null) f.setText(fmt(v)); }
        static String fmt(double v) { return v == Math.rint(v) ? String.valueOf((long) v) : String.valueOf(v); }

        JComponent buildStatusBar() {
            status = new JTextArea(3, 20);
            status.setEditable(false);
            status.setBackground(new Color(0x0f, 0x12, 0x16));
            status.setForeground(new Color(0xcf, 0xd6, 0xe0));
            status.setBorder(BorderFactory.createEmptyBorder(6, 10, 6, 10));
            return status;
        }

        void updateStatus() {
            if (status == null) return;
            StringBuilder sb = new StringBuilder();
            if (currentField != null) sb.append("FIELD: ").append(currentField.name).append("\n");
            if (lastResult == null) sb.append("no plan");
            else {
                sb.append("STATUS: ").append(lastResult.status);
                sb.append("   mode: ").append(planMode);
                if (planMode.equals("withReturn"))
                    sb.append("   return: ").append(lastReversed ? "REVERSE" : "forward");
                sb.append("   segments: ").append(lastResult.segs.size());
                sb.append("   length: ").append(String.format("%.1f", totalDist)).append(" in");
                if (Vx > 0) sb.append("   ~").append(String.format("%.2f", totalDist / Vx)).append(" s at Vx");
                for (String n : lastResult.notes) sb.append("\n- ").append(n);
            }
            status.setText(sb.toString());
        }

        JLabel header(String t) {
            JLabel l = new JLabel(t);
            l.setFont(l.getFont().deriveFont(Font.BOLD, 12f));
            l.setForeground(new Color(0xe6, 0xeb, 0xf2));
            l.setAlignmentX(Component.LEFT_ALIGNMENT);
            l.setBorder(BorderFactory.createEmptyBorder(0, 0, 4, 0));
            return l;
        }

        JComponent modeRow(ButtonGroup bg, String[] modes) {
            JPanel row = new JPanel(new GridLayout(1, modes.length, 4, 0));
            row.setMaximumSize(new Dimension(240, 28));
            row.setAlignmentX(Component.LEFT_ALIGNMENT);
            for (String m : modes) {
                JToggleButton b = new JToggleButton(m);
                b.setSelected(m.equals(mode));
                b.setMargin(new Insets(2, 2, 2, 2));
                b.addActionListener(e -> { mode = m; });
                bg.add(b);
                row.add(b);
            }
            return row;
        }

        JComponent numRow(String label, double val, DoubleSetter setter) {
            JPanel row = new JPanel(new BorderLayout(6, 0));
            row.setMaximumSize(new Dimension(240, 26));
            row.setAlignmentX(Component.LEFT_ALIGNMENT);
            JLabel l = new JLabel(label);
            l.setForeground(new Color(0xbf, 0xc7, 0xd2));
            l.setPreferredSize(new Dimension(120, 22));
            JTextField tf = new JTextField(fmt(val));
            boxes.put(label, tf);                 // register so syncInputs() can update it on field load
            tf.addActionListener(e -> apply(tf, setter));
            tf.addFocusListener(new FocusAdapter() { public void focusLost(FocusEvent e) { apply(tf, setter); } });
            row.add(l, BorderLayout.WEST);
            row.add(tf, BorderLayout.CENTER);
            return row;
        }

        void apply(JTextField tf, DoubleSetter setter) {
            try { setter.set(Double.parseDouble(tf.getText().trim())); }
            catch (NumberFormatException ex) { /* ignore bad input */ }
        }

        JComponent checkRow(String label, boolean val, BoolSetter setter) {
            JCheckBox cb = new JCheckBox(label, val);
            cb.setAlignmentX(Component.LEFT_ALIGNMENT);
            cb.setForeground(new Color(0xbf, 0xc7, 0xd2));
            cb.setBackground(null);
            cb.addActionListener(e -> setter.set(cb.isSelected()));
            return cb;
        }
    }

    interface DoubleSetter { void set(double v); }
    interface BoolSetter { void set(boolean v); }
}
