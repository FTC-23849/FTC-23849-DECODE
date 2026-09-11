package pathviz;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;

/**
 * Scans the repo's .java source files for Field / Obstacle definitions and turns each one it finds
 * into a {@link FieldSpec} the visualizer can load -- so you pick an existing field from a dropdown
 * instead of drawing anything.
 *
 * It reads the SOURCE TEXT (not compiled classes), so it works on any OpMode that builds a field like:
 *     new Field(15.5, 17.5)
 *         .add(new RectObstacle(0, 67, 0, 144))
 *         .add(new TriObstacle(72, 96, 48, 72, 96, 72))
 *         ...
 *         .minClearance(3);
 * Obstacle arguments may be plain numbers OR simple named constants declared in the same file
 * (e.g. ROBOT_WIDTH = 15.5) -- those get resolved. Anything it can't resolve is skipped, and the
 * whole thing is wrapped so it can never crash the app: worst case it returns an empty list and the
 * visualizer falls back to a built-in field.
 */
public class FieldScanner {

    /** One field discovered in the code: its obstacles, robot size, clearance, and (if present) END. */
    public static class FieldSpec {
        public String name;                       // shown in the dropdown
        public String source;                     // file it came from
        public double robotW = 15.5, robotL = 17.5;
        public double minClear = 3;
        public boolean hasEnd = false;
        public double endX, endY;
        public final List<double[]> rects   = new ArrayList<>();  // {lcx,lcy,ucx,ucy}
        public final List<double[]> tris     = new ArrayList<>();  // {ax,ay,bx,by,cx,cy}
        public final List<double[]> circles  = new ArrayList<>();  // {cx,cy,r}

        public int obstacleCount() { return rects.size() + tris.size() + circles.size(); }
        public String toString() { return name; }
    }

    private static final Pattern P_RECT   = Pattern.compile("new\\s+RectObstacle\\s*\\(([^)]*)\\)");
    private static final Pattern P_TRI    = Pattern.compile("new\\s+TriObstacle\\s*\\(([^)]*)\\)");
    private static final Pattern P_CIRCLE = Pattern.compile("new\\s+CircleObstacle\\s*\\(([^)]*)\\)");
    private static final Pattern P_FIELD  = Pattern.compile("new\\s+Field\\s*\\(([^)]*)\\)");
    private static final Pattern P_CLEAR  = Pattern.compile("minClearance\\s*\\(([^)]*)\\)");
    private static final Pattern P_END    = Pattern.compile("\\bEND\\s*=\\s*\\{([^}]*)\\}");
    private static final Pattern P_CONST  = Pattern.compile("([A-Za-z_][A-Za-z0-9_]*)\\s*=\\s*(-?[0-9]+(?:\\.[0-9]+)?)");

    /** Find every field in the repo. Never throws. */
    public static List<FieldSpec> scan() {
        List<FieldSpec> out = new ArrayList<>();
        try {
            File root = findRepoRoot();
            if (root == null) return out;
            try (Stream<Path> walk = Files.walk(root.toPath())) {
                walk.filter(Files::isRegularFile)
                    .filter(p -> p.toString().endsWith(".java"))
                    .filter(p -> {
                        String s = p.toString().replace('\\', '/');
                        return !s.contains("/build/") && !s.contains("/.gradle/") && !s.contains("/.git/");
                    })
                    .forEach(p -> {
                        try {
                            if (Files.size(p) > 1_000_000) return;   // skip anything huge
                            String text = new String(Files.readAllBytes(p));
                            FieldSpec fs = parse(text, fileBase(p));
                            if (fs != null && fs.obstacleCount() > 0) out.add(fs);
                        } catch (Exception ignore) { /* one bad file shouldn't stop the scan */ }
                    });
            }
        } catch (Exception ignore) { /* return whatever we have */ }
        return out;
    }

    /** Parse one file's text into a FieldSpec (or null if it has no obstacles). */
    private static FieldSpec parse(String text, String base) {
        Map<String, Double> consts = new HashMap<>();
        Matcher cm = P_CONST.matcher(text);
        while (cm.find()) consts.put(cm.group(1), Double.parseDouble(cm.group(2)));

        FieldSpec fs = new FieldSpec();
        fs.source = base;

        addAll(P_RECT.matcher(text),   consts, 4, fs.rects);
        addAll(P_TRI.matcher(text),    consts, 6, fs.tris);
        addAll(P_CIRCLE.matcher(text), consts, 3, fs.circles);
        if (fs.obstacleCount() == 0) return null;

        // robot dimensions from `new Field(w, l)`
        Matcher fm = P_FIELD.matcher(text);
        if (fm.find()) {
            double[] wl = args(fm.group(1), consts, 2);
            if (wl != null) { fs.robotW = wl[0]; fs.robotL = wl[1]; }
        }
        // clearance from minClearance(x)
        Matcher clm = P_CLEAR.matcher(text);
        if (clm.find()) {
            double[] c = args(clm.group(1), consts, 1);
            if (c != null) fs.minClear = c[0];
        }
        // END = { x, y }
        Matcher em = P_END.matcher(text);
        if (em.find()) {
            double[] e = args(em.group(1), consts, 2);
            if (e != null) { fs.hasEnd = true; fs.endX = e[0]; fs.endY = e[1]; }
        }

        fs.name = base + "  (" + fs.obstacleCount() + " obstacle" + (fs.obstacleCount() == 1 ? "" : "s") + ")";
        return fs;
    }

    private static void addAll(Matcher m, Map<String, Double> consts, int n, List<double[]> into) {
        while (m.find()) {
            double[] a = args(m.group(1), consts, n);
            if (a != null) into.add(a);
        }
    }

    /** Split "0, 67, 0, 144" into doubles; resolve named constants; return null if any arg is unknown
     *  or the count is wrong. */
    private static double[] args(String inside, Map<String, Double> consts, int expected) {
        String[] parts = inside.split(",");
        if (parts.length != expected) return null;
        double[] out = new double[expected];
        for (int i = 0; i < expected; i++) {
            String t = parts[i].trim();
            try {
                out[i] = Double.parseDouble(t);
            } catch (NumberFormatException nf) {
                Double v = consts.get(t);
                if (v == null) return null;      // an arg we can't resolve -> skip this obstacle
                out[i] = v;
            }
        }
        return out;
    }

    private static String fileBase(Path p) {
        String n = p.getFileName().toString();
        return n.endsWith(".java") ? n.substring(0, n.length() - 5) : n;
    }

    /** Walk up from the working directory looking for the repo root (settings.gradle or .git). */
    private static File findRepoRoot() {
        File dir = new File(System.getProperty("user.dir")).getAbsoluteFile();
        for (int i = 0; i < 10 && dir != null; i++) {
            if (new File(dir, "settings.gradle").exists()
                    || new File(dir, "settings.gradle.kts").exists()
                    || new File(dir, ".git").exists()) return dir;
            dir = dir.getParentFile();
        }
        // fall back to the working directory itself
        return new File(System.getProperty("user.dir")).getAbsoluteFile();
    }

    /** A built-in field to show if the scan finds nothing (mirrors the DECODE field in RandomPointAuto). */
    public static FieldSpec builtinDecode() {
        FieldSpec fs = new FieldSpec();
        fs.source = "built-in";
        fs.name = "DECODE (built-in fallback)";
        fs.robotW = 15.5; fs.robotL = 17.5; fs.minClear = 3;
        fs.hasEnd = true; fs.endX = 48; fs.endY = 96;
        fs.rects.add(new double[]{ 0, 67, 0, 144 });
        fs.tris.add(new double[]{ 0, 144, 25, 144, 0, 110 });
        fs.tris.add(new double[]{ 72, 96, 48, 72, 96, 72 });
        fs.tris.add(new double[]{ 72, 48, 48, 72, 96, 72 });
        return fs;
    }
}
