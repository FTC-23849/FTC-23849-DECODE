package org.firstinspires.ftc.teamcode.opmode.misc;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Config
@TeleOp(name = "Artifact Color Detector", group = "Test")
public class ArtifactColorDetector extends LinearOpMode {

    // ---------- TUNABLES ----------
    public static float MIN_SATURATION   = 0.02f;   // only reject if BOTH sat & val are below these
    public static float MIN_VALUE        = 0.005f;

    public static float GREEN_HUE_CENTER  = 150f;   // adjust if your green reads ~100–140
    public static float PURPLE_HUE_CENTER = 225f;

    public static float COLOR_SENSOR_GAIN = 1f;     // try 2–8 if readings are tiny

    // goBILDA RGB light servo positions
    public static double LED_OFF_POS    = 0.00;
    public static double LED_GREEN_POS  = 0.50;
    public static double LED_PURPLE_POS = 0.70;
    // ------------------------------

    private NormalizedColorSensor colorLeft;
    private NormalizedColorSensor colorRight;
    private Servo rgbLight;

    // for loop-time measurement
    private long lastLoopTimeNs = 0;

    private enum ArtifactColor { GREEN, PURPLE, UNKNOWN }

    private static class Reading {
        ArtifactColor color;
        float hue, sat, val;
        float conf;   // confidence = s * v
        Reading(ArtifactColor c, float h, float s, float v) {
            color = c; hue = h; sat = s; val = v; conf = s * v;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Your hardware names
        colorLeft  = hardwareMap.get(NormalizedColorSensor.class, "leftIntakeColorSensor");
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "rightIntakeColorSensor");
        rgbLight   = hardwareMap.get(Servo.class, "light");

        initSensor(colorLeft);
        initSensor(colorRight);

        telemetry.addLine("ArtifactColorDetector ready");
        telemetry.update();

        waitForStart();

        lastLoopTimeNs = System.nanoTime(); // initialize

        while (opModeIsActive()) {
            // --- measure loop time ---
            long now = System.nanoTime();
            double loopMs = (now - lastLoopTimeNs) / 1e6;  // ns -> ms
            lastLoopTimeNs = now;

            // --- color sensing ---
            Reading L = readAndClassify(colorLeft);
            Reading R = readAndClassify(colorRight);

            ArtifactColor overall = combineByConfidence(L, R);
            showOnRgbLight(overall);

            // --- telemetry ---
            telemetry.addData("Loop Time (ms)", "%.2f", loopMs);

            telemetry.addData("Left Detected",  L.color);
            telemetry.addData("Right Detected", R.color);
            telemetry.addData("Overall",        overall);

            telemetry.addData("Left HSV",  "h=%.1f s=%.3f v=%.3f conf=%.5f",
                    L.hue, L.sat, L.val, L.conf);
            telemetry.addData("Right HSV", "h=%.1f s=%.3f v=%.3f conf=%.5f",
                    R.hue, R.sat, R.val, R.conf);

            telemetry.update();
        }

        rgbLight.setPosition(LED_OFF_POS);
    }

    private void initSensor(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
        sensor.setGain(COLOR_SENSOR_GAIN);
    }

    private Reading readAndClassify(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);

        float h = hsv[0], s = hsv[1], v = hsv[2];

        // Only UNKNOWN if it's BOTH very dark AND very unsaturated
        if (v < MIN_VALUE && s < MIN_SATURATION) {
            return new Reading(ArtifactColor.UNKNOWN, h, s, v);
        }

        // Otherwise, closest hue wins
        float dGreen  = hueDistance(h, GREEN_HUE_CENTER);
        float dPurple = hueDistance(h, PURPLE_HUE_CENTER);
        ArtifactColor c = (dGreen <= dPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        return new Reading(c, h, s, v);
    }

    private float hueDistance(float a, float b) {
        float d = Math.abs(a - b);
        return Math.min(d, 360f - d);
    }

    // If either sensor reports a color, choose the one with higher confidence (s * v).
    // If both UNKNOWN, return UNKNOWN.
    private ArtifactColor combineByConfidence(Reading left, Reading right) {
        boolean leftColor  = left.color  != ArtifactColor.UNKNOWN;
        boolean rightColor = right.color != ArtifactColor.UNKNOWN;

        if (!leftColor && !rightColor) return ArtifactColor.UNKNOWN;
        if (leftColor && !rightColor)  return left.color;
        if (!leftColor && rightColor)  return right.color;

        // both have colors; pick by confidence
        if (left.conf > right.conf) return left.color;
        if (right.conf > left.conf) return right.color;

        // equal confidence tie-breaker: pick the one with higher V (brighter)
        if (left.val > right.val) return left.color;
        if (right.val > left.val) return right.color;

        // final tie-breaker: prefer PURPLE (arbitrary; change if you prefer GREEN)
        return (left.color == ArtifactColor.PURPLE || right.color == ArtifactColor.PURPLE)
                ? ArtifactColor.PURPLE : ArtifactColor.GREEN;
    }

    private void showOnRgbLight(ArtifactColor color) {
        switch (color) {
            case GREEN:  rgbLight.setPosition(LED_GREEN_POS);  break;
            case PURPLE: rgbLight.setPosition(LED_PURPLE_POS); break;
            default:     rgbLight.setPosition(LED_OFF_POS);     break;
        }
    }
}
