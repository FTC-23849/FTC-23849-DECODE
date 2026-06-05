package org.firstinspires.ftc.teamcode.opmode.TeleOp.TeleOpArchive;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.hardware.CustomGoBildaPrismRgbLedDriver;

@Disabled
@TeleOp
@Config
public class PrismLightSortingTest extends OpMode {

    private NormalizedColorSensor colorLeft1;
    private NormalizedColorSensor colorRight1;

    private NormalizedColorSensor colorLeft2;
    private NormalizedColorSensor colorRight2;

    private NormalizedColorSensor colorLeft3;
    private NormalizedColorSensor colorRight3;

    CustomGoBildaPrismRgbLedDriver prism;

    private final float[] hsvBuf = new float[3];

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

    private enum ArtifactColor { GREEN, PURPLE, UNKNOWN }

    private static class Reading {
        ArtifactColor color;
        float hue, sat, val;
        float conf;   // confidence = s * v
        Reading(ArtifactColor c, float h, float s, float v) {
            color = c; hue = h; sat = s; val = v; conf = s * v;
        }
    }

    // LED colors
    private static final int[] RGB_GREEN  = {0, 255, 0};
    private static final int[] RGB_PURPLE = {180, 0, 255};
    private static final int[] RGB_OFF    = {0, 0, 0};

    private static final int BRIGHT_ON = 100;
    private static final int BRIGHT_OFF = 0;

    private ArtifactColor last1 = null;
    private ArtifactColor last2 = null;
    private ArtifactColor last3 = null;


    @Override
    public void init() {

        prism = hardwareMap.get(CustomGoBildaPrismRgbLedDriver.class, "prism");

        // Configure 3 layers as 3 segments: (0-1), (2-3), (4-5)
        // Index 0 is the LED closest to the Prism driver. :contentReference[oaicite:10]{index=10}
        prism.configureSolidLayer(0, 0, 1, BRIGHT_OFF, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, BRIGHT_OFF, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, BRIGHT_OFF, 0, 0, 0);

        colorLeft1 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight1 = hardwareMap.get(NormalizedColorSensor.class, "colorRight1");

        colorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft2");
        colorRight2 = hardwareMap.get(NormalizedColorSensor.class, "colorRight2");

        colorLeft3 = hardwareMap.get(NormalizedColorSensor.class, "colorLeft3");
        colorRight3 = hardwareMap.get(NormalizedColorSensor.class, "colorRight3");

        initSensor(colorLeft1);
        initSensor(colorRight1);

        initSensor(colorLeft2);
        initSensor(colorRight2);

        initSensor(colorLeft3);
        initSensor(colorRight3);

        last1 = last2 = last3 = null;

    }

    @Override
    public void loop() {

        // --- color sensing ---
        Reading L1 = readAndClassify(colorLeft1);
        Reading R1 = readAndClassify(colorRight1);

        ArtifactColor overall1 = combineByConfidence(L1, R1);

        Reading L2 = readAndClassify(colorLeft2);
        Reading R2 = readAndClassify(colorRight2);

        ArtifactColor overall2 = combineByConfidence(L2, R2);

        Reading L3 = readAndClassify(colorLeft3);
        Reading R3 = readAndClassify(colorRight3);

        ArtifactColor overall3 = combineByConfidence(L3, R3);

        // Only update Prism when a sensor's detected state changes (faster + less I2C spam)
        if (last1 == null || overall1 != last1) applyToLayer(prism, 0, overall1);
        if (last2 == null || overall2 != last2) applyToLayer(prism, 1, overall2);
        if (last3 == null || overall3 != last3) applyToLayer(prism, 2, overall3);

        last1 = overall1; last2 = overall2; last3 = overall3;

    }

    private void applyToLayer(CustomGoBildaPrismRgbLedDriver prism, int layer, ArtifactColor c) {
        int[] rgb;
        int bright;

        switch (c) {
            case GREEN:
                rgb = RGB_GREEN; bright = BRIGHT_ON; break;
            case PURPLE:
                rgb = RGB_PURPLE; bright = BRIGHT_ON; break;
            default:
                rgb = RGB_OFF; bright = BRIGHT_OFF; break;
        }

        prism.setLayerBrightness(layer, bright);
        prism.setLayerColor(layer, rgb[0], rgb[1], rgb[2]);
    }

    private void initSensor(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
        sensor.setGain(COLOR_SENSOR_GAIN);
    }

    private Reading readAndClassify(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBuf);

        float h = hsvBuf[0];
        float s = hsvBuf[1];
        float v = hsvBuf[2];

        if (v < MIN_VALUE && s < MIN_SATURATION) {
            return new Reading(ArtifactColor.UNKNOWN, h, s, v);
        }

        float dGreen  = hueDistance(h, GREEN_HUE_CENTER);
        float dPurple = hueDistance(h, PURPLE_HUE_CENTER);
        ArtifactColor c = (dGreen <= dPurple) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;

        return new Reading(c, h, s, v);
    }


    private float hueDistance(float a, float b) {
        float d = Math.abs(a - b);
        return Math.min(d, 360f - d);
    }

    private ArtifactColor combineByConfidence(Reading left, Reading right) {
        boolean leftColor  = left.color  != ArtifactColor.UNKNOWN;
        boolean rightColor = right.color != ArtifactColor.UNKNOWN;

        if (!leftColor && !rightColor) return ArtifactColor.UNKNOWN;
        if (leftColor && !rightColor)  return left.color;
        if (!leftColor && rightColor)  return right.color;

        if (left.conf > right.conf) return left.color;
        if (right.conf > left.conf) return right.color;

        if (left.val > right.val) return left.color;
        if (right.val > left.val) return right.color;

        return (left.color == ArtifactColor.PURPLE || right.color == ArtifactColor.PURPLE)
                ? ArtifactColor.PURPLE : ArtifactColor.GREEN;
    }

//    private void showOnRgbLight(ArtifactColor color) {
//        switch (color) {
//            case GREEN:  rgbLight.setPosition(LED_GREEN_POS);  break;
//            case PURPLE: rgbLight.setPosition(LED_PURPLE_POS); break;
//            default:     rgbLight.setPosition(LED_OFF_POS);    break;
//        }
//    }
}
