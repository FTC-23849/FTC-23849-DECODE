package org.firstinspires.ftc.teamcode.hardware.subsystems;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.CustomGoBildaPrismRgbLedDriver;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

public class MiscSubsystems {
    private final RobotClass robot;
    public VoltageSensor voltageSensor;
    public CustomGoBildaPrismRgbLedDriver prism;
    public Servo rgbLight;
    public ServoImplEx zoneLight;
    public AnalogInput switchCurrent;
    public NormalizedColorSensor colorLeft1, colorRight1, colorLeft2, colorRight2;

    public String allianceColor = "Red";
    public boolean lowVoltage = false;

    // Color Sensor stuff
    public enum ArtifactColor { GREEN, PURPLE, RED, UNKNOWN }
    private ArtifactColor last1 = null, last2 = null, last3 = null;
    private int colorPhase = 0;
    private double lastColorTick = 0;
    public boolean prismSenseEnabled = false;
    private final float[] hsvBuf = new float[3];

    // Log data into a file on control hub
    private static final int MAX_TOTAL_LOG_FILES = 5;
    private BufferedWriter currentLogger;
    private String currentLogPath;
    private long lastLoggedTimeMs;
    private double lastCurrentLogTime = 0;

    public MiscSubsystems(RobotClass robot) {
        this.robot = robot;
    }

    public void init() {
        voltageSensor = robot.hardwareMap.get(VoltageSensor.class, "Control Hub");
        prism = robot.hardwareMap.get(CustomGoBildaPrismRgbLedDriver.class, "prism");
        rgbLight = robot.hardwareMap.get(Servo.class, "light");
        zoneLight = robot.hardwareMap.get(ServoImplEx.class, "zoneLight");
        switchCurrent = robot.hardwareMap.get(AnalogInput.class, "switchCurrent");

        colorLeft1 = robot.hardwareMap.get(NormalizedColorSensor.class, "colorLeft1");
        colorRight1 = robot.hardwareMap.get(NormalizedColorSensor.class, "colorRight1");
        colorLeft2 = robot.hardwareMap.get(NormalizedColorSensor.class, "colorLeft2");
        colorRight2 = robot.hardwareMap.get(NormalizedColorSensor.class, "colorRight2");

        initSensor(colorLeft1); initSensor(colorRight1);
        initSensor(colorLeft2); initSensor(colorRight2);

        prism.configureSolidLayer(0, 0, 1, 100, 0, 0, 0);
        prism.configureSolidLayer(1, 2, 3, 100, 0, 0, 0);
        prism.configureSolidLayer(2, 4, 5, 100, 0, 0, 0);

        setupLogging();
    }

    private void initSensor(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(true);
        sensor.setGain(1.0f);
    }

    public void update() {
        double now = System.currentTimeMillis();
        if (prismSenseEnabled && (now - lastColorTick >= 80)) {
            lastColorTick = now;
            handleColorSensing();
        }

        if (now - lastCurrentLogTime >= 20) {
            logMotorCurrents();
            lastCurrentLogTime = now;
        }

        updateZoneLight();
    }

    private void updateZoneLight() {
        boolean intakeStalled = robot.intake.intakeIsStalled && !robot.drive.isDtStalled();
        if (intakeStalled) {
            zoneLight.setPosition(0.277);
        } else if (robot.intake.thirdBallPresent) {
            zoneLight.setPosition(1.0);
        } else {
            zoneLight.setPosition(0.0);
        }
    }

    private void handleColorSensing() {
        switch (colorPhase) {
            case 0:
                ArtifactColor overall1 = combine(read(colorLeft1), read(colorRight1));
                if (overall1 != last1) applyToLayer(0, overall1);
                last1 = overall1;
                showOnRgbLight(overall1);
                break;
            case 1:
                ArtifactColor overall2 = combine(read(colorLeft2), read(colorRight2));
                if (overall2 != last2) applyToLayer(1, overall2);
                last2 = overall2;
                break;
            case 2:
                ArtifactColor overall3 = predictThird(last1, last2, robot.intake.thirdBallPresent);
                if (overall3 != last3) applyToLayer(2, overall3);
                last3 = overall3;
                break;
        }
        colorPhase = (colorPhase + 1) % 3;
    }

    private ArtifactColor read(NormalizedColorSensor sensor) {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvBuf);
        if (hsvBuf[2] < 0.005f && hsvBuf[1] < 0.02f) return ArtifactColor.UNKNOWN;
        float dG = Math.abs(hsvBuf[0] - 150f); dG = Math.min(dG, 360 - dG);
        float dP = Math.abs(hsvBuf[0] - 225f); dP = Math.min(dP, 360 - dP);
        return (dG <= dP) ? ArtifactColor.GREEN : ArtifactColor.PURPLE;
    }

    private ArtifactColor combine(ArtifactColor c1, ArtifactColor c2) {
        if (c1 != ArtifactColor.UNKNOWN) return c1;
        return c2;
    }

    private ArtifactColor predictThird(ArtifactColor f, ArtifactColor s, boolean present) {
        if (!present || f == ArtifactColor.UNKNOWN || s == ArtifactColor.UNKNOWN) return ArtifactColor.UNKNOWN;
        int g = (f == ArtifactColor.GREEN ? 1 : 0) + (s == ArtifactColor.GREEN ? 1 : 0);
        int p = (f == ArtifactColor.PURPLE ? 1 : 0) + (s == ArtifactColor.PURPLE ? 1 : 0);
        if (p == 2) return ArtifactColor.GREEN;
        if (p == 1 && g == 1) return ArtifactColor.PURPLE;
        return ArtifactColor.RED;
    }

    private void applyToLayer(int layer, ArtifactColor c) {
        int r = 0, g = 0, b = 0;
        if (c == ArtifactColor.GREEN) g = 255;
        else if (c == ArtifactColor.PURPLE) { r = 180; b = 255; }
        else if (c == ArtifactColor.RED) r = 255;
        prism.setLayerColor(layer, r, g, b);
    }

    private void showOnRgbLight(ArtifactColor c) {
        if (c == ArtifactColor.GREEN) rgbLight.setPosition(0.5);
        else if (c == ArtifactColor.PURPLE) rgbLight.setPosition(0.7);
        else rgbLight.setPosition(0.0);
    }

    public void setAllianceBlue() { allianceColor = "Blue"; }

    public void setAllianceRed() { allianceColor = "Red"; }

    public void toggleLowVoltage() { lowVoltage = !lowVoltage; }

    public void enablePrisms() { prismSenseEnabled = true; }

    public void disablePrisms() {
        prismSenseEnabled = false;
        applyToLayer(0, ArtifactColor.UNKNOWN);
        applyToLayer(1, ArtifactColor.UNKNOWN);
        applyToLayer(2, ArtifactColor.UNKNOWN);
    }

    private void setupLogging() {
        long now = System.currentTimeMillis();
        String startStr = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date(now));
        currentLogPath = "/sdcard/FIRST/TeleOpClean_" + startStr + ".csv";
        lastLoggedTimeMs = now;
        try {
            currentLogger = new BufferedWriter(new FileWriter(currentLogPath, false));
            currentLogger.write("wall_clock_ms,lf_A,rf_A,lb_A,rb_A,frontIntake_A,backIntake_A,lShooter_A,rShooter_A,motor_total_A,floodgate_A");
            currentLogger.newLine();
            currentLogger.flush();
        } catch (IOException ignored) {}

        File logDir = new File("/sdcard/FIRST/");
        File[] logFiles = logDir.listFiles((dir, name) -> name.endsWith(".csv"));
        if (logFiles != null && logFiles.length > MAX_TOTAL_LOG_FILES) {
            Arrays.sort(logFiles, (a, b) -> a.getName().compareTo(b.getName()));
            for (int i = 0; i < logFiles.length - MAX_TOTAL_LOG_FILES; i++) {
                logFiles[i].delete();
            }
        }
    }

    private void logMotorCurrents() {
        if (currentLogger == null) return;
        try {
            double lf = robot.drive.leftFront.getCurrent(CurrentUnit.AMPS);
            double rf = robot.drive.rightFront.getCurrent(CurrentUnit.AMPS);
            double lb = robot.drive.leftBack.getCurrent(CurrentUnit.AMPS);
            double rb = robot.drive.rightBack.getCurrent(CurrentUnit.AMPS);
            double fi = robot.intake.frontIntakeMotor.getCurrent(CurrentUnit.AMPS);
            double bi = robot.intake.backIntakeMotor.getCurrent(CurrentUnit.AMPS);
            double ls = robot.shooter.leftShooterMotor.getCurrent(CurrentUnit.AMPS);
            double rs = robot.shooter.rightShooterMotor.getCurrent(CurrentUnit.AMPS);
            double total = lf + rf + lb + rb + fi + bi + ls + rs;
            double floodgate = (switchCurrent.getVoltage() / 3.3) * 80.0;
            lastLoggedTimeMs = System.currentTimeMillis();
            currentLogger.write(String.format(Locale.US, "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                    lastLoggedTimeMs, lf, rf, lb, rb, fi, bi, ls, rs, total, floodgate));
            currentLogger.newLine();
            
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("current/total", total);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } catch (IOException ignored) {}
    }

    public double getVoltage() { return voltageSensor.getVoltage(); }

    public void telemetry() {
        robot.telemetry.addData("Voltage", getVoltage());
        robot.telemetry.addData("Ball 1", last1);
        robot.telemetry.addData("Ball 2", last2);
        robot.telemetry.addData("Ball 3 Pred", last3);
    }

    public void stop() {
        try {
            if (currentLogger != null) {
                currentLogger.flush();
                currentLogger.close();
                String endStr = new SimpleDateFormat("HHmmss", Locale.US).format(new Date(lastLoggedTimeMs));
                String finalPath = currentLogPath.replace(".csv", "_to_" + endStr + ".csv");
                new File(currentLogPath).renameTo(new File(finalPath));
            }
        } catch (IOException ignored) {}
    }
}
