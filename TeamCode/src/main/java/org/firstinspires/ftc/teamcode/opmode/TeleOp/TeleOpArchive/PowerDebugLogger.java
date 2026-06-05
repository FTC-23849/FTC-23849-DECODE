package org.firstinspires.ftc.teamcode.opmode.TeleOp.TeleOpArchive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class PowerDebugLogger {

    private static final String TAG = "POWER_DEBUG";

    private final VoltageSensor voltageSensor;
    private final DcMotorEx[] motors;
    private final String[] motorNames;

    private long lastLogMs = 0;
    private long logIntervalMs = 150;

    private final SimpleDateFormat sdf =
            new SimpleDateFormat("HH:mm:ss.SSS", Locale.US);

    public PowerDebugLogger(HardwareMap hardwareMap,
                            String[] motorNames,
                            DcMotorEx[] motors) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motorNames = motorNames;
        this.motors = motors;
    }

    public void setLogIntervalMs(long intervalMs) {
        this.logIntervalMs = intervalMs;
    }

    public void log(String robotState) {
        long now = System.currentTimeMillis();

        if (now - lastLogMs < logIntervalMs) {
            return;
        }

        lastLogMs = now;

        double voltage = voltageSensor.getVoltage();
        String timeStr = sdf.format(new Date(now));

        StringBuilder sb = new StringBuilder();

        sb.append("t=").append(timeStr);
        sb.append(" | state=").append(robotState);
        sb.append(" | V=").append(String.format(Locale.US, "%.2f", voltage));

        for (int i = 0; i < motors.length; i++) {
            double current = motors[i].getCurrent(CurrentUnit.AMPS);

            sb.append(" | ");
            sb.append(motorNames[i]);
            sb.append("=");
            sb.append(String.format(Locale.US, "%.2fA", current));
        }

        RobotLog.ii(TAG, sb.toString());
    }

    public void event(String message) {
        String timeStr = sdf.format(new Date(System.currentTimeMillis()));
        RobotLog.ii("POWER_EVENT", "t=%s | %s", timeStr, message);
    }

    public void warning(String message) {
        String timeStr = sdf.format(new Date(System.currentTimeMillis()));
        RobotLog.ee("POWER_WARN", "t=%s | %s", timeStr, message);
    }
}