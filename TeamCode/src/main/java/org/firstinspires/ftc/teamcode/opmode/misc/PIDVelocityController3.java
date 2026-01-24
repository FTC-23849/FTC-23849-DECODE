package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController3 {

    // ---- Gains for maintain mode ----
    private double KpMaintain;
    private double KiMaintain;

    // ---- Gains for recovery mode ----
    private double KpRecovery;
    private double KiRecovery;

    // ---- Feedforward (constant) ----
    private double kS;
    private double kV;

    // ---- Thresholds ----
    private double recoveryThreshold = 80;  // abs(error) > this → recovery
    private double maintainThreshold = 60;  // abs(error) < this → maintain

    // ---- Controller state ----
    private double targetVelocity;
    private double lastVelocity = 0;
    private double integralSum = 0.0;
    private boolean recoveringMode = false;

    private static final double NOMINAL_VOLTAGE = 13.0;
    private final ElapsedTime timer = new ElapsedTime();

    // ---- Constructor ----
    public PIDVelocityController3(
            double targetVelocity,
            double KpMaintain, double KiMaintain,
            double KpRecovery, double KiRecovery,
            double kS, double kV
    ) {
        this.targetVelocity = targetVelocity;

        this.KpMaintain = KpMaintain;
        this.KiMaintain = KiMaintain;
        this.KpRecovery = KpRecovery;
        this.KiRecovery = KiRecovery;

        this.kS = kS;
        this.kV = kV;

        timer.reset();
    }

    // ---- Main update ----
    public double update(double currentVelocity, double batteryVoltage,double normalVoltage) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;

        // ---- Mode switching with hysteresis ----
        if (Math.abs(error) > recoveryThreshold) {
            recoveringMode = true;
        } else if (Math.abs(error) < maintainThreshold) {
            recoveringMode = false;
        }

        // ---- Select gains ----
        double Kp = recoveringMode ? KpRecovery : KpMaintain;
        double Ki = recoveringMode ? KiRecovery : KiMaintain;

        // ---- Integral handling ----
        if (!recoveringMode) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0; // prevent windup in recovery
        }

        // ---- PID term ----
        double pid = (Kp * error) + (Ki * integralSum);

        // ---- Feedforward ----
        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        // ---- Combine and voltage compensate ----
        double output = (pid + ff) * (normalVoltage / batteryVoltage);

        // ---- Clamp output ----
        return Math.max(-1.0, Math.min(1.0, output));
    }


    // ---- Target velocity ----
    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getTargetVelocity() { return targetVelocity; }

    // ---- Mode telemetry ----
    public boolean isRecovering() { return recoveringMode; }

    // ---- Set gains dynamically ----
    public void setMaintainGains(double Kp, double Ki) {
        this.KpMaintain = Kp;
        this.KiMaintain = Ki;
    }

    public void setRecoveryGains(double Kp, double Ki) {
        this.KpRecovery = Kp;
        this.KiRecovery = Ki;
    }

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    public void setRecoveryThreshold(double threshold) { this.recoveryThreshold = threshold; }
    public void setMaintainThreshold(double threshold) { this.maintainThreshold = threshold; }
}
