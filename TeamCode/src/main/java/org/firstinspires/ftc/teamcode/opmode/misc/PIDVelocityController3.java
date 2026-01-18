package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController3 {

    // ---- Gains for modes ----
    private double Kp_maintain;
    private double Kp_recovery;

    private double kS_maintain;
    private double kS_recovery;

    private double kV;

    // Thresholds
    private double recoveryThreshold = 80;  // abs(error) above this → recovery
    private double maintainThreshold = 80;  // hysteresis to exit recovery

    private double targetVelocity;
    private double lastTargetVelocity;
    private double lastVelocity = 0;

    private static final double NOMINAL_VOLTAGE = 13.0;
    private ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor with tunable gains
     * @param targetVelocity initial target velocity
     * @param KpMaintain maintain mode P
     * @param KpRecovery recovery mode P
     * @param kSMaintain maintain mode static feedforward
     * @param kSRecovery recovery mode static feedforward
     * @param kV velocity feedforward (same for both modes)
     */
    public PIDVelocityController3(
            double targetVelocity,
            double KpMaintain,
            double KpRecovery,
            double kSMaintain,
            double kSRecovery,
            double kV
    ) {
        this.targetVelocity = targetVelocity;
        this.lastTargetVelocity = targetVelocity;

        this.Kp_maintain = KpMaintain;
        this.Kp_recovery = KpRecovery;

        this.kS_maintain = kSMaintain;
        this.kS_recovery = kSRecovery;

        this.kV = kV;

        timer.reset();
    }

    public double update(double currentVelocity) {
        return update(currentVelocity, NOMINAL_VOLTAGE);
    }

    public double update(double currentVelocity, double batteryVoltage) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;

        // Determine mode
        boolean recovering;
        if (Math.abs(error) > recoveryThreshold) {
            recovering = true;
        } else if (Math.abs(error) < maintainThreshold) {
            recovering = false;
        } else {
            // Keep previous mode if within hysteresis band
            recovering = lastVelocity < targetVelocity; // optional, or store mode
        }

        // Apply mode gains
        double Kp = recovering ? Kp_recovery : Kp_maintain;
        double kS = recovering ? kS_recovery : kS_maintain;

        // PID term (only P)
        double pid = Kp * error;

        // Feedforward term
        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        // Combine PID + feedforward
        double output = pid + ff;

        // Voltage compensation
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltage;
        output *= voltageComp;

        // Clamp output
        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }
    public void setMaintain(double VkPm, double VkSm, double VkV) {
        this.Kp_maintain = VkPm;
        this.kS_maintain = VkSm;
        this.kV = VkV;
    }

    public void setRecovery(double VkPr, double VkSr) {
        this.kS_recovery = VkSr;
        this.Kp_recovery = VkPr;
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setRecoveryThreshold(double threshold) {
        this.recoveryThreshold = threshold;
    }

    public void setMaintainThreshold(double threshold) {
        this.maintainThreshold = threshold;
    }

    // Optional: adjust gains dynamically at runtime
    public void setKpMaintain(double Kp) { this.Kp_maintain = Kp; }
    public void setKpRecovery(double Kp) { this.Kp_recovery = Kp; }

    public void setKSMaintain(double kS) { this.kS_maintain = kS; }
    public void setKSRecovery(double kS) { this.kS_recovery = kS; }

    public void setKV(double kV) { this.kV = kV; }
}
