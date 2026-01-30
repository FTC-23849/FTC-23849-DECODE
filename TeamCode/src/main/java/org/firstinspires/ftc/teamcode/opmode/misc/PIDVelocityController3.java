package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController3 {

    private double KpMaintain;
    private double KiMaintain;

    private double KpRecovery;
    private double KpRecoveryDrive;
    private double KiRecovery;
    private double KsRecovery;

    private double kS;
    private double kV;

    private double recoveryThreshold = 80;
    private double maintainThreshold = 40;
    private boolean lowVoltage;

    private double targetVelocity;
    private double integralSum = 0.0;
    private boolean recoveringMode = false;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDVelocityController3(
            double targetVelocity,
            double KpMaintain, double KiMaintain,
            double KpRecovery, double KiRecovery, double KsRecovery,
            double kS, double kV
    ) {
        this.targetVelocity = targetVelocity;
        this.KpMaintain = KpMaintain;
        this.KiMaintain = KiMaintain;
        this.KpRecovery = KpRecovery;
        this.KiRecovery = KiRecovery;
        this.KsRecovery = KsRecovery;
        this.kS = kS;
        this.kV = kV;
        timer.reset();
    }

    public double update(double currentVelocity, double batteryVoltage, double normalVoltage) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;

        if (Math.abs(error) > recoveryThreshold) {
            recoveringMode = true;
        } else if (Math.abs(error) < maintainThreshold) {
            recoveringMode = false;
        }

        double Kp = recoveringMode ? KpRecovery : KpMaintain;
        Kp = (Math.abs(error) > 40) ? KpRecoveryDrive: Kp;
        double Ki = recoveringMode ? KiRecovery : KiMaintain;
        double kS = recoveringMode ? KsRecovery : this.kS;

        if (!recoveringMode) {
            integralSum += error * dt;
        } else {
            integralSum = 0.0;
        }

        double pid = (Kp * error) + (Ki * integralSum);

        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(error));
        }
        double output = 0;
        if(lowVoltage){
            output = (pid + ff) * (normalVoltage / batteryVoltage);
        }else{
            output = (pid + ff) *0.95; //(normalVoltage / batteryVoltage);
        }

        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getTargetVelocity() { return targetVelocity; }
    public boolean isRecovering() { return recoveringMode; }

    public void setMaintainGains(double Kp, double Ki,double Kpd) {
        this.KpMaintain = Kp;
        this.KiMaintain = Ki;
        this.KpRecoveryDrive = Kpd;
    }

    public void setRecoveryGains(double Kp, double Ki, double Ks) {
        this.KpRecovery = Kp;
        this.KiRecovery = Ki;
        this.KsRecovery = Ks;
    }

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    public void setRecoveryThreshold(double threshold) { this.recoveryThreshold = threshold; }
    public void setMaintainThreshold(double threshold) { this.maintainThreshold = threshold; }
    public void setVoltageStatus(boolean voltage) { this.lowVoltage = voltage; }
}
