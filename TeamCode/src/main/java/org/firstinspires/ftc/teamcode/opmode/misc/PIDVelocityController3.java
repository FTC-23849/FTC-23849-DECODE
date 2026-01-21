package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController3 {

    private double Kp_maintain;
    private double Kp_recovery;

    private double kS_maintain;
    private double kS_recovery;

    private double kV;


    public double recoveryThreshold = 80;


    private double targetVelocity;
    private double lastTargetVelocity;
    private double lastVelocity = 0;
    private ElapsedTime timer = new ElapsedTime();

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


    public double update(double x,double currentVelocity, double batteryVoltage,double NOMINAL_VOLTAGE) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;
        boolean recovering;
        if (Math.abs(error) > recoveryThreshold) {
            recovering = true;
        } else if (Math.abs(error) < recoveryThreshold) {
            recovering = false;
        } else {
            recovering = lastVelocity < targetVelocity;
        }

        double Kp = recovering ? Kp_recovery : Kp_maintain;
        double kS = recovering ? kS_recovery : kS_maintain;
        if(x<2.5){
            kS = recovering ? 0.09 : kS_maintain;
        }
        double pid = Kp * error;
        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        double output = pid + ff;
        double voltageComp = NOMINAL_VOLTAGE / batteryVoltage;
        output *= voltageComp;
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

    public void setKpMaintain(double Kp) { this.Kp_maintain = Kp; }
    public void setKpRecovery(double Kp) { this.Kp_recovery = Kp; }

    public void setKSMaintain(double kS) { this.kS_maintain = kS; }
    public void setKSRecovery(double kS) { this.kS_recovery = kS; }

    public void setKV(double kV) { this.kV = kV; }
}
