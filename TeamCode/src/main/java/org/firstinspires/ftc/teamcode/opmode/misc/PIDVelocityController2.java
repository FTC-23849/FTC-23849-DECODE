package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController2 {

    private double Kp, Ki, Kd;
    private double kS, kV;

    private double targetVelocity;
    private double lastTargetVelocity;

    private double integralSum = 0;
    private double lastVelocity = 0;

    private double maxIntegral = 0.2;
    private double velocityDeadband = 10;

    private ElapsedTime timer = new ElapsedTime();

    public PIDVelocityController2(
            double Kp, double Ki, double Kd,
            double kS, double kV,
            double targetVelocity
    ) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.kS = kS;
        this.kV = kV;
        this.targetVelocity = targetVelocity;
        this.lastTargetVelocity = targetVelocity;
        timer.reset();
    }

    public double update(double currentVelocity) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        if (targetVelocity != lastTargetVelocity) {
            integralSum = 0;
            lastTargetVelocity = targetVelocity;
        }

        double error = targetVelocity - currentVelocity;
        if (Math.abs(error) < velocityDeadband) error = 0;

        double derivative = -(currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;

        double pid = (Kp * error) + (Kd * derivative);

        double ff = 0;
        if (targetVelocity != 0) {
            ff = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        }

        double output = pid + ff;

        if (Math.abs(output) < 1.0) {
            integralSum += error * dt;
            integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        }

        output += Ki * integralSum;

        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void reset() {
        integralSum = 0;
        lastVelocity = 0;
        lastTargetVelocity = targetVelocity;
        timer.reset();
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setFeedforward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }

    public void setVelocityDeadband(double velocityDeadband) {
        this.velocityDeadband = velocityDeadband;
    }
}
