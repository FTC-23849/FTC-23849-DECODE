package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController2 {

    private double Kp, Ki, Kd;
    private double kS;
    private double kV;
    private double targetVelocity;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double maxVelocity = 2000;
    private double maxIntegral = 1.0;

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
        timer.reset();
    }

    public double update(double currentVelocity) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-6;

        double error = targetVelocity - currentVelocity;
        double normalizedError = error / maxVelocity;
        integralSum += normalizedError * dt;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        double derivative = (normalizedError - lastError) / dt;
        lastError = normalizedError;

        double pidOutput = (Kp * normalizedError) + (Ki * integralSum) + (Kd * derivative);
        double sign = Math.signum(targetVelocity);
        double feedforward = (kS * sign) + (kV * targetVelocity);
        double output = pidOutput + feedforward;

        return Math.max(-1.0, Math.min(1.0, output));
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
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

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }
}
