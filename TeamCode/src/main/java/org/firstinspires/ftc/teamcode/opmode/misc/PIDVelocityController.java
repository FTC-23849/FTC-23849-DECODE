package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDVelocityController {
    private double Kp, Ki, Kd, Kv;
    private double targetVelocity;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime tuneTimer = new ElapsedTime();
    private boolean initialPowerReached = false;
    private boolean settled = false;

    public PIDVelocityController(double Kp, double Ki, double Kd, double Kv, double targetVelocity) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kv = Kv;
        this.targetVelocity = targetVelocity;
        timer.reset();
        tuneTimer.reset();
    }

    public double update(double currentVelocity) {
        double error = targetVelocity - currentVelocity;
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 1e-6;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        double pidOutput = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;

        double output = pidOutput + Kv * targetVelocity;

        // Handle timing-based auto feedforward tuning logic
        if (!initialPowerReached && tuneTimer.milliseconds() > 500) {
            tuneTimer.reset();
            initialPowerReached = true;
        }

        if (tuneTimer.milliseconds() > 100) {
            settled = true;
        }

        if (Math.abs(error) > 20 && tuneTimer.milliseconds() > 100 && initialPowerReached && settled) {
            tuneFeedForward(error < 0 ? -1 : 1);
            tuneTimer.reset();
            settled = false;
        }

        return Math.max(-1.0, Math.min(1.0, output));
    }

    public double tuneFeedForward(int direction) {
        double correctionVal = 0.0000002;

        if (direction == 1) Kv += correctionVal;
        else if (direction == -1) Kv -= correctionVal;

        Kv = Math.max(0, Math.min(1, Kv));
        return Kv;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
        tuneTimer.reset();
        initialPowerReached = false;
        settled = false;
    }

    public void setTargetVelocity(double newTarget) {
        this.targetVelocity = newTarget;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void setFeedforward(double Kv) {
        this.Kv = Kv;
    }

    public double getFeedforward() {
        return Kv;
    }
}
