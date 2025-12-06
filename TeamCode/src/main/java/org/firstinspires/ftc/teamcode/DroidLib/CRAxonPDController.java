package org.firstinspires.ftc.teamcode.DroidLib;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class CRAxonPDController {

    // --- PD state for this controller instance ---
    private final ElapsedTime pidTimer = new ElapsedTime();
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private boolean firstRun = true;

    public CRAxonPDController() {
        // runs automatically when you do: new PIDFControllers()
        pidTimer.reset();
        lastTime = pidTimer.seconds();
    }


    /**
     * PD Controller for CR Axon servo using the built-in Analog Encoder
     * @param kp Tuned kP constant
     * @param kd Tuned kD constant
     * @param normalizedTarget Target value. Must be normalized from 0.0V - 3.3V, to 0.0 - 1.0
     * @param normalizedAxonEncoderOutput Output value from the Axon encoder. Must be normalized from 0.0V - 3.3V, to 0.0 - 1.0
     * @return the power to be provided to the Axon servo
     */

    public double Output (double kp, double kd, double normalizedTarget, double normalizedAxonEncoderOutput) {

        // Treat target/input as circular [0, 1)

        // Wrap target into [0, 1)
        normalizedTarget = normalizedTarget % 1.0;
        if (normalizedTarget < 0) normalizedTarget += 1.0;

        // --- P term: circular error in [-0.5, 0.5] ---
        double error = normalizedTarget - normalizedAxonEncoderOutput;

        if (error > 0.5) {
            error -= 1.0;
        } else if (error < -0.5) {
            error += 1.0;
        }

        // --- D term: derivative of error ---
        double now = pidTimer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;  // avoid divide-by-zero

        double derivative = (error - lastError) / dt;

        double power = kp * error + kd * derivative;

        // save state for next loop
        lastError = error;
        lastTime = now;

        return Range.clip(power, -1.0, 1.0);

    }

}
