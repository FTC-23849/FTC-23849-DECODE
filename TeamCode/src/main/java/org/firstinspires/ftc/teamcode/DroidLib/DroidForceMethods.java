package org.firstinspires.ftc.teamcode.DroidLib;

public class  DroidForceMethods {

    public double zeroAndNormalizeAxonEncoder (double rawEncoderVoltage, double zero) {
        double fullScale = 3.3; // Axon encoder range in volts

        // 1) Zero around your chosen zero point
        double zeroed = rawEncoderVoltage - zero;

        // 2) Wrap into [0, fullScale)
        if (zeroed < 0) {
            zeroed += fullScale;
        } else if (zeroed >= fullScale) {
            zeroed -= fullScale;
        }

        // 3) Normalize to [0, 1)
        return zeroed / fullScale;
    }

    public double normalizeTo01(double input, double originalMax) {
        if (originalMax == 0) return 0;  // or throw an exception
        double normalized = input / originalMax;
        // Optional clamp:
        if (normalized < 0) normalized = 0;
        if (normalized > 1) normalized = 1;
        return normalized;
    }

    public double wrapAround0 (double input, double zero, double maxRange) {

        // 1) Zero around your chosen zero point
        double wrapped = input - zero;

        // 2) Wrap into [0, fullScale)
        if (wrapped < 0) {
            wrapped += maxRange;
        } else if (wrapped >= maxRange) {
            wrapped -= maxRange;
        }

        return wrapped;

    }

    public double adjustForCircular01(double target, double input) {
        // Make sure target is in [0, 1)
        target = target % 1.0;
        if (target < 0) target += 1.0;

        double error = target - input;

        // If error is large positive, we want to "pull" measurement up by 1
        // If error is large negative, we "push" it down by 1
        if (error > 0.5) {
            input += 1.0;
        } else if (error < -0.5) {
            input -= 1.0;
        }

        return input;
    }

}
