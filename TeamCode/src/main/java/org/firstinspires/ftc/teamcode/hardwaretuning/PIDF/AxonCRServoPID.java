package org.firstinspires.ftc.teamcode.hardwaretuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DroidLib.CRAxonPDController;
import org.firstinspires.ftc.teamcode.DroidLib.DroidForceMethods;
import org.firstinspires.ftc.teamcode.hardware.Globals;

@Config
@TeleOp
public class AxonCRServoPID extends OpMode {

    public static double kP = 5.0;
    public static double kI = 0.0;
    public static double kD = 0.05;

    private ElapsedTime pidTimer = new ElapsedTime();

    private double lastError = 0.0;
    private double lastTime = 0.0;

    public static double normalizedTarget = 0.0;

    public static double zeroValue = 2.8;

    double encoderVoltage;
    double processedEncoderValue;

    CRServoImplEx leftKickerServo ;
    CRServoImplEx rightKickerServo;
    AnalogInput axonEncoder;

    CRAxonPDController kickerPID = new CRAxonPDController();
    DroidForceMethods DFM = new DroidForceMethods();

    @Override
    public void init() {

        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);
        axonEncoder = hardwareMap.get(AnalogInput.class, "leftKickerEncoder");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


//        pidTimer.reset();
//        lastTime = pidTimer.seconds();

    }

    @Override
    public void loop() {

        encoderVoltage = axonEncoder.getVoltage();
        processedEncoderValue = DFM.zeroAndNormalizeAxonEncoder(encoderVoltage, Globals.KICKER_ZERO);

//        processedEncoderValue = processedEncoderVoltage(encoderVoltage, zeroValue);

//        leftKickerServo.setPower(PDController(kP, kD, normalizedTarget, processedEncoderValue));
//        rightKickerServo.setPower(PDController(kP, kD, normalizedTarget, processedEncoderValue));
//

        leftKickerServo.setPower(kickerPID.Output(Globals.KICKER_kP, Globals.KICKER_kD, Globals.KICKER_IDLE, processedEncoderValue));
        rightKickerServo.setPower(kickerPID.Output(Globals.KICKER_kP, Globals.KICKER_kD, Globals.KICKER_IDLE, processedEncoderValue));

        telemetry.addData("Target: ", Globals.KICKER_IDLE);
        telemetry.addData("Encoder Voltage: ", encoderVoltage);
        telemetry.addData("Processed Encoder Value", processedEncoderValue);

        telemetry.update();

    }

//    public double processedEncoderVoltage(double rawEncoderVoltage, double zero) {
//        double fullScale = 3.3; // Axon encoder range in volts
//
//        // 1) Zero around your chosen zero point
//        double zeroed = rawEncoderVoltage - zero;
//
//        // 2) Wrap into [0, fullScale)
//        if (zeroed < 0) {
//            zeroed += fullScale;
//        } else if (zeroed >= fullScale) {
//            zeroed -= fullScale;
//        }
//
//        // 3) Normalize to [0, 1)
//        return zeroed / fullScale;
//    }
//
//
//
//    public double PDController(double kp, double kd, double target, double input) {
//        // Treat target/input as circular [0, 1)
//
//        // Wrap target into [0, 1)
//        target = target % 1.0;
//        if (target < 0) target += 1.0;
//
//        // --- P term: circular error in [-0.5, 0.5] ---
//        double error = target - input;
//
//        if (error > 0.5) {
//            error -= 1.0;
//        } else if (error < -0.5) {
//            error += 1.0;
//        }
//
//        // --- D term: derivative of error ---
//        double now = pidTimer.seconds();
//        double dt = now - lastTime;
//        if (dt <= 0) dt = 1e-3;  // avoid divide-by-zero
//
//        double derivative = (error - lastError) / dt;
//
//        double power = kp * error + kd * derivative;
//
//        // save state for next loop
//        lastError = error;
//        lastTime = now;
//
//        return Range.clip(power, -1.0, 1.0);
//    }
}
