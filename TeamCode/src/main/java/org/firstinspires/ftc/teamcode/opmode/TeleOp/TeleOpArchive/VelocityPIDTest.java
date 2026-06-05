package org.firstinspires.ftc.teamcode.opmode.TeleOp.TeleOpArchive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController2;

import java.util.ArrayList;
import java.util.Iterator;

@Disabled
@Config
@TeleOp(name = "Velocity PID Example")
public class VelocityPIDTest extends LinearOpMode {

    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;

    private PIDVelocityController2 velocityPID;

    // Dashboard tunables
    public static double TargetVelocity = 900;   // ticks/sec
    public static double Kp = 8;
    public static double Ki = 0.0;
    public static double Kd = 0.08;

    public static double kS = 0.059;
    public static double kV = 0.00035;

    private static final double STDEV_WINDOW_SECONDS = 5.0;
    private final ArrayList<Double> errorSamples = new ArrayList<>();
    private final ArrayList<Double> errorTimestamps = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");

        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        velocityPID = new PIDVelocityController2(
                Kp, Ki, Kd,
                kS, kV,
                TargetVelocity
        );

        waitForStart();

        while (opModeIsActive()) {

            double currentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity())/2;

            // Update tunables
            velocityPID.setPID(Kp, Ki, Kd);
            velocityPID.setFeedforward(kS, kV);
            velocityPID.setTargetVelocity(TargetVelocity);

            double power = velocityPID.update(currentVelocity);

            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);

            double error = TargetVelocity - currentVelocity;
            double now = getRuntime();

            errorSamples.add(error);
            errorTimestamps.add(now);

            Iterator<Double> sampleIter = errorSamples.iterator();
            Iterator<Double> timeIter = errorTimestamps.iterator();

            while (timeIter.hasNext()) {
                double t = timeIter.next();
                sampleIter.next();

                if (now - t > STDEV_WINDOW_SECONDS) {
                    timeIter.remove();
                    sampleIter.remove();
                } else {
                    break;
                }
            }

            double stdev = calculateStdDev(errorSamples);

            telemetry.addData("Target Velocity", TargetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity StDev (5s)", stdev);
            telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }

    private double calculateStdDev(ArrayList<Double> values) {
        if (values.size() < 2) return 0.0;

        double mean = 0.0;
        for (double v : values) mean += v;
        mean /= values.size();

        double variance = 0.0;
        for (double v : values) {
            variance += Math.pow(v - mean, 2);
        }
        variance /= values.size();

        return Math.sqrt(variance);
    }
}
