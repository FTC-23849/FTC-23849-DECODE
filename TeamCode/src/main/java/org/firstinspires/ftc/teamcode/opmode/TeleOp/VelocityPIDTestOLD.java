package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.ArrayList;
import java.util.Iterator;

@Config
@TeleOp(name = "Velocity PID Example OLD")
public class VelocityPIDTestOLD extends LinearOpMode {
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    CRServoImplEx leftKickerServo;
    CRServoImplEx rightKickerServo;
    CRServoImplEx leftBackRoller;
    CRServoImplEx rightBackRoller;
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private PIDVelocityController velocityPID;
    public static double currentVelocity;
    public static double TargetVelocity = 900;
    public static double Kp = 0.00107;
    public static double Ki = 0;
    public static double Kd = 0.000007;
    public static double Kv = 0.000627;

    private static final double STDEV_WINDOW_SECONDS = 5.0;
    private final ArrayList<Double> errorSamples = new ArrayList<>();
    private final ArrayList<Double> errorTimestamps = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);
        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackRoller = hardwareMap.get(CRServoImplEx.class, "leftBackRoller");
        rightBackRoller = hardwareMap.get(CRServoImplEx.class, "rightBackRoller");
        rightBackRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        velocityPID = new   PIDVelocityController(Kp, Ki, Kd, Kv, TargetVelocity);
        velocityPID.setGains(Kp, Ki, Kd);
        velocityPID.setFeedforward(Kv);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = leftShooterMotor.getVelocity();

            velocityPID.setTargetVelocity(TargetVelocity);

            double power = velocityPID.update(currentVelocity);
            velocityPID.setGains(Kp, Ki, Kd);
            velocityPID.setFeedforward(Kv);

            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);

            double error = currentVelocity - TargetVelocity;
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
            telemetry.addData("Samples", errorSamples.size());
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