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

import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController;
import org.firstinspires.ftc.teamcode.hardware.Globals;


@Config
@TeleOp(name = "Velocity PID Example")
public class VelocityPIDTest extends LinearOpMode {
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    CRServoImplEx leftKickerServo;
    CRServoImplEx rightKickerServo;
    CRServoImplEx leftBackRoller;
    CRServoImplEx rightBackRoller;
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private PIDVelocityController velocityPID;

    public static double TargetVelocity = 900;
    public static double Kp = 0.00107;
    public static double Ki = 0;
    public static double Kd = 0.000007;
    public static double Kv = 0.000627;

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

        velocityPID = new PIDVelocityController(Kp, Ki, Kd, Kv, TargetVelocity);

        waitForStart();

        while (opModeIsActive()) {
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);
            backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
            frontIntakeMotor.setPower(Globals.frontIntakeShootSpeed);
            double currentVelocity = leftShooterMotor.getVelocity();
            velocityPID.setTargetVelocity(TargetVelocity);
            velocityPID.setGains(Kp, Ki, Kd);
            velocityPID.setFeedforward(Kv);

            double power = velocityPID.update(currentVelocity);
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);


        }
    }
}
