package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController2;
import org.firstinspires.ftc.teamcode.vision.visionTools;

import java.util.ArrayList;
import java.util.Iterator;

@Config
@TeleOp(name = "Velocity PID + Distance")
public class DistanceTestVelocityPID extends LinearOpMode {
    ServoImplEx leftHood;
    ServoImplEx rightHood;
    CRServoImplEx leftKickerServo;
    GoBildaPinpointDriver pinpoint;
    CRServoImplEx rightKickerServo;
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    Limelight3A limelight;
    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftBackMotor;
    DcMotorEx rightBackMotor;
    DcMotorEx frontIntakeMotor;
    DcMotorEx backIntakeMotor;
    ServoImplEx leftTurretServo;
    ServoImplEx rightTurretServo;
    ServoImplEx leftTongueServo;
    ServoImplEx rightTongueServo;
    ElapsedTime timer = new ElapsedTime();
    private PIDVelocityController2 velocityPID;
    visionTools vision = new visionTools();
    // Dashboard tunables
    public static double TargetVelocity = 900;   // ticks/sec
    public static double Kp = 2;
    public static double Ki = 1.5;
    public static double Kd = 0;
    public static double height = 0.1;
    public static double kS = 0;
    public static double kV = 0.00045;
    public static String alliance = "Blue";
    public static double distance = 0;
    private static final double STDEV_WINDOW_SECONDS = 5.0;
    private final ArrayList<Double> errorSamples = new ArrayList<>();
    private final ArrayList<Double> errorTimestamps = new ArrayList<>();

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntakeMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntakeMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "RB");
        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);
        ElapsedTime kickerStartDelayTimer = new ElapsedTime();
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        velocityPID = new PIDVelocityController2(
                Kp, Ki, Kd,
                kS, kV,
                TargetVelocity
        );
        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(19.970472542,DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        kickerStartDelayTimer.reset();
        waitForStart();
        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        leftTurretServo.setPosition(0.5);
        rightTurretServo.setPosition(0.5);
        rightHood.setDirection(ServoImplEx.Direction.REVERSE);
        vision.mt2pinpoint(pinpoint,limelight);
        while (opModeIsActive()) {
            timer.reset();
            leftHood.setPosition(height);
            rightHood.setPosition(height);
            leftTurretServo.setPosition(0.5);
            rightTurretServo.setPosition(0.5);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFrontMotor.setPower((y + x + rx) / denominator);
            leftBackMotor.setPower((y - x + rx) / denominator);
            rightFrontMotor.setPower((y - x - rx) / denominator);
            rightBackMotor.setPower((y + x - rx) / denominator);

            pinpoint.update();
            distance = vision.groundDistancePinpoint(pinpoint,alliance);
            vision.groundDistancePinpoint(pinpoint,alliance);
            if(gamepad1.a){
                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);
                frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
                backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
            }else{
                frontIntakeMotor.setPower(0);
                backIntakeMotor.setPower(0);
            }
            if(!gamepad1.x){
                kickerStartDelayTimer.reset();
            }
            else{
                // do nothing
            }
            if(gamepad1.x){
                leftTongueServo.setPosition(Globals.tongueShoot);
                rightTongueServo.setPosition(Globals.tongueShoot);

                    if(kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay) {
                        leftKickerServo.setPower(Globals.rollerKickerShoot);
                        rightKickerServo.setPower(Globals.rollerKickerShoot);
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed);
                    }
                    else{
                        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        frontIntakeMotor.setPower(0.7);
                    }

                backIntakeMotor.setPower(Globals.backIntakeShootSpeed);
            }else{
                leftKickerServo.setPower(0);
                rightKickerServo.setPower(0);
            }
            if(gamepad1.b){
                if(alliance.equals("Blue")){
                    alliance = "Red";
                }else if(alliance.equals("Red")){
                    alliance = "Blue";
                }
            }
            if(gamepad1.y){
                vision.mt1pinpoint(pinpoint,limelight);
            }
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
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Target Velocity", TargetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity StDev (5s)", stdev);
            telemetry.addData("Motor Power", power);
            telemetry.addData("Distance",distance);
            telemetry.addData("hood height", height);
            telemetry.addData("looptime", timer.milliseconds());
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
