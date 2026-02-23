package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

@TeleOp(name = "SOTM Tester", group = "Test")
@Config
public class SOTMtest extends OpMode {

    DcMotorEx lf, rf, lb, rb;
    GoBildaPinpointDriver pinpoint;
    visionToolsClean vision = new visionToolsClean();
    MultipleTelemetry dashboardTelemetry;

    public static double kalmanQ = 0.02;
    public static double kalmanR = 0.1;
    public static double sec = 0.45;
    public static double moveAwayAdjustment = 0.0;
    public static double driveScale = 1.0;
    public static String allianceColor = "Red";

    private double lastSpeed = 0;

    @Override
    public void init() {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lf = hardwareMap.get(DcMotorEx.class, "LF");
        rf = hardwareMap.get(DcMotorEx.class, "RF");
        lb = hardwareMap.get(DcMotorEx.class, "LB");
        rb = hardwareMap.get(DcMotorEx.class, "RB");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(19.970472542, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {

        pinpoint.update();

        double y = -gamepad1.left_stick_y * driveScale;
        double x = gamepad1.left_stick_x * 1.1 * driveScale;
        double rx = gamepad1.right_stick_x * driveScale;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        lf.setPower((y + x + rx) / denom);
        lb.setPower((y - x + rx) / denom);
        rf.setPower((y - x - rx) / denom);
        rb.setPower((y + x - rx) / denom);

        Pose2D pose = pinpoint.getPosition();

        double px = pose.getX(DistanceUnit.METER);
        double py = pose.getY(DistanceUnit.METER);
        double pa = pose.getHeading(AngleUnit.DEGREES);
        double vxField = pinpoint.getVelX(DistanceUnit.METER);
        double vyField = pinpoint.getVelY(DistanceUnit.METER);

        double vxFilt = vision.getFilteredVelocityX(vxField, kalmanQ, kalmanR);
        double vyFilt = vision.getFilteredVelocityY(vyField, kalmanQ, kalmanR);

        double currentDist = vision.groundDistancePinpoint(px, py, allianceColor);
        double flightTime = sec * (7.0 / 30.0) * currentDist + 0.05;

        double ax = px + vxFilt * flightTime;
        double ay = py + vyFilt * flightTime;

        double estimatedDist = vision.groundDistancePinpoint(ax, ay, allianceColor);

        double xx = estimatedDist;
        double x2 = xx * xx;
        double x3 = x2 * xx;

        double speed;

        if (estimatedDist < 1.9) {
            speed = 638.8889 * x3 - 3788.8889 * x2 + 6576.9444 * x - 4590.4444;
        } else {
            speed = -1.8579 * x3 - 33.2624 * x2 - 0.5996 * x - 1162.7841;
        }

        if (estimatedDist < 2.0) {
            speed -= 30;
        }

        if (currentDist == -1) {
            speed = lastSpeed;
        }

        lastSpeed = speed;

        dashboardTelemetry.addData("Pos X (m)", px);
        dashboardTelemetry.addData("Pos Y (m)", py);

        dashboardTelemetry.addData("Field VX (m/s)", vxField);
        dashboardTelemetry.addData("Field VY (m/s)", vyField);

        dashboardTelemetry.addData("Filtered VX", vxFilt);
        dashboardTelemetry.addData("Filtered VY", vyFilt);

        dashboardTelemetry.addData("Pred X (m)", ax);
        dashboardTelemetry.addData("Pred Y (m)", ay);

        dashboardTelemetry.addData("Current Dist", currentDist);
        dashboardTelemetry.addData("Estimated Dist", estimatedDist);

        dashboardTelemetry.addData("Flywheel Speed", speed);
        telemetry.addData("heading (deg)", pa);
        double heading = pa;
        double adjustedHeading = heading + 90;
        if (adjustedHeading < 0) adjustedHeading += 360;
        telemetry.addData("adj heading (deg)", adjustedHeading);
        telemetry.update();
        dashboardTelemetry.update();
    }
}
