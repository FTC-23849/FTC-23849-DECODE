package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

@TeleOp(name = "Pinpoint Raw Vel Tester", group = "Test")
public class PinpointVelTest extends OpMode {

    DcMotorEx lf, rf, lb, rb;
    GoBildaPinpointDriver pinpoint;
    visionTools vision = new visionTools();
    MultipleTelemetry dashboardTelemetry;

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

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        lf.setPower((y + x + rx) / denom);
        lb.setPower((y - x + rx) / denom);
        rf.setPower((y - x - rx) / denom);
        rb.setPower((y + x - rx) / denom);

        Pose2D pose = pinpoint.getPosition();

        double vxField = pinpoint.getVelX(DistanceUnit.METER);
        double vyField = pinpoint.getVelY(DistanceUnit.METER);

        double heading = pose.getHeading(AngleUnit.RADIANS);

        double vxRobot = vxField * Math.sin(heading) + vyField * Math.cos(heading);
        double vyRobot = vxField * Math.cos(heading) - vyField * Math.sin(heading);

        double px = pinpoint.getPosX(DistanceUnit.METER);
        double py = pinpoint.getPosY(DistanceUnit.METER);

        double currentDist = vision.groundDistancePinpoint(px, py, "Red");
        double flightTime = (7.0 / 30.0) * currentDist + 0.05;

        double ax = px + vxField * flightTime;
        double ay = py + vyField * flightTime;

        double estimatedDist = vision.groundDistancePinpoint(ax, ay, "Red");
        double fx = estimatedDist;
        double x2 = fx * fx;
        double x3 = x2 * fx;

        double speed = -1.8411 * x3
                + 29.2526 * x2
                - 344.1536 * fx
                - 962.8227;

        dashboardTelemetry.addData("Pose X (m)", pose.getX(DistanceUnit.METER));
        dashboardTelemetry.addData("Pose Y (m)", pose.getY(DistanceUnit.METER));
        dashboardTelemetry.addData("Pose Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        dashboardTelemetry.addLine();
        dashboardTelemetry.addData("Field Vel X (m/s)", vxField);
        dashboardTelemetry.addData("Field Vel Y (m/s)", vyField);
        dashboardTelemetry.addLine();
        dashboardTelemetry.addData("Robot Vel X (strafe m/s)", vxRobot);
        dashboardTelemetry.addData("Robot Vel Y (forward m/s)", vyRobot);
        dashboardTelemetry.addData("Heading Vel (deg/s)", pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        dashboardTelemetry.addLine();
        dashboardTelemetry.addData("speed", speed);
        dashboardTelemetry.update();
    }
}
