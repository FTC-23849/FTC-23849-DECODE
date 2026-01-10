package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "Pinpoint Raw Vel Tester", group = "Test")
public class PinpointVelTest extends OpMode {

    DcMotorEx lf, rf, lb, rb;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {

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

        telemetry.addData("Pose X (m)", pose.getX(DistanceUnit.METER));
        telemetry.addData("Pose Y (m)", pose.getY(DistanceUnit.METER));
        telemetry.addData("Pose Heading (deg)", pose.getHeading(AngleUnit.DEGREES));

        telemetry.addLine();

        telemetry.addData("getVelX (m/s)", pinpoint.getVelX(DistanceUnit.METER));
        telemetry.addData("getVelY (m/s)", pinpoint.getVelY(DistanceUnit.METER));
        telemetry.addData(
                "getHeadingVelocity (deg/s)",
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
        );

        telemetry.update();
    }
}
