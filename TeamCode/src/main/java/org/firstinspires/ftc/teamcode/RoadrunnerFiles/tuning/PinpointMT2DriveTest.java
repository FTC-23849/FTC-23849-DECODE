package org.firstinspires.ftc.teamcode.RoadrunnerFiles.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.visionTools;

@Config
@TeleOp
public class PinpointMT2DriveTest extends LinearOpMode {

    DcMotorEx fl, fr, bl, br;
    ServoImplEx leftTurretServo;
    ServoImplEx rightTurretServo;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    AnalogInput encoder;
    visionTools vision = new visionTools();

    @Override
    public void runOpMode() throws InterruptedException {
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        fl = hardwareMap.get(DcMotorEx.class, "LF");
        fr = hardwareMap.get(DcMotorEx.class, "RF");
        bl = hardwareMap.get(DcMotorEx.class, "LB");
        br = hardwareMap.get(DcMotorEx.class, "RB");

        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.start();
        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        /*
        rightTurretServo.setPosition(0.5);
        leftTurretServo.setPosition(0.5);
        */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setEncoderResolution(19.970472542,DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,-63,-65, AngleUnit.DEGREES,0));
        limelight.pipelineSwitch(9);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            double position = encoder.getVoltage() / 3.2 * 360;
            leftTurretServo.setPosition(0.5);
            rightTurretServo.setPosition(0.5);
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            fl.setPower(y + x + rx);
            bl.setPower(y - x + rx);
            fr.setPower(y - x - rx);
            br.setPower(y + x - rx);

            pinpoint.update();

            double px = pinpoint.getPosition().getX(DistanceUnit.METER);
            double py = pinpoint.getPosition().getY(DistanceUnit.METER);
            double ph = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            Pose2D pose2d = pinpoint.getPosition();
            double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
            double mx = 0;
            double my = 0;
            double myaw = 0;
            double[] MT2 = new double[]{-7,-7,-7};
            limelight.updateRobotOrientation(robotYaw+178);
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                telemetry.addData("result null?", false);
                telemetry.addData("result valid?", result.isValid());
                telemetry.addData("botpose mt1 null?", result.getBotpose() == null);
                telemetry.addData("botpose mt2 null?", result.getBotpose_MT2() == null);
            } else {
                telemetry.addData("result null?", true);
            }

            if (result != null  && result.getBotpose_MT2() != null) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                mx = botpose_mt2.getPosition().x;
                my = botpose_mt2.getPosition().y;
                myaw = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                MT2 = new double[]{mx, my, myaw};
            } else {
                MT2 = new double[]{-2000, -2000, -2000};
            }

            double mt1x = -2000;
            double mt1y = -2000;
            double mt1heading = -2000;
            if (result != null && result.getBotpose() != null) {
                mt1x = result.getBotpose().getPosition().x;
                mt1y = result.getBotpose().getPosition().y;
                mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            }

            telemetry.addData("px", px);
            telemetry.addData("py", py);
            telemetry.addData("pheading", ph);
            telemetry.addData("mx", MT2[0]);
            telemetry.addData("my", MT2[1]);
            telemetry.addData("mheading", MT2[2]);
            double[] AMT2 = vision.adjustMT2Values(MT2[0], MT2[1], MT2[2], 0, -0.07650, 0.0, 0.14281);
            telemetry.addData("ax", AMT2[0]);
            telemetry.addData("ay", AMT2[1]);
            telemetry.addData("aheading", AMT2[2]);
            telemetry.addData("m1x", mt1x);
            telemetry.addData("m1y", mt1y);
            telemetry.addData("m1heading", mt1heading+180);
            telemetry.addData("x diff", px-MT2[0]);
            telemetry.addData("y diff", py-MT2[1]);
            telemetry.addData("pos", position*(19.0/99.0));
            telemetry.update();
        }
    }
}
