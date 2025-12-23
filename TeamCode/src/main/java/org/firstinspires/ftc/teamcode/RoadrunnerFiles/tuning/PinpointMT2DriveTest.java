package org.firstinspires.ftc.teamcode.RoadrunnerFiles.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    visionTools vision = new visionTools();

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotorEx.class, "LF");
        fr = hardwareMap.get(DcMotorEx.class, "RF");
        bl = hardwareMap.get(DcMotorEx.class, "LB");
        br = hardwareMap.get(DcMotorEx.class, "RB");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        waitForStart();

        while (opModeIsActive()) {

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
            double ph = Math.toDegrees(pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
            pinpoint.update();
            Pose2D pose2d = pinpoint.getPosition();
            double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
            double mx = 0;
            double my = 0;
            double myaw = 0;
            double[] MT2 = new double[]{-7,-7,-7};
            limelight.updateRobotOrientation(robotYaw);
            LLResult result = limelight.getLatestResult();
            limelight.updateRobotOrientation(robotYaw);
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    mx = botpose_mt2.getPosition().x;
                    my = botpose_mt2.getPosition().y;
                    myaw = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                    MT2 = new double[]{mx, my, myaw};
                } else {
                    MT2 = new double[]{-2000, -2000, -2000};
                }
            } else {
                MT2 = new double[]{-2000, -2000, -2000};
            }
            double mt1x = result.getBotpose().getPosition().x;
            double mt1y = result.getBotpose().getPosition().y;
            double mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addData("px", px);
            telemetry.addData("py", py);
            telemetry.addData("pheading", ph);

            telemetry.addData("mx", MT2[0]);
            telemetry.addData("my", MT2[1]);
            telemetry.addData("mheading", Math.toDegrees(MT2[2]));
            double[]AMT2 = vision.adjustMT2Values(MT2[0],MT2[1],MT2[2],0,-0.07650, 0.0, 0.14281);
            telemetry.addData("ax", AMT2[0]);
            telemetry.addData("ay", AMT2[1]);
            telemetry.addData("aheading", Math.toDegrees(AMT2[2]));
            telemetry.update();
            telemetry.addData("m1x", mt1x);
            telemetry.addData("m1y", mt1y);
            telemetry.addData("m1heading", mt1heading);
        }
    }
}
