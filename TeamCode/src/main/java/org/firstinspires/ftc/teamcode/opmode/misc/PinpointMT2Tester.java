package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Pinpoint: MT2 Init", group = "Sensor")
public class PinpointMT2Tester extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private IMU imu;
    double robotYaw = 0;
    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void loop() {
        pinpoint.update();
        Pose2D pose2d = pinpoint.getPosition();

        double pinx = pose2d.getX(DistanceUnit.METER);
        double piny = pose2d.getY(DistanceUnit.METER);
        telemetry.addData("Pinpoint Location:", "(" + pinx + ", " + piny + ")");
        robotYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
        telemetry.update();
    }

}
