package org.firstinspires.ftc.teamcode.opmode.misc;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.vision.visionTools;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointMT2Tester extends OpMode {

    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    visionTools vision;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        vision = new visionTools();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pinpoint.update();

        double[] mt2 = vision.getMT2(limelight, pinpoint);
        double mt2X = mt2[0];
        double mt2Y = mt2[1];
        double mt2Yaw = mt2[2];

        Pose2D ppPose = pinpoint.getPosition();
        double ppX = ppPose.getX(DistanceUnit.METER);
        double ppY = ppPose.getY(DistanceUnit.METER);
        double ppYaw = ppPose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("MT2 X", mt2X);
        telemetry.addData("MT2 Y", mt2Y);
        telemetry.addData("MT2 Yaw", mt2Yaw);
        telemetry.addData("Pinpoint X", ppX);
        telemetry.addData("Pinpoint Y", ppY);
        telemetry.addData("Pinpoint Yaw", ppYaw);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke("#b5b33f");
        packet.fieldOverlay().fillCircle((float) mt2X, (float) mt2Y, 5);

        packet.fieldOverlay().setStroke("#3F51B5");
        packet.fieldOverlay().fillCircle((float) ppX, (float) ppY, 5);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
