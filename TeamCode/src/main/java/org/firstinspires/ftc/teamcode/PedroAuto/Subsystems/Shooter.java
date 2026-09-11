package org.firstinspires.ftc.teamcode.PedroAuto.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController3;
import org.firstinspires.ftc.teamcode.vision.visionTools;

@Configurable
public class Shooter {

    DcMotorEx leftShooterMotor, rightShooterMotor;
    ServoImplEx leftTurretServo, rightTurretServo;
    ServoImplEx leftHood, rightHood;

    private final PIDVelocityController3 velocityPID;
    private final visionTools vision = new visionTools();

    public String allianceColor = "Blue";

    private double turretOffset = 0;

    private double turretStartPos = 0;

    public static final double GEAR = 11.9;

    public Shooter(HardwareMap hardwareMap, double turretOffset, double turretStartPos) {

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTurretServo = hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(ServoImplEx.class, "rightTurretServo");

        leftHood = hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood.setDirection(ServoImplEx.Direction.REVERSE);

        this.turretOffset = turretOffset;
        this.turretStartPos = turretStartPos;

        velocityPID = new PIDVelocityController3(
                0, Globals.KpMaintain, Globals.KiMaintain,
                Globals.KpRecovery, Globals.KiRecovery, Globals.KsRecoveryClose,
                Globals.KsFF, Globals.KvFF);

    }

    public void init() {

        leftHood.setPosition(0.28);
        rightHood.setPosition(0.28);

        leftTurretServo.setPosition(turretStartPos);
        rightTurretServo.setPosition(turretStartPos);

    }

    public Command startVelPID(double targetSpeed) {
        return Command.build()
                .setStart(() -> velocityPID.setTargetVelocity(targetSpeed))
                .setExecute(() -> {
                    double vel = leftShooterMotor.getVelocity();
                    velocityPID.setTargetVelocity(targetSpeed);
                    velocityPID.setMaintainGains(Globals.KpMaintain, Globals.KiMaintain, Globals.KpDriveRecovery);
                    velocityPID.setRecoveryGains(Globals.KpRecovery, Globals.KiRecovery, Globals.KsRecoveryClose);
                    velocityPID.setFeedforward(Globals.KsFF, Globals.KvFF);
                    velocityPID.setRecoveryThreshold(Globals.recoveryThreshold);
                    velocityPID.setMaintainThreshold(Globals.maintainThreshold);
                    double power = velocityPID.update(vel, 13.5, Globals.defaultVoltage);
                    leftShooterMotor.setPower(power);
                    rightShooterMotor.setPower(power);
                })
                .setDone(() -> false)
                .requiring(leftShooterMotor, rightShooterMotor);
    }

    public Command startTurretTracking(Follower follower) {
        return Command.build()
                .setExecute(() -> {
                    Pose2D visionPose = toVisionPose(follower.getPose());
                    double cmd = vision.pinpointTurretMovingAUTO(
                            visionPose, GEAR, Globals.sec,
                            leftTurretServo.getPosition(), allianceColor);
                    leftTurretServo.setPosition(cmd - turretOffset);
                    rightTurretServo.setPosition(cmd - turretOffset);
                })
                .setDone(() -> false)
                .requiring(leftTurretServo, rightTurretServo);
    }

    private Pose2D toVisionPose(Pose pedroPose) {
        // 1) Pedro -> RoadRunner/FTC coordinates (known conversion, via Pedro's converter)
        Pose rr = pedroPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);

        // 2) reproduce RRtoPinpoint(...) EXACTLY so the tuned constants apply unchanged
        double x = -rr.getX();
        double y = -rr.getY();
        double deg = Math.toDegrees(rr.getHeading());
        if (deg < 0) deg += 180; else deg -= 180;

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, Math.toRadians(deg));
    }

}
