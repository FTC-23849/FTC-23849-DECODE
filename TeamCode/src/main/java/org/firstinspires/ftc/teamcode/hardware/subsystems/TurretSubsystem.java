package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;

@Config
public class TurretSubsystem {
    private final RobotClass robot;
    public ServoImplEx leftTurretServo, rightTurretServo, frontTurretServo;
    public AnalogInput turretEncoder;

    public static double turretZeroCorrection = -0.009;
    public static double turretCorrection = 0.004;
    public static double gear = 13;
    public static double offsetMultiplier = 0.8;
    public static double trueZero = 318;
    public static double quadratureZero = 0;

    public TurretSubsystem(RobotClass robot) {
        this.robot = robot;
    }

    public void init() {
        leftTurretServo = robot.hardwareMap.get(ServoImplEx.class, "leftTurretServo");
        rightTurretServo = robot.hardwareMap.get(ServoImplEx.class, "rightTurretServo");
        frontTurretServo = robot.hardwareMap.get(ServoImplEx.class, "frontTurretServo");
        turretEncoder = robot.hardwareMap.get(AnalogInput.class, "turretEncoder");

        leftTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        frontTurretServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void start() {
        setTurretPosition(0.5 + turretZeroCorrection);
    }

    public void update() {
    }

    public void adjustTurretCorrection(double correction) {
        if (leftTurretServo.getPosition() < 0.764) turretCorrection += correction;
    }


    public void zeroQuadrature() {
        quadratureZero = robot.drive.rightFront.getCurrentPosition();
        calibrateTurret();
    }

    public void autoCorrectFromQuadrature() {
        double currentQuadratureAngle = robot.drive.rightFront.getCurrentPosition();
        double targetServoPosition = rightTurretServo.getPosition();
        double ticksPerTurretDegree = 4096.0 / 360.0;
        double encoderToTurretRatio = 19.0 / 99.0;
        double quadratureAngleDeg = ((currentQuadratureAngle - quadratureZero) / ticksPerTurretDegree) * encoderToTurretRatio;
        double predictedServoPosition = 0.5 + (quadratureAngleDeg / 360.0) * 0.508 + turretZeroCorrection;
        double servoCorrection = (targetServoPosition - predictedServoPosition) * offsetMultiplier;

        turretZeroCorrection += servoCorrection;
    }

    public void calibrateTurretAndRelocalize() {
        calibrateTurret();
        robot.vision.mt1pinpoint(robot.miscSubsystems.allianceColor);
    }

    public void toggleUsingTurret() {
        robot.shooter.useTurret = !robot.shooter.useTurret;
        if (!robot.shooter.useTurret) {
            setTurretPosition(0.5 + turretZeroCorrection);
        }
    }

    public void updateTracking() {
        if (robot.shooter.backButtonTrue && robot.shooter.useTurret) {
            Pose2D predictedPos = robot.drive.getTurretPredictedPos();
            double turretPos = robot.vision.getTurretAngle(predictedPos, robot.miscSubsystems.allianceColor, leftTurretServo.getPosition(), gear);
            setTurretPosition(turretPos);
        } else {
            setTurretPosition(0.5 + turretZeroCorrection);
        }
    }

    public void setTurretPosition(double pos) {
        leftTurretServo.setPosition(pos);
        rightTurretServo.setPosition(pos);
        frontTurretServo.setPosition(pos);
    }

    public void calibrateTurret() {
        double currentAngle = (turretEncoder.getVoltage() / 3.2 * 360) % 360;
        double EncoderToTurretRatio = 19.0/99.0;
        double encoderOffsetFromZero = trueZero - currentAngle;
        double turretOffsetFromZero = (encoderOffsetFromZero * EncoderToTurretRatio);
        double servoOffsetFromZero = 0.508 * (turretOffsetFromZero/360);
        servoOffsetFromZero *= offsetMultiplier;
        turretZeroCorrection += servoOffsetFromZero;
        setTurretPosition(0.5 + turretZeroCorrection);
    }

    public void telemetry() {
        robot.telemetry.addData("Turret Correction", turretCorrection);
        robot.telemetry.addData("Turret Zero Correction", turretZeroCorrection);
    }
}
