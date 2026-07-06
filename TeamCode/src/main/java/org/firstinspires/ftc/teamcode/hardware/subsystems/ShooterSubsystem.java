package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;
import org.firstinspires.ftc.teamcode.opmode.misc.PIDVelocityController3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterSubsystem {
    private final RobotClass robot;
    public DcMotorEx leftShooterMotor, rightShooterMotor;
    public ServoImplEx leftHood, rightHood;
    public PIDVelocityController3 velocityPID;

    public double targetVelocity = 0;
    public double flywheelCorrection = 0;
    public boolean backButtonTrue = false;
    public boolean usePower = true;
    public boolean useTurret = true;

    public double shotSpeed = 0.9;
    public static double defaultShotSpeed = 0.9;
    public static double lowBatteryShotSpeed = 0.7;
    public static double lockedPosShotSpeed = 0.7;
    public static double moveTowardsGoalshotSpeed = 0.85;
    public static double moveAwayFromGoalshotSpeed = 0.75;

    public static double KpMaintain = 0.003;
    public static double KiMaintain = 0.002;
    public static double KpDriveRecovery = 0.03;
    public static double KpRecovery = 0;
    public static double KiRecovery = 0.001;
    public static double KsRecovery = 0.8;
    public static double KvFF = 0.00042;
    public static double KsFF = 0.055;

    public static class Point {
        public double x;
        public double y;

        public Point() {
        }

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static double flywheelZoneThreshold1 = 1.9;
    public static double flywheelZoneThreshold2 = 2.75;
    public static int flywheelNearDegree = 2;
    public static int flywheelMidDegree = 2;
    public static int flywheelFarDegree = 4;

    //tunable points for regression
    public static Point flywheelPoint1 = new Point(0.9, -559);
    public static Point flywheelPoint2 = new Point(1.15, -650);
    public static Point flywheelPoint3 = new Point(1.4, -737);
    public static Point flywheelPoint4 = new Point(1.65, -820);
    public static Point flywheelPoint5 = new Point(1.9, -899);
    public static Point flywheelPoint6 = new Point(2.05, -842);
    public static Point flywheelPoint7 = new Point(2.2, -879);
    public static Point flywheelPoint8 = new Point(2.4, -924);
    public static Point flywheelPoint9 = new Point(2.6, -964);
    public static Point flywheelPoint10 = new Point(2.75, -991);
    public static Point flywheelPoint11 = new Point(2.8, -1003);
    public static Point flywheelPoint12 = new Point(3.0, -1024);
    public static Point flywheelPoint13 = new Point(3.25, -1073);
    public static Point flywheelPoint14 = new Point(3.5, -1136);
    public static Point flywheelPoint15 = new Point(3.75, -1203);
    public static Point flywheelPoint16 = new Point(4.0, -1268);

    private double[] lastFlywheelRegressionSignature = null;
    private PolynomialFunction nearFlywheelPoly, midFlywheelPoly, farFlywheelPoly;
    public double nearFlywheelR2, midFlywheelR2, farFlywheelR2;

    public ShooterSubsystem(RobotClass robot) {
        this.robot = robot;
    }

    public void init() {
        leftShooterMotor = robot.hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightShooterMotor = robot.hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftHood = robot.hardwareMap.get(ServoImplEx.class, "leftHood");
        rightHood = robot.hardwareMap.get(ServoImplEx.class, "rightHood");
        leftHood.setDirection(ServoImplEx.Direction.REVERSE);

        velocityPID = new PIDVelocityController3(0, KpMaintain, KiMaintain, KpRecovery, KiRecovery, KsRecovery, KsFF, KvFF);
    }

    public void update() {
        updateShotSpeed();
    }

    private void updateShotSpeed() {
        Pose2D robotPos = robot.drive.getRobotPos();
        Pose2D predictedPos = robot.drive.getPredictedPos();
        double predDistance = Math.sqrt(
                Math.pow(predictedPos.getX(DistanceUnit.CM) - robotPos.getX(DistanceUnit.CM), 2) +
                        Math.pow(predictedPos.getY(DistanceUnit.CM) - robotPos.getY(DistanceUnit.CM), 2));

        if (predDistance > 10) {
            double goalX = robot.miscSubsystems.allianceColor.equals("Red") ? -165 : 165;
            double goalY = 0;

            double actualDist = Math.sqrt(
                    Math.pow(robotPos.getX(DistanceUnit.CM) - goalX, 2) +
                            Math.pow(robotPos.getY(DistanceUnit.CM) - goalY, 2));

            double predictedDist = Math.sqrt(
                    Math.pow(predictedPos.getX(DistanceUnit.CM) - goalX, 2) +
                            Math.pow(predictedPos.getY(DistanceUnit.CM) - goalY, 2));

            shotSpeed = predictedDist > actualDist ? moveAwayFromGoalshotSpeed : moveTowardsGoalshotSpeed;
        } else if (robot.miscSubsystems.lowVoltage) {
            shotSpeed = lowBatteryShotSpeed;
        } else if (robot.drive.lockedModeEnabled) {
            shotSpeed = lockedPosShotSpeed;
        } else {
            shotSpeed = defaultShotSpeed;
        }
    }

    private Point[] flywheelPoints() {
        return new Point[]{
                flywheelPoint1, flywheelPoint2, flywheelPoint3, flywheelPoint4,
                flywheelPoint5, flywheelPoint6, flywheelPoint7, flywheelPoint8,
                flywheelPoint9, flywheelPoint10, flywheelPoint11, flywheelPoint12,
                flywheelPoint13, flywheelPoint14, flywheelPoint15, flywheelPoint16
        };
    }

    private double[] flywheelRegressionSignature() {
        Point[] points = flywheelPoints();
        double[] signature = new double[points.length * 2 + 5];
        int i = 0;
        for (Point point : points) {
            signature[i++] = point.x;
            signature[i++] = point.y;
        }
        signature[i++] = flywheelZoneThreshold1;
        signature[i++] = flywheelZoneThreshold2;
        signature[i++] = flywheelNearDegree;
        signature[i++] = flywheelMidDegree;
        signature[i++] = flywheelFarDegree;
        return signature;
    }

    private PolynomialFunction fitZone(List<Point> zonePoints, int configuredDegree) {
        if (zonePoints.size() < 2) return null;

        int degree = Range.clip(configuredDegree, 1, 4);
        degree = Math.min(degree, zonePoints.size() - 1);

        WeightedObservedPoints observations = new WeightedObservedPoints();
        for (Point point : zonePoints) observations.add(point.x, point.y);

        double[] coefficients = PolynomialCurveFitter.create(degree).fit(observations.toList());
        return new PolynomialFunction(coefficients);
    }

    private double computeR2(List<Point> zonePoints, PolynomialFunction poly) {
        if (poly == null || zonePoints.isEmpty()) return Double.NaN;

        double meanY = 0;
        for (Point point : zonePoints) meanY += point.y;
        meanY /= zonePoints.size();

        double ssRes = 0;
        double ssTot = 0;
        for (Point point : zonePoints) {
            double residual = point.y - poly.value(point.x);
            ssRes += residual * residual;
            double diff = point.y - meanY;
            ssTot += diff * diff;
        }

        return ssTot == 0 ? 1 : 1 - (ssRes / ssTot);
    }

    private void refitFlywheelRegressionIfNeeded() {
        double[] signature = flywheelRegressionSignature();
        if (Arrays.equals(signature, lastFlywheelRegressionSignature)) return;
        lastFlywheelRegressionSignature = signature;

        List<Point> nearZone = new ArrayList<>();
        List<Point> midZone = new ArrayList<>();
        List<Point> farZone = new ArrayList<>();
        for (Point point : flywheelPoints()) {
            if (point.x <= flywheelZoneThreshold1) nearZone.add(point);
            else if (point.x <= flywheelZoneThreshold2) midZone.add(point);
            else farZone.add(point);
        }

        nearFlywheelPoly = fitZone(nearZone, flywheelNearDegree);
        midFlywheelPoly = fitZone(midZone, flywheelMidDegree);
        farFlywheelPoly = fitZone(farZone, flywheelFarDegree);

        nearFlywheelR2 = computeR2(nearZone, nearFlywheelPoly);
        midFlywheelR2 = computeR2(midZone, midFlywheelPoly);
        farFlywheelR2 = computeR2(farZone, farFlywheelPoly);
    }

    private double calculateFlywheelSpeed(double distance) {
        refitFlywheelRegressionIfNeeded();

        PolynomialFunction poly = distance <= flywheelZoneThreshold1 ? nearFlywheelPoly
                : distance <= flywheelZoneThreshold2 ? midFlywheelPoly
                : farFlywheelPoly;

        return poly == null ? 0 : poly.value(distance);
    }

    public void adjustFlywheelCorrection(double delta) {
        flywheelCorrection += delta;
    }

    public void resetFlywheelCorrection() {
        flywheelCorrection = 0;
    }

    public void toggleShooter() {
        backButtonTrue = !backButtonTrue;
    }

    public void togglePowerMode() {
        usePower = !usePower;
    }

    public void updateFlywheel(Pose2D predictedPos, double voltage) {
        if (backButtonTrue) {
            double flywheelCurrentVelocity = (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2.0;
            
            if (!usePower) {
                targetVelocity = -920 + flywheelCorrection;
                leftHood.setPosition(0.15);
                rightHood.setPosition(0.15);
            } else {
                double distance = robot.vision.getTurretDistance(predictedPos, robot.miscSubsystems.allianceColor);
                targetVelocity = calculateFlywheelSpeed(distance) + flywheelCorrection;
                double hoodHeight = robot.vision.getHoodHeight(predictedPos, leftHood.getPosition(), robot.miscSubsystems.allianceColor);
                leftHood.setPosition(hoodHeight);
                rightHood.setPosition(hoodHeight);
            }

            velocityPID.setTargetVelocity(targetVelocity);
            velocityPID.setMaintainGains(KpMaintain, KiMaintain, KpDriveRecovery);
            velocityPID.setRecoveryGains(KpRecovery, KiRecovery, KsRecovery);
            velocityPID.setFeedforward(KsFF, KvFF);
            velocityPID.setVoltageStatus(robot.miscSubsystems.lowVoltage);
            
            double power = velocityPID.update(flywheelCurrentVelocity, voltage, 13.15);
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
        } else {
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }
    }

    public void stop() {
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
    }

    public void telemetry() {
        robot.telemetry.addData("Shooter Target", targetVelocity);
        robot.telemetry.addData("Shooter Actual", (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2.0);
        robot.telemetry.addData("Flywheel Near R^2", nearFlywheelR2);
        robot.telemetry.addData("Flywheel Mid R^2", midFlywheelR2);
        robot.telemetry.addData("Flywheel Far R^2", farFlywheelR2);
    }
}
