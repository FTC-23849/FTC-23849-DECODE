package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;
import org.firstinspires.ftc.teamcode.vision.visionToolsClean;

@Config
public class VisionSubsystem {
    private final RobotClass robot;
    public Limelight3A limelight;
    public visionToolsClean visionTools;

    public static double moveAwayFar = 1.2;
    public static double moveAwayTurretFar = 1.4;
    public static double secFar = 0.8;
    public static double turretSecFar = 1.1;

    public static double moveAwayClose = 2;
    public static double moveAwayTurretClose = 0.6;
    public static double secClose = 0.6;
    public static double turretSecClose = 0.8;

    public static double rotationalSec = 0.1;

    public static double red = 0;
    public static double blue = 0;

    public static double xcoeff = 1;
    public static double ycoeff = 1;

    public boolean SOTM = false;
    public static boolean SOTMTesting = false;

    public VisionSubsystem(RobotClass robot) {
        this.robot = robot;
        this.visionTools = new visionToolsClean();
    }

    public void init() {
        limelight = robot.hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(9);
        limelight.setPollRateHz(15);
        limelight.start();
    }

    public void update() {
        //placeholder, nothing here rn
    }

    public Pose2D predictPos(String alliance, Pose2D robotPos, double velX, double velY, double velH) {
        boolean isFarZone = robotPos.getX(DistanceUnit.MM) <= 0;
        double moveAway = isFarZone ? moveAwayFar : moveAwayClose;
        double sec = isFarZone ? secFar : secClose;

        return visionTools.predictPos(alliance, robotPos, velX, velY, velH, sec, rotationalSec, moveAway);
    }

    public Pose2D getTurretPredictedPos(String alliance, Pose2D robotPos, double velX, double velY, double velH) {
        boolean isFarZone = robotPos.getX(DistanceUnit.MM) <= 0;
        double moveAwayTurret = isFarZone ? moveAwayTurretFar : moveAwayTurretClose;
        double turretSec = isFarZone ? turretSecFar : turretSecClose;

        return visionTools.predictPos(alliance, robotPos, velX, velY, velH, turretSec, rotationalSec, moveAwayTurret);
    }

    public void updateSOTM(boolean leftBumperHeld) {
        SOTM = leftBumperHeld || SOTMTesting;

        if (leftBumperHeld) {
            moveAwayFar = 1.2;
            moveAwayTurretFar = 1.4;
            secFar = 0.8;
            turretSecFar = 1.1;
            rotationalSec = 0.1;
            moveAwayClose = 2;
            moveAwayTurretClose = 0.6;
            secClose = 0.6;
            turretSecClose = 0.8;
        } else {
            moveAwayFar = 0;
            moveAwayTurretFar = -0.2;
            secFar = 0;
            turretSecFar = -0.2;
            rotationalSec = 0;
            moveAwayClose = 0;
            moveAwayTurretClose = -0.2;
            secClose = 0;
            turretSecClose = -0.2;
        }
    }

    public void mt1pinpoint(String allianceColor) {
        visionTools.mt1pinpoint(red, blue, robot.drive.pinpoint, allianceColor, limelight);
    }

    public double getTurretDistance(Pose2D predictedPos, String alliance) {
        return visionTools.getTurretDistance(predictedPos, alliance);
    }

    public double getHoodHeight(Pose2D predictedPos, double currentPos, String alliance) {
        return visionTools.hoodHeightRegressor(predictedPos, currentPos, alliance);
    }

    public double getTurretAngle(Pose2D predictedPos, String alliance, double currentPos, double gear) {
        //TODO: clean ts up and js make it one offset
        return visionTools.CalculateTurretAngle360NEW(xcoeff, ycoeff, predictedPos, gear, currentPos,
                TurretSubsystem.turretZeroCorrection, alliance) + TurretSubsystem.turretCorrection;
    }
}
