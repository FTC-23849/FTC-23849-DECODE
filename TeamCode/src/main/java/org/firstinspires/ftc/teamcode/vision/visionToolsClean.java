package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;

public class visionToolsClean {
    public int[] rampOrder = new int[9];
    ElapsedTime timer = new ElapsedTime();
    double hueThresholdPurple = 200;
    double hueThresholdGreen = 100;
    public double correctPos = 0;
    public double TurretPowerTxDebug;
    private double smoothTx = 0;


    public double groundDistancePinpoint(double x, double y, String allianceColor){
        //method overrload
        double groundDistance = 0;
        if (allianceColor.equals("Red")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((-1.8288 - y) * (-1.8288 - y)));
        }if (allianceColor.equals("Blue")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((1.8288 - y) * (1.8288 - y)));
        }
        return  groundDistance;
    }

    public double groundDistancePinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        double x = pinpoint.getPosition().getX(DistanceUnit.METER);
        double y = pinpoint.getPosition().getY(DistanceUnit.METER);
        double groundDistance = 0;
        if (allianceColor.equals("Red")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((-1.8288 - y) * (-1.8288 - y)));
        }if (allianceColor.equals("Blue")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((1.8288 - y) * (1.8288 - y)));
        }
        return  groundDistance ;
    }




    public double hoodHeightRegressor(Pose2D robotPos, double currentHood, String allianceColor){
        double groundDistance = groundDistancePinpoint(robotPos.getX(DistanceUnit.METER),robotPos.getY(DistanceUnit.METER),allianceColor);
        if (groundDistance == -1) {
            return currentHood;
        }else{
            double height = 0;
            if(groundDistance > 1.9){
                height = 0.15;
            }
            if(groundDistance > 2.77){
                height = 0.28;
            }


            return Range.clip(height,0,0.4);

        }
    }

    public double mt1pinpoint(double red,double blue,org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint,String alliance, Limelight3A limelight){
        limelight.pipelineSwitch(9);
        limelight.start();
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid() && result.getBotpose() != null){
            double mt1x = result.getBotpose().getPosition().x;
            double mt1y = result.getBotpose().getPosition().y;
            double mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            if(alliance.equals("Blue")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.METER,mt1x*-1,mt1y*-1,AngleUnit.DEGREES,mt1heading-180+blue /*+2 = 2 deg right*/));
            }else {
                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, mt1x * -1, mt1y * -1, AngleUnit.DEGREES, mt1heading - 180 + red /*+2 = 2 deg right*/));
            }
            return 1;
        }else{
            return 0;
        }

    }

    public double mt1pinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint,String alliance, Limelight3A limelight){
        limelight.pipelineSwitch(9);
        limelight.start();
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid() && result.getBotpose() != null){
            double mt1x = result.getBotpose().getPosition().x;
            double mt1y = result.getBotpose().getPosition().y;
            double mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            if(alliance.equals("Blue")){
                pinpoint.setPosition(new Pose2D(DistanceUnit.METER,mt1x*-1,mt1y*-1,AngleUnit.DEGREES,mt1heading-180 /*+2 = 2 deg right*/));
            }else {
                pinpoint.setPosition(new Pose2D(DistanceUnit.METER, mt1x * -1, mt1y * -1, AngleUnit.DEGREES, mt1heading - 180 /*+2 = 2 deg right*/));
            }
            return 1;
        }else{
            return 0;
        }

    }
    public double mt2pinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, Limelight3A limelight){
        limelight.pipelineSwitch(9);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        double mx,my,myaw = 0;
        if(result != null && result.isValid() && result.getBotpose() != null){
            double mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            limelight.updateRobotOrientation(mt1heading);
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            mx = botpose_mt2.getPosition().x;
            my = botpose_mt2.getPosition().y;
            myaw = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            //mx: 1.6364918134117126 my: -0.265005594950676
            pinpoint.setPosition(new Pose2D(DistanceUnit.METER,mx*-1,my*-1,AngleUnit.DEGREES,myaw-180/*2 deg right*/));

        }
        return 0;
    }

    private double vxEstimate = 0;
    private double vyEstimate = 0;
    private double vxErrCov = 1;
    private double vyErrCov = 1;
    private final double kalmanR = 0.1;
    private final double kalmanQ = 0.01;

    private double lastX;
    private double lastY;
    private double lastTime;

    private double axEstimate = 0;
    private double ayEstimate = 0;
    private double accelErrCovX = 1;
    private double accelErrCovY = 1;

    private final double accelAlpha = 0.25;
    private double lastVx = 0;
    private double lastVy = 0;

    private ElapsedTime accelTimer = new ElapsedTime();

    public double[] calculateVelocity(double currentX, double currentY, double currentTime) {
        double dt = currentTime - lastTime;

        double vx =getFilteredVelocityX  ((currentX - lastX) / dt,0.1,0.01);
        double vy = getFilteredVelocityY  ((currentY - lastY) / dt,0.1,0.01);

        lastX = currentX;
        lastY = currentY;
        lastTime = currentTime;

        return new double[]{vx, vy};
    }

    public double[] calculateAcceleration(double vx, double vy) {
        double dt = accelTimer.seconds();
        accelTimer.reset();

        if(dt == 0){
            dt = 0.001;
        }

        double axRaw = (vx - lastVx) / dt;
        double ayRaw = (vy - lastVy) / dt;

        axEstimate = axEstimate + accelAlpha * (axRaw - axEstimate);
        ayEstimate = ayEstimate + accelAlpha * (ayRaw - ayEstimate);

        lastVx = vx;
        lastVy = vy;

        return new double[]{axEstimate, ayEstimate};
    }

    public double getFilteredVelocityX(double measuredVx,double kalmanQ, double kalmanR){
        vxErrCov += kalmanQ;
        double K = vxErrCov / (vxErrCov + kalmanR);
        vxEstimate = vxEstimate + K * (measuredVx - vxEstimate);
        vxErrCov = (1 - K) * vxErrCov;
        return vxEstimate;
    }

    public double getFilteredVelocityY(double measuredVy,double kalmanQ, double kalmanR){
        vyErrCov += kalmanQ;
        double K = vyErrCov / (vyErrCov + kalmanR);
        vyEstimate = vyEstimate + K * (measuredVy - vyEstimate);
        vyErrCov = (1 - K) * vyErrCov;
        return vyEstimate;
    }
    public double CalculateTurretAngle360NEW(
            double xcoeff,
            double ycoeff,
            Pose2D robotPos,
            double gear,
            double currentPos,
            double turretZero,
            double turretZero2,
            String alliance
    ) {
        double yPos = robotPos.getX(DistanceUnit.METER);
        double xPos = -1*robotPos.getY(DistanceUnit.METER);

        double dist = groundDistancePinpoint(xPos, yPos, alliance);

        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = heading + 90;
        if (adjustedHeading < 0) adjustedHeading += 360;

        double robotX = xPos;
        double robotY = yPos;
        double turretOffsetMeters = -0.0765;

        double sinH = Math.sin(Math.toRadians(adjustedHeading));
        double cosH = Math.cos(Math.toRadians(adjustedHeading));

        double curX = robotX + turretOffsetMeters * (xcoeff * cosH - ycoeff * sinH);
        double curY = robotY + turretOffsetMeters * (xcoeff * sinH + ycoeff * cosH);


        double goalY = 1.8288;
        double goalXRed = 1.8288;
        double goalXBlue = -1.8288;
        if(yPos > 0){
            goalY = 1.6788;
        }
        double goalX = alliance.equals("Red") ? goalXRed : goalXBlue;
        double startingAngle = (180+adjustedHeading)%360;
        double turretAngle = startingAngle - Math.toDegrees(Math.atan2(goalY - curY,goalX - curX));
        turretAngle = ((turretAngle + 180) % 360) - 180;
        //more for left, less for right (turretZero)
        double turretCenter = 0.5+turretZero;
        double ticksPerDegree = (33.0 / gear) / 1800.0;
        double turretPos = turretCenter - turretAngle * ticksPerDegree;
        if(turretPos<0.5){
            turretPos-=turretZero;
            turretPos+=turretZero2;
        }
        return Range.clip(turretPos, 0.246, 0.764 );//Range.clip(turretPos, 0.23, 0.65);
    }
    public double CalculateTurretAngle360(
            double moveAway,
            double xcoeff,
            double ycoeff,
            Pose2D robotPos,
            double xVelocity,
            double yVelocity,
            double headingVelocity,
            double gear,
            double sec,
            double currentPos,
            String alliance
    ) {
        double yPos = robotPos.getX(DistanceUnit.METER);
        double xPos = -1*robotPos.getY(DistanceUnit.METER);

        double dist = groundDistancePinpoint(xPos, yPos, alliance);

        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = heading + 90;
        if (adjustedHeading < 0) adjustedHeading += 360;

        double robotX = xPos;
        double robotY = yPos;
        double turretOffsetMeters = -0.0765;

        double sinH = Math.sin(Math.toRadians(adjustedHeading));
        double cosH = Math.cos(Math.toRadians(adjustedHeading));

        double curX = robotX + xcoeff * sinH * turretOffsetMeters;
        double curY = robotY + ycoeff * cosH * turretOffsetMeters;

        double curYaw = heading;

        double goalY = 1.8288;
        double goalXRed = 1.8288;
        double goalXBlue = -1.8288;
        double goalX = alliance.equals("Red") ? goalXRed : goalXBlue;
        double startingAngle = 270;
        double turretAngle = startingAngle - Math.toDegrees(Math.atan2(goalY - curY,goalX - curX));
        double turretOffset = turretAngle - curYaw;
        turretOffset = ((turretOffset + 180) % 360) - 180;//5 deg overlap

        double turretCenter = 0.5;
        double ticksPerDegree = (33.0 / gear) / 1800.0;

        double turretPos = turretCenter + turretOffset * ticksPerDegree;

        return Range.clip(turretPos, 0.25, 0.75);//Range.clip(turretPos, 0.23, 0.65);
    }
    //OLD CLASS:
//    public double CalculateTurretAngle360(
//            double moveAway,
//            double xcoeff,
//            double ycoeff,
//            Pose2D robotPos,
//            double xVelocity,
//            double yVelocity,
//            double headingVelocity,
//            double gear,
//            double sec,
//            double currentPos,
//            String alliance
//    ) {
//        double xPos = robotPos.getX(DistanceUnit.METER);
//        double yPos = robotPos.getY(DistanceUnit.METER);
//
//        double dist = groundDistancePinpoint(xPos, yPos, alliance);
//
//        double heading = robotPos.getHeading(AngleUnit.DEGREES);
//        double adjustedHeading = heading + 90;
//        if (adjustedHeading < 0) adjustedHeading += 360;
//
//        double robotX = xPos;
//        double robotY = yPos;
//        double turretOffsetMeters = -0.0765;
//
//        double sinH = Math.sin(Math.toRadians(adjustedHeading));
//        double cosH = Math.cos(Math.toRadians(adjustedHeading));
//
//        double curX = robotX + xcoeff * sinH * turretOffsetMeters;
//        double curY = robotY + ycoeff * cosH * turretOffsetMeters;
//
//        double curYaw = heading;
//
//        double goalX = 1.8288;
//        double goalYRed = -1.8288;
//        double goalYBlue = 1.8288;
//        double goalY = alliance.equals("Red") ? goalYRed : goalYBlue;
//        double startingAngle = 270;
//        double turretAngle = startingAngle - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
//        double turretOffset = turretAngle - curYaw;
//        turretOffset = ((turretOffset + 185) % 370) - 185;//5 deg overlap
//
//        double turretCenter = 0.5;
//        double ticksPerDegree = (33.0 / gear) / 1800.0;
//
//        double turretPos = turretCenter + turretOffset * ticksPerDegree;
//
//        return Range.clip(turretPos, 0.237, 0.763);//Range.clip(turretPos, 0.23, 0.65);
//    }

    public double TurretAngle(
            double moveAway,
            double xcoeff,
            double ycoeff,
            Pose2D robotPos,
            double xVelocity,
            double yVelocity,
            double headingVelocity,
            double gear,
            double sec,
            double currentPos,
            String alliance
    ) {
        double xPos = robotPos.getX(DistanceUnit.METER);
        double yPos = robotPos.getY(DistanceUnit.METER);

        double dist = groundDistancePinpoint(xPos, yPos, alliance);

        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = heading + 90;
        if (adjustedHeading < 0) adjustedHeading += 360;

        double robotX = xPos;
        double robotY = yPos;
        double turretOffsetMeters = -0.0765;

        double sinH = Math.sin(Math.toRadians(adjustedHeading));
        double cosH = Math.cos(Math.toRadians(adjustedHeading));

        double curX = robotX + xcoeff * sinH * turretOffsetMeters;
        double curY = robotY + ycoeff * cosH * turretOffsetMeters;

        double curYaw = heading;

        double goalX = 1.8288;
        double goalYRed = -1.8288;
        double goalYBlue = 1.8288;
        double goalY = alliance.equals("Red") ? goalYRed : goalYBlue;
        double startingAngle = 90;
        double turretAngle = startingAngle - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;

        double turretCenter = 0.5;
        double ticksPerDegree = (33.0 / gear) / 1800.0;

        double turretPos = turretCenter + turretOffset * ticksPerDegree;

        return Range.clip(turretPos, 0.23, 0.65);
    }

    public double CalculatedFlywheelSpeed(Pose2D robotPos, String allianceColor) {
        double px = robotPos.getX(DistanceUnit.METER);
        double py = robotPos.getY(DistanceUnit.METER);
        double currentDist = groundDistancePinpoint(px,py,allianceColor);
        double offset = 0.0508;
        if(currentDist < 1.9) {
            offset = 0.0254;
        }
        double adjustedHeading =  robotPos.getHeading(AngleUnit.DEGREES)+ 90;
        if(adjustedHeading < 0 ){ adjustedHeading+= 360; }
        currentDist = groundDistancePinpoint(px,py,allianceColor);

        double x = currentDist;
        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x2 * x2;
        double x5 = x2 * x3;
        double x6 = x3 * x3;
        double speed = 0;
        if(currentDist < 1.9) {
            speed = -14808.14168
                    + 37374.67489 * x
                    - 36462.94734 * x2
                    + 15555.54901 * x3
                    - 2469.13478 * x4;

        }else if(currentDist< 2.75){
            speed = 7895.76427
                    - 10649.81464 * x
                    + 4307.95413 * x2
                    - 586.52521 * x3;
        }else{
            //−19.9936x3+191.7677x2−853.9049x−213.5501

            speed = -110300.75593
                    + 188135.14001 * x
                    - 133074.55282 * x2
                    + 49553.07366 * x3
                    - 10257.33467 * x4
                    + 1119.53079 * x5
                    - 50.36368 * x6;

        }
        return speed;
    }

    public Pose2D predictPos(String allianceColor,Pose2D currPos, double xVel, double yVel, double hVel,double sec,double turretSec, double moveAway){
        double predX = xVel*sec + currPos.getX(DistanceUnit.MM);
        double predY = yVel*sec + currPos.getY(DistanceUnit.MM);
        double currDist = groundDistancePinpoint(currPos.getX(DistanceUnit.METER),currPos.getY(DistanceUnit.METER),allianceColor);
        double predDist = groundDistancePinpoint(predX/1000,predY/1000,allianceColor);
        if(predDist>currDist){
            predX = xVel*moveAway + currPos.getX(DistanceUnit.MM);
            predY = yVel*moveAway + currPos.getY(DistanceUnit.MM);
        }
        double predH = hVel*turretSec + currPos.getHeading(AngleUnit.DEGREES);
        Pose2D predictedPos = new Pose2D(DistanceUnit.MM,predX,predY,AngleUnit.DEGREES,predH);
        return predictedPos;
    }
    public Pose2D RRtoPinpoint(MecanumDrive drive) {
        Pose2d pose = drive.localizer.getPose();
        double x = -pose.position.x;
        double y = -pose.position.y;
        double deg = Math.toDegrees(pose.heading.toDouble()); // raw RR heading (-180..180)

        if (deg < 0){
            deg+=180;
        }else{
            deg-=180;
        }
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, Math.toRadians(deg));
    }
    public double pinpointTurretMovingAUTO(Pose2D pose2d,double gear,double sec, double currentPos, String alliance){

        //double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
        //double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));

        double dist = groundDistancePinpoint(pose2d.getX(DistanceUnit.METER),
                pose2d.getY(DistanceUnit.METER),alliance);
        double flightTime = sec * dist;

        double curX = pose2d.getX(DistanceUnit.METER) /*+ (vx * flightTime)*/;
        double curY = pose2d.getY(DistanceUnit.METER) /*+ (vy * flightTime)*/;

        double curYaw = pose2d.getHeading(AngleUnit.DEGREES) /*+ (flightTime * 0.3) * pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)*/;

        double goalX = 1.8288;
        double goalY = (alliance.equals("Red")) ? -1.8288 : 1.8288;


        double turretAngle = 90 - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;

        double turretPos = 0.5 + (turretOffset * (33.0/gear) / 1800.0);

        return Range.clip(turretPos, 0.25, 0.625);
    }

    public double hoodHeightRegressorAUTO(Pose2D pose, double currentHood, String allianceColor){
        double groundDistance = groundDistancePinpoint(pose.getX(DistanceUnit.METER),
                pose.getY(DistanceUnit.METER),allianceColor);
        if (groundDistance == -1) {
            return currentHood;
        }else{
            double height = 0.1*(groundDistance);
            if(groundDistance > 1.25){
                height = 0.4;
            }

            return Range.clip(height,0,0.4);

        }


    }

    public double FlywheelSpeedRegressorAUTO(Pose2D pose, double moveAwayAdjustment, double sec,
                                             double currentVelocity, String allianceColor){

//        double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
//        double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));


//        double flightTime = sec * currentDist;
//
//        double ax = pinpoint.getPosX(DistanceUnit.METER) + (vx * flightTime);
//        double ay = pinpoint.getPosY(DistanceUnit.METER) + (vy * flightTime);

        double px = pose.getX(DistanceUnit.METER);
        double py = pose.getY(DistanceUnit.METER);

        //double groundDistance = groundDistancePinpoint(ax, ay, allianceColor);
        double groundDistance = groundDistancePinpoint(px, py, allianceColor);

//        if (groundDistance > oldGroundDistance) {
//            double adjustedSec = sec + moveAwayAdjustment;
//            double extendedFlightTime = adjustedSec * currentDist;
//            ax = pinpoint.getPosX(DistanceUnit.METER) + vx * extendedFlightTime;
//            ay = pinpoint.getPosY(DistanceUnit.METER) + vy * extendedFlightTime;
//            groundDistance = groundDistancePinpoint(ax, ay, allianceColor);
//        }

        double x = groundDistance;
        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;
        double x5 = x4 * x;
        double x6 = x5 * x;
        double speed = -1.8411 * x3
                + 29.2526 * x2
                - 344.1536 * x
                - 962.8227;

        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
        //Update
    }


}