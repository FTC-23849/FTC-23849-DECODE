package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.RoadrunnerFiles.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class visionToolsClean {
    public int[] rampOrder = new int[9];
    ElapsedTime timer = new ElapsedTime();
    double hueThresholdPurple = 200;
    double hueThresholdGreen = 100;
    public double correctPos = 0;
    public double TurretPowerTxDebug;
    private double smoothTx = 0;
    public boolean inRange (Limelight3A limelight){
        LLResult result = limelight.getLatestResult();
        double size = result.getTa();
        if ((size < 3.5)&(size > 1.5)){
            return true;
        }else{
            return false;
        }
    }

    public int ObeliskID(Limelight3A limelight){
        limelight.pipelineSwitch(8);
        LLResult result = limelight.getLatestResult();
        int tagID = -1;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId(); // The ID number of the Apriltag
        }
        return tagID;
    }

    public String currentColor(NormalizedColorSensor leftIntakeColorSensor,NormalizedColorSensor rightIntakeColorSensor) {

        NormalizedRGBA leftColor = leftIntakeColorSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightIntakeColorSensor.getNormalizedColors();
        double leftHue = JavaUtil.colorToHue(rightColor.toColor());
        double rightHue = JavaUtil.colorToHue(rightColor.toColor());
        if (leftHue > hueThresholdPurple && rightHue > hueThresholdPurple) {

            return "Purple";
        }else {
            if (leftHue > hueThresholdGreen && rightHue > hueThresholdGreen){
                return "Green";
            }
        }


        return "none";
    }
    public double ballsInRamp (Limelight3A limelight){
        limelight.pipelineSwitch(0);
        LLResult results = limelight.getLatestResult();
        return results.getPythonOutput()[3];
    }
    public double RampIsFull (Limelight3A limelight){
        limelight.pipelineSwitch(2);
        limelight.start();
        LLResult results = limelight.getLatestResult();
        return results.getPythonOutput()[0];
    }

    public double ballinRampOpenCVCloseZone(Mat frame) {

        Mat hsv = new Mat();
        Mat greenMask = new Mat();
        Mat purpleMask = new Mat();

        Scalar GREEN_LOWER = new Scalar(69, 111, 71);
        Scalar GREEN_UPPER = new Scalar(97, 255, 255);

        Scalar PURPLE_LOWER = new Scalar(125, 71, 101);
        Scalar PURPLE_UPPER = new Scalar(146, 232, 255);

        int RAMP_SIZE_THRESHOLD = 40;

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, greenMask);
        Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, purpleMask);

        class Detection {
            int x;
            int type;
            Detection(int x, int type) {
                this.x = x;
                this.type = type;
            }
        }

        ArrayList<Detection> detections = new ArrayList<>();

        ArrayList<MatOfPoint> greenContours = new ArrayList<>();
        Imgproc.findContours(greenMask, greenContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c : greenContours) {
            Rect r = Imgproc.boundingRect(c);
            if (r.width > RAMP_SIZE_THRESHOLD && r.height > RAMP_SIZE_THRESHOLD) {
                detections.add(new Detection(r.x, 1));
                Imgproc.rectangle(frame, r, new Scalar(0, 255, 0), 2);
            }
        }

        ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
        Imgproc.findContours(purpleMask, purpleContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c : purpleContours) {
            Rect r = Imgproc.boundingRect(c);
            if (r.width > RAMP_SIZE_THRESHOLD && r.height > RAMP_SIZE_THRESHOLD) {
                detections.add(new Detection(r.x, 2));
                Imgproc.rectangle(frame, r, new Scalar(255, 0, 255), 2);
            }
        }

        detections.sort(Comparator.comparingInt(d -> d.x));

        for (int i = 0; i < rampOrder.length; i++) {
            rampOrder[i] = 0;
        }

        int start = Math.max(0, rampOrder.length - detections.size());
        for (int i = 0; i < detections.size() && i < rampOrder.length; i++) {
            rampOrder[start + i] = detections.get(i).type;
        }

        return detections.size();
    }

    public double ballinRampOpenCVFarZone(Mat frame) {

        Mat hsv = new Mat();
        Mat greenMask = new Mat();
        Mat purpleMask = new Mat();

        Scalar GREEN_LOWER = new Scalar(69, 111, 71);
        Scalar GREEN_UPPER = new Scalar(97, 255, 255);

        Scalar PURPLE_LOWER = new Scalar(125, 71, 101);
        Scalar PURPLE_UPPER = new Scalar(146, 232, 255);

        int RAMP_SIZE_THRESHOLD = 40;

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, greenMask);
        Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, purpleMask);

        class Detection {
            int y;
            int type;
            Detection(int y, int type) {
                this.y = y;
                this.type = type;
            }
        }

        ArrayList<Detection> detections = new ArrayList<>();

        ArrayList<MatOfPoint> greenContours = new ArrayList<>();
        Imgproc.findContours(greenMask, greenContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c : greenContours) {
            Rect r = Imgproc.boundingRect(c);
            if (r.width > RAMP_SIZE_THRESHOLD && r.height > RAMP_SIZE_THRESHOLD) {
                detections.add(new Detection(r.y, 1));
                Imgproc.rectangle(frame, r, new Scalar(0, 255, 0), 2);
            }
        }

        ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
        Imgproc.findContours(purpleMask, purpleContours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint c : purpleContours) {
            Rect r = Imgproc.boundingRect(c);
            if (r.width > RAMP_SIZE_THRESHOLD && r.height > RAMP_SIZE_THRESHOLD) {
                detections.add(new Detection(r.y, 2));
                Imgproc.rectangle(frame, r, new Scalar(255, 0, 255), 2);
            }
        }

        detections.sort(Comparator.comparingInt(d -> d.y));

        for (int i = 0; i < rampOrder.length; i++) {
            rampOrder[i] = 0;
        }

        int start = Math.max(0, rampOrder.length - detections.size());
        for (int i = 0; i < detections.size() && i < rampOrder.length; i++) {
            rampOrder[start + i] = detections.get(i).type;
        }

        return detections.size();
    }

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
            double height = 0.1*(groundDistance);
            if(groundDistance > 1.25){
                height = 0.4;
            }

            return Range.clip(height,0,0.4);

        }
    }

    public double mt1pinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, Limelight3A limelight){
        limelight.pipelineSwitch(9);
        limelight.start();
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid() && result.getBotpose() != null){
            double mt1x = result.getBotpose().getPosition().x;
            double mt1y = result.getBotpose().getPosition().y;
            double mt1heading = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            pinpoint.setPosition(new Pose2D(DistanceUnit.METER,mt1x*-1,mt1y*-1,AngleUnit.DEGREES,mt1heading-180-3 /*+2 = 2 deg right*/));
            return 1;
        }else{
            return 0;
        }

    }

    public double mt2pinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, Limelight3A limelight){
        limelight.pipelineSwitch(9);
        limelight.start();
        Pose2D pose2d = pinpoint.getPosition();
        double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw+180);
        LLResult result = limelight.getLatestResult();
        double mx,my,myaw = 0;
        if(result != null && result.isValid() && result.getBotpose() != null){
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            mx = botpose_mt2.getPosition().x;
            my = botpose_mt2.getPosition().y;
            myaw = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
            //mx: 1.6364918134117126 my: -0.265005594950676
            pinpoint.setPosition(new Pose2D(DistanceUnit.METER,mx-1,my*-1,AngleUnit.DEGREES,myaw-180/*2 deg right*/));

        }
        return 0;
    }

    private double vxEstimate = 0;
    private double vyEstimate = 0;
    private double vxErrCov = 1;
    private double vyErrCov = 1;
    private final double kalmanR = 0.1;
    private final double kalmanQ = 0.01;

    private double getFilteredVelocityX(double measuredVx){
        vxErrCov += kalmanQ;
        double K = vxErrCov / (vxErrCov + kalmanR);
        vxEstimate = vxEstimate + K * (measuredVx - vxEstimate);
        vxErrCov = (1 - K) * vxErrCov;
        return vxEstimate;
    }

    private double getFilteredVelocityY(double measuredVy){
        vyErrCov += kalmanQ;
        double K = vyErrCov / (vyErrCov + kalmanR);
        vyEstimate = vyEstimate + K * (measuredVy - vyEstimate);
        vyErrCov = (1 - K) * vyErrCov;
        return vyEstimate;
    }

    public double pinpointTurretMoving(Pose2D robotPos, double xVelocity, double yVelocity, double headingVelocity, double gear,double sec, double currentPos, String alliance){
        double vx = xVelocity;
        double vy = yVelocity;
        double dist = groundDistancePinpoint(robotPos.getX(DistanceUnit.METER),robotPos.getY(DistanceUnit.METER), alliance);
        double flightTime = sec * (7.0 / 30.0) * dist + 0.05;
        double curX = robotPos.getX(DistanceUnit.METER) + (vx * flightTime);
        double curY = robotPos.getY(DistanceUnit.METER) + (vy * flightTime);

        double curYaw = robotPos.getHeading(AngleUnit.DEGREES) /*+ (flightTime * 0.3) * pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)*/;

        double goalX = 1.6288;
        double goalY = (alliance.equals("Red")) ? -1.6288 : 1.6288;

        if(dist > 3.0) {
            goalX = 1.6288;
            goalY = (alliance.equals("Blue")) ? 1.6288 : -1.6288;
        }
        if(curX > 1.2){
            goalX = 1.5288;
            goalY = (alliance.equals("Blue")) ? 1.5288 : -1.5288;
        } else if(curX > 0){
            goalX = 1.8288;
            goalY = (alliance.equals("Blue")) ? 1.8288 : -1.8288;
        }
        double turretAngle = 90 - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;

        double turretPos = 0.5 + (turretOffset * (33.0/gear) / 1800.0);

        return Range.clip(turretPos, 0.25, 0.625);
    }
    public double FlywheelSpeedRegressor(Pose2D robotPos, double xVelocity, double yVelocity,double moveAwayAdjustment, double sec, double currentVelocity, String allianceColor) {

        //double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
        //double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));
        double vx = xVelocity;
        double vy = yVelocity;

        double px = robotPos.getX(DistanceUnit.METER);
        double py = robotPos.getY(DistanceUnit.METER);
        double currentDist = groundDistancePinpoint(px,py,allianceColor);
        double flightTime = sec * (7.0 / 30.0) * currentDist + 0.05;
        double ax = px+vx*flightTime;
        double ay = py+vy*flightTime;
        double estimatedDist = groundDistancePinpoint(ax,ay,allianceColor);

        double x = estimatedDist;
        double x2 = x * x;
        double x3 = x2 * x;

        double speed = 6.1716 * x3
                - 106.8394 * x2
                + 199.2194 * x
                - 1208.2518;

        if(estimatedDist<2.0){
            speed -= 30;
        }
        if (currentDist == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
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