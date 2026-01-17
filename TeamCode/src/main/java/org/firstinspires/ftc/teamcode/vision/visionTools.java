package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Thread.sleep;

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

public class visionTools {
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
    public double adjustedTurretAngle(double currentAngle, Limelight3A limelight,double zone) {
        double correction = 0;
        limelight.pipelineSwitch(9);
        double errorMargin = 2;
        double smoothingRange = 0.25;
        double offset = 0;
        int tagID = 20;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId(); // The ID number of the Apriltag
        }
        if (zone == 1){
            offset = 0;
        }else if (zone == 2){
            offset = 4.3;
        }else if (zone == 3){
            offset = 0;
        }
        if (tagID == 24){
            offset = -1*offset;
        }
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            if (Math.abs(tx - smoothTx) > smoothingRange) {
                smoothTx = tx;
            }
            if (Math.abs(smoothTx) <= errorMargin) {
                correctPos = 1;
                return currentAngle;
            }else{
                correctPos = 0;
            }
            double direction;
            if (tx < offset){
                direction = -1;
            }else{
                direction = 1;
            }
            if (zone == 1){
                correction = ((smoothTx - offset) * (1.67)); //30/18
            }else if (zone == 2){
                correction = ((smoothTx - offset) * (1.92 /*25/13*/))-(direction*0.008);
            }else if (zone == 3){
                correction = ((smoothTx - offset) * (1.67)); //30/18
            }
            return -(correction / 1800.0) + currentAngle;
        } else {
            return currentAngle;
        }

    }

    private double filteredTx = 0;
    private double lastError = 0;
    private double integral = 0;
    private long lastTime = System.nanoTime();
    public double TurretPower(Limelight3A limelight, double errorMargin) {
        limelight.pipelineSwitch(9);
        double alpha = 0.25;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lastError = 0;
            integral = 0;
            return 0;
        }

        double tx = result.getTx();
        double direction;
        if (tx > 0){
            direction = -1;
        }else{
            direction = 1;
        }
        if (tx == errorMargin){
            return 0;
        }else {
            return 0.05*direction + 0.15 * tx/24;
        }
    }
    public boolean AprilTagTrackerDriveTrain(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb, Limelight3A limelight,double mode) {
        limelight.pipelineSwitch(9);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lf.setPower(0);
            lb.setPower(0);
            rf.setPower(0);
            rb.setPower(0);
            return false;
        }

        double tx = result.getTx();
        double ta = result.getTa();

        double targetTa = 2.1;
        double sizeError = targetTa - ta;
        double turnError = tx;

        double forward = 0;
        double turn = 0;

        if (Math.abs(sizeError) > 0.2 && mode == 1) {
            double s = Math.abs(sizeError) < 0.3 ? 0.3 : Math.abs(sizeError);
            forward = sizeError > 0 ?  s : -s;
        }

        if (Math.abs(turnError) > 2) {
            turn = (turnError / 24.0) * 0.5;
        }

        double lfPow = forward + turn;
        double lbPow = forward + turn;
        double rfPow = forward - turn;
        double rbPow = forward - turn;

        double max = Math.max(1.0, Math.max(Math.abs(lfPow), Math.abs(rfPow)));

        lf.setPower(lfPow / max);
        lb.setPower(lbPow / max);
        rf.setPower(rfPow / max);
        rb.setPower(rbPow / max);

        return Math.abs(sizeError) <= 0.2 && Math.abs(turnError) <= 2;
    }

    public double TurretPowerPID(Limelight3A limelight, double errorMargin, double kP, double kI, double kD) {
        limelight.pipelineSwitch(9);
        double alpha = 0.25;
        LLResult result = limelight.getLatestResult();
        if (!result.isValid() || result == null ) return 0;
        double tx = result.getTx();
        filteredTx = alpha * tx + (1 - alpha) * filteredTx;
        double error = filteredTx;
        if (Math.abs(error) <= errorMargin) {
            lastError = 0;
            integral = 0;
            return 0;
        }
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        if (dt <= 0) dt = 0.001;
        integral += error * dt;
        integral = Math.max(Math.min(integral, 0.5), -0.5);
        double derivative = (error - lastError) / dt;
        lastError = error;
        double pid = (kP * error) + (kI * integral) + (kD * derivative);
        double scale = 1;
        pid *= scale;
        if (pid > 0) pid += 0.05;
        else if (pid < 0) pid -= 0.05;
        //pid = Math.max(-1, Math.min(1, pid));
        return -pid;
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
    public double adjustedTurretAnglePID(double currentAngle, Limelight3A limelight,double zone, double kP, double kI, double kD) {
        double correction = 0;
        limelight.pipelineSwitch(9);
        double errorMargin = 2;
        double smoothingRange = 0.25;
        double offset = 0;
        int tagID = 20;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId(); // The ID number of the Apriltag
        }
        if (zone == 1){
            offset = 0;
        }else if (zone == 2){
            offset = 4.3;
        }else if (zone == 3){
            offset = 0;
        }
        if (tagID == 24){
            offset = -1*offset;
        }
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            if (Math.abs(tx - smoothTx) > smoothingRange) {
                smoothTx = tx;
            }
            if (Math.abs(smoothTx) <= errorMargin) {
                return currentAngle;
            }
            double direction;
            if (tx < offset){
                direction = -1;
            }else{
                direction = 1;
            }
            double error = smoothTx - offset;
            if (Math.abs(error) <= errorMargin) {
                correctPos = 1;
                lastError = 0;
                integral = 0;
                return 0;
            }else{
                correctPos = 0;
            }

                double turnDeg = ((33.0 / 13.2) * error) / 1800.0;

                if (Math.abs(turnDeg) <= errorMargin) {
                    correctPos = 1;
                    lastError = 0;
                    integral = 0;
                    return currentAngle;
                }else{
                    correctPos = 0;
                }

                long now = System.nanoTime();
                double dt = (now - lastTime) / 1e9;
                lastTime = now;
                if (dt <= 0) dt = 0.001;

                integral += turnDeg * dt;
                integral = Math.max(Math.min(integral, 0.5), -0.5);

                double derivative = (turnDeg - lastError) / dt;
                lastError = turnDeg;

                double pid = (kP * turnDeg) + (kI * integral) + (kD * derivative);

                if (pid > 0) pid += 0.05;
                else if (pid < 0) pid -= 0.05;

                double newPos = currentAngle + pid;
                newPos = Math.max(0.0, Math.min(1.0, newPos));

                return newPos;
            }
         else {
            return currentAngle;
        }

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

    //TODO: use pinpoint pos to change size filtering for balls
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

    public double distance (Limelight3A limelight) {
        //limelight to apriltag distance
        limelight.pipelineSwitch(9);
        LLResult result = limelight.getLatestResult();
        double TA = result.getTa();
        double distance = 1.068 * Math.sqrt((3.05 * Math.cos(Math.toRadians(20))) / TA);
        //distance = referenceDistance * sqrt ( (reference target area * cos(limelight vertical angle) ) / current target area)
        return distance;
    }
    public double groundDistance(Limelight3A limelight){
        double distance = distance(limelight);
        double groundDistance =Math.sqrt((distance*distance) - 0.1505828025);
        if (groundDistance > 50){
            return -1;
        }else{
        return groundDistance;
        }
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
        return  groundDistance - 0.465;
    }
    public double groundDistancePinpoint(double x, double y, String allianceColor){
        //method overrload
        double groundDistance = 0;
        if (allianceColor.equals("Red")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((-1.8288 - y) * (-1.8288 - y)));
        }if (allianceColor.equals("Blue")) {
            groundDistance = Math.sqrt(((1.8288 - x) * (1.8288 - x)) + ((1.8288 - y) * (1.8288 - y)));
        }
        return  groundDistance - 0.465;
    }
    public double FlywheelSpeed(Limelight3A limelight, double currentVelocity, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        double groundDistance = groundDistancePinpoint(pinpoint,allianceColor);
        //double speed = -1 * (0.42 + 0.1505  * groundDistance);
        //distance speed function
        double speed = -1 * (0.4415  + 0.114 * (groundDistance - 1));
        if (groundDistance > 3.1){
            speed += 1 * (0.1 * (groundDistance - 3.1));
        }
        if(groundDistance > 3.4){
            speed-= 1 * (0.1 * (groundDistance - 3.4));
        }
        //pinpoint.update();

        double vx = pinpoint.getVelX(DistanceUnit.METER);
        double vy = pinpoint.getVelY(DistanceUnit.METER);

        double x = pinpoint.getPosX(DistanceUnit.METER);
        double y = pinpoint.getPosY(DistanceUnit.METER);

        double heading = pinpoint.getHeading(AngleUnit.RADIANS); // radians

        double v = Math.sqrt(vx * vx + vy * vy);

        double goalX = 1.8288;
        double goalY = allianceColor.equals("Red") ? -1.8288 : 1.8288;

        double dx = goalX - x;
        double dy = goalY - y;

        double fieldVx = vx * Math.cos(heading) - vy * Math.sin(heading);
        double fieldVy = vx * Math.sin(heading) + vy * Math.cos(heading);

        double dot = dx * fieldVx + dy * fieldVy;

        int direction = dot >= 0 ? 1 : -1;

        speed += v * 0.05 * direction;

        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
    }

    public double FlywheelSpeedRegressor(double sec,Limelight3A limelight, double currentVelocity,
    org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        //pinpoint.update();
        double vx = pinpoint.getVelX(DistanceUnit.METER);
        double vy = pinpoint.getVelY(DistanceUnit.METER);

        double ax = pinpoint.getPosX(DistanceUnit.METER)+vx*sec;
        double ay = pinpoint.getPosY(DistanceUnit.METER)+vy*sec;

        double groundDistance = groundDistancePinpoint(ax,ay,allianceColor);
        //double speed = -1 * (0.42 + 0.1505  * groundDistance);
        //distance speed function(regressor)
        double speed = 0.5;
        /*if(groundDistance < 2.5){
            //close zone(first 2.5 m with hood down)
            //−17.1364x4+155.8192x3−471.9196x2+276.0946x−1321.3433
            double x = groundDistance;
            double x2 = x * x;
            double x3 = x2 * x;
            double x4 = x3 * x;
             speed = -210.6462 * x4
                     + 1303.6487 * x3
                     - 2922.8138 * x2
                     + 2496.6165 * x
                     - 2019.7428;

        }else{}*/
            //far zone (every distance > 2.5 m with hood at 0.25)
            //−102.8806x^4+1241.4264x^3−5505.1432x^2+10360.9329x−8624.4939
        double x = groundDistance;
        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;
        double x5 = x4 * x;

        speed = 26.2827 * x5
                - 337.8205 * x4
                + 1651.8877 * x3
                - 3767.1536 * x2
                + 3661.4842 * x
                - 2523.5832;


        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
    }

    public double FlywheelSpeedRegressor( double sec, double currentVelocity,
                                         org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){

        double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
        double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));

        double currentDist = groundDistancePinpoint(pinpoint, allianceColor);
        double flightTime = sec * currentDist;

        double ax = pinpoint.getPosX(DistanceUnit.METER) + (vx * flightTime);
        double ay = pinpoint.getPosY(DistanceUnit.METER) + (vy * flightTime);

        double px = pinpoint.getPosX(DistanceUnit.METER) + vx;
        double py = pinpoint.getPosY(DistanceUnit.METER) + vy ;

        double groundDistance = groundDistancePinpoint(ax, ay, allianceColor);
        double oldGroundDistance = groundDistancePinpoint(px, py, allianceColor);

        if (groundDistance > oldGroundDistance) {
            double adjustedSec = sec ;
            double extendedFlightTime = adjustedSec * currentDist;
            ax = pinpoint.getPosX(DistanceUnit.METER) + vx * extendedFlightTime;
            ay = pinpoint.getPosY(DistanceUnit.METER) + vy * extendedFlightTime;
            groundDistance = groundDistancePinpoint(ax, ay, allianceColor);
        }

        double x = groundDistance;
        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;
        double x5 = x4 * x;

        double speed = 26.2827 * x5
                - 337.8205 * x4
                + 1651.8877 * x3
                - 3767.1536 * x2
                + 3661.4842 * x
                - 2523.5832;

        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
    }
    /*public double closeZoneflywheelspeed(Limelight3A limelight, double currentVelocity, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        double groundDistance = groundDistancePinpoint(pinpoint,allianceColor);
        double speed = -1 * (0.57 + 0.1 * groundDistance);
        if (groundDistance > 3.3){
            speed += -1 * (0.12 * (groundDistance - 3.2));
        }
        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
    }*/

    public double hoodHeight(Limelight3A limelight, double currentHood, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        double groundDistance = groundDistancePinpoint(pinpoint,allianceColor);
        double hood = 0.1 * groundDistance;
        if (groundDistance == -1) {
            return currentHood;
        } else {
            if (hood > 0.4){
                return 0.4;
            } else {
                return hood;
            }
        }
    }

    public double hoodHeightRegressor(Limelight3A limelight, double currentHood, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){
        double groundDistance = groundDistancePinpoint(pinpoint,allianceColor);
        if (groundDistance == -1) {
            return currentHood;
        }else{
         double height = 0.1*(groundDistance);
         if(groundDistance > 3){
             height = 0.4;
         }

         return Range.clip(height,0,0.4);

        }
    }

    public boolean recycleToColor(String targetColor,
                                  NormalizedColorSensor leftIntakeColorSensor,
                                  NormalizedColorSensor rightIntakeColorSensor,
                                  DcMotorEx frontIntakeMotor,
                                  DcMotorEx backIntakeMotor,
                                  CRServoImplEx leftKickerServo,
                                  CRServoImplEx rightKickerServo,
                                  CRServoImplEx leftBackRoller,
                                  CRServoImplEx rightBackRoller,
                                  ElapsedTime cycleTimer,
                                  double kickerLocation,
                                  double defaultKickerLocation,
                                  int maxAttempts,
                                  boolean xtrue) {
        if (!xtrue) {
            String currentBallColor = currentColor(leftIntakeColorSensor, rightIntakeColorSensor);

            if (currentBallColor.equals(targetColor)) {
                return false;
            }

            if (maxAttempts <= 0) {
                return false;
            }

            cycleTimer.reset();

            leftKickerServo.setPower(Globals.kickerRecycle);
            rightKickerServo.setPower(Globals.kickerRecycle);
            frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed - 0.2);
            backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed - 0.2);
            leftBackRoller.setPower(Globals.backRollersMaxPower);
            rightBackRoller.setPower(Globals.backRollersMaxPower);

            if (cycleTimer.milliseconds() > 1000) {
                frontIntakeMotor.setPower(0);
                backIntakeMotor.setPower(0);
                leftBackRoller.setPower(0);
                rightBackRoller.setPower(0);
                leftKickerServo.setPower(0);
                rightKickerServo.setPower(0);
                return false;
            }
            cycleTimer.reset();
            return true;
        }
        return false;
    }
    public double[] getMT2(Limelight3A limelight, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint){
        limelight.pipelineSwitch(9);
        limelight.start();
        //pinpoint.update();
        Pose2D pose2d = pinpoint.getPosition();
        double robotYaw = pose2d.getHeading(AngleUnit.DEGREES);
        double x = 0;
        double y = 0;
        double yaw = 0;
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                x = botpose_mt2.getPosition().x;
                y = botpose_mt2.getPosition().y;
                yaw = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
                return new double[]{x, y, yaw};
            } else {
                return new double[]{-5, -5, -5};
            }
        } else {
            return new double[]{-5, -5, -5};
        }
    }

    public double[] adjustMT2Values(
            double x,
            double y,
            double mt2Heading,
            double turretAngle,
            double turretOffsetX,
            double turretOffsetY,
            double cameraOffsetX
    ) {
        double x_rel = x - turretOffsetX - cameraOffsetX;
        double y_rel = y - turretOffsetY;

        double x_rot = x_rel * Math.cos(turretAngle) - y_rel * Math.sin(turretAngle);
        double y_rot = x_rel * Math.sin(turretAngle) + y_rel * Math.cos(turretAngle);

        double adjustedX = x_rot + turretOffsetX + cameraOffsetX;
        double adjustedY = y_rot + turretOffsetY;

        double robotHeading = mt2Heading - Math.toDegrees(turretAngle);
        robotHeading = ((robotHeading + 180) % 360 + 360) % 360 - 180;

        return new double[]{adjustedX, adjustedY, robotHeading};
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

    public double updatePinpoint(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, Limelight3A limelight, double turretAngle){
        double[] mt2 = getMT2(limelight, pinpoint);
        double x = mt2[0];
        double y = mt2[1];
        double yaw = mt2[2];
        if (x == y && x == yaw && x == -5) {
            return 0;
        }
        double[] MT2Val = adjustMT2Values(x, y, yaw, turretAngle, -0.07650, 0.0, 0.14281);
        double nx = MT2Val[0];
        double ny = MT2Val[1];
        double nyaw =MT2Val[2];

        // Map MT2 axes (X forward, Y left) to old turret axes (X right, Y forward)
        double turretX = -ny; // left → right
        double turretY = nx;  // forward → forward

        pinpoint.setPosition(new Pose2D(DistanceUnit.METER, turretX, turretY, AngleUnit.DEGREES, nyaw));
        return 1;
    }



    public double pinpointTurret(org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, double currentPos, String alliance){

        //pinpoint.update();
        Pose2D pose2d = pinpoint.getPosition();

        double curX = pose2d.getX(DistanceUnit.METER);
        double curY = pose2d.getY(DistanceUnit.METER);
        double curYaw = pose2d.getHeading(AngleUnit.DEGREES);

        double goalX = 0;
        double goalY = 0;
        if(alliance.equals("Red")){
            goalX = 1.8288;
            goalY = -1.8288;
        } else if(alliance.equals("Blue")){
            goalX = 1.8288;
            goalY = 1.8288;
        }

        if(curX > 1.2){
            goalX = 1.8288;
        }

        double turretAngle = 90 - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;
        double turretPos = 0.5 + (turretOffset * (33.0/13.0) / 1800.0);

        return Range.clip(turretPos, 0.35, 0.85);
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

    public double FlywheelSpeedRegressor(double moveAwayAdjustment, double sec, double currentVelocity,
                                         org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, String allianceColor){

//        double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
//        double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));

        double currentDist = groundDistancePinpoint(pinpoint, allianceColor);
//        double flightTime = sec * currentDist;
//
//        double ax = pinpoint.getPosX(DistanceUnit.METER) + (vx * flightTime);
//        double ay = pinpoint.getPosY(DistanceUnit.METER) + (vy * flightTime);

        double px = pinpoint.getPosX(DistanceUnit.METER) ;
        double py = pinpoint.getPosY(DistanceUnit.METER) ;

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
        double speed = currentVelocity;
        if(groundDistance < 3.0){
            speed = 230.1232 * x6
                    - 2593.3794 * x5
                    + 11696.9697 * x4
                    - 26891.7749 * x3
                    + 33072.9972 * x2
                    - 20795.6573 * x
                    + 3978.801;
        }else{
            speed = -1194.7012 * x4
                    + 16697.1406 * x3
                    - 87063.4798 * x2
                    + 200501.6233 * x
                    - 173785.1687;
        }


        if (groundDistance == -1) {
            return currentVelocity;
        } else {
            return speed;
        }
    }
    public double pinpointTurretMoving(double sec, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, double currentPos, String alliance){
        Pose2D pose2d = pinpoint.getPosition();
        double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
        double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));

        double dist = groundDistancePinpoint(pinpoint, alliance);
        double flightTime = sec * dist;

        double curX = pose2d.getX(DistanceUnit.METER) /*+ (vx * flightTime)*/;
        double curY = pose2d.getY(DistanceUnit.METER) /*+ (vy * flightTime)*/;

        double curYaw = pose2d.getHeading(AngleUnit.DEGREES) /*+ (0.3) * pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)*/;

        double goalX = 1.8288;
        double goalY = (alliance.equals("Red")) ? -1.8288 : 1.8288;

        if(dist > 3.0) {
            goalX = 1.8288;
            goalY = (alliance.equals("Blue")) ? 1.8288 : -1.8288;
        }

        if(curX > 1.2){
            goalX = 1.4288;
        } else if(curX > 0){
            goalX = 1.8288;
            goalY = (alliance.equals("Blue")) ? 1.8288 : -1.8288;
        }

        /*if((curY < 0 && alliance.equals("Blue")) || (curY > 0 && alliance.equals("Red"))){
            goalX = 1.6288;
        }*/

        double turretAngle = 90 - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;

        double turretPos = 0.5 + (turretOffset * (33.0/12.8) / 1800.0);

        return Range.clip(turretPos, 0.25, 0.625);
    }
    public double pinpointTurretMoving(double gear,double sec, org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver pinpoint, double currentPos, String alliance){
        Pose2D pose2d = pinpoint.getPosition();
        double vx = getFilteredVelocityX(pinpoint.getVelX(DistanceUnit.METER));
        double vy = getFilteredVelocityY(pinpoint.getVelY(DistanceUnit.METER));

        double dist = groundDistancePinpoint(pinpoint, alliance);
        double flightTime = sec * dist;

        double curX = pose2d.getX(DistanceUnit.METER) /*+ (vx * flightTime)*/;
        double curY = pose2d.getY(DistanceUnit.METER) /*+ (vy * flightTime)*/;

        double curYaw = pose2d.getHeading(AngleUnit.DEGREES) /*+ (flightTime * 0.3) * pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)*/;

        double goalX = 1.8288;
        double goalY = (alliance.equals("Red")) ? -1.8288 : 1.8288;

        if(dist > 3.0) {
            goalX = 1.8288;
            goalY = (alliance.equals("Blue")) ? 1.8288 : -1.8288;
        }

        if(curX > 1.2){
            goalX = 1.4288;
        } else if(curX > 0){
            goalX = 1.8288;
            goalY = (alliance.equals("Blue")) ? 1.8288 : -1.8288;
        }

        /*if((curY < 0 && alliance.equals("Blue")) || (curY > 0 && alliance.equals("Red"))){
            goalX = 1.6288;
        }*/

        double turretAngle = 90 - Math.toDegrees(Math.atan2(goalX - curX, goalY - curY));
        double turretOffset = turretAngle - curYaw;

        double turretPos = 0.5 + (turretOffset * (33.0/gear) / 1800.0);

        return Range.clip(turretPos, 0.25, 0.625);
    }




}

