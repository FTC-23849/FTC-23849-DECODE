package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

public class visionTools {
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
                correction = ((smoothTx - offset) * (30/18));
            }else if (zone == 2){
                correction = ((smoothTx - offset) * (25/13))-(direction*0.008);
            }else if (zone == 3){
                correction = ((smoothTx - offset) * (30/18));
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

                double turnDeg = ((33.0 / 13.0) * error) / 1800.0;

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


}

