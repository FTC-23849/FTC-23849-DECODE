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

public class visionTools {
    ElapsedTime timer = new ElapsedTime();
    double hueThresholdPurple = 200;
    double hueThresholdGreen = 100;
    public double TurretPowerTxDebug;
    private double smoothTx = 0;

    public double adjustedTurretAngle(double currentAngle, Limelight3A limelight,int zone) {
        limelight.pipelineSwitch(9);
        double errorMargin = 2;
        double smoothingRange = 0.25;
        double offset;
        LLResult result = limelight.getLatestResult();
        LLResultTypes.FiducialResult fiducial = null;
        double tagID = fiducial.getFiducialId();
        if (zone == 1){
            offset = 0;
        }else{
            offset = 9.2;
        }
        if (tagID == 24){
            offset = -offset;
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
            double correction = ((smoothTx - offset) * (13/33.0))+0*(direction*0.004);
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
    private void stopAll(CRServoImplEx leftKick, CRServoImplEx rightKick,
                         DcMotorEx frontIntake, DcMotorEx backIntake,
                         CRServoImplEx leftRoller, CRServoImplEx rightRoller) {
        leftKick.setPower(0);
        rightKick.setPower(0);
        frontIntake.setPower(0);
        backIntake.setPower(0);
        leftRoller.setPower(0);
        rightRoller.setPower(0);
    }
    public int recycleToColor(String targetColor,
                               NormalizedColorSensor leftSensor,
                               NormalizedColorSensor rightSensor,
                               double attempts){
        if (attempts < 3 ) {
            boolean reachedCorrectColor = currentColor(leftSensor, rightSensor).equals(targetColor);
            if (reachedCorrectColor) {
                int correctColor = 1;
                return correctColor;
            } else {
                int correctColor = 0;
                return correctColor;
            }
        }else{
            return -1;
        }
    }
    public boolean recycle(int action,
                              CRServoImplEx leftKickerServo,
                              CRServoImplEx rightKickerServo,
                              DcMotorEx frontIntakeMotor,
                              DcMotorEx backIntakeMotor,
                              CRServoImplEx leftBackRoller,
                              CRServoImplEx rightBackRoller) {


        if (action == 1) {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            leftBackRoller.setPower(0);
            rightBackRoller.setPower(0);
            leftKickerServo.setPower(0);
            rightKickerServo.setPower(0);
            return true;
        }else{
            frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed);
            backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed);
            leftBackRoller.setPower(Globals.backRollersReverse);
            rightBackRoller.setPower(Globals.backRollersReverse);
            leftKickerServo.setPower(Globals.kickerRecycle);
            rightKickerServo.setPower(Globals.kickerRecycle);
            return false;
        }

    }

}

