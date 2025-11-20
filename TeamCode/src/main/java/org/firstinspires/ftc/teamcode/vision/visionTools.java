package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public double adjustedTurretAngle(double currentAngle, Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double Offset = result.getTx();
            double correction = (Offset / 48) * 54.5;
            return correction + currentAngle;
        } else {
            return 0;
        }
    }

    private double filteredTx = 0;
    private double lastError = 0;
    private double integral = 0;
    private long lastTime = System.nanoTime();
    public double TurretPower(Limelight3A limelight, double errorMargin) {
        double alpha = 0.25;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        double tx = result.getTx();
        double direction;
        if (tx > 0){
             direction = 1;
        }else{
            direction = -1;
        }
        if (tx == 0){
            return 0;
        }else {
            return 0.05*direction + 0.15 * direction * tx/24;
        }
    }
    public double TurretPowerPID(Limelight3A limelight, double errorMargin, double kP, double kI, double kD) {
        double alpha = 0.25;
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) return 0;
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
        double scale = Math.min(1.0, Math.abs(error) / 15.0);
        pid *= scale;
        if (pid > 0) pid += 0.05;
        else if (pid < 0) pid -= 0.05;
        pid = Math.max(-1, Math.min(1, pid));
        return pid;
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
        LLResult results = limelight.getLatestResult();
        return results.getPythonOutput()[3];
    }
    public void recycleToColor(double color, CRServoImplEx leftKickerServo, CRServoImplEx rightKickerServo,
                               DcMotorEx frontIntakeMotor, DcMotorEx backIntakeMotor,
                               CRServoImplEx leftBackRoller, CRServoImplEx rightBackRoller,
                               NormalizedColorSensor leftIntakeColorSensor, NormalizedColorSensor rightIntakeColorSensor) {
        String detectedColor = currentColor(leftIntakeColorSensor, rightIntakeColorSensor);



        int cycles = 0;
        while (cycles < 2) {
            if ((color == 0 && !detectedColor.equals("Green")) || (color == 1 && !detectedColor.equals("Purple")||detectedColor.equals("none"))) {
                leftKickerServo.setPower(Globals.kickerRecycle);
                rightKickerServo.setPower(Globals.kickerRecycle);
                frontIntakeMotor.setPower(Globals.frontIntakeRecycleSpeed);
                backIntakeMotor.setPower(Globals.backIntakeRecycleSpeed);
                leftBackRoller.setPower(Globals.backRollersReverse);
                rightBackRoller.setPower(Globals.backRollersReverse);

                timer.reset();
                while (timer.milliseconds() < 500) {

                }
                cycles+=1;
                detectedColor = currentColor(leftIntakeColorSensor, rightIntakeColorSensor);
            } else {
                leftKickerServo.setPower(0);
                rightKickerServo.setPower(0);
                frontIntakeMotor.setPower(0);
                backIntakeMotor.setPower(0);
                leftBackRoller.setPower(0);
                rightBackRoller.setPower(0);
                break;
            }
        }
    }

}
