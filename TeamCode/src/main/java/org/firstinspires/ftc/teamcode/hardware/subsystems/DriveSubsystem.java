package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;
import org.firstinspires.ftc.teamcode.opmode.Auto.PoseStorage;

@Config
public class DriveSubsystem {
    //variables
    private final RobotClass robot;
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public GoBildaPinpointDriver pinpoint;

    public boolean lockedModeEnabled = false;
    public boolean parkModeEnabled = false;
    private Pose2D anchorPos = null;
    private double anchorX, anchorY, anchorYaw;
    private double lastHeadingErrorLocked = 0, lastXErrorLocked = 0, lastYErrorLocked = 0;
    private boolean rightBumperHeld = false;
    private boolean rightBumperFirstTime = false;

    public static double kP_Lock = 0.16;
    public static double kP_Lock_Small = 0.07;
    public static double kD_Lock = 0.0007;
    public static double PIDDeadband = 8;
    public static double kPRot_Lock = 0.08;
    public static double kDRot_Lock = 0.00028;

    private Pose2D robotPos;
    private Pose2D predictedPos;
    private Pose2D turretPredictedPos;
    private double velX, velY, velH;

    private final ElapsedTime dtTimer = new ElapsedTime();
    private double lastTime = 0;
    private double dt = 0.01;

    public static double dtStallThreshold = 20.0;
    public static double dtStallConfirmMs = 250;
    public static double dtStallIntakeDisableSec = 3.0;
    private double dtHighCurrentStartTime = -1;
    private double dtStallTriggerTime = -1;
    private boolean dtMotorStalled = false;

    private enum InitStage { RESET, WAIT_AFTER_RESET, RECAL_IMU, WAIT_AFTER_RECAL, SET_POSE, WAIT_AFTER_SET, UPDATE_AND_PRINT, DONE }
    private InitStage initStage = InitStage.RESET;
    private final ElapsedTime initTimer = new ElapsedTime();

    public DriveSubsystem(RobotClass robot) {
        this.robot = robot;
    }

    //init drive motors + pinpoint
    public void init() {
        leftFront = robot.hardwareMap.get(DcMotorEx.class, "LF");
        rightFront = robot.hardwareMap.get(DcMotorEx.class, "RF");
        leftBack = robot.hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = robot.hardwareMap.get(DcMotorEx.class, "RB");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(96.6511963161, -2.55558368232, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(19.970472542, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        dtTimer.reset();
        lastTime = dtTimer.seconds();
    }

    public void initLoop() {
        switch (initStage) {
            case RESET:
                pinpoint.resetPosAndIMU();
                initTimer.reset();
                initStage = InitStage.WAIT_AFTER_RESET;
                break;
            case WAIT_AFTER_RESET:
                if (initTimer.milliseconds() >= 500) initStage = InitStage.RECAL_IMU;
                break;
            case RECAL_IMU:
                pinpoint.recalibrateIMU();
                initTimer.reset();
                initStage = InitStage.WAIT_AFTER_RECAL;
                break;
            case WAIT_AFTER_RECAL:
                if (initTimer.milliseconds() >= 1000) initStage = InitStage.SET_POSE;
                break;
            case SET_POSE:
                pinpoint.setPosition(PoseStorage.currentPose);
                initTimer.reset();
                initStage = InitStage.WAIT_AFTER_SET;
                break;
            case WAIT_AFTER_SET:
                if (initTimer.milliseconds() >= 1000) initStage = InitStage.UPDATE_AND_PRINT;
                break;
            case UPDATE_AND_PRINT:
                pinpoint.update();
                robot.telemetry.addData("file current pose", PoseStorage.currentPose);
                robot.telemetry.addData("pinpoint current pose", pinpoint.getPosition().toString());
                initStage = InitStage.DONE;
                break;
            case DONE:
                robot.telemetry.addData("Init", "DONE");
                robot.telemetry.addData("file current pose", PoseStorage.currentPose);
                robot.telemetry.addData("pinpoint current pose", pinpoint.getPosition().toString());
                break;
        }
        robot.telemetry.addData("InitStage", initStage);
        robot.telemetry.addData("t(ms)", (int) initTimer.milliseconds());
        robot.telemetry.update();
    }

    public void update() {
        double currentTime = dtTimer.seconds();
        dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.001;
        lastTime = currentTime;
        updateDtStall(currentTime);

        pinpoint.update();
        //sotm stuff
        robotPos = pinpoint.getPosition();
        velX = pinpoint.getVelX(DistanceUnit.MM);
        velY = pinpoint.getVelY(DistanceUnit.MM);
        velH = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        predictedPos = robot.vision.predictPos(robot.miscSubsystems.allianceColor, robotPos, velX, velY, velH);
        turretPredictedPos = robot.vision.getTurretPredictedPos(robot.miscSubsystems.allianceColor, robotPos, velX, velY, velH);
    }

    private void updateDtStall(double currentTime) {
        double dtMotorDraw = leftFront.getCurrent(CurrentUnit.AMPS) + rightFront.getCurrent(CurrentUnit.AMPS)
                + leftBack.getCurrent(CurrentUnit.AMPS) + rightBack.getCurrent(CurrentUnit.AMPS);

        if (dtMotorDraw > dtStallThreshold) {
            if (dtHighCurrentStartTime < 0) {
                dtHighCurrentStartTime = currentTime;
            } else if (!dtMotorStalled && (currentTime - dtHighCurrentStartTime) * 1000.0 >= dtStallConfirmMs) {
                dtMotorStalled = true;
                dtStallTriggerTime = currentTime;
            }
        } else {
            dtHighCurrentStartTime = -1;
        }

        if (dtMotorStalled && (currentTime - dtStallTriggerTime) >= dtStallIntakeDisableSec) {
            dtMotorStalled = false;
            dtHighCurrentStartTime = -1;
            dtStallTriggerTime = -1;
        }
    }

    public boolean isDtStalled() {
        return dtMotorStalled;
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public void cornerRelocalize(String allianceColor) {
        if ("Blue".equals(allianceColor)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -63, -65, AngleUnit.DEGREES, 0));
        } else if ("Red".equals(allianceColor)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -63, 65, AngleUnit.DEGREES, 0));
        }
    }

    //actual drive method ( * 1.1 to dix drift)
    public void driveRobot(Gamepad gamepad) {
        if (lockedModeEnabled || parkModeEnabled) return;

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);
    }

    public void lockedPos(boolean rightBumperPressed) {
        if (rightBumperPressed) {
            rightBumperFirstTime = !rightBumperHeld;
            rightBumperHeld = true;
        } else {
            rightBumperHeld = false;
            rightBumperFirstTime = false;
            lockedModeEnabled = false;
        }

        if (rightBumperFirstTime) {
            lockedModeEnabled = !lockedModeEnabled;
            anchorPos = null;

            if (lockedModeEnabled) {
                anchorPos = robotPos;
                anchorX = -1 * anchorPos.getY(DistanceUnit.METER);
                anchorY = anchorPos.getX(DistanceUnit.METER);
                anchorYaw = ((robotPos.getHeading(AngleUnit.DEGREES) + 90) % 360 + 360) % 360;
            } else {
                lastXErrorLocked = 0;
                lastYErrorLocked = 0;
                lastHeadingErrorLocked = 0;
            }
        }
    }

    public void runLockedPos(Gamepad gamepad) {
        if (lockedModeEnabled || parkModeEnabled) {
            if (anchorPos == null) {
                anchorPos = robotPos;
                anchorX = -1 * anchorPos.getY(DistanceUnit.METER);
                anchorY = anchorPos.getX(DistanceUnit.METER);
                anchorYaw = ((robotPos.getHeading(AngleUnit.DEGREES) + 90) % 360 + 360) % 360;
            }
            runLockedPos(dt);
        } else {
            anchorPos = null;
        }
    }

    private void runLockedPos(double dt) {
        double heading = robotPos.getHeading(AngleUnit.DEGREES);
        double adjustedHeading = ((heading + 90) % 360 + 360) % 360;

        double headingError = anchorYaw - adjustedHeading;
        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;

        double headingDerivative = (headingError - lastHeadingErrorLocked) / dt;
        double rotPower = kPRot_Lock * Math.signum(headingError) * Math.sqrt(Math.abs(headingError)) + kDRot_Lock * headingDerivative;
        lastHeadingErrorLocked = headingError;

        double xPos = -1 * robotPos.getY(DistanceUnit.METER);
        double yPos = robotPos.getX(DistanceUnit.METER);

        double xError = (anchorX - xPos) * 100;
        double yError = (anchorY - yPos) * 100;
        double dist = Math.sqrt(xError * xError + yError * yError);
        double relAngle = Math.atan2(yError, xError) - Math.toRadians(adjustedHeading);

        xError = dist * -Math.sin(relAngle);
        yError = dist * Math.cos(relAngle);

        double xD = (xError - lastXErrorLocked) / dt;
        double yD = (yError - lastYErrorLocked) / dt;
        
        double kP = (dist < PIDDeadband) ? kP_Lock_Small : kP_Lock;
        double xPower = kP * Math.signum(xError) * Math.sqrt(Math.abs(xError)) + kD_Lock * xD;
        double yPower = kP * Math.signum(yError) * Math.sqrt(Math.abs(yError)) + kD_Lock * yD;

        lastXErrorLocked = xError;
        lastYErrorLocked = yError;

        leftFront.setPower(Math.max(-1, Math.min(1, yPower + xPower - rotPower)));
        rightFront.setPower(Math.max(-1, Math.min(1, yPower - xPower + rotPower)));
        leftBack.setPower(Math.max(-1, Math.min(1, yPower - xPower - rotPower)));
        rightBack.setPower(Math.max(-1, Math.min(1, yPower + xPower + rotPower)));
    }

    public Pose2D getRobotPos() { return robotPos; }
    public Pose2D getPredictedPos() { return predictedPos; }
    public Pose2D getTurretPredictedPos() { return turretPredictedPos; }

    public void telemetry() {
        robot.telemetry.addData("Robot Pos", robotPos);
        robot.telemetry.addData("Locked", lockedModeEnabled);
    }
}
