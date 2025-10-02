package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public double targetPIDF;
    public int targetRTP;

    public boolean outtakeReached;
    public boolean outtakeRetracted;

    public void init() {
        setPivot(DepositPivotState.TRANSFER);
        setTurret(DepositTurretState.STRAIGHT);
        setWrist(DepositWristState.TRANSFER);
        setClaw(DepositClawState.OPEN);
        setOuttakeTargetPIDF(0);
        outtakePIDF.setTolerance(3);
    }

    public enum DepositPivotState {
        TRANSFER,
        SAMPLE_SCORE,
        SPECIMEN_INTAKE,
        SPECIMEN_SCORE,
        HANG
    }

    public enum DepositTurretState {
        STRAIGHT,
        SPECIMEN_RIGHT,
        SPECIMEN_LEFT
    }

    public enum DepositWristState {
        TRANSFER,
        SAMPLE_SCORE,
        SPECIMEN_INTAKE,
        HIGH_SPEC_SCORE,
        LOW_SPEC_SCORE,
        HANG
    }

    public enum DepositClawState {
        OPEN,
        CLOSED
    }

    public static DepositPivotState depositPivotState = DepositPivotState.TRANSFER;
    public static DepositTurretState depositTurretState = DepositTurretState.STRAIGHT;
    public static DepositWristState depositWristState = DepositWristState.TRANSFER;
    public static DepositClawState depositClawState = DepositClawState.OPEN;
    private static final PIDFController outtakePIDF = new PIDFController(0,0,0, 0);

    public void setOuttakeTargetPIDF(double targetPIDF) {
        this.targetPIDF = Range.clip(targetPIDF, 0, OUTTAKE_SLIDES_MAX_EXTENSION);
        outtakePIDF.setSetPoint(this.targetPIDF);
    }

    public void setOuttakeTargetRTP(int targetRTP, double power) {
        this.targetRTP = Range.clip(targetRTP, 0, OUTTAKE_SLIDES_MAX_EXTENSION);

        robot.outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.outtakeSlideMotor_right.setTargetPosition(targetRTP);
        robot.outtakeSlideMotor_left.setTargetPosition(targetRTP);

        robot.outtakeSlideMotor_right.setPower(power);
        robot.outtakeSlideMotor_left.setPower(power);
    }

    public void autoUpdateSlides() {
        double power = outtakePIDF.calculate(robot.outtakeSlideMotor_right.getCurrentPosition(), targetPIDF);
        outtakeReached = outtakePIDF.atSetPoint()
                || (targetPIDF == 0 && ((robot.outtakeSlideMotor_right.getCurrentPosition() < 5)
                || (robot.outtakeSlideMotor_left.getCurrentPosition() < 5)));

        outtakeRetracted = (targetPIDF <= 0) && outtakeReached;

        if (outtakeRetracted) {
            robot.outtakeSlideMotor_right.setPower(0);
            robot.outtakeSlideMotor_left.setPower(0);
        } else {
            robot.outtakeSlideMotor_right.setPower(power);
            robot.outtakeSlideMotor_left.setPower(power);
        }
    }

    public void setPivot(DepositPivotState depositPivotState) {
        switch(depositPivotState) {
            case TRANSFER:
                robot.outtakePivotRight.setPosition(OUTTAKE_PIVOT_TRANSFER);
                robot.outtakePivotLeft.setPosition(OUTTAKE_PIVOT_TRANSFER);
                break;
            case SAMPLE_SCORE:
                robot.outtakePivotRight.setPosition(OUTTAKE_PIVOT_SAMPLE_SCORE);
                robot.outtakePivotLeft.setPosition(OUTTAKE_PIVOT_SAMPLE_SCORE);
                break;
            case SPECIMEN_INTAKE:
                robot.outtakePivotRight.setPosition(OUTTAKE_PIVOT_SPECIMEN_PICKUP);
                robot.outtakePivotLeft.setPosition(OUTTAKE_PIVOT_SPECIMEN_PICKUP);
                break;
            case SPECIMEN_SCORE:
                robot.outtakePivotRight.setPosition(OUTTAKE_PIVOT_SPECIMEN_SCORE);
                robot.outtakePivotLeft.setPosition(OUTTAKE_PIVOT_SPECIMEN_SCORE);
                break;
            case HANG:
                robot.outtakePivotRight.setPosition(OUTTAKE_PIVOT_TRANSFER);
                robot.outtakePivotLeft.setPosition(OUTTAKE_PIVOT_TRANSFER);
                break;
        }
    }

    public void setTurret(DepositTurretState depositTurretState) {
        switch(depositTurretState) {
            case STRAIGHT:
                robot.outtakeTurret.setPosition(OUTTAKE_TURRET_STRAIGHT);
                break;
            case SPECIMEN_RIGHT:
                robot.outtakeTurret.setPosition(OUTTAKE_TURRET_RIGHT90);
                break;
            case SPECIMEN_LEFT:
                robot.outtakeTurret.setPosition(OUTTAKE_TURRET_LEFT90);
                break;
        }
    }

    public void setTurretVariable(double pos) {
        robot.outtakeTurret.setPosition(pos);
    }

    public void setWrist(DepositWristState depositWristState) {
        switch(depositWristState) {
            case TRANSFER:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_TRANSFER);
                break;
            case SAMPLE_SCORE:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_SAMPLE_SCORE);
                break;
            case SPECIMEN_INTAKE:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_SPECIMEN_PICKUP);
                break;
            case HIGH_SPEC_SCORE:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE);
                break;
            case LOW_SPEC_SCORE:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_LOW_SPECIMEN_SCORE);
                break;
            case HANG:
                robot.outtakeWrist.setPosition(OUTTAKE_WRIST_HANG);
                break;
        }
    }

    public void setClaw(DepositClawState depositClawState) {
        switch(depositClawState) {
            case OPEN:
                robot.outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                robot.outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSE);
                break;
        }
    }

    /*@Override
    public void periodic() {
        autoUpdateSlides();
    }*/

}
