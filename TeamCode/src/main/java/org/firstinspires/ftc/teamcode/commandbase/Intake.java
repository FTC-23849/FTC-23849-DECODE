package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public double target;

    // Reached set position
    public boolean extendoReached;
    // Fully retracted
    public boolean extendoRetracted;

    public void init() {
        setPivot(IntakePivotState.TRANSFER);
        setTurret(IntakeTurretState.STRAIGHT);
        setWrist(IntakeWristState.STRAIGHT);
        setClaw(IntakeClawState.OPEN);
        setExtendoTarget(0);
        extendoPIDF.setTolerance(3);
    }

    public enum IntakePivotState {
        TRANSFER,
        HOVER,
        PICKUP,
        SIDE_DROP,
        OUTTAKE_AVOID
    }

    public enum IntakeTurretState {
        STRAIGHT,
        RIGHT_INTAKE,
        LEFT_INTAKE,
        RIGHT_SIDE_DROP,
        LEFT_SIDE_DROP
    }

    public enum IntakeWristState {
        STRAIGHT,
        RIGHT_90,
        LEFT_90
    }

    public enum IntakeClawState {
        OPEN,
        CLOSED
    }

    public static IntakePivotState intakePivotState = IntakePivotState.TRANSFER;
    public static IntakeTurretState intakeTurretState = IntakeTurretState.STRAIGHT;
    public static IntakeWristState intakeWristState = IntakeWristState.STRAIGHT;
    public static IntakeClawState intakeClawState = IntakeClawState.OPEN;
    private static final PIDFController extendoPIDF = new PIDFController(0,0,0, 0);


    public void setExtendoTarget(double target) {
        this.target = Range.clip(target, 0, INTAKE_MOTOR_MAX_EXTENSION);
        extendoPIDF.setSetPoint(this.target);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(robot.intakeMotor.getCurrentPosition(), this.target);
        extendoReached = (extendoPIDF.atSetPoint() && target > 0) || (robot.intakeMotor.getCurrentPosition() <= 3 && target == 0);
        extendoRetracted = (target <= 0) && extendoReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0 && !extendoReached) {
            extendoPower -= 0.2;
        } else if (!extendoReached) {
            extendoPower += 0.2;
        }

        if (extendoReached) {
            robot.intakeMotor.setPower(0);
        } else {
            robot.intakeMotor.setPower(extendoPower);
        }
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch(intakePivotState) {
            case TRANSFER:
                robot.intakePivot.setPosition(INTAKE_PIVOT_TRANSFER);
                break;
            case HOVER:
                robot.intakePivot.setPosition(INTAKE_PIVOT_PICKUP_READY);
                break;
            case PICKUP:
                robot.intakePivot.setPosition(INTAKE_PIVOT_PICKUP);
                break;
            case SIDE_DROP:
                robot.intakePivot.setPosition(INTAKE_PIVOT_DROP);
                break;
            case OUTTAKE_AVOID:
                robot.intakePivot.setPosition(INTAKE_PIVOT_AVOID);
                break;
        }
    }

    public void setTurret(IntakeTurretState intakeTurretState) {
        switch(intakeTurretState) {
            case STRAIGHT:
                robot.intakeTurret.setPosition(INTAKE_TURRET_TRANSFER);
                break;
            case RIGHT_INTAKE:
                robot.intakeTurret.setPosition(INTAKE_TURRET_PICKUP_RIGHT);
                break;
            case LEFT_INTAKE:
                robot.intakeTurret.setPosition(INTAKE_TURRET_PICKUP_LEFT);
                break;
            case RIGHT_SIDE_DROP:
                robot.intakeTurret.setPosition(INTAKE_TURRET_DROP_RIGHT);
                break;
            case LEFT_SIDE_DROP:
                robot.intakeTurret.setPosition(INTAKE_TURRET_DROP_LEFT);
                break;
        }
    }

    public void setWrist(IntakeWristState intakeWristState) {
        switch(intakeWristState) {
            case STRAIGHT:
                robot.intakeWrist.setPosition(INTAKE_WRIST_STRAIGHT);
                break;
            case RIGHT_90:
                robot.intakeWrist.setPosition(INTAKE_WRIST_RIGHT90);
                break;
            case LEFT_90:
                robot.intakeWrist.setPosition(INTAKE_WRIST_LEFT90);
                break;
        }
    }

    public void setClaw(IntakeClawState intakeClawState) {
        switch(intakeClawState) {
            case OPEN:
                robot.intakeClaw.setPosition(INTAKE_CLAW_OPEN);
                break;
            case CLOSED:
                robot.intakeClaw.setPosition(INTAKE_CLAW_CLOSE);
                break;
        }
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
    }

}
