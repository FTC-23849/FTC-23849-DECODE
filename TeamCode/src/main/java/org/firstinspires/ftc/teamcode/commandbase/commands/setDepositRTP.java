package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setDepositRTP extends CommandBase {

    private final Robot robot;
    private final Deposit.DepositPivotState pivotState;
    private final Deposit.DepositTurretState turretState;
    private final Deposit.DepositWristState wristState;
    private final Deposit.DepositClawState clawState;
    private final int target;

    ElapsedTime timer;

    private double index;

    public setDepositRTP(Robot robot, Deposit.DepositPivotState pivotState, Deposit.DepositTurretState turretState,
                         Deposit.DepositWristState wristState, Deposit.DepositClawState clawState, int target) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.turretState = turretState;
        this.wristState = wristState;
        this.clawState = clawState;
        this.target = target;

        addRequirements(robot.deposit);
    }

    @Override
    public void initialize() {
        if (Deposit.depositPivotState.equals(this.pivotState) && Deposit.depositTurretState.equals(this.turretState)
                && Deposit.depositWristState.equals(this.wristState) && Deposit.depositClawState.equals(this.clawState)
                && robot.deposit.targetRTP == this.target) {
            // Set index to 1 if input parameters is same as current state
            index = 1;
        } else if (Deposit.depositPivotState.equals(Deposit.DepositPivotState.TRANSFER)) {
            // Move intake pivot so outtake does not hit intake while going up
            robot.intake.setPivot(Intake.IntakePivotState.OUTTAKE_AVOID);
            // Move slides
            robot.deposit.setOuttakeTargetRTP(target, 1.0);

            index = 2;

            timer.reset();
        } else {
            robot.deposit.setOuttakeTargetRTP(target, 1.0);

            index = 3;
        }
    }

    @Override
    public void execute() {
        if (index == 2) {
            robot.deposit.setPivot(pivotState);
            robot.deposit.setTurret(turretState);
            robot.deposit.setWrist(wristState);
            robot.deposit.setClaw(clawState);

            if (timer.milliseconds() >= 300) {
                robot.intake.setPivot(Intake.IntakePivotState.TRANSFER);
            }

            index = 1;
        }

        if (index == 3) {
            robot.deposit.setPivot(pivotState);
            robot.deposit.setTurret(turretState);
            robot.deposit.setWrist(wristState);
            robot.deposit.setClaw(clawState);

            index = 1;
        }
    }

    @Override
    public boolean isFinished() {
        return (!robot.outtakeSlideMotor_right.isBusy() || !robot.outtakeSlideMotor_left.isBusy()) && index == 1;
    }

}
