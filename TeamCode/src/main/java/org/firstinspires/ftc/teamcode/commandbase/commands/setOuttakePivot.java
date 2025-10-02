package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setOuttakePivot extends CommandBase {

    private final Robot robot;
    private final Deposit.DepositPivotState pivotState;

    public setOuttakePivot(Robot robot, Deposit.DepositPivotState pivotState) {
        this.robot = robot;
        this.pivotState = pivotState;

        addRequirements(robot.deposit);
    }

    @Override
    public void initialize() {
        robot.deposit.setPivot(Deposit.depositPivotState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
