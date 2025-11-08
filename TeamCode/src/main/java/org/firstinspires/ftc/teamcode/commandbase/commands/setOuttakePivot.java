package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setOuttakePivot extends CommandBase {

    private final Robot robot;
    private final Shooter.DepositPivotState pivotState;

    public setOuttakePivot(Robot robot, Shooter.DepositPivotState pivotState) {
        this.robot = robot;
        this.pivotState = pivotState;

        addRequirements(robot.shooter);
    }

    @Override
    public void initialize() {
        robot.shooter.setPivot(pivotState);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
