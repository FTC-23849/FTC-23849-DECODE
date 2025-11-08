package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setDepositPIDF extends CommandBase {

    private final Robot robot;
    private final Shooter.DepositPivotState pivotState;
    private final Shooter.DepositTurretState turretState;
    private final Shooter.DepositWristState wristState;
    private final Shooter.DepositClawState clawState;
    private final double target;

    ElapsedTime timer;

    private double index;

    public setDepositPIDF(Robot robot, Shooter.DepositPivotState pivotState, Shooter.DepositTurretState turretState,
                          Shooter.DepositWristState wristState, Shooter.DepositClawState clawState, double target) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.turretState = turretState;
        this.wristState = wristState;
        this.clawState = clawState;
        this.target = target;

        addRequirements(robot.shooter);
    }

    @Override
    public void initialize() {
        if (Shooter.depositPivotState.equals(this.pivotState) && Shooter.depositTurretState.equals(this.turretState)
                && Shooter.depositWristState.equals(this.wristState) && Shooter.depositClawState.equals(this.clawState)
                && robot.shooter.targetPIDF == this.target) {
            // Set index to 1 if input parameters is same as current state
            index = 1;
        } else if (Shooter.depositPivotState.equals(Shooter.DepositPivotState.TRANSFER)) {
            // Move intake pivot so outtake does not hit intake while going up
            robot.intake.setPivot(Intake.IntakePivotState.OUTTAKE_AVOID);
            // Move slides
            robot.shooter.setOuttakeTargetPIDF(target);

            index = 2;

            timer.reset();
        } else {
            robot.shooter.setOuttakeTargetPIDF(target);

            index = 3;
        }
    }

    @Override
    public void execute() {
        if (index == 2) {
            robot.shooter.setPivot(pivotState);
            robot.shooter.setTurret(turretState);
            robot.shooter.setWrist(wristState);
            robot.shooter.setClaw(clawState);

            if (timer.milliseconds() >= 300) {
                robot.intake.setPivot(Intake.IntakePivotState.TRANSFER);
            }

            index = 1;
        }

        if (index == 3) {
            robot.shooter.setPivot(pivotState);
            robot.shooter.setTurret(turretState);
            robot.shooter.setWrist(wristState);
            robot.shooter.setClaw(clawState);

            index = 1;
        }
    }

    @Override
    public boolean isFinished() {
        return robot.shooter.outtakeReached && index == 1;
    }

}
