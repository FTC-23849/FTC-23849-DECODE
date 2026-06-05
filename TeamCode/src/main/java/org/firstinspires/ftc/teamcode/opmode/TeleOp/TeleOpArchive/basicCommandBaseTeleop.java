package org.firstinspires.ftc.teamcode.opmode.TeleOp.TeleOpArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commandbase.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.commands.setOuttakePivot;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp
public class basicCommandBaseTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    GamepadEx gamepad;

    @Override
    public void initialize() {

        gamepad = new GamepadEx(gamepad1);

        robot.init(hardwareMap);

        super.reset();

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new setOuttakePivot(robot, Shooter.depositPivotState.TRANSFER));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new setOuttakePivot(robot, Shooter.depositPivotState.SAMPLE_SCORE));

        super.run();

    }

}
