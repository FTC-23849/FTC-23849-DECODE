package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotClass;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

@TeleOp(name = "TeleOpClean")
@Config
public class TeleOpClean extends OpMode {

    private RobotClass robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(120);

        robot = new RobotClass(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void init_loop() {
        robot.initLoop();
    }

    @Override
    public void loop() {
        robot.update();

        // Drive
        robot.drive.driveRobot(gamepad1);
        robot.drive.lockedPos(gamepad1.right_bumper);
        robot.drive.runLockedPos(gamepad1);
        if (gamepad2.x) robot.drive.recalibrateIMU();
        if (gamepad1.start) robot.drive.recalibrateIMU();
        if (gamepad2.a) robot.drive.cornerRelocalize(robot.miscSubsystems.allianceColor);

        // SOTM (u need to hold left bumper on gp1 for it to run)
        robot.vision.updateSOTM(gamepad1.left_bumper);

        // Intake TODO: maybe make enums for diff intake speeds?
        if (!robot.intake.isLocked(gamepad1.right_stick_button)) {
            if (gamepad1.left_trigger < 0.1) robot.intake.resetKickerDelay();

            boolean manualIntakePressed = gamepad1.right_trigger > IntakeSubsystem.MANUAL_INTAKE_TRIGGER_THRESHOLD;
            if (!manualIntakePressed) robot.intake.resetManualIntakeState();

            if (manualIntakePressed) {
                robot.intake.runManualIntake();

            } else if (gamepad1.b) {
                robot.intake.runKicker();

            } else if (gamepad1.a) {
                robot.intake.runIntakeReversed();

            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.runIntakeToShoot();

            } else if (gamepad1.dpad_up) {
                robot.intake.runDirectShoot();

            } else {
                robot.intake.stopIntake();
            }
        }
        if (gamepad1.dpadDownWasReleased() || gamepad2.dpadRightWasReleased()) robot.intake.startRecycle();
        robot.intake.updateRecycle();

        // Shooter
        if (gamepad1.backWasReleased()) robot.shooter.toggleShooter();
        if (gamepad2.dpad_up) robot.shooter.adjustFlywheelCorrection(-2);
        if (gamepad2.dpad_down) robot.shooter.adjustFlywheelCorrection(2);
        if (gamepad2.y) robot.shooter.resetFlywheelCorrection();
        if (gamepad2.rightBumperWasReleased()) robot.shooter.togglePowerMode();
        robot.shooter.updateFlywheel(robot.drive.getPredictedPos(), robot.miscSubsystems.getVoltage());

        // Turret
        if (gamepad2.backWasReleased()) robot.turret.adjustTurretCorrection(0.001);
        if (gamepad2.startWasReleased()) robot.turret.adjustTurretCorrection(-0.001);
        if (gamepad1.xWasReleased()) robot.turret.calibrateTurretAndRelocalize();
        if (gamepad1.y) robot.turret.zeroQuadrature();
        if (gamepad2.right_trigger > 0.7f) robot.turret.autoCorrectFromQuadrature();
        if (gamepad2.leftBumperWasReleased()) robot.turret.toggleUsingTurret();
        robot.turret.updateTracking();

        // random stuff
        if (gamepad1.dpad_left) robot.miscSubsystems.setAllianceBlue();
        if (gamepad1.dpad_right) robot.miscSubsystems.setAllianceRed();
        if (gamepad2.bWasReleased()) robot.miscSubsystems.toggleLowVoltage();
        if (gamepad2.right_stick_button) robot.miscSubsystems.enablePrisms();
        if (gamepad2.left_stick_button) robot.miscSubsystems.disablePrisms();

        robot.telemetry();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
