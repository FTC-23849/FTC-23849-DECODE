package org.firstinspires.ftc.teamcode.hardwaretuning.motor;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class motorEncoders extends OpMode {
    public static boolean RESET_ALL_ENCODERS = true;

    private final Robot robot = Robot.getInstance();
    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Resets command scheduler
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);

        telemetry.addData("rightEncoder", robot.outtakeSlideMotor_right.getCurrentPosition());
        telemetry.addData("leftEncoder", robot.outtakeSlideMotor_left.getCurrentPosition());
    }

    @Override
    public void loop() {
        timer.reset();

        if (RESET_ALL_ENCODERS) {
            robot.outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("rightEncoder", robot.outtakeSlideMotor_right.getCurrentPosition());
        telemetry.addData("leftEncoder", robot.outtakeSlideMotor_left.getCurrentPosition());

        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}