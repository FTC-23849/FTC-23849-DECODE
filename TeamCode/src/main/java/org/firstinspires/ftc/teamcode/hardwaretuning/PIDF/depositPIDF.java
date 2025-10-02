package org.firstinspires.ftc.teamcode.hardwaretuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class depositPIDF extends OpMode {
    public static double p = 0; // Old: 0.011
    public static double i = 0;
    public static double d = 0; // Old: 0.0002
    public static double f = 0; // Old: 0.00016

    public static int target = 0;
    public static double maxPowerConstant = 1.0;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);
    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int motorPos = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(2);

        robot.outtakeSlideMotor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSlideMotor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.outtakeSlideMotor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.outtakeSlideMotor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("target", target);
        telemetry.addData("max power", (f * motorPos) + maxPowerConstant);
    }

    @Override
    public void loop() {
        timer.reset();

        motorPos = (robot.outtakeSlideMotor_right.getCurrentPosition()
                + robot.outtakeSlideMotor_left.getCurrentPosition()) / 2;

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(target);

        double maxPower = (f * motorPos) + maxPowerConstant;
        double power = Range.clip(slidePIDF.calculate(motorPos, target), -maxPower, maxPower);

        robot.outtakeSlideMotor_right.setPower(power);
        robot.outtakeSlideMotor_left.setPower(power);

        telemetry.addData("encoder position", motorPos);
        telemetry.addData("target", target);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}