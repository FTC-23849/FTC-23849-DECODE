package org.firstinspires.ftc.teamcode.PedroAuto.Subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;

public class Intake {

    private final DcMotorEx frontMotor, backMotor;
    private final CRServoImplEx leftKickerServo, rightKickerServo;
    private final ServoImplEx leftTongueServo, rightTongueServo;

    public static double INTAKE_SPEED = 1.0;

    public Intake (HardwareMap hardwareMap) {

        frontMotor = hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        backMotor = hardwareMap.get(DcMotorEx.class, "backIntakeMotor");

        leftKickerServo = hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        leftKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTongueServo = hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

    }

    public void init() {

        frontMotor.setPower(0.0);
        backMotor.setPower(0.0);
        leftKickerServo.setPower(0.0);
        rightKickerServo.setPower(0.0);
        rightTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);

    }

    public Command intakeOn() {

        return Command.build()
                .setStart(() -> {
                    frontMotor.setPower(INTAKE_SPEED);
                    backMotor.setPower(INTAKE_SPEED);
                    leftTongueServo.setPosition(Globals.tongueIntake);
                    rightTongueServo.setPosition(Globals.tongueIntake);
                })
                .setDone(() -> true)
                .requiring(frontMotor, backMotor);

    }

    public Command intakeOff() {

        return Command.build()
                .setStart(() -> {
                    frontMotor.setPower(0.0);
                    backMotor.setPower(0.0);
                    leftTongueServo.setPosition(Globals.tongueIntake);
                    rightTongueServo.setPosition(Globals.tongueIntake);
                })
                .setDone(() -> true)
                .requiring(frontMotor, backMotor);

    }

    public Command transferOn() {

        return Command.build()
                .setStart(() -> {
                    frontMotor.setPower(INTAKE_SPEED);
                    backMotor.setPower(INTAKE_SPEED);
                    leftKickerServo.setPower(1.0);
                    rightKickerServo.setPower(1.0);
                    leftTongueServo.setPosition(Globals.tongueShoot);
                    rightTongueServo.setPosition(Globals.tongueShoot);
                })
                .setDone(() -> true)
                .requiring(frontMotor, backMotor, leftKickerServo, rightKickerServo);

    }

    public Command transferOff() {

        return Command.build()
                .setStart(() -> {
                    frontMotor.setPower(0.0);
                    backMotor.setPower(0.0);
                    leftKickerServo.setPower(0.0);
                    rightKickerServo.setPower(0.0);
                    leftTongueServo.setPosition(Globals.tongueIntake);
                    rightTongueServo.setPosition(Globals.tongueIntake);
                })
                .setDone(() -> true)
                .requiring(frontMotor, backMotor, leftKickerServo, rightKickerServo);

    }

    public Command kickersOn() {

        return Command.build()
                .setStart(() -> {
                    leftKickerServo.setPower(1.0);
                    rightKickerServo.setPower(1.0);
                    leftTongueServo.setPosition(Globals.tongueShoot);
                    rightTongueServo.setPosition(Globals.tongueShoot);
                })
                .setDone(() -> true)
                .requiring(leftKickerServo, rightKickerServo);

    }

    public Command kickersOff() {

        return Command.build()
                .setStart(() -> {
                    leftKickerServo.setPower(0.0);
                    rightKickerServo.setPower(0.0);
                    leftTongueServo.setPosition(Globals.tongueIntake);
                    rightTongueServo.setPosition(Globals.tongueIntake);
                })
                .setDone(() -> true)
                .requiring(leftKickerServo, rightKickerServo);

    }

}
