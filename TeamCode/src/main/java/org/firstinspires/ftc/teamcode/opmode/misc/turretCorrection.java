package org.firstinspires.ftc.teamcode.opmode.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;


@TeleOp
public class turretCorrection extends OpMode {

    CRServoImplEx turretServo;
    double totalCurrent;
    boolean tipped = false;

    @Override

    public void init() {
        turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");

    }

    @Override
    public void loop() {

        }


    }



