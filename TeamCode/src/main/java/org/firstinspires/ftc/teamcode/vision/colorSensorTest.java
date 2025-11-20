package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation. YawPitchRollAngles;

@Autonomous
public class colorSensorTest extends OpMode {
    private NormalizedColorSensor  leftColorSensor;
    private NormalizedColorSensor  rightColorSensor;



    @Override
    public void init() {
        leftColorSensor= hardwareMap.get(NormalizedColorSensor.class, "leftIntakeColorSensor");
        rightColorSensor= hardwareMap.get(NormalizedColorSensor.class, "rightIntakeColorSensor");
    }

    @Override
    public void start() {

    }



    @Override
    public void loop() {
        NormalizedRGBA leftColor = leftColorSensor.getNormalizedColors();
        NormalizedRGBA rightColor = rightColorSensor.getNormalizedColors();
        telemetry.addData("Hue", JavaUtil.colorToHue(rightColor.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(rightColor.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(rightColor.toColor()));
        telemetry.addData("leftHue", JavaUtil.colorToHue(leftColor.toColor()));
        telemetry.addData("leftSaturation", "%.3f", JavaUtil.colorToSaturation(leftColor.toColor()));
        telemetry.addData("leftValue", "%.3f", JavaUtil.colorToValue(leftColor.toColor()));
        }
    }
