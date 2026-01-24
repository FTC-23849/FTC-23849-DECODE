package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vision.visionTools;

public class PoseStorage {

    public static Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

}
