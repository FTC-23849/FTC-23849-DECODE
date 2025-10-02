package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.PoseUpdater;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.w3c.dom.ls.LSException;

public class Robot {

    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    public DcMotorEx outtakeSlideMotor_left;
    public DcMotorEx outtakeSlideMotor_right;

    public DcMotorEx intakeMotor;
    public Servo intakeTurret;
    public Servo intakePivot;
    public Servo intakeWrist;
    public Servo intakeClaw;
    public Servo outtakePivotLeft;
    public Servo outtakePivotRight;
    public Servo outtakeTurret;
    public Servo outtakeWrist;
    public Servo outtakeClaw;
    public ServoImplEx leftPTO;
    public ServoImplEx rightPTO;
    public Servo displayLight;
    public DistanceSensor alignerDistanceSensor;

    public Deposit deposit;
    public Intake intake;
    public Drive drive;

    public Follower follower;
    public PoseUpdater poseUpdater;

    // Singleton Implementation

    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {

        leftFrontMotor  = hardwareMap.get(DcMotorEx.class,"LF");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RF");
        leftBackMotor = hardwareMap.get(DcMotorEx.class,"LB");
        rightBackMotor = hardwareMap.get(DcMotorEx.class,"RB");
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(BRAKE);
        leftBackMotor.setZeroPowerBehavior(BRAKE);
        rightFrontMotor.setZeroPowerBehavior(BRAKE);
        rightBackMotor.setZeroPowerBehavior(BRAKE);


        outtakeSlideMotor_left = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_left");
        outtakeSlideMotor_right = hardwareMap.get(DcMotorEx.class,"outtakeSlideMotor_right");
        outtakeSlideMotor_left.setDirection(DcMotorEx.Direction.REVERSE);

        outtakeSlideMotor_left.setZeroPowerBehavior(BRAKE);
        outtakeSlideMotor_right.setZeroPowerBehavior(BRAKE);

        outtakeSlideMotor_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideMotor_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeTurret = hardwareMap.get(Servo.class, "intakeTurret");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeTurret = hardwareMap.get(Servo.class, "outtakeTurret");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        leftPTO = hardwareMap.get(ServoImplEx.class, "leftPto");
        rightPTO = hardwareMap.get(ServoImplEx.class, "rightPto");
        displayLight = hardwareMap.get(Servo.class, "displayLight");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        alignerDistanceSensor = hardwareMap.get(DistanceSensor.class, "alignerDistanceSensor");

        intake = new Intake();
        deposit = new Deposit();
        drive = new Drive();

    }

    public enum RobotState {
        INITIALIZED,
        SAMPLE_INTAKING,
        TRANSFERRING,
        TRANSFERRED,
        SAMPLE_SCORING,
        SPECIMEN_INTAKING,
        SPECIMEN_SCORING
    }

    public static RobotState robotState;

}
