package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    // VPID

    public static double VKp = 0.01;
    public static double VKi = 0.003;
    public static double VKd = 0;
    public static double VkS = 0;
    public static double VkV = 0.00042;
    public static double sec = 0;

    public static double KpMaintain = 0.003;
    public static double KiMaintain = 0.002;

    public static double KpDriveRecovery = 0.03;

    public static double KpRecovery = 0;
    public static double KiRecovery = 0.001;
    public static double KsRecovery = 1;
    public static double KsRecoveryClose = 0.55;
    public static double KvFF = 0.00042;
    public static double KsFF = 0.055 ;


    public static double recoveryThreshold = 60;
    public static double maintainThreshold = 40;
    public static double defaultVoltage = 13.15;

    // Front Intake
    public static double frontIntakeIntakeSpeed = 1;
    public static double frontIntakeShootSpeed = 1; //-0.7
    public static double frontIntakeRecycleSpeed = 0.5;//-0.67
    public static double frontIntakeReverseSpeed = -1;

    // Back Intake
    public static double backIntakeIntakeSpeed = -1;
    public static double backIntakeReverseSpeed = 1;
    public static double backIntakeShootSpeed = 1;
    public static double backIntakeRecycleSpeed = -0.3;

    public static double backRollersMaxPower = 1;
    public static double backRollersReverse = -1;

    // Kicker
    public static double rollerKickerShoot = -1.0;
    public static double rollerKickerRecycle = 1.0;
    public static double tongueIntake = 0.03;
    public static double tongueShoot = 0.0;
    public static double tongueRecycle = 0.18;

    public static double kickerShoot = -0.6;
    public static double kickerRecycle = 0.4;

    public static double defaultKickerLocation = 0.2;
    public static double defaultKickerLocationAuto = 0.9;
    public static double defaultKickerStopDelay = 500;

    // Kicker PID
    public static double KICKER_kP = 5;
    public static double KICKER_kD = 0.02;

    public static double KICKER_ZERO = 2.8;

    public static double KICKER_IDLE = 0.07;
    public static double KICKER_RECYCLE = 0.31;

    //new kicker
    public static double kickerStartDelay = 100;


    // Shooter
    public static double defaultCloseZonePower = -0.67;
    public static double defaultCloseZonePowerAuto = -0.71;
    public static double defaultFarZonePower = -0.92;
    public static double defaultFarZonePowerAuto = -0.92;


    // Tipper
    public static double tipperRetracted = 0.23;
    public static double tipperExtended = 0.5;





    //OLD DO NOT USE
    // Intake

    // Intake Turret
    public static double INTAKE_TURRET_TRANSFER = 0.89;
    public static double INTAKE_TURRET_PICKUP_LEFT = 0.62;
    public static double INTAKE_TURRET_PICKUP_RIGHT = 0.12; // was 12
    public static double INTAKE_TURRET_PICKUP_STRAIGHT = 0.37;
    public static double INTAKE_TURRET_DROP_LEFT = 0.67;
    public static double INTAKE_TURRET_DROP_RIGHT = 0.03;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER = 0.37;
    public static double INTAKE_PIVOT_PICKUP_READY = 0.11;
    public static double INTAKE_PIVOT_PICKUP = 0.06;
    public static double INTAKE_PIVOT_DROP = 0.25;
    public static double INTAKE_PIVOT_AVOID = 0.7;

    // Intake Wrist
    public static double INTAKE_WRIST_STRAIGHT = 0.4;// 0.07 works, 0.574 should, done math to test
    public static double INTAKE_WRIST_LEFT90 = 0.66;
    public static double INTAKE_WRIST_RIGHT90 = 0.07;

    // Intake Claw
    public static double INTAKE_CLAW_OPEN = 0.6;
    public static double INTAKE_CLAW_CLOSE = 0.40;

    // Extendo
    public static int INTAKE_MOTOR_RETRACT = 0;
    public static int INTAKE_MOTOR_MAX_EXTENSION = 750;
    public static int INTAKE_LOWER_ENCODER_TICKS = 250;


    // Outtake

    // Outtake Pivot
    public static double OUTTAKE_PIVOT_TRANSFER = 0;
    public static double OUTTAKE_PIVOT_SAMPLE_SCORE = 0.8;
    public static double OUTTAKE_PIVOT_SPECIMEN_PICKUP = 1;
    public static double OUTTAKE_PIVOT_SPECIMEN_SCORE = 0.35;
    public static double OUTTAKE_PIVOT_HANG = 0.6;

    // Outtake Turret
    public static double OUTTAKE_TURRET_STRAIGHT = 0.49;
    public static double OUTTAKE_TURRET_LEFT90 = 0.17;
    public static double OUTTAKE_TURRET_RIGHT90 = 0.85;

    // Outtake Wrist
    public static double OUTTAKE_WRIST_TRANSFER = 0.85;
    public static double OUTTAKE_WRIST_SPECIMEN_PICKUP = 0.8;
    public static double OUTTAKE_WRIST_SAMPLE_SCORE = 0.5;
    public static double OUTTAKE_WRIST_HIGH_SPECIMEN_SCORE = 0.63;
    public static double OUTTAKE_WRIST_LOW_SPECIMEN_SCORE = 0.6;
    public static double OUTTAKE_WRIST_HANG = 0.5;

    // Outtake Claw
    public static double OUTTAKE_CLAW_OPEN = 0.4;
    public static double OUTTAKE_CLAW_CLOSE = 0.51;

    // Outtake Slides
    public static int OUTTAKE_SLIDES_RETRACT = 0;
    public static int OUTTAKE_SLIDES_MAX_EXTENSION = 800;
    public static int OUTTAKE_SLIDES_SAMPLE_SCORE = 800;
    public static int OUTTAKE_SLIDES_HIGH_SPECIMEN_SCORE = 375;
    public static int OUTTAKE_SLIDES_LOW_SPECIMEN_SCORE = 0;


    // PTO
    public static double LEFT_PTO_DISENGAGE = 0.25;
    public static double LEFT_PTO_ENGAGE = 1; // 0.46 is loose // 0.51 is still slipping
    public static double RIGHT_PTO_DISENGAGE = 0.5;
    public static double RIGHT_PTO_ENGAGE = 0.0; // 0.3 is loose, 0.2 is still too lose


    // Hang
    public static int OUTTAKE_MOTOR_L2_PREPARE = -500;
    public static int DRIVE_L2_HANG = 0;
    public static int OUTTAKE_MOTOR_L3_PREPARE = -850;
    public static int DRIVE_L3_HANG = 0;
    public static double DRIVE_HANG_LOWER = -0.2;


}