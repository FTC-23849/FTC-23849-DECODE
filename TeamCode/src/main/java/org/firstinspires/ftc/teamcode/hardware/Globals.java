package org.firstinspires.ftc.teamcode.hardware;

public class Globals {

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

    // Outtake Pviot
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
