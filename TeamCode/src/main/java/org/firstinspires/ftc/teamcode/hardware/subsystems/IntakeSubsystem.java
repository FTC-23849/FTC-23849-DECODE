package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.RobotClass;

@Config
public class IntakeSubsystem {
    private final RobotClass robot;
    public DcMotorEx frontIntakeMotor, backIntakeMotor;
    public CRServoImplEx leftKickerServo, rightKickerServo;
    public ServoImplEx leftTongueServo, rightTongueServo;
    public DigitalChannel bottomLeftLaser, bottomRightLaser;

    public boolean intakeIsStalled = false;
    public boolean thirdBallPresent = false;
    public boolean recyclerIsRunning = false;
    
    private ElapsedTime recyclerTimer = new ElapsedTime();
    private ElapsedTime thirdBallConfirmTimer = new ElapsedTime();
    private ElapsedTime kickerStartDelayTimer = new ElapsedTime();
    
    private boolean started = false;
    private boolean intakeStarted = false;
    private boolean thirdBallConfirmTimerStarted = false;

    // Manual intake settings
    private boolean manualIntakeWasPressed = false;
    private boolean manualIntakeLatchedOff = false;
    private boolean manualIntakeAlreadyFullMode = false;
    private boolean manualIntakeThirdBallConfirmTimerStarted = false;
    private ElapsedTime manualIntakeThirdBallConfirmTimer = new ElapsedTime();
    private ElapsedTime manualIntakeAlreadyFullTimer = new ElapsedTime();

    public double targetIntakePower = 0;

    public static double RECYCLE_TONGUE_DOWN_MS = 400;
    public static double THIRD_BALL_CONFIRM_MS = 400;
    public static double RECYCLE_INTAKE_MAX_MS = 2000;
    public static double MANUAL_INTAKE_TRIGGER_THRESHOLD = 0.1;
    public static double MANUAL_INTAKE_THIRD_BALL_CONFIRM_MS = 200;
    public static double MANUAL_INTAKE_ALREADY_FULL_RUN_MS = 800;
    public static double frontIntakeAmpLimit = 8.5;
    public static double backIntakeAmpLimit = 8.5;

    public IntakeSubsystem(RobotClass robot) {
        this.robot = robot;
    }

    public void init() {
        frontIntakeMotor = robot.hardwareMap.get(DcMotorEx.class, "frontIntakeMotor");
        backIntakeMotor = robot.hardwareMap.get(DcMotorEx.class, "backIntakeMotor");
        frontIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftKickerServo = robot.hardwareMap.get(CRServoImplEx.class, "leftKickerServo");
        rightKickerServo = robot.hardwareMap.get(CRServoImplEx.class, "rightKickerServo");
        rightKickerServo.setDirection(CRServoImplEx.Direction.REVERSE);

        leftTongueServo = robot.hardwareMap.get(ServoImplEx.class, "leftGateServo");
        rightTongueServo = robot.hardwareMap.get(ServoImplEx.class, "rightGateServo");
        leftTongueServo.setDirection(ServoImplEx.Direction.REVERSE);

        bottomLeftLaser = robot.hardwareMap.get(DigitalChannel.class, "bottomLeftLaser");
        bottomRightLaser = robot.hardwareMap.get(DigitalChannel.class, "bottomRightLaser");
        bottomLeftLaser.setMode(DigitalChannel.Mode.INPUT);
        bottomRightLaser.setMode(DigitalChannel.Mode.INPUT);
        
        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);
    }

    public void update() {
        thirdBallPresent = bottomLeftLaser.getState() && bottomRightLaser.getState();
        
        // Stall detection logic
        frontIntakeMotor.setCurrentAlert(frontIntakeAmpLimit, CurrentUnit.AMPS);
        backIntakeMotor.setCurrentAlert(backIntakeAmpLimit, CurrentUnit.AMPS);
        boolean intakeStallFront = frontIntakeMotor.isOverCurrent();
        boolean intakeStallBack = backIntakeMotor.isOverCurrent();

        if (intakeStallFront || intakeStallBack) {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            intakeIsStalled = true;
        } else {
            intakeIsStalled = false;
        }
    }

    public boolean isLocked(boolean rightStickButton) {
        return recyclerIsRunning || rightStickButton;
    }

    public void resetKickerDelay() {
        kickerStartDelayTimer.reset();
    }

    public void runManualIntake() {
        boolean disabled = intakeIsStalled || robot.drive.isDtStalled();
        if (shouldRunManualIntake(true) && !disabled) {
            frontIntakeMotor.setPower(-Globals.frontIntakeIntakeSpeed);
            backIntakeMotor.setPower(Globals.backIntakeIntakeSpeed);
            targetIntakePower = Globals.frontIntakeIntakeSpeed;
        } else {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            targetIntakePower = 0;
        }
        leftTongueServo.setPosition(Globals.tongueIntake);
        rightTongueServo.setPosition(Globals.tongueIntake);
    }

    public void runKicker() {
        leftKickerServo.setPower(1.0);
        rightKickerServo.setPower(1.0);
    }

    public void runIntakeReversed() {
        if (intakeIsStalled || robot.drive.isDtStalled()) return;
        frontIntakeMotor.setPower(-Globals.frontIntakeReverseSpeed);
        backIntakeMotor.setPower(Globals.backIntakeReverseSpeed);
        targetIntakePower = -Globals.frontIntakeReverseSpeed;
    }

    public void runIntakeToShoot() {
        boolean disabled = intakeIsStalled || robot.drive.isDtStalled();
        leftTongueServo.setPosition(Globals.tongueShoot);
        rightTongueServo.setPosition(Globals.tongueShoot);
        if (kickerStartDelayTimer.milliseconds() > Globals.kickerStartDelay && !disabled) {
            leftKickerServo.setPower(Globals.rollerKickerShoot);
            rightKickerServo.setPower(Globals.rollerKickerShoot);
            frontIntakeMotor.setPower(-Globals.frontIntakeShootSpeed * robot.shooter.shotSpeed);
            backIntakeMotor.setPower(-Globals.backIntakeShootSpeed * robot.shooter.shotSpeed);
            targetIntakePower = -Globals.frontIntakeShootSpeed * robot.shooter.shotSpeed;
        } else {
            if (!disabled) frontIntakeMotor.setPower(0.7);
            leftKickerServo.setPower(Globals.rollerKickerShoot);
            rightKickerServo.setPower(Globals.rollerKickerShoot);
            targetIntakePower = 0.7;
        }
    }

    public void runDirectShoot() {
        leftTongueServo.setPosition(Globals.tongueShoot);
        rightTongueServo.setPosition(Globals.tongueShoot);
        leftKickerServo.setPower(Globals.rollerKickerShoot);
        rightKickerServo.setPower(Globals.rollerKickerShoot);
    }

    public void startRecycle() {
        if (recyclerIsRunning) return;
        recyclerIsRunning = true;
        started = false;
    }

    public void updateRecycle() {
        if (recyclerIsRunning) {
            if (!started) {
                started = true;
                recyclerTimer.reset();
            }

            double t = recyclerTimer.milliseconds();
            if (t < RECYCLE_TONGUE_DOWN_MS) {
                leftTongueServo.setPosition(Globals.tongueRecycle);
                rightTongueServo.setPosition(Globals.tongueRecycle);
                leftKickerServo.setPower(Globals.rollerKickerRecycle);
                rightKickerServo.setPower(Globals.rollerKickerRecycle);
                frontIntakeMotor.setPower(0);
                backIntakeMotor.setPower(0);
                targetIntakePower = 0;
            } else {
                leftTongueServo.setPosition(Globals.tongueIntake);
                rightTongueServo.setPosition(Globals.tongueIntake);
                leftKickerServo.setPower(0);
                rightKickerServo.setPower(0);
                if (!intakeIsStalled) {
                    frontIntakeMotor.setPower(-1.0);
                    backIntakeMotor.setPower(-1.0);
                    targetIntakePower = -1;
                }

                if (t - RECYCLE_TONGUE_DOWN_MS >= RECYCLE_INTAKE_MAX_MS || (thirdBallPresent && t - RECYCLE_TONGUE_DOWN_MS > THIRD_BALL_CONFIRM_MS)) {
                    recyclerIsRunning = false;
                    stopIntake();
                }
            }
        }
    }

    private boolean shouldRunManualIntake(boolean intakePressed) {
        if (!intakePressed) {
            resetManualIntakeState();
            return false;
        }

        // stops intake after 3rd ball is detected (no longer functional)
        if (!manualIntakeWasPressed) {
            manualIntakeWasPressed = true;
            manualIntakeLatchedOff = false;
            manualIntakeThirdBallConfirmTimerStarted = false;
            manualIntakeThirdBallConfirmTimer.reset();

            manualIntakeAlreadyFullMode = thirdBallPresent;
            manualIntakeAlreadyFullTimer.reset();
        }

        if (manualIntakeLatchedOff) {
            return false;
        }

        if (manualIntakeAlreadyFullMode) {
            if (manualIntakeAlreadyFullTimer.milliseconds() >= MANUAL_INTAKE_ALREADY_FULL_RUN_MS) {
                manualIntakeLatchedOff = true;
                manualIntakeAlreadyFullMode = false;
                manualIntakeThirdBallConfirmTimerStarted = false;
                manualIntakeThirdBallConfirmTimer.reset();
                return false;
            }
            return true;
        }

        if (thirdBallPresent) {
            if (!manualIntakeThirdBallConfirmTimerStarted) {
                manualIntakeThirdBallConfirmTimerStarted = true;
                manualIntakeThirdBallConfirmTimer.reset();
            }
            if (manualIntakeThirdBallConfirmTimer.milliseconds() >= MANUAL_INTAKE_THIRD_BALL_CONFIRM_MS) {
                manualIntakeLatchedOff = true;
                return false;
            }
        } else {
            manualIntakeThirdBallConfirmTimerStarted = false;
            manualIntakeThirdBallConfirmTimer.reset();
        }

        return true;
    }

    public void resetManualIntakeState() {
        manualIntakeWasPressed = false;
        manualIntakeLatchedOff = false;
        manualIntakeThirdBallConfirmTimerStarted = false;
        manualIntakeAlreadyFullMode = false;
        manualIntakeThirdBallConfirmTimer.reset();
        manualIntakeAlreadyFullTimer.reset();
    }

    public void stopIntake() {
        leftKickerServo.setPower(0);
        rightKickerServo.setPower(0);
        frontIntakeMotor.setPower(0);
        backIntakeMotor.setPower(0);
        targetIntakePower = 0;
    }

    public void stop() {
        stopIntake();
    }

    public void telemetry() {
        robot.telemetry.addData("Intake Stall", intakeIsStalled);
        robot.telemetry.addData("Third Ball", thirdBallPresent);
    }
}
