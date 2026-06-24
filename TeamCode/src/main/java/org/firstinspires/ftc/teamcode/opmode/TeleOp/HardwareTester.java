package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.List;

/**
 * Hardware Tester
 * ----------------
 * Menu-driven test OpMode for Team 23849 DECODE robot.
 *
 * NAVIGATION (gamepad1):
 *   - dpad_up / dpad_down : move selection cursor
 *   - A                   : select / enter a menu level
 *   - B                   : go back a menu level
 *
 * Screen 1: pick a CATEGORY (Drivetrain, Intake, Shooter, Servos, CRServos,
 *           Sensors, Other)
 * Screen 2: pick a specific PART in that category
 * Screen 3: CONTROL the selected part. The controls shown depend on part type:
 *
 *   MOTOR (DcMotorEx):
 *     - right_trigger : drive forward (power = trigger)
 *     - left_trigger  : drive reverse (power = trigger)
 *     - X             : toggle BRAKE / FLOAT zero-power behavior
 *     - Y             : reset encoder
 *     Telemetry: power, velocity, current, encoder position
 *
 *   SERVO (ServoImplEx, position 0..1):
 *     - dpad_left  : position -= step
 *     - dpad_right : position += step
 *     - left/right_bumper : decrease / increase step size
 *     - X : snap to 0.0   Y : snap to 0.5   right_stick_button : snap to 1.0
 *     Telemetry: commanded position, step
 *
 *   CRSERVO (continuous, power -1..1):
 *     - right_trigger : forward    left_trigger : reverse
 *     - X : stop (power 0)
 *     Telemetry: commanded power
 *
 *   SENSOR (read-only): live telemetry for color / digital / analog / voltage
 */
@TeleOp(name = "Hardware Tester", group = "test")
public class HardwareTester extends OpMode {

    // ---------- device type tags ----------
    private enum DeviceType { MOTOR, SERVO, CRSERVO, DIGITAL, ANALOG, COLOR, VOLTAGE, SPECIAL }

    private static class Device {
        final String name;
        final DeviceType type;
        final Object handle;          // the actual hardware object (may be null for SPECIAL)

        Device(String name, DeviceType type, Object handle) {
            this.name = name;
            this.type = type;
            this.handle = handle;
        }
    }

    private static class Category {
        final String name;
        final List<Device> devices = new ArrayList<>();
        Category(String name) { this.name = name; }
    }

    // ---------- menu state ----------
    private enum Screen { CATEGORY, PART, CONTROL }
    private Screen screen = Screen.CATEGORY;

    private final List<Category> categories = new ArrayList<>();
    private int catCursor = 0;
    private int partCursor = 0;

    // ---------- per-part runtime state ----------
    private double servoStep = 0.02;
    private double servoTarget = 0.5;
    private boolean motorBrake = true;

    // ---------- edge-detect helpers ----------
    private boolean lastUp, lastDown, lastA, lastB, lastX, lastY;
    private boolean lastLB, lastRB, lastRSB;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---------- hardware handles we may need to read ----------
    private VoltageSensor controlHubVoltage;

    @Override
    public void init() {
        // ---- Drivetrain ----
        Category drive = new Category("Drivetrain");
        addMotor(drive, "LF", true);
        addMotor(drive, "RF", false);
        addMotor(drive, "LB", true);
        addMotor(drive, "RB", false);

        // ---- Intake ----
        Category intake = new Category("Intake");
        addMotor(intake, "frontIntakeMotor", true);
        addMotor(intake, "backIntakeMotor", true);

        // ---- Shooter ----
        Category shooter = new Category("Shooter");
        addMotorRaw(shooter, "leftShooterMotor", false, true);   // FLOAT, no encoder run
        addMotorRaw(shooter, "rightShooterMotor", true, true);

        // ---- Servos (positional) ----
        Category servos = new Category("Servos");
        addServo(servos, "leftTurretServo", false, true);
        addServo(servos, "rightTurretServo", false, true);
        addServo(servos, "frontTurretServo", false, true);
        addServo(servos, "leftHood", true, false);
        addServo(servos, "rightHood", false, false);
        addServo(servos, "leftTipper", false, false);
        addServo(servos, "rightTipper", false, false);
        addServo(servos, "leftGateServo", true, false);   // left tongue
        addServo(servos, "rightGateServo", false, false);  // right tongue
        addServo(servos, "light", false, false);
        addServo(servos, "zoneLight", false, false);

        // ---- CRServos (continuous) ----
        Category crservos = new Category("CR Servos");
        addCRServo(crservos, "leftKickerServo", false);
        addCRServo(crservos, "rightKickerServo", true);

        // ---- Sensors ----
        Category sensors = new Category("Sensors");
        addColor(sensors, "colorLeft1");
        addColor(sensors, "colorRight1");
        addColor(sensors, "colorLeft2");
        addColor(sensors, "colorRight2");
        addDigital(sensors, "bottomLeftLaser");
        addDigital(sensors, "bottomRightLaser");
        addAnalog(sensors, "turretEncoder");
        addAnalog(sensors, "leftKickerEncoder");
        controlHubVoltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        sensors.devices.add(new Device("Control Hub Voltage", DeviceType.VOLTAGE, controlHubVoltage));

        // ---- Other (pinpoint, limelight) ----
        Category other = new Category("Other");
        try {
            GoBildaPinpointDriver pinpoint =
                    hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            other.devices.add(new Device("pinpoint", DeviceType.SPECIAL, pinpoint));
        } catch (Exception ignored) {}
        try {
            Limelight3A ll = hardwareMap.get(Limelight3A.class, "Limelight");
            other.devices.add(new Device("Limelight", DeviceType.SPECIAL, ll));
        } catch (Exception ignored) {}

        categories.add(drive);
        categories.add(intake);
        categories.add(shooter);
        categories.add(servos);
        categories.add(crservos);
        categories.add(sensors);
        categories.add(other);

        telemetry.addLine("Hardware Tester ready.");
        telemetry.addLine("dpad to move, A select, B back.");
        telemetry.update();
    }

    // ---------------- builders (with try/catch so a missing device skips, not crashes) ----------------

    private void addMotor(Category cat, String name, boolean reverse) {
        addMotorRaw(cat, name, reverse, false);
    }

    private void addMotorRaw(Category cat, String name, boolean reverse, boolean floatZero) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            m.setZeroPowerBehavior(floatZero ? DcMotor.ZeroPowerBehavior.FLOAT
                    : DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cat.devices.add(new Device(name, DeviceType.MOTOR, m));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.MOTOR, null));
        }
    }

    private void addServo(Category cat, String name, boolean reverse, boolean wideRange) {
        try {
            ServoImplEx s = hardwareMap.get(ServoImplEx.class, name);
            s.setDirection(reverse ? ServoImplEx.Direction.REVERSE
                    : ServoImplEx.Direction.FORWARD);
            if (wideRange) s.setPwmRange(new PwmControl.PwmRange(500, 2500));
            cat.devices.add(new Device(name, DeviceType.SERVO, s));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.SERVO, null));
        }
    }

    private void addCRServo(Category cat, String name, boolean reverse) {
        try {
            CRServoImplEx s = hardwareMap.get(CRServoImplEx.class, name);
            s.setDirection(reverse ? CRServoImplEx.Direction.REVERSE
                    : CRServoImplEx.Direction.FORWARD);
            cat.devices.add(new Device(name, DeviceType.CRSERVO, s));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.CRSERVO, null));
        }
    }

    private void addColor(Category cat, String name) {
        try {
            NormalizedColorSensor c = hardwareMap.get(NormalizedColorSensor.class, name);
            if (c instanceof SwitchableLight) ((SwitchableLight) c).enableLight(true);
            c.setGain(1f);
            cat.devices.add(new Device(name, DeviceType.COLOR, c));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.COLOR, null));
        }
    }

    private void addDigital(Category cat, String name) {
        try {
            DigitalChannel d = hardwareMap.get(DigitalChannel.class, name);
            d.setMode(DigitalChannel.Mode.INPUT);
            cat.devices.add(new Device(name, DeviceType.DIGITAL, d));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.DIGITAL, null));
        }
    }

    private void addAnalog(Category cat, String name) {
        try {
            AnalogInput a = hardwareMap.get(AnalogInput.class, name);
            cat.devices.add(new Device(name, DeviceType.ANALOG, a));
        } catch (Exception e) {
            cat.devices.add(new Device(name + " (MISSING)", DeviceType.ANALOG, null));
        }
    }

    // ---------------- main loop ----------------

    @Override
    public void loop() {
        // read edges once per loop
        boolean up   = gamepad1.dpad_up   && !lastUp;
        boolean down = gamepad1.dpad_down && !lastDown;
        boolean a    = gamepad1.a && !lastA;
        boolean b    = gamepad1.b && !lastB;
        boolean x    = gamepad1.x && !lastX;
        boolean y    = gamepad1.y && !lastY;
        boolean lb   = gamepad1.left_bumper  && !lastLB;
        boolean rb   = gamepad1.right_bumper && !lastRB;
        boolean rsb  = gamepad1.right_stick_button && !lastRSB;

        switch (screen) {
            case CATEGORY: handleCategory(up, down, a); break;
            case PART:     handlePart(up, down, a, b);  break;
            case CONTROL:  handleControl(a, b, x, y, lb, rb, rsb); break;
        }

        telemetry.addLine("loop ms: " + (int) loopTimer.milliseconds());
        loopTimer.reset();
        telemetry.update();

        lastUp = gamepad1.dpad_up;     lastDown = gamepad1.dpad_down;
        lastA = gamepad1.a;            lastB = gamepad1.b;
        lastX = gamepad1.x;            lastY = gamepad1.y;
        lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;
        lastRSB = gamepad1.right_stick_button;
    }

    // ----- screen 1: choose category -----
    private void handleCategory(boolean up, boolean down, boolean a) {
        if (up)   catCursor = wrap(catCursor - 1, categories.size());
        if (down) catCursor = wrap(catCursor + 1, categories.size());
        if (a) {
            partCursor = 0;
            screen = Screen.PART;
            return;
        }

        telemetry.addLine("=== SELECT CATEGORY ===");
        telemetry.addLine("dpad: move   A: select");
        telemetry.addLine("");
        for (int i = 0; i < categories.size(); i++) {
            telemetry.addLine((i == catCursor ? " > " : "   ")
                    + categories.get(i).name
                    + " (" + categories.get(i).devices.size() + ")");
        }
    }

    // ----- screen 2: choose part -----
    private void handlePart(boolean up, boolean down, boolean a, boolean b) {
        Category cat = categories.get(catCursor);
        if (up)   partCursor = wrap(partCursor - 1, cat.devices.size());
        if (down) partCursor = wrap(partCursor + 1, cat.devices.size());
        if (b) { screen = Screen.CATEGORY; return; }
        if (a) {
            // init defaults for the part we just entered
            servoStep = 0.02;
            servoTarget = 0.5;
            motorBrake = true;
            screen = Screen.CONTROL;
            return;
        }

        telemetry.addLine("=== " + cat.name + " ===");
        telemetry.addLine("dpad: move   A: select   B: back");
        telemetry.addLine("");
        for (int i = 0; i < cat.devices.size(); i++) {
            Device d = cat.devices.get(i);
            telemetry.addLine((i == partCursor ? " > " : "   ")
                    + d.name + "  [" + d.type + "]");
        }
    }

    // ----- screen 3: control the part -----
    private void handleControl(boolean a, boolean b, boolean x, boolean y,
                               boolean lb, boolean rb, boolean rsb) {
        Device d = categories.get(catCursor).devices.get(partCursor);
        if (b) { stopDevice(d); screen = Screen.PART; return; }

        telemetry.addLine("=== TESTING: " + d.name + " ===");
        telemetry.addLine("B: stop & back");
        telemetry.addLine("");

        if (d.handle == null) {
            telemetry.addLine("Device not found on hardwareMap.");
            return;
        }

        switch (d.type) {
            case MOTOR:   controlMotor(d, x, y); break;
            case SERVO:   controlServo(d, x, y, lb, rb, rsb); break;
            case CRSERVO: controlCRServo(d, x); break;
            case DIGITAL: readDigital(d); break;
            case ANALOG:  readAnalog(d); break;
            case COLOR:   readColor(d); break;
            case VOLTAGE: readVoltage(d); break;
            case SPECIAL: readSpecial(d); break;
        }
    }

    // ---------------- per-type controllers ----------------

    private void controlMotor(Device d, boolean toggleBrake, boolean resetEnc) {
        DcMotorEx m = (DcMotorEx) d.handle;

        if (toggleBrake) {
            motorBrake = !motorBrake;
            m.setZeroPowerBehavior(motorBrake ? DcMotor.ZeroPowerBehavior.BRAKE
                    : DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (resetEnc) {
            DcMotor.RunMode prev = m.getMode();
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double power = gamepad1.right_trigger - gamepad1.left_trigger; // -1..1
        m.setPower(power);

        telemetry.addLine("RT: fwd   LT: rev   X: brake/float   Y: reset enc");
        telemetry.addData("power", "%.2f", power);
        telemetry.addData("zeroPower", motorBrake ? "BRAKE" : "FLOAT");
        telemetry.addData("velocity", "%.1f", m.getVelocity());
        telemetry.addData("encoder", m.getCurrentPosition());
        telemetry.addData("current (A)", "%.2f", m.getCurrent(CurrentUnit.AMPS));
    }

    private void controlServo(Device d, boolean snap0, boolean snap5,
                              boolean smaller, boolean bigger, boolean snap1) {
        ServoImplEx s = (ServoImplEx) d.handle;

        if (gamepad1.dpad_left)  servoTarget -= servoStep;
        if (gamepad1.dpad_right) servoTarget += servoStep;
        if (smaller) servoStep = Math.max(0.001, servoStep / 2.0);
        if (bigger)  servoStep = Math.min(0.2, servoStep * 2.0);
        if (snap0) servoTarget = 0.0;
        if (snap5) servoTarget = 0.5;
        if (snap1) servoTarget = 1.0;

        servoTarget = Math.max(0.0, Math.min(1.0, servoTarget));
        s.setPosition(servoTarget);

        telemetry.addLine("dpad L/R: move   bumpers: step size");
        telemetry.addLine("X: 0.0   Y: 0.5   RstickBtn: 1.0");
        telemetry.addData("commanded pos", "%.3f", servoTarget);
        telemetry.addData("step", "%.3f", servoStep);
    }

    private void controlCRServo(Device d, boolean stop) {
        CRServoImplEx s = (CRServoImplEx) d.handle;
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        if (stop) power = 0;
        s.setPower(power);

        telemetry.addLine("RT: fwd   LT: rev   X: stop");
        telemetry.addData("commanded power", "%.2f", power);
    }

    private void readDigital(Device d) {
        DigitalChannel ch = (DigitalChannel) d.handle;
        telemetry.addData("state", ch.getState());
    }

    private void readAnalog(Device d) {
        AnalogInput ai = (AnalogInput) d.handle;
        double v = ai.getVoltage();
        telemetry.addData("voltage", "%.3f", v);
        telemetry.addData("angle (v/3.2*360)", "%.1f", (v / 3.2 * 360) % 360);
    }

    private void readColor(Device d) {
        NormalizedColorSensor c = (NormalizedColorSensor) d.handle;
        NormalizedRGBA rgba = c.getNormalizedColors();
        float[] hsv = new float[3];
        android.graphics.Color.colorToHSV(rgba.toColor(), hsv);
        telemetry.addData("R", "%.3f", rgba.red);
        telemetry.addData("G", "%.3f", rgba.green);
        telemetry.addData("B", "%.3f", rgba.blue);
        telemetry.addData("Hue", "%.1f", hsv[0]);
        telemetry.addData("Sat", "%.3f", hsv[1]);
        telemetry.addData("Val", "%.3f", hsv[2]);
    }

    private void readVoltage(Device d) {
        VoltageSensor v = (VoltageSensor) d.handle;
        telemetry.addData("voltage", "%.2f", v.getVoltage());
    }

    private void readSpecial(Device d) {
        if (d.handle instanceof GoBildaPinpointDriver) {
            GoBildaPinpointDriver p = (GoBildaPinpointDriver) d.handle;
            p.update();
            Pose2D pos = p.getPosition();
            telemetry.addData("X (mm)", "%.1f", pos.getX(DistanceUnit.MM));
            telemetry.addData("Y (mm)", "%.1f", pos.getY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("(X: recalibrate IMU)");
            if (gamepad1.x) p.recalibrateIMU();
        } else if (d.handle instanceof Limelight3A) {
            Limelight3A ll = (Limelight3A) d.handle;
            telemetry.addData("limelight", ll.isConnected() ? "connected" : "no");
        }
    }

    // ---------------- safety ----------------

    private void stopDevice(Device d) {
        if (d.handle == null) return;
        switch (d.type) {
            case MOTOR:   ((DcMotorEx) d.handle).setPower(0); break;
            case CRSERVO: ((CRServoImplEx) d.handle).setPower(0); break;
            default: break; // servos hold position; sensors do nothing
        }
    }

    @Override
    public void stop() {
        for (Category cat : categories) {
            for (Device d : cat.devices) stopDevice(d);
        }
    }

    private int wrap(int i, int size) {
        if (size == 0) return 0;
        return ((i % size) + size) % size;
    }
}