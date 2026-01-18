package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Five Laser Counter", group = "Sensor")
public class LaserBeamBreakTest extends LinearOpMode {

    // Define the hardware map names for all five sensors
    private final String[] SENSOR_NAMES = {
            "laserSensor0", "laserSensor1", "laserSensor2", "laserSensor3", "laserSensor4"
    };
    private Servo rgbLight;

    // Arrays to hold the DigitalChannel objects and their state history
    private DigitalChannel[] lasers = new DigitalChannel[SENSOR_NAMES.length];
    private boolean[] previousStates = new boolean[SENSOR_NAMES.length];

    // Variable to hold the running total count
    private int objectCount = 0;

    @Override
    public void runOpMode() {

        // --- INITIALIZATION ---

        // Loop through the names to initialize each DigitalChannel object
        for (int i = 0; i < SENSOR_NAMES.length; i++) {
            try {
                // Initialize sensor from hardware map
                lasers[i] = hardwareMap.get(DigitalChannel.class, SENSOR_NAMES[i]);
                rgbLight = hardwareMap.get(Servo.class, "rgbLight");
                // Set the digital channel direction to INPUT
                lasers[i].setMode(DigitalChannel.Mode.INPUT);

            } catch (Exception e) {
                // Report any configuration error to the driver station
                telemetry.addData("ERROR", "Could not find sensor: " + SENSOR_NAMES[i]);
                telemetry.update();
                // Abort the OpMode if a sensor is missing
                return;
            }
        }

        telemetry.addData("Status", "Five Sensors Initialized. Waiting for START...");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // --- POST-START INITIALIZATION ---

        // Initialize previousStates array with the current reading of the sensors
        // to prevent counting objects that are already there at the start of the match.
        for (int i = 0; i < lasers.length; i++) {
            previousStates[i] = lasers[i].getState();
        }

        // --- MAIN LOOP ---

        while (opModeIsActive()) {

            // Loop through all five sensors to read and process their state
            for (int i = 0; i < lasers.length; i++) {

                boolean currentState = lasers[i].getState();

                // --- Logic for Incrementing/Decrementing ---

                // Condition 1: Object Enters (Beam Broken: State changed from LOW/Clear to HIGH/Broken)
                if (currentState && !previousStates[i]) {
                    objectCount++;
                    // Optional: Add logging for debugging
                    telemetry.addData("Event", "Sensor " + i + " BROKEN -> Count: " + objectCount);
                }
                // Condition 2: Object Leaves (Beam Cleared: State changed from HIGH/Broken to LOW/Clear)
                else if (!currentState && previousStates[i]) {
                    objectCount--;
                    // Optional: Add logging for debugging
                    telemetry.addData("Event", "Sensor " + i + " CLEARED -> Count: " + objectCount);
                }

                // Update the previous state for the next loop iteration
                previousStates[i] = currentState;
            }

            // Safety check: Ensure the count never drops below zero
            if (objectCount < 0) {
                objectCount = 0;
            }
            if (objectCount == 3) {
                rgbLight.setPosition(0.5); //green
            }
            if (objectCount > 3) {
                rgbLight.setPosition(0.71); //purple
            }
            if (objectCount == 1 || objectCount == 2 || objectCount == 0) {
                rgbLight.setPosition(1);
            }
            /// --- TELEMETRY UPDATE ---
            telemetry.addData("--------------------", "--------------------");
            telemetry.addData("TOTAL OBJECT COUNT", objectCount);
            telemetry.addData("S0-S4 Status (T=Broken)", "%b, %b, %b, %b, %b",
                    lasers[0].getState(), lasers[1].getState(), lasers[2].getState(),
                    lasers[3].getState(), lasers[4].getState());
            telemetry.update();
        }
    }
}