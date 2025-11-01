package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class colorsensingTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private Servo beaconLight;  // The beacon is controlled by a servo
    private double red, green, blue;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the color sensor and beacon servo
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        beaconLight = hardwareMap.get(Servo.class, "beaconLight"); // Servo that controls the beacon

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get the color detected by the sensor
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();

            // Check if the detected color is closer to green or purple
            if (isGreen(red, green, blue)) {
                // Move the servo to position for green (e.g., position 0.0)
                beaconLight.setPosition(0.0);  // Green
            } else if (isPurple(red, green, blue)) {
                // Move the servo to position for purple (e.g., position 1.0)
                beaconLight.setPosition(1.0);  // Purple
            }

            // Add a small delay to make it responsive
            sleep(50);
        }
    }

    // Helper method to check if the detected color is closer to green
    private boolean isGreen(double red, double green, double blue) {
        return green > red && green > blue;
    }

    // Helper method to check if the detected color is closer to purple
    private boolean isPurple(double red, double green, double blue) {
        return red > blue && red > green;
    }
}
