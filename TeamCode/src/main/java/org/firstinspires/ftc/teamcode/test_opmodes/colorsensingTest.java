package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class colorsensingTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private Servo beaconLight;  // The beacon is controlled by a servo
    private double red, green, blue;

    private double lastColorTime;  // To track the time when the color was last seen
    private double lastColorPosition = 0.611;  // Default to blue initially
    private static final double BLUE_POSITION = 0.611;
    private static final double GREEN_POSITION = 0.450;
    private static final double PURPLE_POSITION = 0.722;
    private static final double TIMEOUT = 10_000;  // Timeout in milliseconds (10 seconds)

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the color sensor and beacon servo
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        beaconLight = hardwareMap.get(Servo.class, "beaconLight"); // Servo that controls the beacon

        // Start with the servo in blue position
        beaconLight.setPosition(BLUE_POSITION);

        // Store the current time to track when color was last seen
        lastColorTime = System.currentTimeMillis();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get the color detected by the sensor
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();

            // Check if it's time to reset to blue based on the timeout
            if (System.currentTimeMillis() - lastColorTime > TIMEOUT) {
                // If no green or purple detected for 10 seconds, reset to blue
                lastColorPosition = BLUE_POSITION;
                beaconLight.setPosition(BLUE_POSITION);
            }

            // Check if the detected color is closer to green
            if (isGreen(red, green, blue)) {
                // Only change to green if it's not already green
                if (lastColorPosition != GREEN_POSITION) {
                    lastColorPosition = GREEN_POSITION;
                    beaconLight.setPosition(GREEN_POSITION);
                    lastColorTime = System.currentTimeMillis();  // Update the last color seen time
                }
            }
            // Check if the detected color is closer to purple
            else if (isPurple(red, green, blue)) {
                // Only change to purple if it's not already purple
                if (lastColorPosition != PURPLE_POSITION) {
                    lastColorPosition = PURPLE_POSITION;
                    beaconLight.setPosition(PURPLE_POSITION);
                    lastColorTime = System.currentTimeMillis();  // Update the last color seen time
                }
            }

            // Add a small delay to make the program responsive
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
