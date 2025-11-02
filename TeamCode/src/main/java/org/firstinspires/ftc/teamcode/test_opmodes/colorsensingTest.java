package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "colorsensingTest")
public class colorsensingTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private Servo leftLED;
    private Servo rightLED;

    private double red, green, blue;
    private long lastSeenTime;

    // Servo positions
    private static final double GREEN_POS = 0.5;
    private static final double BLUE_POS = 0.611;
    private static final double PURPLE_POS = 0.772;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize color sensor and both beacon LEDs
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Start blue
        setLEDs(BLUE_POS);
        lastSeenTime = System.currentTimeMillis();

        waitForStart();

        while (opModeIsActive()) {

            // Read color values
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();

            boolean colorDetected = false;

            // Detect colors
            if (isGreen(red, green, blue)) {
                setLEDs(GREEN_POS);
                lastSeenTime = System.currentTimeMillis();
                colorDetected = true;
                telemetry.addLine("Detected: Green");
            } else if (isPurple(red, green, blue)) {
                setLEDs(PURPLE_POS);
                lastSeenTime = System.currentTimeMillis();
                colorDetected = true;
                telemetry.addLine("Detected: Purple");
            }

            // If no color detected for 3 seconds, turn blue
            if (!colorDetected && System.currentTimeMillis() - lastSeenTime > 3000) {
                setLEDs(BLUE_POS);
                telemetry.addLine("No color detected for 3s → Blue");
            }

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();

            sleep(50);
        }
    }

    // Helper: set both LEDs to same position
    private void setLEDs(double position) {
        leftLED.setPosition(position);
        rightLED.setPosition(position);
    }

    // Helper: detect green
    private boolean isGreen(double red, double green, double blue) {
        return green > red * 1.3 && green > blue * 1.3 && green > 50;  // add small threshold
    }

    // Helper: detect purple (mix of red + blue, low green)
    private boolean isPurple(double red, double green, double blue) {
        return red > 60 && blue > 60 && green < (red + blue) / 3;
    }
}
