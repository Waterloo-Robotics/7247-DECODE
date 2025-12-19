package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="artifactLEDs")
public class artifactLEDs extends OpMode {

    private RevColorSensorV3 color1;
    private RevColorSensorV3 color2;
    private RevColorSensorV3 color3;

    private Servo led1;
    private Servo led2;
    private Servo led3;
    private static final double WHITE  = 1.0;
    private static final double GREEN  = 0.5;
    private static final double PURPLE = 0.722;




    public void init() {
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        color3 = hardwareMap.get(RevColorSensorV3.class, "color3");

        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");

        led1.setPosition(WHITE);
        led2.setPosition(WHITE);
        led3.setPosition(WHITE);


    }

    @Override
    public void loop() {

        int r1 = color1.red();
        int g1 = color1.green();
        int b1 = color1.blue();
        int total1 = r1 + g1 + b1;

        if (total1 > 600 && g1 > r1 + 150 && g1 > b1 + 150) {
            led1.setPosition(GREEN);
        } else if (total1 > 600 && r1 > 300 && b1 > 300 && g1 < r1 && g1 < b1) {
            led1.setPosition(PURPLE);
        } else {
            led1.setPosition(WHITE);
        }

        // Sensor 2
        int r2 = color2.red();
        int g2 = color2.green();
        int b2 = color2.blue();
        int total2 = r2 + g2 + b2;

        if (total2 > 600 && g2 > r2 + 150 && g2 > b2 + 150) {
            led2.setPosition(GREEN);
        } else if (total2 > 600 && r2 > 300 && b2 > 300 && g2 < r2 && g2 < b2) {
            led2.setPosition(PURPLE);
        } else {
            led2.setPosition(WHITE);
        }

        // Sensor 3
        int r3 = color3.red();
        int g3 = color3.green();
        int b3 = color3.blue();
        int total3 = r3 + g3 + b3;

        if (total3 > 600 && g3 > r3 + 150 && g3 > b3 + 150) {
            led3.setPosition(GREEN);
        } else if (total3 > 600 && r3 > 300 && b3 > 300 && g3 < r3 && g3 < b3) {
            led3.setPosition(PURPLE);
        } else {
            led3.setPosition(WHITE);
        }


          }

}
