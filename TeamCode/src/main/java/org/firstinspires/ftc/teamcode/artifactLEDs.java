package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="artifactLEDs")
public class artifactLEDs extends OpMode {

    private RevColorSensorV3 color1a;
    private RevColorSensorV3 color2a;
    private RevColorSensorV3 color3a;
    private RevColorSensorV3 color1b;
    private RevColorSensorV3 color2b;
    private RevColorSensorV3 color3b;
    private Servo led1;
    private Servo led2;
    private Servo led3;

    public void init() {
        color1a = hardwareMap.get(RevColorSensorV3.class, "color1a");
        color2a = hardwareMap.get(RevColorSensorV3.class, "color2a");
        color3a = hardwareMap.get(RevColorSensorV3.class, "color3a");
        color1b = hardwareMap.get(RevColorSensorV3.class, "color1b");
        color2b = hardwareMap.get(RevColorSensorV3.class, "color2b");
        color3b = hardwareMap.get(RevColorSensorV3.class, "color3b");

    }

    @Override
    public void loop() {



          }

}
