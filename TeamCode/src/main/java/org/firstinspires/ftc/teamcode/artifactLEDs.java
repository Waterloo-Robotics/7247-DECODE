package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="artifactLEDs")
public class artifactLEDs extends OpMode {

  private Servo led1;
  private Servo led2;
  private Servo led3;
  private ColorSensor cs1;
  private ColorSensor cs2;
  private ColorSensor cs3;

    private String color1 = "NONE";
    private String color2 = "NONE";
    private String color3 = "NONE";


    public void init() {
        hardwareMap.get(RevColorSensorV3.class, "cs1");
        hardwareMap.get(RevColorSensorV3.class, "cs2");
        hardwareMap.get(RevColorSensorV3.class, "cs3");
    }

    @Override
    public void loop() {

          }
}
