package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.flywheelModule;

@TeleOp(name="Indexer Test", group="TestOpMode")
public class IndexerTest extends OpMode {

    private Servo SlotO;
    private Servo SlotT;
    private Servo SlotTH;

    @Override
    public void init(){

        SlotO = hardwareMap.get(Servo.class, "SlotOne");
        SlotT = hardwareMap.get(Servo.class, "SlotTwo");
        SlotTH = hardwareMap.get(Servo.class, "SlotThree");
    }
    public void init_loop() {
    }


    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            SlotO.setPosition(1);

        }

        if (gamepad1.a) {
            SlotT.setPosition(1);

        }

        if (gamepad1.b) {
            SlotTH.setPosition(1);

        }
    }
}

