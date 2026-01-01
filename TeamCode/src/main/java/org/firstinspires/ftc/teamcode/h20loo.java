package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.FCDrivebaseModule;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.modules.Table2D;
import org.firstinspires.ftc.teamcode.modules.flywheelModule;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="h20", group="LinearOpMode")
public class h20loo extends OpMode {

    /* ---------- Drive Motors ---------- */
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backIntake;
    private DcMotor frontIntake;
    private DcMotor turretRotation;
    private DcMotor flywheel;
    private Servo ball1;
    private Servo ball2;
    private Servo ball3;
    private Servo hood;
    private Servo linearServo;
    private RevColorSensorV3 color1a;
    private RevColorSensorV3 color1b;
    private RevColorSensorV3 color2a;
    private RevColorSensorV3 color2b;
    private RevColorSensorV3 color3a;
    private RevColorSensorV3 color3b;
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    /* ---------- Variables ---------- */
    private double hoodPosition = 1; // start with hood down
    private double flywheelRPM;
    FCDrivebaseModule drivebase;
    float[] distance = {22, 30, 35, 40,44,52,56,69,81,125,126};
    private float[] flywheel_speed = {2600, 2750, 2850, 3020, 3070, 3170, 3190, 3120, 3300, 3900,3900};
    private float[] hood_angle = { (float)0.75, (float)0.75, (float)0.75, (float)0.65, (float)0.65,(float)0.65,(float)0.65,(float)0.65,(float)0.65,(float)0.6, (float)0.55};
    private Table2D flywheel_speed_table = new Table2D(distance, flywheel_speed);
    private Table2D hood_angle_table = new Table2D(distance, hood_angle);
    boolean AutoTargeting;
    GoBildaPinpointDriver pinpoint;
    // this is used for color sensors and the robot picking which ball to launch
    private List<Integer> availablePurples = new ArrayList<>();
    private List<Integer> availableGreens = new ArrayList<>();
    private boolean isLaunching = false;
    private double launchEndTime = 0;
    private static final double LAUNCH_DURATION = 0.4;

    private boolean launchAllInProgress = false;
    private List<Integer> launchAllQueue = new ArrayList<>();
    private int launchAllIndex = 0;
    private double nextLaunchTime = 0;
    private static final double TIME_BETWEEN_SHOTS = 0.52;  // tune this value

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");  // ORANGE & port 1 on EXPANSION hub
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // GREEN & port 2 on CONTROL hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //BLUE & port 1 on CONTROL hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");  // RED  & port 0 on EXPANSION hub
        backIntake= hardwareMap.get(DcMotor.class, "backIntake"); // YELLOW & port 2 on EXPANSION hub
        frontIntake= hardwareMap.get(DcMotor.class, "frontIntake"); // PURPLE & port 0 on CONTROL
        flywheel = hardwareMap.get(DcMotor.class, "flywheel"); // WHITE & port 3 onCONTROL hub
        turretRotation = hardwareMap.get(DcMotor.class, "turretRotation"); // GREY & port 3 on EXPANSION hub
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // 12c Bus 0 on EXPANSION hub
        ball1 = hardwareMap.get(Servo.class, "ball1"); // RED servo port 0 on EXPANSION hub
        ball2 = hardwareMap.get(Servo.class, "ball2"); // YELLOW servo port 5 on CONTROL hub
        ball3 = hardwareMap.get(Servo.class, "ball3"); // ORANGE servo port 0  on CONTROL hub
        linearServo = hardwareMap.get(Servo.class, "linearServo"); // GREEN servo port 1 on EXPANSION hub
        hood = hardwareMap.get(Servo.class, "hood"); // BLUE & servo port 1 on CONTROL hub
        color1a = hardwareMap.get(RevColorSensorV3.class, "color1a"); // BLUE & 12c Bus 3 on EXPANSION hub
        color1b = hardwareMap.get(RevColorSensorV3.class, "color1b"); // PURPLE & 12c Bus 2 on EXPANSION hub
        color2a = hardwareMap.get(RevColorSensorV3.class, "color2a"); // YELLOW & 12c Bus 3 on CONTROL hub
        color2b = hardwareMap.get(RevColorSensorV3.class, "color2b"); // GREEN & 12c Bus 2 on CONTROL hub
        color3a = hardwareMap.get(RevColorSensorV3.class, "color3a"); // ORANGE & 12c Bus 1 on CONTROL hub
        color3b = hardwareMap.get(RevColorSensorV3.class, "color3b"); // RED & 12c Bus 0 on CONTROL hub
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        drivebase = new FCDrivebaseModule(backLeft, backRight, frontLeft, frontRight, pinpoint);

        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // --- Modules ---

        // flywheel
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        // limelight
        llModule = new LimelightProcessingModule(limelight, telemetry);
        limelight.start();

        // Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* rest position for servos
        for some reason 3 is different from the rest
        dont ask me why, i literally dont know
        someone please fix with the srs programmer and update the code
         */
        ball1.setPosition(1.0);
        ball2.setPosition(1.0);
        ball3.setPosition(0);
    }

    @Override
    public void loop() {
        updateAvailableBalls();

        // launch all button
        if (gamepad2.yWasPressed() && !isLaunching && !launchAllInProgress) {
            launchAllQueue.clear();
            if (!detectColor(color1a, color1b).equals("EMPTY")) launchAllQueue.add(1);
            if (!detectColor(color2a, color2b).equals("EMPTY")) launchAllQueue.add(2);
            if (!detectColor(color3a, color3b).equals("EMPTY")) launchAllQueue.add(3);

            if (!launchAllQueue.isEmpty()) {
                launchAllInProgress = true;
                launchAllIndex = 0;
                nextLaunchTime = getRuntime();
            }
        }
        if (launchAllInProgress && !isLaunching) {
            double now = getRuntime();
            if (now >= nextLaunchTime) {
                int pocket = launchAllQueue.get(launchAllIndex);
                launchPocket(pocket);

                isLaunching = true;
                launchEndTime = now + LAUNCH_DURATION;

                launchAllIndex++;
                if (launchAllIndex >= launchAllQueue.size()) {
                    launchAllInProgress = false;
                } else {
                    nextLaunchTime = now + TIME_BETWEEN_SHOTS;
                }
            }
        }

        if (gamepad2.bWasPressed() && !isLaunching) {
            launchNext("PURPLE");
        }
        if (gamepad2.xWasPressed() && !isLaunching) {
            launchNext("GREEN");
        }

        if (isLaunching) {
            if (getRuntime() >= launchEndTime) {
                resetAllServos(); // return all servos to rest pos
                isLaunching = false;
            }
        }

        // hood software limit
        if(hoodPosition <= .4){
            hoodPosition = .4;
        }

        float rpm = 0;
        float angle = 1;
        boolean limelight_available = false;

        Pose2D pose = llModule.limelightResult();

        float limelight_distance = 0;
        if (pose != null) {
            limelight_distance = (float) (1.75*(float) -pose.getX(DistanceUnit.INCH));

            if (limelight_distance < 81 || limelight_distance > 110) {
                limelight_available = true;
            }

            if(limelight_available){
                rpm =  (flywheel_speed_table.Lookup(limelight_distance));
                angle = hood_angle_table.Lookup(limelight_distance);
            }
            else if (hood_angle_table.Lookup(limelight_distance) <= .45) {
                angle = .45F;
            }
        }

        /* ---------------- DRIVE CODE ---------------- */
        pinpoint.update();
        drivebase.update_Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.options) {
            pinpoint.update();
            pinpoint.resetPosAndIMU();
        }

        flywheelRPM += gamepad2.right_trigger * 50;
        flywheelRPM -= gamepad2.left_trigger * 50;

        flywheelRPM = Math.max(0, Math.min(4200, flywheelRPM));
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- BALL / INTAKE / TRANSFER CONTROL ---------------- */
        double intakePower = 0.0;

        // --- Touchpad reverses both while held ---
        if (gamepad2.touchpad || gamepad1.touchpad) {
            intakePower = -1.0;      // reverse
            flywheelRPM = 3500;
        }

        // --- Ball 1 (B) ---

        if (gamepad1.x || gamepad2.x) {
            ball1.setPosition(0);

        }

        if (gamepad1.a || gamepad2.a) {
            ball2.setPosition(0);

        }

        if (gamepad1.b || gamepad2.b) {
            ball3.setPosition(1);

        }

        if (gamepad1.y || gamepad2.y) {
            ball3.setPosition(0);
            ball2.setPosition(1);
            ball1.setPosition(1);
        }


        // --- Flywheel Stop (A) ---
//        if (gamepad2.a || gamepad1.a) {
//            flywheelRPM = 0;
//            hoodPosition =1;
//
//        }

        // Apply powers
        frontIntake.setPower(intakePower);
        backIntake.setPower(intakePower);
        flywheelControl.set_speed((int) flywheelRPM);

        telemetry.addData("Intake Power", intakePower);
//        if (gamepad2.dpad_up) {
//            hoodPosition -= 0.005;
//        } else if (gamepad2.dpad_down) {
//            hoodPosition += 0.005;
//        }
        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        if(gamepad2.touchpadWasPressed()){
            AutoTargeting = !AutoTargeting;

        }
        if(AutoTargeting) {
            flywheelRPM = rpm;
            hoodPosition = angle;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            // add in stuff to make turret face front of bot
        }
        if (gamepad2.dpadRightWasPressed()) {
            // add in stuff to make turret face back of bot
        }

        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);



        /* ---------------- LIMELIGHT TELEMETRY ---------------- */


        if (pose != null) {
            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Limelight", "No valid target");
        }

        /* ---------------- GENERAL TELEMETRY ---------------- */
        telemetry.addLine("=== POCKET CONTENTS ===");

        telemetry.addData("Pocket 1", detectColor(color1a, color1b));
        telemetry.addData("Pocket 2", detectColor(color2a, color2b));
        telemetry.addData("Pocket 3", detectColor(color3a, color3b));

        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID Power", flywheelControl.pid_power);
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.addData("AutoTargeting",AutoTargeting);
        telemetry.update();
    }

    private void updateAvailableBalls() {
        availablePurples.clear();
        availableGreens.clear();

        String c1 = detectColor(color1a, color1b);
        String c2 = detectColor(color2a, color2b);
        String c3 = detectColor(color3a, color3b);

        if ("PURPLE".equals(c1)) availablePurples.add(1);
        if ("PURPLE".equals(c2)) availablePurples.add(2);
        if ("PURPLE".equals(c3)) availablePurples.add(3);

        if ("GREEN".equals(c1)) availableGreens.add(1);
        if ("GREEN".equals(c2)) availableGreens.add(2);
        if ("GREEN".equals(c3)) availableGreens.add(3);

        // Already naturally sorted since we add in order 1->2->3
    }

    private void launchNext(String color) {
        List<Integer> available = color.equals("PURPLE") ? availablePurples : availableGreens;

        if (available.isEmpty()) {
            // Nothing to launch
            return;
        }

        // Launch the lowest-numbered pocket
        int pocket = available.remove(0); // remove it so it's no longer considered available

        launchPocket(pocket);

        isLaunching = true;
        launchEndTime = getRuntime() + LAUNCH_DURATION;
    }

    private void launchPocket(int pocket) {
        switch (pocket) {

            /////// ughhhhhhh we gotta like test this??? ughhh i hate work why cant we just cast a spell to know the correct servo positions
            // ready to change this later
            case 1:
                ball1.setPosition(0);
                break;
            case 2:
                ball2.setPosition(0);
                break;
            case 3:
                ball3.setPosition(1.0);
                break;
        }
    }

    private void resetAllServos() {
        ball1.setPosition(1.0);
        ball2.setPosition(1.0);
        ball3.setPosition(0.0);
    }

    // ------- this is put as a string to decrease our loop time -----------
    private String detectColor(RevColorSensorV3 sensorA, RevColorSensorV3 sensorB) {
        int red   = (sensorA.red()   + sensorB.red())   / 2;
        int green = (sensorA.green() + sensorB.green()) / 2;
        int blue  = (sensorA.blue()  + sensorB.blue())  / 2;

        // Use distance to detect if a ball is present
        double avgDistance = (sensorA.getDistance(DistanceUnit.CM) +
                sensorB.getDistance(DistanceUnit.CM)) / 2.0;

        if (avgDistance > 3.0) {
            return "EMPTY";
        }

        if (green > red + 50 && green > blue + 50) {
            return "GREEN";
        } else if (red + blue > green + 80) {
            return "PURPLE";
        } else {
            return "UNKNOWN";
        }
    }
}