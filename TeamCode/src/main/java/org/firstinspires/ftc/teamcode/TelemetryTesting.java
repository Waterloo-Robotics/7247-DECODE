package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


// -------------------------------------------------------------------------------
@TeleOp
public class TelemetryTesting extends OpMode {
private Limelight3A limelight;
    private IMU imu;

    private boolean tag_found_21;
    private boolean tag_found_22;
    private boolean tag_found_23;
    private boolean tagID;

    //grabs from TestIMU.java
    TestIMU bench = new TestIMU();

    // --- motors





    // marks patterns as false at the start

    // ---
    private String detectedPattern = "None";
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;


    // --------------------------------------------------

    @Override
    public void init() {
        bench.init(hardwareMap);
        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // imu
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // april tags
        tag_found_21 = false;
        tag_found_22= false;
        tag_found_23 = false;
        tagID = false;

        // motors ---
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");


    }

    @Override
    public void start() {
        // limelight
        limelight.start();

    }



    @Override
    public void loop() {
        // ------





        //-----------------------------------



        // IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            // show tag distance status in telem
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tag X", llResult.getTx());
            telemetry.addData("Tag Y", llResult.getTy());
            telemetry.addData("Tag Area", llResult.getTa());

        }
        // ---
        telemetry.addData("Bot Angle", bench.getHeading(AngleUnit.DEGREES));
        // ---
        // shows motif
        if (detectedPattern.equals("None")) {
        if (tag_found_21)
        {
            detectedPattern = "GREEN PURPLE PURPLE";

        }
        else if (tag_found_22)
        {
            detectedPattern = "PURPLE GREEN PURPLE";

        }
        else if (tag_found_23){
            detectedPattern = "PURPLE PURPLE GREEN";

        } }

        telemetry.addData("Pattern", detectedPattern);
        telemetry.update();
        // end of motif stuff

    }

}
// Starting Odometry here
