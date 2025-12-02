package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class FieldPositionEstimation {

    private GoBildaPinpointDriver pinpoint;
    private Pose2D field_position;
    public Pose2D relative_robot_position;
    private boolean on_red_side; // what is this for?

    private Limelight3A limelight;
    private Pose2D limelight_position;

    public FieldPositionEstimation(GoBildaPinpointDriver pinpoint, boolean on_red_side)
    {
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(155, -25, DistanceUnit.MM);
        this.on_red_side = on_red_side;
        this.pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    public void attachLimelight(Limelight3A limelight) {
        this.limelight = limelight;
        this.limelight.pipelineSwitch(0);
        this.limelight.start();
    }
    public void update_from_pinpoint() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double rot = pinpoint.getHeading(AngleUnit.DEGREES);
        this.relative_robot_position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, rot);
    }
    public void update_from_limelight() {
        if (this.limelight == null) return;

        LLResult llResult = this.limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> results = llResult.getFiducialResults();
            if (results.isEmpty()) return;

            Pose3D robotPoseField = results.get(0).getRobotPoseFieldSpace();
            if (robotPoseField == null) return;

            double x = robotPoseField.getPosition().toUnit(DistanceUnit.INCH).x;
            double y = robotPoseField.getPosition().toUnit(DistanceUnit.INCH).y;
            double heading = robotPoseField.getOrientation().getYaw(AngleUnit.DEGREES);

            this.limelight_position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        }
    }

    // Fuse the data from IMU and Limelight
    public Pose2D getFusedFieldPosition() {
        // If Limelight position is available, use it
        if (limelight_position != null) {
            this.field_position = limelight_position;
        }
        // Otherwise, use the relative IMU position
        else if (relative_robot_position != null) {
            this.field_position = relative_robot_position;
        }

        return this.field_position;
    }
    public void reset_pinpoint() {
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
    }
}
