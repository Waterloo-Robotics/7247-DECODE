package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightProcessingModule {

    Telemetry telemetry;

    public Limelight3A limelight;
    public LimelightProcessingModule(Limelight3A limelight, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        this.limelight = limelight;
        this.limelight.pipelineSwitch(0);
        this.limelight.start();
    }
    /* ------ */
    /* limelightResult
     *  The goal of this function is to return the robot position
     *  relative to the april tag.
     * */
    public Pose2D limelightResult() {
        LLResult llResult = this.limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> results = llResult.getFiducialResults();

            // now it gets field space and not target space so the telemetry should be correct
            Pose3D robot_pose = results.get(0).getRobotPoseFieldSpace();

            double x = robot_pose.getPosition().toUnit(DistanceUnit.INCH).x;
            double y = robot_pose.getPosition().toUnit(DistanceUnit.INCH).y;
            double rot = robot_pose.getOrientation().getYaw(AngleUnit.DEGREES);

            return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, rot);
        }
        return null;
    }

}
