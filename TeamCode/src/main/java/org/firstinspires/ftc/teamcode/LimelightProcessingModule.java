package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightProcessingModule {

public Limelight3A limelight;
    public LimelightProcessingModule(Limelight3A limelight)
    {
    this.limelight = limelight;
    limelight.pipelineSwitch(0);

    }
    /* ------ */
    /* limelightResult
    *  The goal of this function is to return the robot position
    *  relative to the april tag.
    * */
    public Pose2D limelightResult() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid())
        {
            Pose3D robot_pose = llResult.getBotpose();
            double x = robot_pose.getPosition().toUnit(DistanceUnit.INCH).x;
            double y = robot_pose.getPosition().toUnit(DistanceUnit.INCH).y;
            double rot = robot_pose.getOrientation().getYaw(AngleUnit.DEGREES);
            return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, rot);
        }
        return null;
    }
}
