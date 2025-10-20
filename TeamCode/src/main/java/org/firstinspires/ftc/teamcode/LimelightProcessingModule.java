package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightProcessingModule {

    public DcMotor limelight;
    public LimelightProcessingModule(DcMotor dcMotor) {
        this.limelight = (DcMotor)dcMotor;
    }

    public void limelightResult(LLResult) {
        LLResult llResult = limelight.getLatestResult();
    }
}
