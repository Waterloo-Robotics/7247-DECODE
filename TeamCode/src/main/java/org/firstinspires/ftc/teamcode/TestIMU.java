

/* THIS IS NOT A TELEOP, THIS IS IMU CODE! PLEASE DO NOT MODIFY UNLESS YOU KNOW WHAT YOU ARE DOING, BECAUSE THIS TOOK ME FOREVER, AND I DO NOT WANT IT TO MESS UP

IF WORK = NO TOUCH
TOUCH = BREAK
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TestIMU {

    private IMU imu;

    public void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT

        );

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public double getHeading(AngleUnit degrees) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }




}
