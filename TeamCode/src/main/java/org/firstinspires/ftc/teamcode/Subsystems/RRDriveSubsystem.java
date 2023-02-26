package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class RRDriveSubsystem extends SubsystemBase {

    SampleMecanumDrive robot;

    public RRDriveSubsystem(HardwareMap hardwareMap){

        robot = new SampleMecanumDrive(hardwareMap);

    }



}
