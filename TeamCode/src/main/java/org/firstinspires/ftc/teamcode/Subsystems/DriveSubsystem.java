package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;


/*
This is our drive subsystem. This subsystem is for our mecanum chassis with 4 DC Motors.
 */
public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem(Motor lf, Motor rf, Motor lb, Motor rb) {
        m_drive = new MecanumDrive(lf, rf, lb, rb);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param str the commanded strafe movement
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation movement
     */

    public void drive(double str, double fwd, double rot) {
        m_drive.driveRobotCentric(-str, -fwd, -rot);
    }

    public void dpad_frontal(double fwd){
        m_drive.driveRobotCentric(0, -fwd, 0);
    }

    public void dpad_lateral(double str){
        m_drive.driveRobotCentric(-str, 0, 0);
    }

    public void rotire(double rot){
        m_drive.driveRobotCentric(0, 0, -rot);
    }
}


