package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.NU_MAI_POT;


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
        m_drive.driveRobotCentric(-str * limitare_vit, -fwd * limitare_vit, -rot * limitare_vit);
    }

    public void dpad_frontal(double fwd, boolean lb, boolean rb){
        if(lb)
            fwd = fwd * 0.3;
        else if(rb)
            fwd = fwd * 1.2;

        if(fwd > 1)
            fwd = 1;

        m_drive.driveRobotCentric(0, -fwd, 0);
    }

    public void dpad_lateral(double str, boolean lb, boolean rb){
        if(lb)
            str = str * 0.3;
        else if(rb)
            str = str * 1.2;

        if(str > 1)
            str = 1;

        m_drive.driveRobotCentric(-str, 0, 0);
    }

    public void rotire(double rot, boolean lb, boolean rb){
        if(lb)
            rot = rot * 0.3;
        else if(rb)
            rot = rot * 1.2;

        if(rot > 1)
            rot = 1;

        m_drive.driveRobotCentric(0, 0, -rot);
    }
}


