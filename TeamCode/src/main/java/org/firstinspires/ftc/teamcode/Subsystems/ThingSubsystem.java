package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.NU_MAI_POT;


public class ThingSubsystem extends SubsystemBase {

    private final Servo left, right;

    public static boolean isActive = false;

    public static double poz_inchis_left = NU_MAI_POT.poz_inchis_st;
    public static double poz_inchis_right = NU_MAI_POT.poz_inchis_dr;

    public static double poz_deschis_left = NU_MAI_POT.poz_deschis_st;
    public static double poz_deschis_right = NU_MAI_POT.poz_deschis_dr;

    public ThingSubsystem(Servo left, Servo right)
    {
        this.left = left;
        this.right = right;
    }

    public void apuca()
    {
        left.setPosition(poz_inchis_left);
        right.setPosition(poz_inchis_right);

        isActive = true;
    }

    public void arunca()
    {
        left.setPosition(poz_deschis_left);
        right.setPosition(poz_deschis_right);

        isActive = false;
    }
}
