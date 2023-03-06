package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.ThreadInfo;

public class LiftSubsystem extends SubsystemBase {

    private Motor brat, brat_pe_sub;

    public boolean isMoving = false;

    private static int poz_jos = 0;
    private static int poz_low = 0;
    private static int poz_medium = 0;
    private static int poz_high = 0;

    public LiftSubsystem(Motor brat, Motor brat_pe_sub)
    {
        this.brat = brat;
        this.brat_pe_sub = brat_pe_sub;
    }

    public void liftJos() {
        isMoving = true;
        brat.setRunMode(Motor.RunMode.PositionControl);
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat.setTargetPosition(poz_jos);
        brat_pe_sub.setTargetPosition(poz_jos);

        brat.set(0);
        brat_pe_sub.set(0);

        // eroare maxima in ticks
        brat.setPositionTolerance(13.6);
        brat_pe_sub.setPositionTolerance(13.6);

        // perform the control loop
        while (!brat.atTargetPosition() || !brat_pe_sub.atTargetPosition()) {
            brat.set(1);
            brat_pe_sub.set(1);
        }

        isMoving = false;
        brat.stopMotor();
        brat_pe_sub.stopMotor();// stop the motor
    }

    public void liftLow() {
        isMoving = true;
        brat.setRunMode(Motor.RunMode.PositionControl);
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat.setTargetPosition(poz_low);
        brat_pe_sub.setTargetPosition(poz_low);

        brat.set(0);
        brat_pe_sub.set(0);

        // eroare maxima in ticks
        brat.setPositionTolerance(13.6);
        brat_pe_sub.setPositionTolerance(13.6);

        // perform the control loop
        while (!brat.atTargetPosition() || !brat_pe_sub.atTargetPosition()) {
            brat.set(1);
            brat_pe_sub.set(1);
        }

        isMoving = false;
        brat.stopMotor();
        brat_pe_sub.stopMotor();// stop the motor
    }

    public void liftMediu() {
        isMoving = true;
        brat.setRunMode(Motor.RunMode.PositionControl);
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat.setTargetPosition(poz_medium);
        brat_pe_sub.setTargetPosition(poz_medium);

        brat.set(0);
        brat_pe_sub.set(0);

        // eroare maxima in ticks
        brat.setPositionTolerance(13.6);
        brat_pe_sub.setPositionTolerance(13.6);

        // perform the control loop
        while (!brat.atTargetPosition() || !brat_pe_sub.atTargetPosition()) {
            brat.set(1);
            brat_pe_sub.set(1);
        }

        isMoving = false;
        brat.stopMotor();
        brat_pe_sub.stopMotor();// stop the motor
    }

    public void liftHigh() {
        isMoving = true;
        brat.setRunMode(Motor.RunMode.PositionControl);
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat.setTargetPosition(poz_high);
        brat_pe_sub.setTargetPosition(poz_high);

        brat.set(0);
        brat_pe_sub.set(0);

        // eroare maxima in ticks
        brat.setPositionTolerance(13.6);
        brat_pe_sub.setPositionTolerance(13.6);

        // perform the control loop
        while (!brat.atTargetPosition() || !brat_pe_sub.atTargetPosition()) {
            brat.set(1);
            brat_pe_sub.set(1);
        }

        isMoving = false;
        brat.stopMotor();
        brat_pe_sub.stopMotor();// stop the motor
    }

    public void liftPower(double power){

        if(power == 0) {
            isMoving = false;
            ThreadInfo.use = true;
        } else {
            isMoving = true;
            ThreadInfo.use = false;
            ThreadInfo.target = brat_pe_sub.getCurrentPosition();
        }

        brat.set(power);
        brat_pe_sub.set(power);

    }

}
