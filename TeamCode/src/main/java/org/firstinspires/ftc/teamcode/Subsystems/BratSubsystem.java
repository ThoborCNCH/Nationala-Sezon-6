package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BratSubsystem extends SubsystemBase {

    final TouchSensor magnet;

    final CRServo servo;

    final double powerRoteste = 0.5;

    final ElapsedTime timer;
    final double timerMagnet = 0.65;

    public BratSubsystem(CRServo servo, TouchSensor magnet){
        this.magnet = magnet;
        this.servo = servo;

        timer = new ElapsedTime();
    }

    public void rotesteCentruTimer(double speed){
        timer.reset();
        while (!magnetAtingere() || timer.seconds() <= timerMagnet) {
            servo.setPower(speed);
        }
        servo.setPower(0);
    }

    public void rotesteCentru(double speed){
        while (!magnetAtingere()) {
            servo.setPower(speed);
        }
        servo.setPower(0);
    }

    public void rotesteThing(double speed){
        servo.setPower(speed);
    }

    private boolean magnetAtingere(){
        return magnet.isPressed();
    }
}
