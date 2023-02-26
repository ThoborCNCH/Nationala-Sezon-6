package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous
public class Idk extends LinearOpMode {
    CRServo servo;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "sus");

        waitForStart();
        while(opModeIsActive()){

            servo.getController().pwmEnable();
            servo.getController().setServoPosition(0, 1);
            sleep(500);
            servo.getController().setServoPosition(0, 0);

            stop();
        }

    }
}
