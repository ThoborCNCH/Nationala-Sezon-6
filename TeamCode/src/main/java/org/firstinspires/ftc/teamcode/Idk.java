package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@Autonomous
@Disabled
public class Idk extends LinearOpMode {
    CRServo servo;
    public static double poz1 = 1;
    public static double poz2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "sus");
        servo.getController().pwmEnable();

        waitForStart();
        while (opModeIsActive()) {

            servo.getController().setServoPosition(0, poz1);

            telemetry.addData("poza: ", servo.getController().getServoPosition(0));
            telemetry.update();
//            sleep(1000);
//            servo.getController().setServoPosition(0, poz2);

            stop();
        }

    }
}
