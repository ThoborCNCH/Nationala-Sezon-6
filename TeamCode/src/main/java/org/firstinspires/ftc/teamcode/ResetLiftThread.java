package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class ResetLiftThread extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//
//        DcMotorEx brat,brat_pe_sub;
//        CRServo sus;
//        Servo left,right;
//        TouchSensor magnet;
//
//        brat = hardwareMap.get(DcMotorEx.class, "brat");
//        brat_pe_sub = hardwareMap.get(DcMotorEx.class, "brat_pe_sub");
//        sus = hardwareMap.get(CRServo.class, "sus");
//        left = hardwareMap.get(Servo.class, "gheara_stanga");
//        right = hardwareMap.get(Servo.class, "gheara_dreapta");
//        magnet = hardwareMap.get(TouchSensor.class, "magnet");
//
//        ArmcPIDF armcPIDF = new ArmcPIDF(brat_pe_sub, brat, sus, magnet, left, right);
//        Thread reset = new Thread(armcPIDF);

        waitForStart();

        ThreadInfoStanga.shouldClose = true;

//        reset.join();

        stop();
    }
}
