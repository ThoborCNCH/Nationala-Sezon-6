package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@Autonomous
public class Implementare extends LinearOpMode {

    public static int target = 0;
    public static boolean shouldClose = false;
    public static boolean tele = true;
    public static boolean use = true;

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx brat_pe_sub = hardwareMap.get(DcMotorEx.class, "brat_pe_sub");
        DcMotorEx brat = hardwareMap.get(DcMotorEx.class, "brat");

        CRServo servo = hardwareMap.get(CRServo.class, "sus");
        TouchSensor magnet = hardwareMap.get(TouchSensor.class, "magnet");

        Servo left = hardwareMap.get(Servo.class, "gheara_stanga");
        Servo right = hardwareMap.get(Servo.class, "gheara_dreapta");

        ArmcPIDF armcPIDF = new ArmcPIDF(brat_pe_sub, brat, servo, magnet, left, right);

        Thread noi = new Thread(armcPIDF);

        waitForStart();
        noi.start();

        while (opModeIsActive()) {
            ThreadInfo.shouldClose = shouldClose;
            ThreadInfo.target = target;
            ThreadInfo.use = tele;
        }

        noi.join();
    }
}
