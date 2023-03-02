package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
class ThreadInfoStanga {
    public static int target = 0;
    public static boolean shouldClose = false;
    public static boolean use = true;
    public static int fr;
    public static boolean useTele = true;
    public static double servo_speed = 0;
    public static boolean closed_hand = false;
}

//MAI ESTE O ZI PANA LA REGIONALA SI CODUL CU IMPLEMENTAREA
//NOASTRA DE PIDF SI THREADS PT LIFT NU FUNCTIONEAZA MULTUMIM
//ROSOPHIA CA DE ALTA ERAM VAI NOI

@Config
class ArmcSTPIDF implements Runnable {
    DcMotorEx ridicareSlide, brat;
    CRServo servo;
    TouchSensor magnet;
    Servo left, right;

    public ArmcSTPIDF(DcMotorEx ridicareSlide, DcMotorEx brat, CRServo servo, TouchSensor magnet, Servo left, Servo right) {
        this.ridicareSlide = ridicareSlide;
        this.brat = brat;
        this.servo = servo;
        this.magnet = magnet;
        this.left = left;
        this.right = right;

//        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ridicareSlide.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public static double ppd = 0.0;
    public static double ppu = 0.0;
    public static double pd = 0.0021;
    public static double pu = 0.002;
    public static double d = 0;
    public static double i = 0.0002;
    public static double Kf = 0.001;

    public static double LPC = 2.6;

    double error = 0;
    double derivate = 0;
    double lastError = 0;
    double integralSum = 0;

    public void run() {
        ElapsedTime timer2 = new ElapsedTime();
        double outp = 0;
        int tc = 0;
        timer2.reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!ThreadInfoStanga.shouldClose) {
            if (ThreadInfoStanga.useTele) {
                ++tc;
                if (timer2.seconds() >= 1.0) {
                    ThreadInfoStanga.fr = (int) (tc / timer2.seconds());
                    tc = 0;
                    timer2.reset();
                }
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("fr", ThreadInfoStanga.fr);
                pack.put("Target", ThreadInfoStanga.target);
                pack.put("Current", ridicareSlide.getCurrentPosition());
                pack.put("Power", outp);
                dashboard.sendTelemetryPacket(pack);
            }
            if (ThreadInfoStanga.use) {
                if (ridicareSlide.getCurrentPosition() > 600 && ThreadInfoStanga.servo_speed == -1) {
                    servo.setPower(-1);
                } else if (ThreadInfoStanga.servo_speed == 1 && !getMagnetAtingere()) {
                    servo.setPower(1);
                } else {
                    servo.setPower(0);
                }

//                if(ThreadInfo.closed_hand) {
//                    left.setPosition(NU_MAI_POT.poz_inchis_st);
//                    right.setPosition(NU_MAI_POT.poz_inchis_dr);
//                } else
//                {
//                    left.setPosition(NU_MAI_POT.poz_deschis_st);
//                    right.setPosition(NU_MAI_POT.poz_deschis_dr);
//                }

                error = ThreadInfoStanga.target - ridicareSlide.getCurrentPosition();
                //derivate = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                if (error < -220) {
                    outp = /*-(ppd * error * error) +*/ (pd * error) + (d * derivate) + (i * integralSum) + Kf;
                } else {
                    outp = /*(ppu * error * error) +*/ (pu * error) + (d * derivate) + (i * integralSum) + Kf;
                }
                if (ThreadInfoStanga.target < 150 && ridicareSlide.getCurrentPosition() < 150) {
                    outp /= LPC;
                }
                ridicareSlide.setPower(outp);
                brat.setPower(-outp);

                lastError = error;
                timer.reset();
            } else {
                error = derivate = lastError = integralSum = 0;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        ridicareSlide.setPower(0);
        brat.setPower(0);
    }


    private boolean getMagnetAtingere() {
        return magnet.isPressed();
    }
}
