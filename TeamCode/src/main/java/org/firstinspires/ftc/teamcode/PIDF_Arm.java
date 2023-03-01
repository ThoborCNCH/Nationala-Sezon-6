//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Config
//@TeleOp
//public class PIDF_Arm extends OpMode {
//    private PIDController controller;
//    public static double p = 0.03, i = 0, d = 0.0001;
//    public static double f = 0.5;
//
//    public static int target = 2000;
//
//    private final double ticks_in_degree = 537.7 / 360.0;
//
//    private DcMotorEx brat, brat_pe_sub;
//
//    @Override
//    public void init() {
//        controller = new PIDController(p, i, d);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        brat = hardwareMap.get(DcMotorEx.class, "brat");
//        brat_pe_sub = hardwareMap.get(DcMotorEx.class, "brat_pe_sub");
//
//        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        brat_pe_sub.setDirection(DcMotorSimple.Direction.REVERSE);
//
//    }
//
//    @Override
//    public void loop() {
//        controller.setPID(p, i, d);
//        int armpos = brat_pe_sub.getCurrentPosition();
//        double pid = controller.calculate(armpos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
//        double power = pid * ff;
//
//        brat_pe_sub.setPower(power);
//        brat.setPower(-power);
//        telemetry.addData("pos ", armpos);
//        telemetry.addData("target ", target);
//        telemetry.addData("power ", power);
//        telemetry.addData("speid", brat_pe_sub.getVelocity() * ticks_in_degree);
//        telemetry.update();
//    }
//}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF_Arm extends LinearOpMode {
    private PIDController controller;
    public static double p = 0.0018, i = 0, d = 0;
    public static double f = 0.2;

    public static int target = 2000;

    private final double ticks_in_degree = 537.7 / (3.543307 * Math.PI);

    private DcMotorEx brat, brat_pe_sub;

    @Override
    public void runOpMode() throws InterruptedException {
        inits();
        waitForStart();
        while (opModeIsActive()) {
            loops();
        }
    }

    public void inits() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        brat = hardwareMap.get(DcMotorEx.class, "brat");
        brat_pe_sub = hardwareMap.get(DcMotorEx.class, "brat_pe_sub");

        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brat_pe_sub.setDirection(DcMotorSimple.Direction.REVERSE);
        brat_pe_sub.setTargetPositionTolerance(10);

    }

    public void loops() {
        controller.setPID(p, i, d);
        int armpos = brat_pe_sub.getCurrentPosition();
        double pid = controller.calculate(armpos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid;

        brat_pe_sub.setPower(power);
        brat.setPower(-power);
        telemetry.addData("pos ", armpos);
        telemetry.addData("target ", target);
        telemetry.addData("ff ", ff);
        telemetry.addData("pid  ", pid);
        telemetry.update();
    }
}
