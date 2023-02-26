package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_st;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@TeleOp
public class TestGheara extends LinearOpMode {
    private Servo left, right;

    public static double lo = poz_deschis_st;
    public static double ro = poz_deschis_dr;
    public static double lc = poz_inchis_st;
    public static double rc = poz_inchis_dr;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(Servo.class, "gheara_stanga");
        right = hardwareMap.get(Servo.class, "gheara_dreapta");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad1.a) {
                inchide();
            } else if (gamepad1.b) {
                deschide();
            }
        }

    }

    public void inchide() {
        left.setPosition(lc);
        right.setPosition(rc);
    }

    public void deschide() {
        left.setPosition(lo);
        right.setPosition(ro);
    }

    ;
}
