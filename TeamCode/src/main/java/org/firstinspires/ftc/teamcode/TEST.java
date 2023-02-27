package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.STACK_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.power_brat_dc;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_st;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Autonomous
public class TEST extends LinearOpMode {
    SampleMecanumDrive robot;
    TrajectorySequence first;
    Trajectory f;
    TrajectorySequence s;
    TrajectorySequence stack_1;
    TrajectorySequence stack_2;
    TrajectorySequence back_junction;
    TrajectorySequence back_junction_after_math;
    private PIDController controller;
    public static double p = 0.03, i = 0, d = 0.0001;

    Motor brat_pe_sub, brat;
    private Servo left, right;
    public VoltageSensor batteryVoltageSensor;

    CRServo servo;
    public TouchSensor magnet;

    Thread ridica_si_rot;

    Thread coboara_si_rot_stack_1;
    Thread coboara_si_rot_stack_2;
    Thread ridica;


    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = new SampleMecanumDrive(hardwareMap);

        controller = new PIDController(p, i, d);

        brat_pe_sub = new Motor(hardwareMap, "brat_pe_sub");
        brat = new Motor(hardwareMap, "brat");
//        brat_pe_sub = hardwareMap.get(DcMotor.class, "brat_pe_sub");
//        brat = hardwareMap.get(DcMotor.class, "brat");

        brat_pe_sub.setInverted(true);
//        brat.setDirection(DcMotor.Direction.REVERSE);
//        brat_pe_sub.setDirection(DcMotor.Direction.REVERSE);


        left = hardwareMap.get(Servo.class, "gheara_stanga");
        right = hardwareMap.get(Servo.class, "gheara_dreapta");

        servo = hardwareMap.get(CRServo.class, "sus");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");

        robot = new SampleMecanumDrive(hardwareMap);

        brat.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        brat_pe_sub.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        brat_pe_sub.resetEncoder();
//        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat_pe_sub.setPositionTolerance(10);


//        brat_pe_sub.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Thread inchide = new Thread(() -> {
            left.setPosition(poz_inchis_st);
            right.setPosition(poz_inchis_dr);
        });

        Thread deschide = new Thread(() -> {
            left.setPosition(poz_deschis_st);
            right.setPosition(poz_deschis_dr);
        });

        coboara_si_rot_stack_1 = new Thread(() -> {
            this.deschide();
            sleep(200);
            back_thing();
            lift(350, coboara_si_rot_stack_1);
        });

        coboara_si_rot_stack_2 = new Thread(() -> {
            sleep(200);
            back_thing();
            lift(250, coboara_si_rot_stack_2);
        });

        Thread coboara_si_rot_stack_3 = new Thread(() -> {
            ridica(300, 0.1, LiftMode.DOWN);
        });

        Thread coboara_si_rot_stack_4 = new Thread(() -> {
            ridica(200, 0.1, LiftMode.DOWN);
        });

        Thread coboara_si_rot_stack_5 = new Thread(() -> {
            ridica(100, 0.1, LiftMode.DOWN);
        });

        ridica_si_rot = new Thread(() -> {
            inchide();
            sleep(200);
            lift(2100, ridica_si_rot);
            sleep(500);
            rotesteThing(1);
        });

        ridica = new Thread(() -> {
            lift(2100, ridica);
        });


        robot.setPoseEstimate(START_DR_RED_BLUE);


        first = robot.trajectorySequenceBuilder(START_DR_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    ridica.start();
                })
                .lineTo(new Vector2d(40, -25))
                .splineTo(new Vector2d(34.5, -3), Math.toRadians(120))
                .addDisplacementMarker(this::deschide)
                .waitSeconds(0.2)
                .build();


        stack_1 = robot.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(coboara_si_rot_stack_1::start)
                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(90)))
                .splineTo(new Vector2d(66, -5.7), Math.toRadians(0))
                .addDisplacementMarker(this::inchide)
                .waitSeconds(0.4)
                .build();

        back_junction = robot.trajectorySequenceBuilder(stack_1.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -4.8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .addDisplacementMarker(this::deschide)
                .build();

        stack_2 = robot.trajectorySequenceBuilder(back_junction.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_2::start)
                .lineToLinearHeading(new Pose2d(63.3, -6, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.2)
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction_after_math = robot.trajectorySequenceBuilder(stack_2.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -4.8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.1)
                .addDisplacementMarker(this::deschide)
                .build();


        waitForStart();

        this.inchide();

        while (opModeIsActive()) {
            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
            telemetry.update();
            this.inchide();
            sleep(400);

            robot.followTrajectorySequence(first);
//            ridica.join(0);
//            telemetry.addData("ridica thred: ", ridica.getState());
            robot.followTrajectorySequence(stack_1);
//            coboara_si_rot_stack_1.join(0);
            robot.followTrajectorySequence(back_junction);
//            ridica_si_rot.join(0);
            robot.followTrajectorySequence(stack_2);
//            coboara_si_rot_stack_2.join(0);
            robot.followTrajectorySequence(back_junction_after_math);
//            ridica_si_rot.join(0);
//            robot.followTrajectory(f);
//            robot.update();
//
//            robot.followTrajectory(stack_1);
////            robot.update();/
//
//
//            robot.followTrajectory(back_junction);
//            robot.update();
//            robot.followTrajectory(stack_2);
//
//            robot.followTrajectory(back_junction);


            sleep(30000);
            stop();
        }
    }


    private void start_brat(double power) {
//        brat.setPower(power);
//        brat_pe_sub.setPower(power);

        brat.set(power);
        brat_pe_sub.set(power);
    }


    private void stop_brat() {
        brat.stopMotor();
        brat_pe_sub.stopMotor();
//        brat_pe_sub.setPower(0);
//        brat.setPower(0);
    }


    public void inchide() {
        left.setPosition(poz_inchis_st);
        right.setPosition(poz_inchis_dr);
    }

    public void deschide() {
        left.setPosition(poz_deschis_st);
        right.setPosition(poz_deschis_dr);
    }

    public void ridica(int ticks, double power, LiftMode up_or_down) {
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);
        brat.setRunMode(Motor.RunMode.VelocityControl);
//        brat_pe_sub.resetEncoder();

        brat_pe_sub.setTargetPosition(ticks);

        brat_pe_sub.set(0);
        brat.set(0);

//        brat.setPositionTolerance(1);
//        brat_pe_sub.setPositionTolerance(13.6);

        double power_brat = power;
        if (ticks < 0)
            power_brat = -power;

        while (opModeIsActive() && !getMagnetAtingere()) {
            rotesteThing(-0.85);
        }
        rotesteThing(0);

        while (!brat_pe_sub.atTargetPosition() && opModeIsActive()) {
            brat_pe_sub.set(power);
            brat.set(power_brat);
        }

        brat_pe_sub.stopMotor();
        brat.stopMotor();

        //        if (up_or_down == LiftMode.DOWN) {
//            brat.stopMotor();
//        }

//        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        brat_pe_sub.setTargetPosition(ticks);
//
//
//        while (brat_pe_sub.isBusy() && opModeIsActive()) {
////            brat.setPower(power_brat);
//
//            if (up_or_down == LiftMode.UP) {
//                if (brat_pe_sub.getCurrentPosition() > 1000) {
//                    rotesteThing(1);
//                }
//            } else if (up_or_down == LiftMode.DOWN) {
//                brat_pe_sub.setPower(-power);
//                if (opModeIsActive() && !getMagnetAtingere()) {
//                    rotesteThing(-1);
//                } else {
//                    rotesteThing(0);
//                }
//            }
//        }

//        brat.set(0);
//        brat_pe_sub.set(0);
//        brat.setPower(0);
//        brat_pe_sub.setPower(0);
//
//        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brat_pe_sub.set(0.4);
    }

    private void ridica_lift(int tick, double sped) {
        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);

        brat_pe_sub.setTargetPosition(tick);
        double delta = Math.abs(tick) - Math.abs(brat_pe_sub.getCurrentPosition());
        double first = delta, second = delta * 2, third = second + first, fourth = second * 2;

        double power_real = sped / 2;

        while (!brat_pe_sub.atTargetPosition() && opModeIsActive()) {
            int pos = brat_pe_sub.getCurrentPosition();

            if (pos <= first)
                power_real = sped / 8;
            if (pos >= first && pos <= second)
                power_real = sped;
            if (pos >= third && pos <= fourth)
                power_real = sped / 8;

            double power_brat = power_real;
            if (pos >= tick) {
                power_brat = -power_brat;
            }
            brat_pe_sub.set(power_real);
            brat.set(power_brat);
        }

        brat_pe_sub.stopMotor();
        brat.stopMotor();
    }

    private void back_thing() {
        while (opModeIsActive() && !getMagnetAtingere()) {
            rotesteThing(-0.6);
        }
        rotesteThing(0);
    }

    private void rotesteThing(double speed) {
        servo.setPower(speed);
    }

    private boolean getMagnetAtingere() {
        return magnet.isPressed();
    }

    private void lift(double target, Thread thread) {
        this.inchide();
//        while (opModeIsActive() ) {
            controller.setPID(p, i, d);
            int armpos = brat_pe_sub.getCurrentPosition();
            double pid = controller.calculate(armpos, target);
            double power = pid;

            brat_pe_sub.set(power);
            brat.set(-power);
//        }
    }

    private void lift(double target, Thread thread, boolean alternative){
        this.inchide();
        while (opModeIsActive() && brat.getCurrentPosition() <)
    }
}


/**
 * Roses are red
 * Violets are blue
 * I wanna stick my dick inside you
 */