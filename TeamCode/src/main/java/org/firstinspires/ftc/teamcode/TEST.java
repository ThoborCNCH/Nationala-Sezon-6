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

    //    Motor brat_pe_sub;
//    Motor brat;
    Motor brat_pe_sub, brat;
    private Servo left, right;
    public VoltageSensor batteryVoltageSensor;

    CRServo servo;
    public TouchSensor magnet;


    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = new SampleMecanumDrive(hardwareMap);

        brat_pe_sub = new Motor(hardwareMap, "brat_pe_sub");
        brat = new Motor(hardwareMap, "brat");
//        brat_pe_sub = hardwareMap.get(DcMotor.class, "brat_pe_sub");
//        brat = hardwareMap.get(DcMotor.class, "brat");

        brat.setInverted(true);
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

        Thread coboara_si_rot_stack_1 = new Thread(() -> {
            this.deschide();
            sleep(200);
            back_thing();
            ridica_lift(350, 0.4);
        });

        Thread coboara_si_rot_stack_2 = new Thread(() -> {
            sleep(400);

            // nigger

            back_thing();
            ridica_lift(250, 0.4);
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

        Thread ridica_si_rot = new Thread(() -> {
            brat.setRunMode(Motor.RunMode.RawPower);
            brat_pe_sub.setRunMode(Motor.RunMode.RawPower);

//            ridica(1200, 0.6, LiftMode.UP);

            inchide();
            sleep(400);
            start_brat(1);
            sleep(500);
            rotesteThing(1);
        });
        Thread ridica_si_rot_v2 = new Thread(() -> {
            brat.setRunMode(Motor.RunMode.RawPower);
            brat_pe_sub.setRunMode(Motor.RunMode.RawPower);

//            ridica(1200, 0.6, LiftMode.UP);

            inchide();
            start_brat(1);
            sleep(500);
            rotesteThing(1);
        });

        Thread ridica = new Thread(() -> {
            brat.setRunMode(Motor.RunMode.RawPower);
            brat_pe_sub.setRunMode(Motor.RunMode.RawPower);

            inchide();
            sleep(400);
            start_brat(0.9);
        });


        robot.setPoseEstimate(START_DR_RED_BLUE);

        f = robot.trajectoryBuilder(robot.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    ridica_si_rot.start();
                })
                .lineToLinearHeading(new Pose2d(43.5, -40, Math.toRadians(0)))
                .addDisplacementMarker(() -> robot.followTrajectorySequence(s))
                .build();

        s = robot.trajectorySequenceBuilder(f.end())
                .lineToSplineHeading(new Pose2d(39.2, -6, Math.toRadians(-10)))
                .addDisplacementMarker(this::deschide)
                .waitSeconds(0.2)
                .build();

        first = robot.trajectorySequenceBuilder(START_DR_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    ridica.start();
//                    ridica_si_rot.start();
                })
//                .setTurnConstraint(15, 15)
//                .setTangent(Math.toRadians(75))
//                .lineToSplineHeading(new Pose2d(40, -15, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(38, -6, Math.toRadians(100)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(38, -6, Math.toRadians(100)), Math.toRadians(100))
                .lineTo(new Vector2d(40, -25))
                .splineTo(new Vector2d(34.5, -3), Math.toRadians(120))
                .addDisplacementMarker(this::deschide)
                .waitSeconds(0.2)
                .build();

        stack_1 = robot.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(coboara_si_rot_stack_1::start)
                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(90)))
                .splineTo(new Vector2d(66.5, -6), Math.toRadians(0))
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction = robot.trajectorySequenceBuilder(stack_1.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot_v2::start)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -6, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .waitSeconds(0.5)
                .addDisplacementMarker(this::deschide)
                .build();

        stack_2 = robot.trajectorySequenceBuilder(back_junction.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_2::start)
                .lineToLinearHeading(new Pose2d(64, -6, Math.toRadians(0)))
                .addDisplacementMarker(this::inchide)
                .waitSeconds(0.2)
                .build();

        back_junction_after_math = robot.trajectorySequenceBuilder(stack_2.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot_v2::start)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(36, -6, Math.toRadians(0)))
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
            robot.followTrajectorySequence(stack_1);
            robot.followTrajectorySequence(back_junction);
            robot.followTrajectorySequence(stack_2);
            robot.followTrajectorySequence(back_junction_after_math);

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

    public void rotesteThing(double speed) {
        servo.setPower(speed);
    }

    public boolean getMagnetAtingere() {
        return magnet.isPressed();
    }

}


/**
 * Roses are red
 * Violets are blue
 * I wanna stick my dick inside you
 */