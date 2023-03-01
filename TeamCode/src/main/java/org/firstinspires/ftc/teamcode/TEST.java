package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.STACK_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.START_DR_RED_BLUE;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.PRE_POSITION_DR_RED_BLUE_KKK;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.TIMER_SENZOR_DR;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.power_brat_dc;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_st;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
@Config
public class TEST extends LinearOpMode {
    SampleMecanumDrive robot;
    TrajectorySequence first;
    Trajectory f;
    TrajectorySequence s;
    TrajectorySequence stack_1;
    TrajectorySequence stack_2;
    TrajectorySequence stack_3;
    TrajectorySequence back_junction;
    TrajectorySequence back_junction_after_math;
    TrajectorySequence back_junction_3;
    Thread coboara_si_rot_stack_3, coboara_si_rot_stack, coboara_stack;

    ArmcPIDF armcPIDF;

    public static boolean auto = false;

    private PIDController controller;
    public static double p = 0.002, i = 0, d = 0.0;

    DcMotorEx brat_pe_sub, brat;
    private Servo left, right;
    public VoltageSensor batteryVoltageSensor;

    CRServo servo;
    public TouchSensor magnet;

    Thread ridica_si_rot;

    Thread coboara_si_rot_stack_1;
    Thread coboara_si_rot_stack_2;
    Thread ridica;
    ElapsedTime time;

    Thread liftController;

    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        robot = new SampleMecanumDrive(hardwareMap);

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        brat_pe_sub = hardwareMap.get(DcMotorEx.class, "brat_pe_sub");
        brat = hardwareMap.get(DcMotorEx.class, "brat");

//        brat_pe_sub.setDirection(DcMotorEx.Direction.REVERSE);

        left = hardwareMap.get(Servo.class, "gheara_stanga");
        right = hardwareMap.get(Servo.class, "gheara_dreapta");

        servo = hardwareMap.get(CRServo.class, "sus");
        magnet = hardwareMap.get(TouchSensor.class, "magnet");

        armcPIDF = new ArmcPIDF(brat_pe_sub, brat, servo, magnet, left, right);

        liftController = new Thread(armcPIDF);

        robot = new SampleMecanumDrive(hardwareMap);

        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brat_pe_sub.setTargetPositionTolerance(1);
        time = new ElapsedTime();

        time.startTime();
        ridica_si_rot = new Thread(() -> {
            inchide();
            sleep(200);
            ThreadInfo.servo_speed = 1;
            ThreadInfo.target = 2000;
        });

        coboara_stack = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            ThreadInfo.target = 290;
        });

        coboara_si_rot_stack_2 = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
//            sleep(100);
            back_thing();
            ThreadInfo.target = 230;

        });

        coboara_si_rot_stack_3 = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            sleep(150);
            back_thing();
            ThreadInfo.target = 130;
        });

        robot.setPoseEstimate(START_DR_RED_BLUE);

        this.inchide();
        //ThreadInfo.closed_hand = true;

        first = robot.trajectorySequenceBuilder(START_DR_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    this.inchide();
//                    ridica.start();
                    ThreadInfo.target = 2000;
                })
                .lineTo(new Vector2d(40, -25))
                .splineTo(new Vector2d(33.7, -3.8), Math.toRadians(120))
                .build();

        stack_1 = robot.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(1, coboara_stack::start)
                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(90)))
                .splineTo(new Vector2d(65.7, -5.85), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(28))
                .addDisplacementMarker(() -> {
                    //ThreadInfo.closed_hand = false;
                })
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction = robot.trajectorySequenceBuilder(stack_1.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .lineToLinearHeading(new Pose2d(36.2, -4.5, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .addDisplacementMarker(() -> {
                    //ThreadInfo.closed_hand = true;
                })
                .build();

        stack_2 = robot.trajectorySequenceBuilder(back_junction.end())
                .addTemporalMarker(0, this::deschide)
                .addTemporalMarker(0.12, coboara_si_rot_stack_2::start)
                .lineToLinearHeading(new Pose2d(65.5, -6.65, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(28))
//                .waitSeconds(0.1)
//                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(() -> {
                    //ThreadInfo.closed_hand = false;
                })
                .build();

        back_junction_after_math = robot.trajectorySequenceBuilder(stack_2.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .lineToLinearHeading(new Pose2d(36, -4.6, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .waitSeconds(0.07)
                .addDisplacementMarker(() -> {
                    this.deschide();
                })
                .build();

        stack_3 = robot.trajectorySequenceBuilder(back_junction_after_math.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_3::start)
                .lineToLinearHeading(new Pose2d(65.5, -6.65, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(22))
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction_3 = robot.trajectorySequenceBuilder(stack_3.end())
                .addDisplacementMarker(ridica_si_rot::start)
                .lineToLinearHeading(new Pose2d(36, -4.8, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .build();
        ElapsedTime overall_timer = new ElapsedTime();

        liftController.start();
        telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
        telemetry.update();

        waitForStart();
        overall_timer.startTime();

        this.inchide();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
            telemetry.update();
            this.inchide();
//            sleep(400);

            robot.followTrajectorySequence(first);
//            ridica.join();
//            telemetry.addData("ridica thred: ", ridica.getState());
            robot.followTrajectorySequence(stack_1);
//            coboara_si_rot_stack_1.interrupt();
            coboara_stack.join();

            robot.followTrajectorySequence(back_junction);
//            ridica_si_rot.interrupt();
            ridica_si_rot.join();

            ElapsedTime timer = new ElapsedTime();
            timer.startTime();

            robot.followTrajectorySequence(stack_2);
//            coboara_si_rot_stack_2.interrupt();

            robot.followTrajectorySequence(back_junction_after_math);
//            ridica_si_rot.interrupt();
            telemetry.addLine(String.valueOf(timer.seconds()));

            robot.followTrajectorySequence(stack_3);
//            coboara_si_rot_stack_3.interrupt();

            robot.followTrajectorySequence(back_junction_3);
//            ridica_si_rot.interrupt();
            this.deschide();

            //parc st
            Trajectory fs = robot.trajectoryBuilder(back_junction_3.end())
                    .addTemporalMarker(0, () -> {
                        coboara_si_rot_stack_3.start();
                    })
                    .lineToLinearHeading(new Pose2d(17, -14, Math.toRadians(0)))
                    .build();
            robot.followTrajectory(fs);

            ThreadInfo.shouldClose = true;

            telemetry.addData("Overall timer: ", overall_timer.seconds());
            telemetry.update();
            sleep(300000);
            stop();
        }
        ThreadInfo.servo_speed = 0;

        ThreadInfo.shouldClose = true;

        telemetry.addData("te rog: ", ThreadInfo.shouldClose);
        telemetry.update();
    }


    private void start_brat(double power) {
        brat.setPower(power);
        brat_pe_sub.setPower(power);

//        brat.set(power);
//        brat_pe_sub.set(power);
    }


    private void stop_brat() {
//        brat.stopMotor();
//        brat_pe_sub.stopMotor();
        brat_pe_sub.setPower(0);
        brat.setPower(0);
    }


    public void inchide() {
        left.setPosition(poz_inchis_st);
        right.setPosition(poz_inchis_dr);
    }

    public void deschide() {
        left.setPosition(poz_deschis_st);
        right.setPosition(poz_deschis_dr);
    }

    private void back_thing() {
        time.reset();
        while (opModeIsActive() && !getMagnetAtingere()) {
            rotesteThing(-0.9);
            if (time.seconds() >= TIMER_SENZOR_DR)
                break;
        }
        rotesteThing(0);
    }

    private void rotesteThing(double speed) {
        servo.setPower(speed);
    }

    private boolean getMagnetAtingere() {
        return magnet.isPressed();
    }

    private void lift(double target, Thread thread, boolean alternative, LiftMode liftMode) {
        boolean ok = true;
        if (!alternative)
            try {
                throw new Exception("Esti prost?");
            } catch (Exception e) {
            }
        while (opModeIsActive() && brat_pe_sub.getCurrentPosition() != target && !thread.isInterrupted()) {
            controller.setPID(p, i, d);
            int armpos = brat_pe_sub.getCurrentPosition();
            double pid = controller.calculate(armpos, target);
            double power = pid;

            brat_pe_sub.setPower(power);
            brat.setPower(-power);

            if (liftMode == LiftMode.UP && brat_pe_sub.getCurrentPosition() > 600) {
                rotesteThing(1);
            } else {
                rotesteThing(0);
            }
            double dif;

            if (liftMode == LiftMode.UP || liftMode == LiftMode.NEUTRAL) {
                dif = target - brat_pe_sub.getCurrentPosition();
            } else {
                dif = Math.abs(target - brat_pe_sub.getCurrentPosition());
            }

            if (((liftMode == LiftMode.UP && target <= brat_pe_sub.getCurrentPosition()) ||
                    (liftMode == LiftMode.UP && target - 40 <= brat_pe_sub.getCurrentPosition()) ||
                    (liftMode == LiftMode.NEUTRAL && target - 40 <= brat_pe_sub.getCurrentPosition()) ||
                    (liftMode == LiftMode.NEUTRAL && target <= brat_pe_sub.getCurrentPosition())) ||
                    (liftMode == LiftMode.DOWN && target >= brat_pe_sub.getCurrentPosition())
            )
                thread.interrupt();

            telemetry.addData("pos ", armpos);
            telemetry.addData("target ", target);
            telemetry.addData("dif ", dif);
            telemetry.update();

//            if (dif > 2) ok = false;
        }
//        thread.interrupt();
    }
}


/**
 * Roses are red
 * Violets are blue
 * I wanna stick my dick inside of you
 */