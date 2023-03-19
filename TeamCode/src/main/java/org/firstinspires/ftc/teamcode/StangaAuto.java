package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.START_ST_RED_BLUE;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.TIMER_SENZOR_DR;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_deschis_st;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_dr;
import static org.firstinspires.ftc.teamcode.NU_MAI_POT.poz_inchis_st;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "STANGA")
@Config
public class StangaAuto extends LinearOpMode {

    /*
     *
     *          _.-/`)                              .-.
     *         // / / )                           __| |__
     *      .=// / / / )                         [__   __]
     *     //`/ / / / /      __________             | |
     *    // /     ` /      /          \            | |
     *   ||         /       | WE PRAY  |            | |
     *    \\       /        | TO WORK  |            '-'
     *     ))    .'         \_________/
     *    //    /
     *         /
     *
     */

    Thread camera_thread;
    SampleMecanumDrive robot;
    TrajectorySequence first;
    Trajectory f;
    TrajectorySequence s;
    TrajectorySequence stack_1;
    TrajectorySequence stack_2;
    TrajectorySequence stack_4;
    Trajectory fs, rot;
    TrajectorySequence stack_3;
    TrajectorySequence back_junction;
    TrajectorySequence back_junction_after_math;
    TrajectorySequence back_junction_3;
    TrajectorySequence back_junction_4;

    Thread coboara_si_rot_stack_3, coboara_si_rot_stack, coboara_stack;

    AprilTagDetection tagOfInterest;
    double cx = 402.145;
    double cy = 221.506;
    double fx = 578.272;
    double fy = 578.272;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    int numFramesWithoutDetection = 0;


    // UNITS ARE METERS
    double tagsize = 0.166;

    ArmcSTPIDF armcPIDF;
//    DetectieRunnable detectieRunnable;

    public static boolean auto = false;

    private PIDController controller;
    public static double p = 0.002, i = 0, d = 0.0;

    DcMotorEx brat_pe_sub, brat;
    private Servo left, right;
    public VoltageSensor batteryVoltageSensor;

    CRServo servo;
    public TouchSensor magnet;

    Thread ridica_si_rot;

    Thread coboara_si_rot_stack_2;
    Thread coboara_si_rot_stack_4;
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

        armcPIDF = new ArmcSTPIDF(brat_pe_sub, brat, servo, magnet, left, right);

        liftController = new Thread(armcPIDF);

        robot = new SampleMecanumDrive(hardwareMap);

        brat_pe_sub.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat_pe_sub.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brat_pe_sub.setTargetPositionTolerance(1);
        time = new ElapsedTime();

        time.startTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        tagOfInterest = new AprilTagDetection();
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                telemetry.addData("eroare: ", String.valueOf(errorCode));
                telemetry.update();
                tagOfInterest.id = 3;
            }
        });

        camera_thread = new Thread(camera::closeCameraDevice);

        ridica_si_rot = new Thread(() -> {
            inchide();
            sleep(20);
            ThreadInfoStanga.target = 2007;
            ThreadInfoStanga.servo_speed = -1;
        });

        coboara_stack = new Thread(() -> {
            this.deschide();
            ThreadInfoStanga.servo_speed = 0;
            ThreadInfoStanga.target = 180;
            this.servo.setPower(-0.1);
            sleep(70);
            this.servo.setPower(0);
        });

        coboara_si_rot_stack_2 = new Thread(() -> {
            this.deschide();
            ThreadInfoStanga.servo_speed = 0;
            sleep(120);
            back_thing(1);
            ThreadInfoStanga.target = 120;
        });

        coboara_si_rot_stack_3 = new Thread(() -> {
            this.deschide();
            ThreadInfoStanga.servo_speed = 0;
            sleep(60);
            back_thing(1);
            ThreadInfoStanga.target = 60;
        });

        coboara_si_rot_stack_4 = new Thread(() -> {
            this.deschide();
            ThreadInfoStanga.servo_speed = 0;
            sleep(100);
            back_thing(1);
            ThreadInfo.target = 20;
        });


        robot.setPoseEstimate(START_ST_RED_BLUE);

        this.inchide();

        first = robot.trajectorySequenceBuilder(START_ST_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    ThreadInfoStanga.target = 2003;
                })
                .lineTo(new Vector2d(-42, -30))
                .splineTo(new Vector2d(-34.7, -5), Math.toRadians(60))
                .build();

        stack_1 = robot.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(1, coboara_stack::start)
                .lineToLinearHeading(new Pose2d(-38.8, -17, Math.toRadians(90)))
//                .splineTo(new Vector2d(-65.7, -6.6), Math.toRadians(180),
                .splineTo(new Vector2d(-66.3, -6.6), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(this::inchide)

                .build();

        back_junction = robot.trajectorySequenceBuilder(stack_1.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .waitSeconds(0.6)
                .lineToLinearHeading(new Pose2d(-50, -7.2, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(-34.9, -7.5), Math.toRadians(20)) //cplm e cu headingul
//end new
                .waitSeconds(0.7)
                .addDisplacementMarker(this::deschide)
                .build();

        stack_2 = robot.trajectorySequenceBuilder(back_junction.end())
                .addTemporalMarker(0, this::deschide)
                .addTemporalMarker(0.12, coboara_si_rot_stack_2::start)
                .lineToLinearHeading(new Pose2d(-66, -8.2, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        back_junction_after_math = robot.trajectorySequenceBuilder(stack_2.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(-52, -7.2, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(-36.4, -7.5), Math.toRadians(20)) //cplm e cu headingul
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    this.deschide();
                })
                .build();


        stack_3 = robot.trajectorySequenceBuilder(back_junction_after_math.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_3::start)
                .lineToLinearHeading(new Pose2d(-65.8, -9.25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addDisplacementMarker(this::inchide)
                .build();


        back_junction_3 = robot.trajectorySequenceBuilder(stack_3.end())
                .addDisplacementMarker(ridica_si_rot::start)
                .setReversed(true)
                .waitSeconds(0.5)
//                .splineToLinearHeading(new Pose2d(-35.8, -4.55, Math.toRadians(180)), Math.toRadians(40))
                .lineToLinearHeading(new Pose2d(-46, -7.8, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(-36.4, -7.8), Math.toRadians(20)) //cplm e cu headingul

                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .build();
/*
        stack_4 = robot.trajectorySequenceBuilder(back_junction_3.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_4::start)
                .lineToLinearHeading(new Pose2d(-65.5, -7.7, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(22))
                .addDisplacementMarker(this::inchide)
                .build();
        back_junction_4 = robot.trajectorySequenceBuilder(stack_4.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                //.waitSeconds(0.7)
                //
                .lineToLinearHeading(new Pose2d(-46, -7.8, Math.toRadians(180)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(20))
                 .splineTo(new Vector2d(-34.8, -4), Math.toRadians(20)) //cplm e cu headingul
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .build();
*/
        ElapsedTime overall_timer = new ElapsedTime();

        ThreadInfoStanga.shouldClose = false;
        ThreadInfo.target = 0;
        ThreadInfo.servo_speed = 0;

        liftController.start();

        telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
//        telemetry.update();

        while (opModeInInit()) {

            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }
                    for (AprilTagDetection tag : detections) {
                        if (tag.id == 1 || tag.id == 2|| tag.id == 3) {
                            telemetry.addLine(String.valueOf(tag.id));
                            telemetry.update();
                            tagOfInterest = tag;
                        }
                    }
                }
            }
            telemetry.addData("id: ", tagOfInterest.id);
            telemetry.update();
        }

        waitForStart();
        overall_timer.startTime();

        this.inchide();

        while (opModeIsActive() && !isStopRequested()) {
//            DetectionInfo.closeCamera = true;

            telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));
            telemetry.update();
            this.inchide();
            camera.closeCameraDevice();

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
            coboara_si_rot_stack_3.interrupt();

            robot.followTrajectorySequence(back_junction_3);
            ridica_si_rot.interrupt();

//            robot.followTrajectorySequence(stack_4);

//            robot.followTrajectorySequence(back_junction_4);


            this.deschide();

            telemetry.addData("id 2: ", tagOfInterest.id);
            telemetry.update();

            switch (tagOfInterest.id) {
                case 1:
                    stanga();
                    break;
                case 2:
                    mijloc();
                    break;
                case 3:
                    dreapta();
                    break;
                default:
                    mijloc();
                    break;
            }

            robot.followTrajectory(fs);

//            if(tagOfInterest.id == 3 || tagOfInterest.id == 2)
//                robot.turn(Math.toRadians(-90));

            ThreadInfoStanga.shouldClose = true;
            liftController.join();


            telemetry.addData("Overall timer: ", overall_timer.seconds());
            telemetry.update();
            sleep(300000);
            stop();
        }

        ThreadInfoStanga.servo_speed = 0;

        ThreadInfoStanga.shouldClose = true;
        liftController.join();

        telemetry.addData("te rog: ", ThreadInfoStanga.shouldClose);
        telemetry.update();
    }

    public void inchide() {
        left.setPosition(poz_inchis_st);
        right.setPosition(poz_inchis_dr);
    }

    public void deschide() {
        left.setPosition(poz_deschis_st);
        right.setPosition(poz_deschis_dr);
    }

    private void back_thing(double power) {
        time.reset();
        while (opModeIsActive() && !getMagnetAtingere()) {
            rotesteThing(power);
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


    private void dreapta() {
        fs = robot.trajectoryBuilder(back_junction_after_math.end())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(-18, -8, Math.toRadians(180)))
                .build();
    }

    private void mijloc() {
        fs = robot.trajectoryBuilder(back_junction_after_math.end())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(-40.5, -14, Math.toRadians(180)))
                .build();
    }

    private void stanga() {
        fs = robot.trajectoryBuilder(back_junction_after_math.end())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(-64, -6.65, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(22))
                .build();
    }
}

/**
 * Roses are red
 * Violets are blue
 * I wanna stick my dick inside of you
 */
