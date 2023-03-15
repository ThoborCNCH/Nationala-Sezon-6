package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NU_MAI_POT.START_DR_RED_BLUE;
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

@Autonomous(name = "DREAPTA")
@Config
public class TEST extends LinearOpMode {

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
    Trajectory fs;
    TrajectorySequence stack_3;
    TrajectorySequence back_junction;
    TrajectorySequence back_junction_after_math;
    TrajectorySequence back_junction_3;
    TrajectorySequence back_junction_4;
    Thread coboara_si_rot_stack_3, coboara_si_rot_stack_4, coboara_stack;

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

    ArmcPIDF armcPIDF;
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
//            inchide();
            sleep(20);
            ThreadInfo.target = 2003;
            ThreadInfo.servo_speed = 1;
        });
        ThreadInfo.servo_speed = 0;

        coboara_stack = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            ThreadInfo.target = 289;
            this.servo.setPower(0.1);
            sleep(70);
            this.servo.setPower(0);
        });

        coboara_si_rot_stack_2 = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            sleep(120);
            back_thing(-0.75);
            ThreadInfo.target = 213;

        });

        coboara_si_rot_stack_3 = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            sleep(120);
            back_thing(-0.75);
            sleep(50);
            ThreadInfo.target = 103;
        });

        coboara_si_rot_stack_4 = new Thread(() -> {
            this.deschide();
            ThreadInfo.servo_speed = 0;
            sleep(150);
            back_thing(-1);
            ThreadInfo.target = 20;
        });

        robot.setPoseEstimate(START_DR_RED_BLUE);

        this.inchide();
        first = robot.trajectorySequenceBuilder(START_DR_RED_BLUE)
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    ThreadInfo.target = 2000;
                })
                .lineTo(new Vector2d(40, -25))
                .splineTo(new Vector2d(32.3, -4), Math.toRadians(120))
                .build();

        stack_1 = robot.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(1, coboara_stack::start)
                .lineToLinearHeading(new Pose2d(40, -17, Math.toRadians(90)))
                .splineTo(new Vector2d(65.7, -5.95), Math.toRadians(0),
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
                .waitSeconds(0.15)

//new
                .lineToLinearHeading(new Pose2d(48, -7, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(32, -5.7), Math.toRadians(160)) //cplm e cu headingul
//end new
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .addDisplacementMarker(() -> {
                    //ThreadInfo.closed_hand = true;
                })
                .build();

        stack_2 = robot.trajectorySequenceBuilder(back_junction.end())
                .addTemporalMarker(0, this::deschide)
//                0.12
                .addTemporalMarker(0, coboara_si_rot_stack_2::start)
                .lineToLinearHeading(new Pose2d(65.4, -7.1, Math.toRadians(0)),
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
                //.waitSeconds(0.7)
                //+
                .lineToLinearHeading(new Pose2d(47, -7, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(32.8, -5.07), Math.toRadians(160)) //cplm e cu headingul
                .waitSeconds(0.07)
                .addDisplacementMarker(() -> {
                    this.deschide();
                })
                .build();

        stack_3 = robot.trajectorySequenceBuilder(back_junction_after_math.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_3::start)
                .lineToLinearHeading(new Pose2d(65.8, -7.3, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(22))
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction_3 = robot.trajectorySequenceBuilder(stack_3.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                //.waitSeconds(0.7)
                //
                .lineToLinearHeading(new Pose2d(46, -7, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(32.8, -5.07), Math.toRadians(160)) //cplm e cu headingul
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .build();

// 6.65
        stack_4 = robot.trajectorySequenceBuilder(back_junction_3.end())
                .addDisplacementMarker(this::deschide)
                .addTemporalMarker(0, coboara_si_rot_stack_4::start)
                .lineToLinearHeading(new Pose2d(65.4, -7.7, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(22))
                .addDisplacementMarker(this::inchide)
                .build();

        back_junction_4 = robot.trajectorySequenceBuilder(stack_4.end())
                .addDisplacementMarker(this::inchide)
                .addDisplacementMarker(ridica_si_rot::start)
                //.waitSeconds(0.7)
                //
                .lineToLinearHeading(new Pose2d(46, -7, Math.toRadians(0)))
//                .splineToConstantHeading(new Vector2d(36.1, -4.5), Math.toRadians(-45))
                .splineTo(new Vector2d(33, -5), Math.toRadians(160)) //cplm e cu headingul
                .waitSeconds(0.07)
                .addDisplacementMarker(this::deschide)
                .build();


        ElapsedTime overall_timer = new ElapsedTime();

        ThreadInfoStanga.shouldClose = false;
        ThreadInfo.target = 0;
        ThreadInfo.servo_speed = 0;
        
        liftController.start();

        telemetry.addData("baterie: ", String.valueOf(batteryVoltageSensor.getVoltage()));

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
                        if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
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

            robot.followTrajectorySequence(stack_4);

            robot.followTrajectorySequence(back_junction_4);
            this.deschide();

            robot.update();
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


            ThreadInfo.shouldClose = true;
            liftController.join();


            telemetry.addData("Overall timer: ", overall_timer.seconds());
            telemetry.update();
            sleep(300000);
            stop();
        }
        ThreadInfo.servo_speed = 0;

        ThreadInfo.shouldClose = true;
        liftController.join();

        telemetry.addData("te rog: ", ThreadInfo.shouldClose);
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


    private void stanga() {
        fs = robot.trajectoryBuilder(robot.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(18, -8, Math.toRadians(0)))
                .build();
    }

    private void mijloc() {
        fs = robot.trajectoryBuilder(robot.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(40.5, -14, Math.toRadians(0)))
                .build();
    }

    private void dreapta() {
        fs = robot.trajectoryBuilder(robot.getPoseEstimate())
                .addTemporalMarker(0, () -> {
                    this.inchide();
                    coboara_si_rot_stack_3.start();
                })
                .lineToLinearHeading(new Pose2d(61, -6.65, Math.toRadians(0)),
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
