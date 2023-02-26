//package org.firstinspires.ftc.teamcode.TeleOP;
//
//import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.limitare_vit;
//import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_de_putin;
//import static org.firstinspires.ftc.teamcode.Amin.NU_MAI_POT.power_thing_slow;
//import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.LOW_POWER;
//import static org.firstinspires.ftc.teamcode.Amin.incercareDetectie3Patrate.NuSeMaiUmbla.MEDIUM_POWER;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
////AUIMA AUE AUIMA AUE IN THE JUNGLE THE MIGHTY JUNGLE THE LION SLEEP TOONIGHTTTTT
//
//
//@TeleOp
//public class Telemeu extends LinearOpMode {
//    SampleMecanumDrive robot;
//    double mana;
//    double brat;
//
//    int servoGheara = 0;
//
//    double multi = 1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new SampleMecanumDrive(hardwareMap);
//        robot.brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.brat_pe_sub.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
////        waitForStart();
//
//        while (!opModeIsActive() && !isStopRequested()) {
//        }
//
//        while (opModeIsActive()) {
//
//            if (gamepad1.left_bumper) {
//                multi = 0.7;
//            }
//            if (!gamepad1.left_bumper) {
//                multi = 1;
//            }
//
//            //          MISCARE JOYSTICK
//            if (!(gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down)) {
//
//                robot.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y * limitare_vit * multi,
//                                -gamepad1.left_stick_x * limitare_vit * multi,
//                                -gamepad1.right_stick_x * limitare_vit * multi//X ul trebuie
//                        )
//                );
//            }
//
//            if (servoGheara != 0 && !robot.getMagnetAtingere()) {
//                if (servoGheara == -1)
//                    robot.rotesteThing(-1);
//                else
//                    robot.rotesteThing(1);
//            } else {
//                servoGheara = 0;
//                robot.rotesteThing(0);
//            }
////            if (gamepad2.x && servoGheara != 0 && !robot.getMagnetAtingere()) {
////                if (servoGheara == -1)
////                    robot.rotesteThing(-1);
////                else if (servoGheara == 1)
////                    robot.rotesteThing(1);
////            } else
////                robot.rotesteThing(0);
//
//            // miscari din dpad uri
//            if (gamepad1.dpad_up) {
//                robot.bagaViteza(MEDIUM_POWER * multi, MEDIUM_POWER * multi, MEDIUM_POWER * multi, MEDIUM_POWER * multi);
//                //                robot.setWeightedDrivePower(
//                //                        new Pose2d(
//                //                                0.3,
//                //                                0,
//                //                                0
//                //                        )
//                //                );
//            } else if (gamepad1.dpad_down) {
//                robot.bagaViteza(-MEDIUM_POWER * multi, -MEDIUM_POWER * multi, -MEDIUM_POWER * multi, -MEDIUM_POWER * multi);
//                //                robot.setWeightedDrivePower(
//                //                        new Pose2d(
//                //                                -0.3,
//                //                                0,
//                //                                0
//                //                        )
//                //                );
//            } else if (gamepad1.dpad_left) {
//                robot.bagaViteza(-MEDIUM_POWER * multi, MEDIUM_POWER * multi, MEDIUM_POWER * multi, -MEDIUM_POWER * multi);
//                //                robot.setWeightedDrivePower(
//                //                        new Pose2d(
//                //                                0,
//                //                                -0.3,
//                //                                0
//                //                        )
//                //                );
//            } else if (gamepad1.dpad_right) {
//                robot.bagaViteza(MEDIUM_POWER * multi, -MEDIUM_POWER * multi, -MEDIUM_POWER * multi, MEDIUM_POWER * multi);
//                //                robot.setWeightedDrivePower(
//                //                        new Pose2d(
//                //                                0,
//                //                                0.3,
//                //                                0
//                //                        )
//                //                );
//            } else if (!(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)) {
//                robot.bagaViteza(0, 0, 0, 0);
//            }
//            //            while(gamepad1.dpad_right) {
//            //                robot.bagaViteza(MEDIUM_POWER, -MEDIUM_POWER, -MEDIUM_POWER, MEDIUM_POWER);
//            //            }
//            //            while(gamepad1.dpad_up) {
//            //                robot.bagaViteza(LOW_POWER, LOW_POWER, LOW_POWER, LOW_POWER);
//            //            }
//            //            while(gamepad1.dpad_left) {
//            //                robot.bagaViteza(-MEDIUM_POWER, MEDIUM_POWER, MEDIUM_POWER, -MEDIUM_POWER);
//            //            }
//
//            // rotiri fine din triggere
//            while (gamepad1.right_trigger != 0) {
//                robot.bagaViteza(LOW_POWER * multi, -LOW_POWER * multi, LOW_POWER * multi, -LOW_POWER * multi);
//            }
//            while (gamepad1.left_trigger != 0) {
//                robot.bagaViteza(-LOW_POWER * multi, LOW_POWER * multi, -LOW_POWER * multi, LOW_POWER * multi);
//            }
//
//            //            if (gamepad2.dpad_up || gamepad2.dpad_down) {
//            //                if (gamepad2.dpad_up)
//            //                    brat = -power_de_cosmin;
//            //                else
//            //                    brat = power_de_cosmin;
//            ////                robot.se_ridica_brat(brat);
//            //            } else {
//            //                brat = 0;
//            ////                robot.se_ridica_brat(brat);
//            //            }
//            //            robot.se_ridica_brat(brat);
//
//            //            ASTA NOUA
//            if (gamepad2.dpad_right || gamepad2.dpad_left) {
//                if (gamepad2.dpad_left) {
//                    mana = power_thing_slow;
//                    servoGheara = 0;
//
//                } else if (gamepad2.dpad_right) {
//                    mana = -power_thing_slow;
//                    servoGheara = 0;
//
//                } else
//                    mana = 0;
//                ///////////////--------------AM ADAUGAT AICI ONDITIE
//                if (servoGheara == 0) {
//                    robot.rotesteThing(mana);
//                }
//
////                servoGheara = 0;
//
//            } else if (servoGheara == 0)
//                ///////////////--------------AM ADAUGAT AICI ONDITIE
//                robot.rotesteThing(0);
//
//            if (gamepad2.left_bumper || gamepad2.right_bumper) {
//                if (gamepad2.left_bumper)
//                    brat = -power_de_putin;
//                else
//                    brat = power_de_putin;
//                //                robot.se_ridica_brat(brat);
//
//                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
//                    if (gamepad2.left_trigger != 0) {
//                        mana = 1;
//                        servoGheara = 0;
//                    } else if (gamepad2.right_trigger != 0) {
//                        mana = -1;
//                        servoGheara = 0;
//                    } else
//                        mana = 0;
//                    servoGheara = 0;
//                    robot.rotesteThing(mana);
//                } else if (servoGheara != 0) {
//                    //////////////////////////////-----------------am adaugat aici conditie
//                    robot.rotesteThing(0);
//                }
//
//
//                if (gamepad2.dpad_up)
//                    robot.apuca(0.1, 1);
//
//
//                if (gamepad2.a)
//                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
//                else if (gamepad2.b)
//                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);
//
//                if (gamepad2.y)
//                    servoGheara = 1;
//                if (gamepad2.x)
//                    servoGheara = -1;
//
//
//            } else {
//                //                robot.se_ridica_brat(0);
//                brat = 0;
//
//                if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
//                    if (gamepad2.left_trigger != 0)
//                        mana = 1;
//                    else if (gamepad2.right_trigger != 0)
//                        mana = -1;
//                    else
//                        mana = 0;
//
//                    servoGheara = 0;
//                    robot.rotesteThing(mana);
//                } else
//                    ///////////////--------------AM ADAUGAT AICI ONDITIE
//                    if (servoGheara == 0)
//                        robot.rotesteThing(0);
//
//                if (gamepad2.dpad_up)
//                    robot.apuca(0.1, 1);
//
//                if (gamepad2.a)
//                    robot.apuca(NU_MAI_POT.poz_deschis_st, NU_MAI_POT.poz_deschis_dr);
//                if (gamepad2.b)
//                    robot.apuca(NU_MAI_POT.poz_inchis_st, NU_MAI_POT.poz_inchis_dr);
//                if (gamepad2.y)
//                    servoGheara = 1;
//                if (gamepad2.x)
//                    servoGheara = -1;
//            }
//            robot.se_ridica_brat(brat);
//        }
//    }
//}
