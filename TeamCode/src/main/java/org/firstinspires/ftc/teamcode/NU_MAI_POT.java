package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class NU_MAI_POT {
    public static double LF_DIRECTION = 1;
    public static double RF_DIRECTION = 0;
    public static double LR_DIRECTION = 1;
    public static double RR_DIRECTION = 0;

    public static double fr = 1;
    public static double lr = 1;
    public static double rr = 0;

    public static int high_junc = 1000;
    public static int mediu_junc = 500;
    public static int low_junc = 300;
    public static int jos_junc = 0;

    public static double TIMER_SENZOR = 0.65;
    public static double TIMER_SENZOR_DR = 0.65;

    public static double power_top = 0.3;
    public static double power_brat_dc = .6;
    public static double power_brat_cr = 1;
    public static double power_coborare = -0.5;
    public static double power_brat_dc_cob = .7;
    public static double power_thing_slow = 0.3;

    public static double power_de_putin = 0.8;

    public static int cob1 = 365;
    public static int cob2 = 20;
    public static int cob3 = 406;

    public static double power_reven = 0.2;

    public static double poz_deschis_st = 0.32; //0.18
    public static double poz_deschis_dr = 0.69; //0.65
    public static double poz_inchis_st = 0.2; //0
    public static double poz_inchis_dr = 0.9; //0.75

    public static double poz_deschis_st_AUTO = 0.7; //0.7
    public static double poz_deschis_dr_AUTO = 0.7; //0.4

    public static double poz_inschis_st_AUTO = 0.11; //0.7
    public static double poz_inschis_dr_AUTO = 0.97; //0.4


    public static boolean GLISIERE_ENCODER = true;
    public static double DISTANTA_GLISIERA = 74; //CM
    public static int GLISIERA_COUNTS = 0;

    public static double limitare_vit = .9;

    public static Pose2d START_DR_RED_BLUE = new Pose2d(40.5, -58, Math.toRadians(90));
    public static Pose2d START_ST_RED_BLUE = new Pose2d(-40.5, -58, Math.toRadians(90));
    public static Pose2d INTRE_TOT_DR_RED_BLUE = new Pose2d(43, -10, Math.toRadians(90));
    public static Vector2d INTRE_TOT_DR_RED_BLUE_VECTOR = new Vector2d(43, -10);
    public static Pose2d PRE_POSITION_DR_RED_BLUE2 = new Pose2d(43, -10, Math.toRadians(90));

    public static Pose2d PRE_POSITION_DR_RED_BLUE = new Pose2d(42.8, -6.8, Math.toRadians(139));
    public static Pose2d PRE_POSITION_DR_RED_BLUE_KKK = new Pose2d(43, -5.5, Math.toRadians(0)); //-5.9 inainte de filmare
    public static Pose2d PRE_POSITION_DR_RED_BLUE3 = new Pose2d(43.5, -5.5, Math.toRadians(138));

    public static Pose2d JUNCTION_PUNE_INAINTE_DR_RED_BLUE = new Pose2d(38.3, -4.5, Math.toRadians(90));
    public static Pose2d JUNCTION_PUNE_DR_RED_BLUE = new Pose2d(38.3, -1.5, Math.toRadians(130));
    public static Vector2d JUNCTION_PUNE_DR_RED_BLUE_VECTOR = new Vector2d(32, -3.5);
    public static Pose2d STACK_DR_RED_BLUE = new Pose2d(67, -9, Math.toRadians(10)); //ERA 60 BY SENZOR LA X
    public static Pose2d STACK_DR_RED_BLUE2 = new Pose2d(67.2, -9.6, Math.toRadians(0));
    public static Pose2d JUNCTION_THING_DR_RED_BLUE = new Pose2d(36.99, -5.88, Math.toRadians(0));
    public static Pose2d JUNCTION_THING_DR_RED_BLUE2 = new Pose2d(36.8, -5.87, Math.toRadians(0));
    public static Vector2d STACK_DR_RED_BLUE_VECTOR = new Vector2d(68, -8.5);
    public static Pose2d BACK_A_LITTLE_DR_RED_BLUE = new Pose2d(-36, -40, Math.toRadians(180));

    public static Pose2d PRE_POSITION_ST_RED_BLUE = new Pose2d(-43.6, -10, Math.toRadians(90));
    public static Pose2d PRE_POSITION_ST_RED_BLUE_KKK = new Pose2d(-35, -5.2, Math.toRadians(180));
    public static Pose2d JUNCTION_THING_ST_RED_BLUE2 = new Pose2d(-36, -6.2, Math.toRadians(180)); //era -36
    public static Pose2d JUNCTION_THING_ST_RED_BLUE = new Pose2d(-36.7, -5.5, Math.toRadians(180));
    public static Pose2d STACK_ST_RED_BLUE = new Pose2d(-62.8, -8.2, Math.toRadians(180));
    public static Pose2d STACK_ST_RED_BLUE2 = new Pose2d(-62.9, -9, Math.toRadians(180));


    public static Vector2d JUNCTION_PUNE_ST_RED_BLUE_VECTOR = new Vector2d(-31, -3.5);

}
