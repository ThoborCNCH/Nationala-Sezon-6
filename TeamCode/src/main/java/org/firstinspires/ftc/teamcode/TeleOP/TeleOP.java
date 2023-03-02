package org.firstinspires.ftc.teamcode.TeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.BratCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.LiftCommand;
import org.firstinspires.ftc.teamcode.Commands.ThingCommand;
import org.firstinspires.ftc.teamcode.NU_MAI_POT;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.BratSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ThingSubsystem;

@TeleOp
public class TeleOP extends CommandOpMode {
    private Motor lf;
    private Motor rf;
    private Motor lb;
    private Motor rb;



    GamepadEx driver2;
    GamepadEx driver1;

    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;


    SampleMecanumDrive robot;

    private Motor brat, brat_pe_sub;
    private CRServo sus;
    private Servo left, right;

    private TouchSensor magnet;

    private LiftSubsystem liftSubsystem;
    private LiftCommand liftCommand;

    private BratSubsystem bratSubsystem;
    private BratCommand bratCommand;

    private ThingSubsystem thingSubsystem;
    private ThingCommand thingCommand;

    private InstantCommand strangeCommand;
    private InstantCommand deschideCommand;

    private InstantCommand bratRotesteSTCommand;
    private InstantCommand bratRotesteDRCommand;
    private InstantCommand bratStangaAUTOCommand;
    private InstantCommand bratDreaptaAUTOCommand;
    private InstantCommand bratOpresteRotireCommand;

    private InstantCommand bratRotesteLeftTriggerCommand;
    private InstantCommand bratRotesteRightTriggerCommand;
    private InstantCommand bratOpresteRotireTriggerCommand;

    private InstantCommand liftRidicaCommand;
    private InstantCommand liftCoboaraCommand;
    private InstantCommand liftRidicaSlowCommand;
    private InstantCommand liftCoboaraSlowCommand;
    private InstantCommand liftOpresteCommand;

    private InstantCommand driveForwardDpadCommand;
    private InstantCommand driveBackDpadCommand;
    private InstantCommand driveStrafeLeftDpadCommand;
    private InstantCommand driveStrafeRightDpadCommand;
    private InstantCommand driveRotateLeftCommand;
    private InstantCommand driveRotateRightCommand;
    private InstantCommand driveStopDpadCommand;

    private Thread rotesteCentruSTThread;
    private Thread rotesteCentruDRThread;

    TriggerReader triggerReader;

    Button liftRidica, liftCoboara, liftStopST, liftStopDR;
    Button liftRidicaPad, liftCoboaraPad, liftStopPadSus, liftStopPadJos;

    Button apuca, arunca;

    Button rotesteAutoStanga, rotesteAutoDreapta;
    Button rotesteStanga, rotesteDreapta, rotesteStopST, rotesteStopDR;
    Trigger rotesteSTBratTrigger, rotesteDRBratTrigger, rotesteStopBratSTTrigger, rotesteStopBratDRTrigger;

    Button driveFrontal, driveBack, driveFrontalStop, driveBackStop;
    Button driveLeft, driveRight, driveLeftStop, driveRightStop;

    Trigger driveRotLeft, driveRotRight, driveRotLeftStop, driveRotRightStop;

    double power_ridica = 0.6;
    double power_coboara = -0.6;
    double power_ridica_slow = 0.5;
    double power_coboara_slow = -0.45;
    double power_roteste = 0.6;


    @Override
    public void initialize() {

        brat = new Motor(hardwareMap, "brat");
        brat_pe_sub = new Motor(hardwareMap, "brat_pe_sub");

        sus = hardwareMap.get(CRServo.class, "sus");

        left = hardwareMap.get(Servo.class, "gheara_stanga");
        right = hardwareMap.get(Servo.class, "gheara_dreapta");

        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        lb = new Motor(hardwareMap, "lr");
        rb = new Motor(hardwareMap, "rr");

        lf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        magnet = hardwareMap.get(TouchSensor.class, "magnet");

        brat.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        brat_pe_sub.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        brat.setInverted(true);
        brat_pe_sub.setInverted(true);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        liftSubsystem = new LiftSubsystem(brat, brat_pe_sub);
        liftCommand = new LiftCommand(liftSubsystem);

        bratSubsystem = new BratSubsystem(sus, magnet);
        bratCommand = new BratCommand(bratSubsystem);

        thingSubsystem = new ThingSubsystem(left, right);
        thingCommand = new ThingCommand(thingSubsystem);

        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);
        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);

        triggerReader = new TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER);

        rotesteCentruSTThread = new Thread(()->{
            ElapsedTime timer = new ElapsedTime();

            timer.reset();
           while (!magnet.isPressed() || timer.seconds() <= NU_MAI_POT.TIMER_SENZOR)
                sus.setPower(1);
           sus.setPower(0);
        });

        rotesteCentruDRThread = new Thread(()->{
            ElapsedTime timer = new ElapsedTime();

            timer.reset();
            while (!magnet.isPressed() || timer.seconds() <= NU_MAI_POT.TIMER_SENZOR)
                sus.setPower(-1);
            sus.setPower(0);
        });

        strangeCommand = new InstantCommand(() -> {
            thingSubsystem.apuca();
        }, thingSubsystem);

        deschideCommand = new InstantCommand(() -> {
            thingSubsystem.arunca();
        }, thingSubsystem);

        bratDreaptaAUTOCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteCentruTimer(-1);
        }, bratSubsystem);

        bratStangaAUTOCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteCentruTimer(1);
        }, bratSubsystem);

        liftRidicaCommand = new InstantCommand(() -> {
            liftSubsystem.liftPower(1);
        }, liftSubsystem);

        liftCoboaraCommand = new InstantCommand(() -> {
            liftSubsystem.liftPower(power_coboara);
        }, liftSubsystem);

        liftOpresteCommand = new InstantCommand(() -> {
            liftSubsystem.liftPower(0);
        }, liftSubsystem);

        liftRidicaSlowCommand = new InstantCommand(() -> {
            liftSubsystem.liftPower(power_ridica_slow);
        }, liftSubsystem);

        liftCoboaraSlowCommand = new InstantCommand(() -> {
            liftSubsystem.liftPower(power_coboara_slow);
        });

        bratRotesteSTCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(0.6);
        }, bratSubsystem);

        bratRotesteDRCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(-0.6);
        }, bratSubsystem);

        bratOpresteRotireCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(0);
        }, bratSubsystem);

        bratRotesteLeftTriggerCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(-driver2.getTrigger((GamepadKeys.Trigger.LEFT_TRIGGER)));
        }, bratSubsystem);

        bratRotesteRightTriggerCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        }, bratSubsystem);

        bratOpresteRotireTriggerCommand = new InstantCommand(() -> {
            bratSubsystem.rotesteThing(0);
        }, bratSubsystem);

        driveForwardDpadCommand = new InstantCommand(()->{
           driveSubsystem.dpad_frontal(0.6, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveBackDpadCommand = new InstantCommand(()->{
            driveSubsystem.dpad_frontal(-0.6, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveStrafeLeftDpadCommand = new InstantCommand(()->{
            driveSubsystem.dpad_lateral(-0.6, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveStrafeRightDpadCommand = new InstantCommand(()->{
           driveSubsystem.dpad_lateral(0.6, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveRotateLeftCommand = new InstantCommand(()->{
            driveSubsystem.rotire(-0.2, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveRotateRightCommand = new InstantCommand(()->{
           driveSubsystem.rotire(0.2, driver1.getButton(GamepadKeys.Button.LEFT_BUMPER), driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        }, driveSubsystem);

        driveStopDpadCommand = new InstantCommand(()->{
            driveSubsystem.drive(0, 0, 0);
        }, driveSubsystem);


        //bumpere lift
        liftRidica = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenHeld(liftRidicaCommand);
        liftCoboara = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenHeld(liftCoboaraCommand);
        liftStopST = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenReleased(liftOpresteCommand);
        liftStopDR = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenReleased(liftOpresteCommand);

        //DPAD lift
        liftRidicaPad = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenHeld(liftRidicaSlowCommand);
        liftCoboaraPad = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenHeld(liftCoboaraSlowCommand);
        liftStopPadSus = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenReleased(liftOpresteCommand);
        liftStopPadJos = new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenReleased(liftOpresteCommand);

        //butoane apuca arunca
        apuca = new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(strangeCommand);
        arunca = new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(deschideCommand);

        //DPAD rotire thing
        rotesteStanga = new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whileHeld(bratRotesteSTCommand);
        rotesteDreapta = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whileHeld(bratRotesteDRCommand);
        rotesteStopST = new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whenReleased(bratOpresteRotireCommand);
        rotesteStopDR = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenReleased(bratOpresteRotireCommand);

//        rotesteAutoDreapta = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(rotesteCentruSTThread::start);
//        rotesteAutoStanga = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(rotesteCentruDRThread::start);

//        #################################
//        rotesteAutoStanga = new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(bratStangaAUTOCommand);
//        rotesteAutoDreapta = new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(bratDreaptaAUTOCommand);

        rotesteSTBratTrigger = new Trigger(()->(driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)).whileActiveContinuous(bratRotesteSTCommand);
        rotesteStopBratSTTrigger = new Trigger(()->(driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) == 0).whenActive(bratOpresteRotireCommand);
        rotesteDRBratTrigger = new Trigger(()->(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) != 0).whileActiveContinuous(bratRotesteDRCommand);
        rotesteStopBratDRTrigger = new Trigger(()->(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) == 0).whenActive(bratOpresteRotireCommand);

        //CONTROL DPAD SASIU
        driveFrontal = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whileHeld(driveForwardDpadCommand);
        driveFrontalStop = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenReleased(driveStopDpadCommand);
        driveBack = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whileHeld(driveBackDpadCommand);
        driveBackStop = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenReleased(driveStopDpadCommand);
        driveLeft = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whileHeld(driveStrafeLeftDpadCommand);
        driveLeftStop = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenReleased(driveStopDpadCommand);
        driveRight = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whileHeld(driveStrafeRightDpadCommand);
        driveRightStop = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenReleased(driveStopDpadCommand);

        //ROTIRE TRIGGERS SASIU
        driveRotLeft = new Trigger(()->(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)).whileActiveContinuous(driveRotateLeftCommand);
        driveRotLeftStop = new Trigger(()->(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0)).whenActive(driveStopDpadCommand);
        driveRotLeft = new Trigger(()->(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0)).whileActiveContinuous(driveRotateRightCommand);
        driveRotLeftStop = new Trigger(()->(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0)).whenActive(driveStopDpadCommand);

        register(driveSubsystem, liftSubsystem, bratSubsystem, thingSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);

    }
}
