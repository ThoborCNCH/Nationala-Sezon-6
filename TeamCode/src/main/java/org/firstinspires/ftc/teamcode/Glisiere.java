package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class Glisiere extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Motor brat, brat_pe_sub;

        brat = new Motor(hardwareMap, "brat");
        brat_pe_sub = new Motor(hardwareMap, "brat_pe_sub");

        brat.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        brat_pe_sub.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        brat.setInverted(true);
        brat_pe_sub.setInverted(true);

        double power = 0;

        waitForStart();

        while (opModeIsActive()){

            if(gamepad2.right_bumper)
                power = 0.6;
            else if(gamepad2.left_bumper)
                power = -0.6;
            else
                power = 0;

            brat.set(power);
            brat_pe_sub.set(power);

        }

    }
}
