package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class LiftPIDF extends LinearOpMode {

    private Motor brat, brat_pe_sub;

    ElevatorFeedforward elevatorFeedforward;

    public double power = 0.6;

    public static int targetPosition = 1800;

    @Override
    public void runOpMode() throws InterruptedException {

        brat = new Motor(hardwareMap, "brat");
        brat_pe_sub = new Motor(hardwareMap, "brat_pe_sub");


        brat_pe_sub.setRunMode(Motor.RunMode.PositionControl);
        brat_pe_sub.resetEncoder();

        brat_pe_sub.setTargetPosition(targetPosition);

        brat.setInverted(true);
        brat_pe_sub.setInverted(true);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double delta = Math.abs(targetPosition) - Math.abs(brat_pe_sub.getCurrentPosition());
            double first = delta, second = delta * 2, third = second + first, fourth = second * 2;

            double power_real = power / 2;

            while (!brat_pe_sub.atTargetPosition() && opModeIsActive()) {

                int pos = brat_pe_sub.getCurrentPosition();

                if (pos <= first)
                    power_real = power / 8;
                if (pos >= first && pos <= second)
                    power_real = power;
                if (pos >= third && pos <= fourth)
                    power_real = power / 8;

                double power_brat = power_real;
                if (pos >= targetPosition)
                    power_brat = -power_brat;

                brat_pe_sub.set(power_real);
                brat.set(power_real);

                telemetry.addData("Actual:", brat_pe_sub.getCurrentPosition());
                telemetry.addData("Targer:", targetPosition);
                telemetry.addData("Power:", power);
                telemetry.update();
            }

            brat.set(0.05);
            brat_pe_sub.set(0.05);
            stop();
        }

    }
}
