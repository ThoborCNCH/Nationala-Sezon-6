package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {

    LiftSubsystem liftSubsystem;

    public LiftCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
}
