package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.BratSubsystem;

public class BratCommand extends CommandBase {

    BratSubsystem bratSubsystem;

    public BratCommand(BratSubsystem bratSubsystem){
        this.bratSubsystem = bratSubsystem;
        addRequirements(bratSubsystem);
    }
}
