package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ThingSubsystem;

public class ThingCommand extends CommandBase {

    ThingSubsystem thingSubsystem;

    public ThingCommand(ThingSubsystem thingSubsystem){
        this.thingSubsystem = thingSubsystem;
        addRequirements(thingSubsystem);
    }
}
