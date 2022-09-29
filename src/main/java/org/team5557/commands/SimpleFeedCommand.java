package org.team5557.commands;

import org.team5557.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleFeedCommand extends CommandBase {

    private FeederSubsystem feeder;
    private double feederSpeed;

    public SimpleFeedCommand(FeederSubsystem feederSubsytem, double feederMotorOutput){
        feeder = feederSubsytem;
        feederSpeed = feederMotorOutput;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.setFeederMotorOutput(feederSpeed);
        System.out.println("feeder running");
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stopFeeder();
    }
}

