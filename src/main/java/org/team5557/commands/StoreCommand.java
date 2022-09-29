package org.team5557.commands;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StoreCommand extends CommandBase{
    

    private IntakeSubsystem intake;
    private double storeSpeed;

    public StoreCommand(IntakeSubsystem intake, double storeSpeed) {
        this.intake = intake;
        this.storeSpeed = storeSpeed;

        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake.setStoreMotorOutput(storeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setStoreMotorOutput(0.0);
    }
}
