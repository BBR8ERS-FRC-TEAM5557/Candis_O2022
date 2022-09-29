package org.team5557.commands;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleIndexCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private double storeSpeed;

    public SimpleIndexCommand(IntakeSubsystem intake, double storeSpeed) {
        this.intake = intake;
        this.storeSpeed = storeSpeed;

    }


    @Override
    public void execute() {
        //intake.setIntakeMotorOutput(intakeSpeed);
        intake.setStoreMotorOutput(storeSpeed);
    }


    @Override
    public void end(boolean interrupted) {
        intake.setStoreMotorOutput(0.0);
    }
}
