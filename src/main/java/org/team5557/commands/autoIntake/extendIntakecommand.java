package org.team5557.commands.autoIntake;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class extendIntakecommand extends CommandBase {
    private final IntakeSubsystem intake;
    
    public extendIntakecommand(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extendIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
