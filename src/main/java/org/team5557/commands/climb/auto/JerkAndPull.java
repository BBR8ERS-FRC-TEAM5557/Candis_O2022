package org.team5557.commands.climb.auto;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class JerkAndPull extends CommandBase {
    private IntakeSubsystem m_intake;

    public JerkAndPull(IntakeSubsystem intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.retractIntakeViolently();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
