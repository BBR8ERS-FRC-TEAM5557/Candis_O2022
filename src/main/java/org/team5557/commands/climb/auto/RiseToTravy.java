package org.team5557.commands.climb.auto;

import org.team5557.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RiseToTravy extends CommandBase {
    private ClimberSubsystem m_climb;

    public RiseToTravy(ClimberSubsystem climb) {
        m_climb = climb;
    }

    @Override
    public void initialize() {
        m_climb.setHeight(10);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
