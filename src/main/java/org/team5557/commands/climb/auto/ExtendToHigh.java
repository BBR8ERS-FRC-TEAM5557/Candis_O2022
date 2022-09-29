package org.team5557.commands.climb.auto;

import org.team5557.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendToHigh extends CommandBase {
    private ClimberSubsystem m_climb;

    public ExtendToHigh(ClimberSubsystem climb) {
        m_climb = climb;
    }

    @Override
    public void initialize() {
        m_climb.setHeight(180);
    }

    @Override
    public boolean isFinished() {
        if (m_climb.getLeftHeight() >= 179 && m_climb.getRightHeight() >= 179) {
            return true;
        }
        return false;
    }
    
}
