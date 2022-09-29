package org.team5557.commands.climb.auto;

import org.team5557.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendToMid extends CommandBase{
    private ClimberSubsystem m_climb;

    public ExtendToMid(ClimberSubsystem climb) {
        m_climb = climb;
    }

    @Override
    public void initialize() {
        m_climb.setHeight(185);
    }

    @Override
    public boolean isFinished() {
        if (m_climb.getLeftHeight() >= 184 && m_climb.getRightHeight() >= 184) {
            return true;
        }
        return false;
    }


}
