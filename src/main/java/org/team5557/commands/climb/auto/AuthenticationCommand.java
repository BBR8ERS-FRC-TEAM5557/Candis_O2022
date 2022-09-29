package org.team5557.commands.climb.auto;

import org.frcteam2910.library.robot.input.XboxController;
import org.team5557.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AuthenticationCommand extends CommandBase {
    protected Timer m_timer = new Timer();
    XboxController m_controller;
    private ClimberSubsystem m_climb;


    public AuthenticationCommand(XboxController secondaryController, ClimberSubsystem climb) {
        m_controller = secondaryController;
        m_climb = climb;
    }

    @Override
    public void initialize() {
        m_climb.setMinandMaxOutput(1.0);
    }


    @Override
    public boolean isFinished() {
        if (m_controller.getBButton().get() == true) {
            return true;
        }
        return false;
    }

    
}
