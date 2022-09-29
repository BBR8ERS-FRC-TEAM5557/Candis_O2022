package org.team5557.commands.climb.auto;

import org.team5557.subsystems.ClimberSubsystem;
import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RiseToMidAndLatch extends CommandBase {
    private ClimberSubsystem m_climb;
    private IntakeSubsystem m_intake;

    public RiseToMidAndLatch(ClimberSubsystem climb, IntakeSubsystem intake) {
        m_climb = climb;
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_climb.setHeight(9);
    }

    @Override
    public boolean isFinished() {
        if (m_climb.getLeftHeight() <= 10 && m_climb.getRightHeight() <= 10) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Timer.delay(0.25);
        m_intake.extendIntake();
        Timer.delay(1.5);
        m_climb.setMinandMaxOutput(0.1);
        /*
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        */
        //m_climb.kMaxOutput
        m_climb.setHeight(35);
    }
}
