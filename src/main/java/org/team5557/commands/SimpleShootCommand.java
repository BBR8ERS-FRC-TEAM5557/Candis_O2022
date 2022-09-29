package org.team5557.commands;

import org.team5557.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SimpleShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private double m_motorSpeed;

    public SimpleShootCommand(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.m_motorSpeed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.shootFlywheel(m_motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.idleFlywheel();
    }
}
