package org.team5557.commands.climb;

import org.team5557.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroClimberCommand extends CommandBase{

    private final ClimberSubsystem climber;

    private long zeroVelocityTimestamp;
    private static final long CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 250;

    public ZeroClimberCommand(ClimberSubsystem climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.isZeroing = true;
        zeroVelocityTimestamp = Long.MAX_VALUE;
        climber.disableSoftLimits();
    }

    @Override
    public void execute() {
        climber.setMotorOutput(-0.1);
        if (zeroVelocityTimestamp == Long.MAX_VALUE) {
            zeroVelocityTimestamp = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - zeroVelocityTimestamp >= CLIMBER_ZERO_VELOCITY_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        //climber.configureSoftLimits();
        climber.setMotorOutput(0.0);
        climber.zeroHeight();
        climber.enableSoftLimits();
        climber.setHeight(0);
        climber.isZeroing = false;
    }
}
