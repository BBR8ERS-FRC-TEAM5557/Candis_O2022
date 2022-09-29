package org.team5557.commands.feed;

import org.team5557.subsystems.FeederSubsystem;
import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Feed2Balls extends CommandBase {

    private FeederSubsystem feeder;
    private IntakeSubsystem intake;
    private long feedBallsTimestamp;
    private long FEED_BALLS_TIME_PERIOD = 2750;
    private long COMMAND_TIME_PERIOD = 3500;
    private int feedSpeed;

    public Feed2Balls(int feederMotorSpeed, FeederSubsystem feederSubsytem, IntakeSubsystem intakeSubsystem){
        feeder = feederSubsytem;
        intake = intakeSubsystem;
        feedSpeed = feederMotorSpeed;

        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        feedBallsTimestamp = Long.MAX_VALUE;
    }

    @Override
    public void execute() {
        if (feedBallsTimestamp == Long.MAX_VALUE) {
            feedBallsTimestamp = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - feedBallsTimestamp >= FEED_BALLS_TIME_PERIOD) {
            intake.setStoreMotorOutput(0.6);
        }
        if (System.currentTimeMillis() - feedBallsTimestamp >= 750) {
            feeder.setFeederMotorOutput(feedSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - feedBallsTimestamp >= COMMAND_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stopFeeder();
        intake.setStoreMotorOutput(0);
    }
}
