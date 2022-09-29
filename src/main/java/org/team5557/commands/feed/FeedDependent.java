package org.team5557.commands.feed;

import org.team5557.subsystems.FeederSubsystem;
import org.team5557.subsystems.IntakeSubsystem;
import org.team5557.subsystems.ShooterSubsystem;
import org.team5557.subsystems.SwerveSubsystemMK2;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedDependent extends CommandBase {

    private FeederSubsystem feeder;
    private IntakeSubsystem intake;
    private long feedBallsTimestamp = -1;
    private long COMMAND_TIME_PERIOD = 1500;
    private double feedSpeed;
    private ShooterSubsystem shooter;
    private SwerveSubsystemMK2 swerve;

    public FeedDependent(double feederMotorSpeed, FeederSubsystem feederSubsytem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, SwerveSubsystemMK2 swerveSubsystem) {
        feeder = feederSubsytem;
        intake = intakeSubsystem;
        swerve = swerveSubsystem;
        feedSpeed = feederMotorSpeed;
        shooter = shooterSubsystem;

        addRequirements(intake, feeder);
    }

    @Override
    public void initialize() {
        feeder.setFeederMotorOutput(feedSpeed);
    }

    @Override
    public void execute() {
        if (shooter.isFlywheelAtTargetVelocity() && swerve.getAngularVelocity() < 0.5 && swerve.getVelocity().length < 10.0) {
            intake.setStoreMotorOutput(0.7);
            intake.setIntakeMotorOutput(-0.5);

            if (feedBallsTimestamp < 0) {
                feedBallsTimestamp = System.currentTimeMillis();
            }
        }
        else {
            intake.setStoreMotorOutput(0);
            intake.setIntakeMotorOutput(0);
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - feedBallsTimestamp >= COMMAND_TIME_PERIOD;
    }

    @Override
    public void end(boolean interrupted) {
        feedBallsTimestamp = -1;
        feeder.stopFeeder();
        intake.setStoreMotorOutput(0);
        intake.setIntakeMotorOutput(0);

        try {
            intake.retractIntakeDelicately();
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}