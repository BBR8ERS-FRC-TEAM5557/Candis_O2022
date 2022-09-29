package org.team5557.commands.feed;

import org.team5557.commands.SimpleShootCommand;
import org.team5557.subsystems.FeederSubsystem;
import org.team5557.subsystems.IntakeSubsystem;
import org.team5557.subsystems.ShooterSubsystem;
import org.team5557.subsystems.SwerveSubsystemMK2;
import org.team5557.util.ShooterConstantTuner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoFeednShoot extends ParallelCommandGroup {

    private FeederSubsystem feeder;
    private IntakeSubsystem intake;
    private long feedBallsTimestamp;
    private long FEED_BALLS_TIME_PERIOD = 2750;
    private long COMMAND_TIME_PERIOD = 3500;
    private int feedSpeed;

    public AutoFeednShoot(FeederSubsystem feederSubsystem, IntakeSubsystem intake, ShooterSubsystem shooter, SwerveSubsystemMK2 swerve){
		addCommands(
			new FeedDependent(5000, feederSubsystem, intake, shooter, swerve),
			new SimpleShootCommand(shooter, 3750)

		);
    }

}