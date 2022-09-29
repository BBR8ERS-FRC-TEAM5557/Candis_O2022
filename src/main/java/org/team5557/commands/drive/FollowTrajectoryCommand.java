package org.team5557.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team5557.subsystems.SwerveSubsystemMK2;
import org.frcteam2910.library.control.Trajectory;

public class FollowTrajectoryCommand extends CommandBase {
    private final SwerveSubsystemMK2 drivetrain;
    private final Trajectory trajectory;

    public FollowTrajectoryCommand(SwerveSubsystemMK2 drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().follow(trajectory);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.getFollower().cancel();
        System.out.println("canceled");
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
    }
}
