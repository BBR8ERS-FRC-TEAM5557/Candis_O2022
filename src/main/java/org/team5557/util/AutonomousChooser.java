package org.team5557.util;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.team5557.Constants;
import org.team5557.RobotContainer;
import org.team5557.commands.*;
import org.team5557.commands.autoIntake.changeStoreAndIntake;
import org.team5557.commands.autoIntake.extendIntakecommand;
import org.team5557.commands.autoIntake.retractIntakecommand;
import org.team5557.commands.drive.FollowTrajectoryCommand;
import org.team5557.commands.ejects.DumpTwoBallsCommand;
import org.team5557.commands.feed.FeedDependent;
import org.team5557.commands.feed.RefactorBalls;
import org.frcteam2910.library.control.Trajectory;
import org.frcteam2910.library.math.RigidTransform2;
import org.frcteam2910.library.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final double shooterVelocity = Constants.highShooterRPM;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(RobotContainer container, AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.addOption("4 Ball Auto", AutonomousMode.FOUR_BALL);
        autonomousModeChooser.addOption("2 + 2 Ball Auto", AutonomousMode.TWO_PLUS_TWO_BALL);
        autonomousModeChooser.addOption("2 Ball Auto", AutonomousMode.TWO_BALL);
        autonomousModeChooser.addOption("Shoot and Scoot", AutonomousMode.ONE_BALL);
        autonomousModeChooser.setDefaultOption("Nothin but Shoot", AutonomousMode.DO_NOTHING_BUTSHOOT);
        autonomousModeChooser.addOption("Do Nothing", AutonomousMode.DO_NOTHING);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }


    private SequentialCommandGroup get4BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFourBallPartOne(), 270);

        command.addCommands(new extendIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), -1.0, 0.4));
        follow(command, container, trajectories.getFourBallPartOne());

        command.addCommands(new WaitCommand(0.25));
        command.addCommands(new retractIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), 0.0, 0.0));

        follow(command, container, trajectories.getFourBallPartTwo());
        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));


        command.addCommands(new extendIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), -1.0, 0.4));

        follow(command, container, trajectories.getFourBallPartThree());
        command.addCommands(new WaitCommand(0.75));

        follow(command, container, trajectories.getFourBallPartFour());
        command.addCommands(new WaitCommand(0.75));


        command.addCommands(new retractIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), 0.0, 0.0));

        follow(command, container, trajectories.getFourBallPartFive());
        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));

        return command;
    }

    private Command get2Plus2BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallPartOne(), 135);

        command.addCommands(new extendIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), -1.0, 0.4));
        follow(command, container, trajectories.getTwoBallPartOne());

        command.addCommands(new WaitCommand(0.25));
        command.addCommands(new retractIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), 0.0, 0.0));

        follow(command, container, trajectories.getTwoBallPartTwo());
        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));

        command.addCommands(new extendIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), -1.0, 0.4));
        follow(command, container, trajectories.getTwoPlusTwoBallPartThree());
        follow(command, container, trajectories.getTwoPlusTwoBallPartFour());
        command.addCommands(new WaitCommand(0.25));


        follow(command, container, trajectories.getTwoPlusTwoBallPartFive());
        command.addCommands(new WaitCommand(0.25));

        follow(command, container, trajectories.getTwoPlusTwoBallPartSix());
        follow(command, container, trajectories.getTwoPlusTwoBallPartSeven());
        command.addCommands(new DumpTwoBallsCommand(container.getIntakeSubsystem(), container.getFeederSubsystem())
            .withTimeout(1.5));

        return command;
    }

    private Command get2BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoBallPartOne(), 135);

        command.addCommands(new extendIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), -1.0, 0.4));
        follow(command, container, trajectories.getTwoBallPartOneSlow());

        command.addCommands(new WaitCommand(0.25));
        command.addCommands(new retractIntakecommand(container.getIntakeSubsystem()));
        command.addCommands(new changeStoreAndIntake(container.getIntakeSubsystem(), 0.0, 0.0));

        follow(command, container, trajectories.getTwoBallPartTwoSlow());
        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));


        return command;
    }

    private SequentialCommandGroup get1BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getScootBack(), 0);

        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));

        command.addCommands(new WaitCommand(8.0));
        follow(command, container, trajectories.getScootBack());

        return command;
    }

    private SequentialCommandGroup getDoNothingButShootCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new RefactorBalls(container.getIntakeSubsystem(), container.getFeederSubsystem())
                    .alongWith((new SimpleShootCommand(container.getShooterSubsystem(), shooterVelocity).withTimeout(4.0)))
                    .andThen(new FeedDependent(5000, container.getFeederSubsystem(), container.getIntakeSubsystem(),
                            container.getShooterSubsystem(), container.getDrivetrainSubsystemMK2())).withTimeout(3.0));
        return command;
    }

    private SequentialCommandGroup getDoNothingCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        return command;
    }


    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case FOUR_BALL:
                return get4BallAutoCommand(container);
            case TWO_PLUS_TWO_BALL:
                return get2Plus2BallAutoCommand(container);
            case TWO_BALL:
                return get2BallAutoCommand(container);
            case ONE_BALL:
                return get1BallAutoCommand(container);
            case DO_NOTHING_BUTSHOOT:
                return getDoNothingButShootCommand(container);
            case DO_NOTHING:
                return getDoNothingCommand(container);
        }
        return get4BallAutoCommand(container);
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystemMK2(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory,
            double initialRotation) {
        /*
         * audience frame of refference:
         * 0 = intake facing right
         * 90 = intake facing away
         * 180 = intake facing left
         * 270 = intake facing down
         */
        command.addCommands(new InstantCommand(
                () -> container.getDrivetrainSubsystemMK2().resetGyroAngle(Rotation2.fromDegrees(-initialRotation))));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystemMK2().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(),
                        Rotation2.fromDegrees(initialRotation)))));
    }

    private enum AutonomousMode {
        FOUR_BALL,
        TWO_BALL,
        TWO_PLUS_TWO_BALL,
        DO_NOTHING,
        ONE_BALL,
        DO_NOTHING_BUTSHOOT
    }
}
