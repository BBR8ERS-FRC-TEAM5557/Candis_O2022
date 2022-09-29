package org.team5557.commands.feed;

import org.team5557.commands.SimpleFeedCommand;
import org.team5557.commands.SimpleIndexCommand;
import org.team5557.commands.autoIntake.extendIntakecommand;
import org.team5557.subsystems.FeederSubsystem;
import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RefactorBalls extends SequentialCommandGroup {
    public RefactorBalls(IntakeSubsystem intake, FeederSubsystem feeder) {
        addCommands(new extendIntakecommand(intake),
        new WaitCommand(1.0),
        new SimpleFeedCommand(feeder, -4000).alongWith(new SimpleIndexCommand(intake, -0.3)).withTimeout(0.5)
        );

        //addRequirements(intake, feeder);
    }

    @Override
    public void end(boolean interrupted) {

        System.out.println("Refactor Finished");
    }
}
