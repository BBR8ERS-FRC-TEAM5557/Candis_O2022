package org.team5557.commands.climb.auto;

import org.frcteam2910.library.robot.input.XboxController;
import org.team5557.subsystems.ClimberSubsystem;
import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbAuto extends SequentialCommandGroup {

    public ClimbAuto(XboxController primaryController, XboxController secondaryController, ClimberSubsystem climb, IntakeSubsystem intake) {
        addCommands(new ExtendToMid(climb),
        new AuthenticationCommand(secondaryController, climb),
        new RiseToMidAndLatch(climb, intake),
        new AuthenticationCommand(secondaryController, climb),
        new ExtendToHigh(climb),
        new AuthenticationCommand(secondaryController, climb),
        new JerkAndPull(intake),
        //new AuthenticationCommand(secondaryController),
        new RiseToMidAndLatch(climb, intake),
        new AuthenticationCommand(secondaryController, climb),
        new ExtendToHigh(climb),
        new AuthenticationCommand(secondaryController, climb),
        new JerkAndPull(intake),
        //new AuthenticationCommand(secondaryController),
        new RiseToTravy(climb)
        );
    }
    
}
