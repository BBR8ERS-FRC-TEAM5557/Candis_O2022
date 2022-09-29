package org.team5557.commands.ejects;

import org.frcteam2910.library.robot.input.XboxController;
import org.team5557.subsystems.FeederSubsystem;
import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DumpTwoBallsCommand extends CommandBase{

    private IntakeSubsystem intake;
    private FeederSubsystem feeder;

    public DumpTwoBallsCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
        this.intake = intake;
        this.feeder = feeder;

        addRequirements(intake);
    }
    @Override
    public void execute() {
        intake.setStoreMotorOutput(-1.0);
        feeder.setFeederMotorOutput(-1000);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setStoreMotorOutput(0.0);
        feeder.stopFeeder();
        //controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble,0.0);
    }
}