package org.team5557.commands.autoIntake;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class changeStoreAndIntake extends CommandBase {
    private final IntakeSubsystem intake;
    private double intakeSpeed;
    private double storeSpeed;




    public changeStoreAndIntake(IntakeSubsystem intake, double intakeSpeed, double storeSpeed) {
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.storeSpeed = storeSpeed;


        //addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeMotorOutput(intakeSpeed);
        intake.setStoreMotorOutput(storeSpeed);
        //controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble,1.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
