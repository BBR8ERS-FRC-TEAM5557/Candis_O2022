package org.team5557.commands.autoIntake;

import org.team5557.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class retractIntakecommand extends CommandBase {
    private final IntakeSubsystem intake;
    //private final double intakeSpeed;
   // private double storeSpeed;



    public retractIntakecommand(IntakeSubsystem intake) {//, double intakeSpeed, double storeSpeed) {
        this.intake = intake;
        //this.intakeSpeed = intakeSpeed;
        //this.storeSpeed = storeSpeed;


        //addRequirements(intake);
    }

    @Override
    public void initialize() {
        try {
            intake.retractIntakeDelicately();
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        //controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble,1.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
