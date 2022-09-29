package org.team5557.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team5557.subsystems.IntakeSubsystem;
import org.frcteam2910.library.robot.input.XboxController;

public class SimpleIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double intakeSpeed;
    private double storeSpeed;

    private XboxController controller;



    public SimpleIntakeCommand(IntakeSubsystem intake, XboxController controller, double intakeSpeed, double storeSpeed) {
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.storeSpeed = storeSpeed;

        this.controller = controller;


        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.extendIntake();
        controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble,1.0);
    }

    @Override
    public void execute() {
        intake.setIntakeMotorOutput(intakeSpeed);
        intake.setStoreMotorOutput(storeSpeed);
    }


    @Override
    public void end(boolean interrupted) {
        intake.setStoreMotorOutput(0.0);
        intake.setIntakeMotorOutput(0.0);

        controller.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble,0.0);
        try {
            intake.retractIntakeDelicately();
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}
