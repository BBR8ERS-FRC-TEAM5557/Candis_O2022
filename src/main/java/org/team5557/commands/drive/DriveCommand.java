package org.team5557.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.library.math.Vector2;
import org.frcteam2910.library.robot.input.Axis;
import org.team5557.subsystems.SwerveSubsystemMK2;

public class DriveCommand extends CommandBase {
    private SwerveSubsystemMK2 drivetrainSubsystemMK2;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public DriveCommand(SwerveSubsystemMK2 drivetrain, Axis forward, Axis strafe, Axis rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystemMK2 = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystemMK2.drive(new Vector2(forward.get(true), strafe.get(true)), rotation.get(true), true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystemMK2.drive(Vector2.ZERO, 0, false);
    }

}
