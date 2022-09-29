package org.team5557.util;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.library.math.Rotation2;
import org.team5557.RobotContainer;
import org.team5557.subsystems.SwerveSubsystemMK2;


public class DriverReadout {

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");
        

        /*
        tab.addNumber("Pressure", () -> container.getSuperstructure().getCurrentPressure())
                .withSize(2, 2)
                .withPosition(0, 0)
                .withWidget(BuiltInWidgets.kDial);
                */
        tab.add("Autonomous Mode", container.getAutonomousChooser().getAutonomousModeChooser())
            .withSize(2, 1)
            .withPosition(0, 0);
        tab.add("Zero Gyroscope", new ZeroGyroscope(container.getDrivetrainSubsystemMK2()))
            .withSize(2, 1)
            .withPosition(2, 0);
        tab.add("Climb Enabled", container.getClimberSubsystem().getClimbEnabled())
            .withSize(1, 1)
            .withPosition(4, 0);
        tab.add("Pressure", container.getIntakeSubsystem().getPressure())
            .withSize(1, 1)
            .withPosition(5, 0)
            .withWidget(BuiltInWidgets.kDial);
        tab.add("Flywheel Velocity", container.getShooterSubsystem().getFlywheelVelocity())
            .withSize(1, 1)
            .withPosition(6, 0)
            .withWidget(BuiltInWidgets.kGraph);
        tab.add("Is at Height", container.getClimberSubsystem().isAtHeight())
            .withSize(1, 1)
            .withPosition(7, 0);
        tab.add("Gyro Reading", container.getDrivetrainSubsystemMK2().getGyroAngle())
            .withSize(2,2)
            .withPosition(0, 1)
            .withWidget(BuiltInWidgets.kGyro);

        //tab.add("Limelight", )

        //tab.add("Intake Camera", container.getVisionSubsystem().getUsbCamera().enumerateSources());

    }


    private static class ZeroGyroscope extends CommandBase {
        private final SwerveSubsystemMK2 drivetrain;

        public ZeroGyroscope(SwerveSubsystemMK2 drivetrain) {
            this.drivetrain = drivetrain;

            setName("Zero Gyroscope");
        }

        @Override
        public void initialize() {
            drivetrain.resetGyroAngle(Rotation2.ZERO);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
