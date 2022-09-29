package org.team5557.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class VisionSubsystem implements Subsystem {

    UsbCamera camera = CameraServer.startAutomaticCapture("Intake Camera", 0);

    private final SwerveSubsystemMK2 drivetrain;

    public VisionSubsystem(SwerveSubsystemMK2 drivetrainSubsystem) {
        drivetrain = drivetrainSubsystem;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    }
    
    public UsbCamera getUsbCamera() {
        return camera;
    }
}