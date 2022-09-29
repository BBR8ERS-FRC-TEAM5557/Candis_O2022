// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frcteam2910.library.math.RigidTransform2;
import org.frcteam2910.library.robot.UpdateManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Robot instance = null;
  //private static final Logger LOGGER = new Logger(Robot.class);
  
  private RobotContainer robotContainer = new RobotContainer();
  private UpdateManager updateManager = new UpdateManager(
    robotContainer.getDrivetrainSubsystemMK2()
  );

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    updateManager.startLoop(5.0e-3);
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.getDrivetrainSubsystemMK2().resetPose(RigidTransform2.ZERO);
    robotContainer.getAutonomousCommand().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new DriveCommand());
    robotContainer.getShooterSubsystem().idleFlywheel();
    //robotContainer.getDrivetrainSubsystemMK2().resetPose(new RigidTransform2(new Vector2(18,18), Rotation2.ZERO));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.print(robotContainer.getDrivetrainSubsystem().getVelocity());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
