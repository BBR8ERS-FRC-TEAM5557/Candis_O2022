// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frcteam2910.library.math.Rotation2;
import org.frcteam2910.library.robot.input.Axis;
import org.frcteam2910.library.robot.input.XboxController;
import org.frcteam2910.library.robot.input.DPadButton.Direction;
import org.team5557.commands.*;
import org.team5557.commands.climb.ZeroClimberCommand;
import org.team5557.commands.climb.auto.ClimbAuto;
import org.team5557.commands.drive.DriveCommand;
import org.team5557.commands.drive.DriveFOCommand;
import org.team5557.commands.ejects.EjectFromIntakeCommand;
import org.team5557.commands.feed.Feed2Balls;
import org.team5557.commands.feed.FeedDependent;
import org.team5557.commands.feed.RefactorBalls;
import org.team5557.subsystems.*;
import org.team5557.util.AutonomousChooser;
import org.team5557.util.AutonomousTrajectories;
import org.team5557.util.DriverReadout;
import org.team5557.util.ShooterConstantTuner;

import java.io.IOException;


public class RobotContainer {
  //Controllers
  private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
  private final XboxController secondaryController = new XboxController(1);

  private final DriverReadout driverReadout;
  private AutonomousTrajectories autonomousTrajectories;
  private final AutonomousChooser autonomousChooser;

  //Subsystems
  private final SwerveSubsystemMK2 drivetrainSubsystemMK2 = new SwerveSubsystemMK2();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(this);
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystemMK2);

  //Commands
  private final DriveCommand driveBase = new DriveCommand(drivetrainSubsystemMK2, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis());
  private ShooterConstantTuner tunableNumbers;


  public RobotContainer() {
    try {
      autonomousTrajectories = new AutonomousTrajectories(SwerveSubsystemMK2.TRAJECTORY_CONSTRAINTS);
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonomousChooser = new AutonomousChooser(this, autonomousTrajectories);

    primaryController.getLeftXAxis().setInverted(true);
    primaryController.getRightXAxis().setInverted(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystemMK2);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(climberSubsystem);
    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(visionSubsystem);

    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystemMK2, driveBase);

    driverReadout = new DriverReadout(this);
    tunableNumbers = new ShooterConstantTuner();
    System.out.println(tunableNumbers.getHighFeederRPM());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    //drivebase
    primaryController.getBackButton().whenPressed(
      () -> drivetrainSubsystemMK2.resetGyroAngle(Rotation2.ZERO)
    );

    //climber
    secondaryController.getDPadButton(Direction.UP).whenPressed( () -> climberSubsystem.setHeight(185));//climberSubsystem.getDesiredHeight() + 10) );
    secondaryController.getDPadButton(Direction.DOWN).whenPressed( () -> climberSubsystem.setHeight(0));//climberSubsystem.getDesiredHeight() - 10) );
    secondaryController.getDPadButton(Direction.LEFT).whenPressed(new ZeroClimberCommand(climberSubsystem));  
    secondaryController.getYButton().whenPressed( () -> climberSubsystem.changeClimbEnabled());
    secondaryController.getYButton().whenPressed( () -> shooterSubsystem.stopFlywheel());
    secondaryController.getAButton().whenPressed(new ClimbAuto(primaryController, secondaryController, climberSubsystem, intakeSubsystem));//(new AutomatedClimbSequence(climberSubsystem, intakeSubsystem, primaryController));

    //intake
    primaryController.getLeftTriggerAxis().getButton(0.5).whileHeld(new SimpleIntakeCommand(intakeSubsystem, primaryController, -1.0, 0.4));
    primaryController.getLeftBumperButton().whileHeld(new DriveFOCommand(drivetrainSubsystemMK2, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
    
    //shooter
    primaryController.getRightTriggerAxis().getButton(0.5).whenPressed(new SimpleShootCommand(shooterSubsystem, tunableNumbers.getHighRPM()).withTimeout(4.0));
    //primaryController.getRightTriggerAxis().getButton(0.5).whileHeld(command)
    primaryController.getRightBumperButton().whenPressed(new RefactorBalls(intakeSubsystem, feederSubsystem).andThen(new FeedDependent(5000, feederSubsystem, intakeSubsystem, shooterSubsystem, drivetrainSubsystemMK2)).withTimeout(4.0));
    primaryController.getRightBumperButton().whenPressed(new SimpleShootCommand(shooterSubsystem, Constants.highShooterRPM).withTimeout(4.0));

    primaryController.getXButton().whenPressed(new SimpleShootCommand(shooterSubsystem, Constants.lowShooterRPM).withTimeout(3.5)
      .alongWith(new Feed2Balls(2000, feederSubsystem, intakeSubsystem)));
      

    //ejects
    secondaryController.getXButton().whenPressed(new EjectFromIntakeCommand(intakeSubsystem, feederSubsystem, primaryController).withTimeout(0.5).andThen(new SimpleFeedCommand(feederSubsystem, -200).withTimeout(0.6)), true);
    
    
    /*
    primaryController.getYButton().whenPressed(new SimpleShootCommand(shooterSubsystem, tunableNumbers.getHighRPM()).withTimeout(3.5)
      .alongWith(new Feed2Balls(2000, feederSubsystem, intakeSubsystem)));
    */
  }

  private Axis getDriveForwardAxis() {
    return primaryController.getLeftYAxis();
  }

  private Axis getDriveStrafeAxis() {
      return primaryController.getLeftXAxis();
  }

  private Axis getDriveRotationAxis() {
      return primaryController.getRightXAxis();
  }

  public SwerveSubsystemMK2 getDrivetrainSubsystemMK2() {
    return drivetrainSubsystemMK2;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public XboxController getPrimaryController() {
    return primaryController;
  }
  
  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(this);
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  public FeederSubsystem getFeederSubsystem() {
    return feederSubsystem;
  }

  public ClimberSubsystem getClimberSubsystem() {
    return climberSubsystem;
  }

  public ShooterConstantTuner getTunableNumbers() {
    return tunableNumbers;
  }

}
