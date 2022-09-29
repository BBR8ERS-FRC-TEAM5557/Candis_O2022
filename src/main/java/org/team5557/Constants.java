// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team5557;

import org.frcteam2910.library.math.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final double highShooterRPM = 3750;
    public static final double lowShooterRPM = 2000;

	/**
	 * How many milliseconds (ms) are in a second.
	 */
	public static final double MILLISECONDS = 1e3;

	/**
	 * How many nanoseconds (ns) are in a second.
	 */
	public static final double NANOSECONDS = 1e9;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 8;

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 0;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 2;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 3;


    //ZERO SWERVEEEEE
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(313.8);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(253.7);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(180.3);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(181.8);
    //ZERO SWERVEEEE

    public static final int CLIMBER_RIGHT_MOTOR_MASTER = 9;
    public static final int CLIMBER_LEFT_MOTOR_MASTER = 10;
    public static final int CLIMBER_LEFT_MOTOR_INVERSE = 11;
    public static final int CLIMBER_RIGHT_MOTOR_INVERSE = 12;
    public static final int INTAKE_MOTOR_CONTROLLER = 17;
    public static final int STORE_MOTOR_CONTROLLER = 16;
    public static final int UPLIFT_MOTOR_CONTROLLER = 15;
    public static final int SHOOTER_FLYWHEEL_MOTOR = 14;
    public static final int SHOOTER_FLYWHEEL_MOTORSLAVE = 13;

    public static final Vector2 kGoal = new Vector2(324, 162);


    //public static final int BOTTOM_INTAKE_EXTENSION_SOLENOID = 0;
    //public static final int TOP_INTAKE_EXTENSION_SOLENOID = 1;

    public static final int PRIMARY_CONTROLLER_PORT = 0;


    //public static final int CLIMBER_LOCK_SOLENOID_PORT = 2;


    //public static final int PIGEON_PORT = 20;

    public static final int PRESSURE_SENSOR_PORT = 0;


}
