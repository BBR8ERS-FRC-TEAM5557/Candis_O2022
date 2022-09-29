package org.frcteam2910.library.robot.subsystems;

import org.frcteam2910.library.drivers.Gyroscope;
import org.frcteam2910.library.math.Vector2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Drivetrain extends Subsystem {
	public abstract Gyroscope getGyroscope();
	
	public abstract double getMaximumVelocity();
	public abstract double getMaximumAcceleration();

	@Override
	public abstract void updateKinematics(double timestamp);

	public abstract Vector2 getKinematicPosition();

	public abstract Vector2 getKinematicVelocity();

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putString("Drivetrain position", getKinematicPosition().toString());
		SmartDashboard.putNumber("Drivetrain X velocity", getKinematicVelocity().x);
		SmartDashboard.putNumber("Drivetrain Y velocity", getKinematicVelocity().y);

		SmartDashboard.putNumber("Drivetrain angle", getGyroscope().getAngle().toDegrees());
	}

	@Override
	public void zeroSensors() {}
}