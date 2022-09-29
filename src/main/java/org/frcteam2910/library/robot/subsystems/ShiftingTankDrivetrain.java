package org.frcteam2910.library.robot.subsystems;

@Deprecated
public abstract class ShiftingTankDrivetrain extends TankDrivetrain {

	public ShiftingTankDrivetrain(double trackWidth) {
		super(trackWidth);
	}

	public abstract boolean inHighGear();

	public abstract void setHighGear(boolean highGear);
}
