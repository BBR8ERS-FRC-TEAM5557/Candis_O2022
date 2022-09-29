package org.frcteam2910.library.robot.commands;

import org.frcteam2910.library.robot.subsystems.ShiftingTankDrivetrain;

import edu.wpi.first.wpilibj.command.Command;

@Deprecated
public class SetDrivetrainGearCommand extends Command {
	private final ShiftingTankDrivetrain drivetrain;
	private final boolean highGear;

	public SetDrivetrainGearCommand(ShiftingTankDrivetrain drivetrain, boolean highGear) {
		this.drivetrain = drivetrain;
		this.highGear = highGear;
	}

	@Override
	protected void initialize() {
		drivetrain.setHighGear(highGear);
	}

	@Override
	protected boolean isFinished() {
		return true;
	}
}
