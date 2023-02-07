// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** To be used in AUTO ONLY for command groups. This will do nothing and potentially harm the robot process in teleop */
public class Wait extends CommandBase {
	private int targetTicks;
	private int count;

	/** To be used in AUTO ONLY for command groups. This will do nothing and potentially harm the robot process in teleop */
	public Wait(int sleepTime_ms) {
		targetTicks = (int) (sleepTime_ms / 20);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		count = 0;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		count++;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return count >= targetTicks;
	}
}
