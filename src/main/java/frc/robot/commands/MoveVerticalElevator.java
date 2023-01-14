// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.VerticalElevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveVerticalElevator extends CommandBase {
	private VerticalElevator elevator;
	private double speed;

	/** Creates a new MoveVerticalMotor. */
	public MoveVerticalElevator(VerticalElevator elevator, double speed) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.elevator = elevator;
		this.speed = speed;

		addRequirements(elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		elevator.setSpeed(speed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		elevator.setSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
