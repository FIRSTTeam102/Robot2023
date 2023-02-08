// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetElevatorPosition extends InstantCommand {
	private Elevator elevator;
	private double height_m;

	/** Creates a new SetElevatorPosition. */
	public SetElevatorPosition(Elevator elevator, double height_m) {
		this.elevator = elevator;
		this.height_m = height_m;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		elevator.setPosition(height_m);
	}
}
