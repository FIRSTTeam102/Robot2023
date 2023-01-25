// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetArmPosition extends InstantCommand {
	private Arm arm;
	private double position;

	/** 
	 * Creates a new SetArmPosition. 
	 * 
	 * @param arm Arm subsystem to utilize
	 * @param position position in percentage of max extension on the range [0, 1] (0 is fully in, 1 is max extension)
	 */
	public SetArmPosition(Arm arm, double position) {
		this.arm = arm;
		this.position = position;

		addRequirements(arm);
	}

	/**
	 * Creates a new SetArmPosition with position set to 0 (fully in). Essentially a reset command
	 * 
	 * @param arm Arm subsystem to utilize
	 */
	public SetArmPosition(Arm arm) {
		this.arm = arm;
		this.position = 0;

		addRequirements(arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		arm.setPos(position);
	}
}
