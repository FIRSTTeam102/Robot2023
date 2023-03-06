// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleLightControlMode extends CommandBase {
	private ControlMode controlMode;

	public ToggleLightControlMode(ControlMode controlMode) {
		this.controlMode = controlMode;
	}

	@Override
	public void initialize() {
		Lights.setControlMode(controlMode);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		Lights.setControlMode(ControlMode.Regular);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
