// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualArmControl extends CommandBase {
	private Arm arm;
	private CommandXboxController operatorController;

	/** Creates a new ManualArmControl. */
	public ManualArmControl(Arm arm, CommandXboxController operatorController) {
		this.arm = arm;
		this.operatorController = operatorController;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double xAxis = operatorController.getRightX();

		arm.setSpeed(xAxis);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
