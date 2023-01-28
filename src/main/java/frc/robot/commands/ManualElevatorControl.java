// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualElevatorControl extends CommandBase {
	private Elevator elevator;
	private CommandXboxController operatorController;

	/** Creates a new ManualElevatorControl. */
	public ManualElevatorControl(Elevator elevator, CommandXboxController operatorcController) {
		this.elevator = elevator;
		this.operatorController = operatorcController;

		addRequirements(elevator);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double yAxis = operatorController.getLeftY();

		elevator.setSpeed(yAxis);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		elevator.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
