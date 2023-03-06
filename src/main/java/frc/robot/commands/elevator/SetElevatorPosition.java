package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetElevatorPosition extends InstantCommand {
	private Elevator elevator;
	private double height_m;

	public SetElevatorPosition(Elevator elevator, double height_m) {
		this.elevator = elevator;
		this.height_m = height_m;
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		elevator.inManualMode = false;
		elevator.setPosition(height_m);
	}
}
