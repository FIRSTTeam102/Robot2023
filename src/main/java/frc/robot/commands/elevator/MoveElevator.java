package frc.robot.commands.elevator;

import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveElevator extends CommandBase {
	private Elevator elevator;
	private double speed;

	public MoveElevator(Elevator elevator, double speed) {
		this.elevator = elevator;
		this.speed = speed;

		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		elevator.setSpeed(speed);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		elevator.setSpeed(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
