package frc.robot.commands;

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
