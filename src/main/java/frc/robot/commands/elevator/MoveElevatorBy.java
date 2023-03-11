package frc.robot.commands.elevator;

import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveElevatorBy extends CommandBase {
	private Elevator elevator;
	private double deltaPos_m;
	private double targetPos_m;

	public MoveElevatorBy(Elevator elevator, double deltaDist_m) {
		this.elevator = elevator;
		this.deltaPos_m = deltaDist_m;
	}

	@Override
	public void initialize() {
		targetPos_m = elevator.inputs.position_m + deltaPos_m;
		elevator.inManualMode = false;
		elevator.setPosition(targetPos_m);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(targetPos_m - elevator.inputs.position_m) <= AutoConstants.elevatorTolerance_m;
	}

	@Override
	public void end(boolean interrupted) {
		// elevator.stop();
	}
}
