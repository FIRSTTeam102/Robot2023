package frc.robot.commands.elevator;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualElevatorControl extends CommandBase {
	private Elevator elevator;
	private CommandXboxController operatorController;

	public ManualElevatorControl(Elevator elevator, CommandXboxController operatorController) {
		this.elevator = elevator;
		this.operatorController = operatorController;
		addRequirements(elevator);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double yAxis = operatorController.getLeftY();

		yAxis = MathUtil.applyDeadband(yAxis, OperatorConstants.stickDeadband);
		elevator.setSpeed(yAxis);
	}

	@Override
	public void end(boolean interrupted) {
		elevator.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
