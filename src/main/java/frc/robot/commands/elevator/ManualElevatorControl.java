package frc.robot.commands.elevator;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ManualElevatorControl extends CommandBase {
	private Elevator elevator;
	private DoubleSupplier inputSupplier;

	public ManualElevatorControl(Elevator elevator, DoubleSupplier inputSupplier) {
		this.elevator = elevator;
		this.inputSupplier = inputSupplier;
		addRequirements(elevator);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		// todo: invert input?
		elevator.setSpeed(MathUtil.applyDeadband(inputSupplier.getAsDouble(), OperatorConstants.stickDeadband) * -.4);
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
