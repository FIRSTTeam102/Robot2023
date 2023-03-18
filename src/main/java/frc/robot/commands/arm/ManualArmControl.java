package frc.robot.commands.arm;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class ManualArmControl extends CommandBase {
	private Arm arm;
	private DoubleSupplier inputSupplier;

	public ManualArmControl(Arm arm, DoubleSupplier inputSupplier) {
		this.arm = arm;
		this.inputSupplier = inputSupplier;
		addRequirements(arm);
		arm.setManualModeInput(inputSupplier);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (arm.inManualMode)
			arm.setSpeed(
				scaleInput(MathUtil.applyDeadband(inputSupplier.getAsDouble(), OperatorConstants.operatorJoystickDeadband)));
	}

	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	public static double scaleInput(double input) {
		return input == 0
			? 0
			: Math.copySign(.6 * Math.abs(input) + .1, input);
	}
}
