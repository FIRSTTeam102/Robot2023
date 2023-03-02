package frc.robot.commands.arm;

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
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		// todo: invert input?
		arm.setSpeed(scaleInput(MathUtil.applyDeadband(-inputSupplier.getAsDouble(), .2)));
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
			: Math.copySign(.55 * Math.abs(input) + .4, input);
	}
}
