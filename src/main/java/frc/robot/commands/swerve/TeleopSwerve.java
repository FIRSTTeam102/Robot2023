package frc.robot.commands.swerve;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
	private double rotation;
	private Translation2d translation;
	private boolean openLoop = true;

	public boolean fieldRelative = true;

	public InstantCommand toggleFieldRelative() {
		return new InstantCommand(() -> {
			fieldRelative = !fieldRelative;
		});
	}

	public InstantCommand zeroYaw() {
		return new InstantCommand(swerve::zeroYaw);
	}

	private Swerve swerve;
	private DoubleSupplier driveSupplier;
	private DoubleSupplier strafeSupplier;
	private DoubleSupplier turnSupplier;

	public TeleopSwerve(Swerve swerve,
		DoubleSupplier driveSupplier,
		DoubleSupplier strafeSupplier,
		DoubleSupplier turnSupplier) {
		this.swerve = swerve;
		addRequirements(swerve);
		this.driveSupplier = driveSupplier;
		this.strafeSupplier = strafeSupplier;
		this.turnSupplier = turnSupplier;
	}

	@Override
	public void execute() {
		translation = new Translation2d(
			modifyAxis(-driveSupplier.getAsDouble()),
			modifyAxis(-strafeSupplier.getAsDouble()))
				.times(SwerveConstants.maxVelocity_mps * OperatorConstants.drivePercent);
		rotation = modifyAxis(-turnSupplier.getAsDouble())
			* SwerveConstants.maxAngularVelocity_radps * OperatorConstants.turnPercent;
		swerve.drive(translation, rotation, fieldRelative);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		super.end(interrupted);
	}

	private static final double cubicWeight = 0.4;
	private static final double weightExponent = 3.5;
	private static final double minOutput = 0.1;

	// custom input scaling
	// @see https://desmos.com/calculator/7wy4gmgdpv
	private static double modifyAxis(double value) {
		// value = MathUtil.applyDeadband(value, OperatorConstants.stickDeadband);
		// return Math.copySign(value * value, value);

		double absValue = Math.abs(value);
		if (absValue < OperatorConstants.stickDeadband)
			return 0;

		return Math.copySign(
			(cubicWeight * Math.pow(absValue, weightExponent) + (1 - cubicWeight) * absValue + minOutput) / (1 + minOutput),
			value);
	}
}
