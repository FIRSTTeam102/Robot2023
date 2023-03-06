package frc.robot.commands.swerve;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
	public boolean fieldRelative = true;
	public boolean overrideSpeed = false;

	public CommandBase toggleFieldRelative() {
		return Commands.runOnce(() -> fieldRelative = !fieldRelative);
	};

	public CommandBase holdToggleFieldRelative() {
		return Commands.startEnd(
			() -> fieldRelative = !fieldRelative,
			() -> fieldRelative = !fieldRelative);
	};

	public CommandBase zeroYaw() {
		return Commands.runOnce(swerve::zeroYaw);
	}

	private Swerve swerve;
	private Arm arm;
	private Elevator elevator;
	private DoubleSupplier driveSupplier;
	private DoubleSupplier strafeSupplier;
	private DoubleSupplier turnSupplier;
	private BooleanSupplier overrideSpeedSupplier;
	private BooleanSupplier preciseModeSupplier;

	/**
	 * @param overrideSpeedSupplier forces swerve to run at normal speed when held, instead of slow if scoring mechanism is out
	 */
	public TeleopSwerve(DoubleSupplier driveSupplier, DoubleSupplier strafeSupplier, DoubleSupplier turnSupplier,
		BooleanSupplier overrideSpeedSupplier, BooleanSupplier preciseModeSupplier, Swerve swerve, Arm arm,
		Elevator elevator) {
		addRequirements(swerve);
		this.driveSupplier = driveSupplier;
		this.strafeSupplier = strafeSupplier;
		this.turnSupplier = turnSupplier;
		this.overrideSpeedSupplier = overrideSpeedSupplier;
		this.preciseModeSupplier = preciseModeSupplier;
		this.swerve = swerve;
		this.arm = arm;
		this.elevator = elevator;
	}

	private double rotation;
	private Translation2d translation;

	private double maxPercent = 1.0;
	private static double maxArmDist_m = Arm.nutDistToArmDist(ArmConstants.maxNutDist_m) - ArmConstants.dangerZone_m;

	@Override
	public void execute() {
		maxPercent = overrideSpeedSupplier.getAsBoolean() ? 1.0
			: 1 - 0.5 * (
			// bigger coefficient = more of a speed decrease the farther out it is
			1 * (arm.getArmDist_m() / maxArmDist_m)
				+ 0.5 * ((elevator.inputs.position_m - ArmConstants.dangerZone_m) / ElevatorConstants.maxHeight_m));

		if (preciseModeSupplier.getAsBoolean())
			maxPercent *= 0.3;

		translation = new Translation2d(
			modifyAxis(driveSupplier.getAsDouble()),
			modifyAxis(strafeSupplier.getAsDouble()))
				.times(SwerveConstants.maxVelocity_mps * maxPercent);

		rotation = modifyAxis(turnSupplier.getAsDouble())
			* SwerveConstants.maxAngularVelocity_radps
			* maxPercent;
		// * (overrideSpeedSupplier.getAsBoolean() ? 0.7 : 0.5);

		swerve.drive(translation, rotation, fieldRelative);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
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
		return (absValue < OperatorConstants.stickDeadband) ? 0
			: Math.copySign(
				(cubicWeight * Math.pow(absValue, weightExponent) + (1 - cubicWeight) * absValue + minOutput) / (1 + minOutput),
				value);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("field oriented", () -> fieldRelative, null);
	}
}
