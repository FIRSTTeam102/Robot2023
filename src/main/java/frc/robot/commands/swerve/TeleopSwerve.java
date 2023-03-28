package frc.robot.commands.swerve;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

	public CommandBase holdRotateAroundPiece() {
		return Commands.startEnd(
			() -> swerve.setCenterRotation(SwerveConstants.trackWidth_m + 0.1, 0),
			() -> swerve.setCenterRotation(0, 0));
	}

	private Alert zeroedYaw = new Alert("never zeroed yaw", AlertType.Warning);

	public class ZeroYaw extends InstantCommand {
		public ZeroYaw() {
			zeroedYaw.set(true);
		}

		@Override
		public void initialize() {
			swerve.zeroYaw();
			zeroedYaw.set(false);
		}

		@Override
		public boolean runsWhenDisabled() {
			return true;
		}
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
		BooleanSupplier overrideSpeedSupplier, BooleanSupplier preciseModeSupplier,
		Swerve swerve, Arm arm, Elevator elevator) {
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
	private double driveMax = 1.0;
	private double turnMaxPercent = 1.0;

	// todo: tune, see if stopping takes too long and maybe add an override
	private static final double accelerationLimit_mps2 = 9;
	private SlewRateLimiter driveLimiter = new SlewRateLimiter(accelerationLimit_mps2);
	private SlewRateLimiter strafeLimiter = new SlewRateLimiter(accelerationLimit_mps2);

	private static final double normalMaxPercent = 0.75;
	private static final double maxArmDist_m = Arm.nutDistToArmDist(ArmConstants.maxNutDist_m)
		- ArmConstants.dangerZone_m;

	@Override
	public void execute() {
		if (overrideSpeedSupplier.getAsBoolean()) {
			driveMax = 1.0;
			turnMaxPercent = 0.9;
		} else {
			driveMax = normalMaxPercent * (1 - (0.4 /* how much of the decrease to use */ * (
			// bigger coefficient = more of a speed decrease the farther out it is
			0.9 * (arm.getArmDist_m() / maxArmDist_m)
				+ 0.5 * ((elevator.inputs.position_m - ArmConstants.dangerZone_m) / ElevatorConstants.maxHeight_m))));
			if (driveMax < 0.1) // bug?
				driveMax = 0.1;
			turnMaxPercent = driveMax * 0.9;
		}

		if (preciseModeSupplier.getAsBoolean()) {
			driveMax *= 0.3;
			turnMaxPercent *= 0.2;
		}

		driveMax *= SwerveConstants.maxVelocity_mps; // turn percent into velocity

		translation = new Translation2d(
			driveLimiter.calculate(driveMax * modifyAxis(driveSupplier.getAsDouble())),
			strafeLimiter.calculate(driveMax * modifyAxis(strafeSupplier.getAsDouble())));

		// Logger.getInstance().recordOutput("TeleopSwerve/translationX_mps", translation.getX());
		// Logger.getInstance().recordOutput("TeleopSwerve/translationY_mps", translation.getY());

		rotation = modifyAxis(turnSupplier.getAsDouble())
			* SwerveConstants.maxAngularVelocity_radps
			* turnMaxPercent;

		swerve.drive(translation, rotation, fieldRelative);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	private static final double cubicWeight = 0.3;
	private static final double weightExponent = 6.5;
	private static final double minOutput = 0.1;

	// custom input scaling
	// @see https://desmos.com/calculator/7wy4gmgdpv
	private static double modifyAxis(double value) {
		// value = MathUtil.applyDeadband(value, OperatorConstants.xboxStickDeadband);
		// return Math.copySign(value * value, value);

		double absValue = Math.abs(value);
		return (absValue < OperatorConstants.xboxStickDeadband) ? 0
			: Math.copySign(
				(cubicWeight * Math.pow(absValue, weightExponent) + (1 - cubicWeight) * absValue + minOutput) / (1 + minOutput),
				value);
	}
}
