package frc.robot.commands.swerve;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

// angles are between +/- pi

public class ChargeStationBalance extends CommandBase {
	private final double maxSpeed_mps = 0.15 * SwerveConstants.maxVelocity_mps;

	/** stops driving when within @fieldcal */
	private final double maxAngle_rad = Units.degreesToRadians(3);

	private final PIDController driveController = new PIDController(2.1, 0, 0.025);

	private Swerve swerve;

	public ChargeStationBalance(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);

		SmartDashboard.putData("balance drive pid", driveController);
		driveController.setTolerance(maxAngle_rad);
		driveController.setSetpoint(0);
		driveController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		var tilt_rad = swerve.getTilt_rad();
		Logger.getInstance().recordOutput("Balance/tilt_rad", tilt_rad);

		var outputVelocity_mps = MathUtil.clamp(driveController.calculate(tilt_rad), -maxSpeed_mps, maxSpeed_mps);
		Logger.getInstance().recordOutput("Balance/outputVelocity_mps", outputVelocity_mps);

		swerve.drive(new Translation2d(outputVelocity_mps, 0), 0, true);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return driveController.atSetpoint();
	}

	// // todo:, also don't pass borders as parameters
	// public boolean geofence(double blueTopY, double blueBottomY, double blueLeftX, double blueRightX,
	// double redTopY, double redBottomY, double redLeftX, double redRightX) {
	// return (swerve.translationY > blueTopY - SwerveConstants.robotfence
	// && swerve.translationY < blueBottomY - SwerveConstants.robotfence
	// && swerve.translationX > blueLeftX - SwerveConstants.robotfence
	// && swerve.translationX < blueRightX - SwerveConstants.robotfence)
	// || (swerve.translationY > redTopY - SwerveConstants.robotfence
	// && swerve.translationY < redBottomY - SwerveConstants.robotfence
	// && swerve.translationX > redLeftX - SwerveConstants.robotfence
	// && swerve.translationX < redRightX - SwerveConstants.robotfence);
	// }
}
