package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

/**
 * swerve module hardware abstraction interface
 */
public interface SwerveModuleIO {
	@AutoLog
	public static class SwerveModuleIOInputs {
		double drivePositionDeg = 0.0;
		double driveDistanceMeters = 0.0;
		double driveVelocityMetersPerSec = 0.0;
		double driveAppliedPercentage = 0.0;
		double[] driveCurrentAmps = new double[] {};
		// double[] driveTempCelsius = new double[] {};

		double angleAbsolutePositionDeg = 0.0;
		double anglePositionDeg = 0.0;
		double angleVelocityRevPerMin = 0.0;
		double angleAppliedPercentage = 0.0;
		double[] angleCurrentAmps = new double[] {};
		// double[] angleTempCelsius = new double[] {};
	}

	/** Updates the set of inputs. */
	public default void updateInputs(SwerveModuleIOInputs inputs) {}

	/** Run the drive motor at the specified percentage of full power. */
	public default void setDriveMotorPercentage(double percentage) {}

	/** Run the drive motor at the specified velocity. */
	public default void setDriveVelocity(double velocity) {}

	/** Run the turn motor to the specified angle. */
	public default void setAnglePosition(double degrees) {}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {}

	/** Enable or disable brake mode on the turn motor. */
	public default void setAngleBrakeMode(boolean enable) {}

	public default SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
		return desiredState;
	}
}
