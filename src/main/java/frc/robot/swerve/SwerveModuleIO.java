package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

/**
 * swerve module hardware abstraction interface
 */
public interface SwerveModuleIO extends AutoCloseable {
	@AutoLog
	public static class SwerveModuleIOInputs {
		double drivePosition_deg = 0.0;
		double driveDistance_m = 0.0;
		double driveVelocity_mps = 0.0;
		double driveAppliedPercentage = 0.0;
		double[] driveCurrentAmps = new double[] {};
		// double[] driveTempCelsius = new double[] {};

		double angleAbsolutePosition_rad = 0.0;
		double anglePosition_rad = 0.0;
		double angleVelocity_rpm = 0.0;
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
	public default void setAnglePosition(Rotation2d angls) {}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {}

	/** Enable or disable brake mode on the turn motor. */
	public default void setAngleBrakeMode(boolean enable) {}

	public default void close() throws Exception {}
}
