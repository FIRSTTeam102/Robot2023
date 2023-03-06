package frc.robot.swerve;

import org.littletonrobotics.junction.AutoLog;

/** swerve module hardware abstraction interface */
public interface SwerveModuleIO extends AutoCloseable {
	@AutoLog
	public static class SwerveModuleIOInputs {
		double drivePosition_rad = 0.0;
		double driveDistance_m = 0.0;
		double driveVelocity_mps = 0.0;
		double driveAppliedPercentage = 0.0;
		double driveCurrent_A = 0.0;
		double driveTemperature_C = 0.0;

		double angleAbsolutePosition_rad = 0.0;
		double anglePosition_rad = 0.0;
		double angleVelocity_radps = 0.0;
		double angleAppliedPercentage = 0.0;
		double angleCurrent_A = 0.0;
		double angleTemperature_C = 0.0;
	}

	/** Updates the set of inputs. */
	public default void updateInputs(SwerveModuleIOInputs inputs) {}

	/** Run the drive motor at the specified percentage of full power. */
	public default void setDriveMotorPercentage(double percentage) {}

	/** Run the drive motor at the specified velocity. */
	public default void setDriveVelocity(double velocity) {}

	/** Run the angle motor at the specified voltage. */
	public default void setAngleVoltage(double voltage) {}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {}

	public default double getCharacterizationVelocity() {
		return 0.0;
	}

	public default void close() throws Exception {}
}
