package frc.robot.swerve;

public record SwerveModuleConstants(
	int driveMotorId,
	int angleMotorId,
	int encoderId,
	double angleOffset_deg) {
	// public double angleOffset_rad() {
	// return Math.toRadians(this.angleOffset_deg);
	// }
}