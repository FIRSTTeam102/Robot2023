package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record SwerveModuleConstants(
	int driveMotorId,
	int angleMotorId,
	int encoderId,
	double angleOffset_deg) {
	public double angleOffset_rad() {
		return Math.toRadians(this.angleOffset_deg);
	}

	public Rotation2d angleOffset_2d() {
		return Rotation2d.fromDegrees(this.angleOffset_deg);
	}
}