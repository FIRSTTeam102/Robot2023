package frc.robot.constants;

public final class AutoConstants {
	/* pathplanner */
	// todo:?
	public static final double maxVelocity_mps = 4.97;
	public static final double maxAcceleration_mps2 = 3;
	public static final double balanceMaxVelocity_mps = 1.5;
	public static final double slowerMaxVelocity_mps = 3.5;

	public static final double armTolerance_m = 0.1;
	public static final double elevatorTolerance_m = 0.15;

	/* rotates the robot to give the gamepiece limelight a better view */
	public static final double leftGamepieceAngle_rad = Math.toRadians(0);
	public static final double rightGamepieceAngle_rad = Math.toRadians(-5);

	/* auto path PID values */
	public static final double autoDriveKp = 12.0;
	public static final double autoDriveKi = 0; // 0.02?
	public static final double autoDriveKd = 0.2;
	public static final double autoAngleKp = 6.3;
	public static final double autoAngleKi = 0;
	public static final double autoAngleKd = 0.5;
}
