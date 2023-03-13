package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
	public static final double minNutDist_m = 0.34925;
	public static final double maxNutDist_m = 0.07;

	public static final int motorId = 12;

	public static final double sectionCount = 4;
	public static final double armSectionLength_m = Units.inchesToMeters(13.75);
	public static final double conversionFactor_m_per_rotation = 0.005 /* rotation of shaft */
		/ 3 /* gear ratio */;
	public static final double conversionFactor_mps_per_rpm = conversionFactor_m_per_rotation / 60;

	/** minimum distance to not hit the swerve modules when elevator is down */
	public static final double dangerZone_m = .32;
	/** clipping distance for mid level on grid */
	public static final double gridSafeZone_m = .7;

	// todo: tuning
	public static final double kP = 17.0;
	public static final double kI = 0.01;
	public static final double kIZone = 0.05;
	public static final double kD = 0.08;
	public static final double kF = 0;

	public static final double maxOutput = .85;
	public static final double minOutput = -.65;
}