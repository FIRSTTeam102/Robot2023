package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
	public static final int motorId = 12;

	public static final double sectionCount = 4;
	public static final double maxNutDist_m = Units.inchesToMeters(13.5);
	public static final double minNutDist_m = Units.inchesToMeters(2.441);
	public static final double armSectionLength_m = Units.inchesToMeters(13.75);
	public static final double conversionFactor_m_per_rotation = Units.inchesToMeters(-0.3333);
	public static final double conversionFactor_mps_per_rpm = conversionFactor_m_per_rotation / 60;

	public static final double moduleDangerZone_m = Units.inchesToMeters(20);

	// todo: tuning
	public static final double kP = 1.0;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double kIZone = 0;
	public static final double kF = 0;

	public static final double minOutput = -1;
	public static final double maxOutput = 1;
}