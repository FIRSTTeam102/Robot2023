package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ArmConstants {
	public static final double minNutDist_m = Units.inchesToMeters(13.5);
	public static final double maxNutDist_m = Units.inchesToMeters(4.92);

	// just for testing please change please please
	public static final double lowExtension_m = Units.inchesToMeters(12);
	public static final double midExtension_m = Units.inchesToMeters(25);
	public static final double highExtension_m = Units.inchesToMeters(41);
	public static final double doubleSubstationExtension_m = 1; // todo:

	public static final int motorId = 12;

	public static final double sectionCount = 4;
	public static final double armSectionLength_m = Units.inchesToMeters(13.75);
	public static final double conversionFactor_m_per_rotation = Units.inchesToMeters(0.1) /* rotation of shaft */
		/ 3 /* gear ratio */;
	public static final double conversionFactor_mps_per_rpm = conversionFactor_m_per_rotation / 60;

	public static final double dangerZone_m = .39;

	// todo: tuning
	public static final double kP = 11.0;
	public static final double kI = 0.005;
	public static final double kIZone = 0.1;
	public static final double kD = 0.1;
	public static final double kF = 0;

	public static final double minOutput = -.7;
	public static final double maxOutput = .7;
}