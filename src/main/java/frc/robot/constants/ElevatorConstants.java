package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
	/* target heights */
	// fixme: actual values
	public static final double lowHeight_m = 0;
	public static final double midHeight_m = 0.5;
	public static final double highHeight_m = 1;

	/* ports */
	public static final int motorId = 10;
	public static final int topSwitchPort = 1;
	public static final int bottomSwitchPort = 2;

	/* speeds */
	public static final double percentSpeed = 0.6;

	public static final double minOuput = -1;
	public static final double mMaxOutput = 1;

	/* pid stuff */
	public static final double kP = 0.0;
	public static final double kI = 0.0;
	public static final double kIZone = 0.0;
	public static final double kD = 0.0;

	public static final double feedForward_V = 0.0;

	// /**
	// * characterization values (feed forward)
	// * {@link edu.wpi.first.math.controller.ElevatorFeedforward#ElevatorFeedforward(double, double, double, double)}
	// */
	// public static final double kS = 0.2;
	// public static final double kG = 1.0;
	// public static final double kV = 5.0;
	// public static final double kA = 0.2;

	/* sizes */
	public static final double gearRatio = 12; // motor rotations per main shaft rotation
	public static final double conversionFactor_m_per_rotation = 1 /* rotation of motor */
		* (1 / gearRatio) /* rotation/rotation of main shaft */
		* 0.089; /* m/rotation */
	public static final double minHeight_m = Units.inchesToMeters(0);
	public static final double maxHeight_m = Units.inchesToMeters(50);
	public static final double carriageMass_kg = Units
		.lbsToKilograms(4.893 /* inner elevator */ + 8.166 /* carriage + arm */ + 6 /* grabber */);
	public static final double drumRadius_m = Units.inchesToMeters(0.5);
}