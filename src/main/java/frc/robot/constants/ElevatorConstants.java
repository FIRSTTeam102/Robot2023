package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
	public static final double dangerZone_m = .11;

	public static final double coneMoveDownHeight_m = -0.15;

	/* ports */
	public static final int motorId = 11;

	/* speeds */
	public static final double percentSpeed = 0.6;

	public static final double maxOutput = .7;
	public static final double minOuput = -maxOutput;

	/* pid stuff */
	public static final double kP = 1;
	public static final double kI = 0.00065;
	public static final double kIZone = 0.1;
	public static final double kD = 0.0;

	public static final double feedForward_V = 0.63;

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
		* 0.2282; /* m/rotation */
	public static final double conversionFactor_mps_per_rpm = conversionFactor_m_per_rotation / 60;
	public static final double minHeight_m = 0.02;
	public static final double maxHeight_m = 1.24;
	public static final double carriageMass_kg = Units
		.lbsToKilograms(4.893 /* inner elevator */ + 9 /* carriage + arm */ + 6 /* grabber */);
	public static final double drumRadius_m = Units.inchesToMeters(0.5);
}