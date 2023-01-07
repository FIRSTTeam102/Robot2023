package frc.robot;

public class Conversions {
	/**
	 * @param counts Falcon counts
	 * @param gearRatio gear ratio between Falcon and mechanism
	 * @return degrees of rotation of mechanism
	 */
	public static double falconToDegrees(double counts, double gearRatio) {
		return counts * (360.0 / (gearRatio * 2048.0));
	}

	/**
	 * @param degrees degrees of rotation of mechanism
	 * @param gearRatio gear ratio between Falcon and mechanism
	 * @return Falcon counts
	 */
	public static double degreesToFalcon(double degrees, double gearRatio) {
		return degrees / (360.0 / (gearRatio * 2048.0));
	}

	/**
	* @param counts CANCoder counts
	* @param gearRatio gear ratio between CANCoder and mechanism
	* @return degrees of rotation of mechanism
	*/
	public static double cancoderToDegrees(double counts, double gearRatio) {
		return counts * (360.0 / (gearRatio * 4096.0));
	}

	/**
	 * @param velocityCounts Falcon velocity counts
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return RPM of mechanism
	 */
	public static double falconToRpm(double velocityCounts, double gearRatio) {
		return velocityCounts * (600.0 / 2048.0) / gearRatio;
	}

	/**
	 * @param RPM RPM of mechanism
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return Falcon velocity counts
	 */
	public static double rpmToFalcon(double RPM, double gearRatio) {
		return RPM * gearRatio * (2048.0 / 600.0);
	}

	/**
	* @param velocityCounts CANCoder velocity counts
	* @param gearRatio gear ratio between CANCoder and mechanism
	* @return RPM of mechanism
	*/
	public static double cancoderToRpm(double velocityCounts, double gearRatio) {
		return velocityCounts * (600.0 / 4096.0) / gearRatio;
	}

	/**
	 * @param RPM RPM of mechanism
	 * @param gearRatio gear ratio between CANCoder and mechanism
	 * @return CANCoder velocity counts
	 */
	public static double rpmToCancoder(double RPM, double gearRatio) {
		return RPM * gearRatio * (4096.0 / 600.0);
	}

	/**
	 * @param velocityCounts Falcon velocity counts
	 * @param circumference circumference of wheel
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return velocity MPS
	 */
	public static double falconToMps(double velocityCounts, double circumference, double gearRatio) {
		return (falconToRpm(velocityCounts, gearRatio) * circumference) / 60;
	}

	/**
	 * @param velocity velocity MPS
	 * @param circumference circumference of wheel
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return Falcon velocity counts
	 */
	public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
		return rpmToFalcon(((velocity * 60) / circumference), gearRatio);
	}

	/**
	 * @param positionCounts Falcon position counts
	 * @param circumference circumference of wheel
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return position
	 */
	public static double falconToM(double positionCounts, double circumference, double gearRatio) {
		return ((positionCounts / 2048.0) / gearRatio) * circumference;
	}

	/**
	 * @param position position
	 * @param circumference circumference of wheel
	 * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
	 * @return Falcon position counts
	 */
	public static double mToFalcon(double position, double circumference, double gearRatio) {
		return 2048.0 * (position * gearRatio) / circumference;
	}

	/**
	 * @param position position
	 * @param circumference circumference of wheel
	 * @param gearRatio gear ratio between CANCoder and mechanism
	 * @return CANCoder position counts
	 */
	public static double mToCancoder(double position, double circumference, double gearRatio) {
		return 4096.0 * (position * gearRatio) / circumference;
	}
}