package frc.robot;

import frc.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorControllerPort = 1;
		public static final double stickDeadband = 0.02;
	}

	public static class SwerveConstants {
		public static final int pigeonId = 1;
		public static final boolean gyroInverted = false;

		public static final SwerveModuleConstants frontLeft = new SwerveModuleConstants(21, 22, 1, 0.0);
		public static final SwerveModuleConstants frontRight = new SwerveModuleConstants(23, 24, 2, .0);
		public static final SwerveModuleConstants backRight = new SwerveModuleConstants(25, 26, 3, 0.0);
		public static final SwerveModuleConstants backLeft = new SwerveModuleConstants(27, 28, 4, 0.0);

		// the left-to-right distance between the drivetrain wheels, should be measured from center to center
		public static final double trackWidth_m = 1.0;
		// the front-to-back distance between the drivetrain wheels, should be measured from center to center
		public static final double wheelBase_m = 1.0;

		public static final Translation2d[] moduleTranslations = {
			new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
			new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
			new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
			new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0)
		};

		/**
		 * MK4i module config
		 * @see https://github.com/SwerveDriveSpecialties/swerve-lib/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java
		 */
		public static final double wheelDiameter_m = 0.10033;
		public static final double wheelCircumference_m = wheelDiameter_m * Math.PI;
		public static final double driveGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
		public static final boolean driveInverted = true;
		public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
		public static final double angleGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
		public static final boolean angleInverted = false;
		public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
		public static final double maxSpeed_mps = Units.feetToMeters(16.3);
		public static final double maxAngularVelocity_radps = 11.5;
		public static final boolean encoderInverted = false;

		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		/* angle motor PID values */
		public static final double angleKp = 0.3; // todo: calibrate
		public static final double angleKi = 0.0;
		public static final double angleKd = 0.005;
		public static final double angleKf = 0.0;

		/* drive motor PID values */
		public static final double driveKp = 0.05; // todo: calibrate
		public static final double driveKi = 0.0;
		public static final double driveKd = 0.0;
		public static final double driveKf = 0.0;

		/* drive motor characterization values (feed forward) */
		public static final double driveKs = (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
		public static final double driveKv = (2.44 / 12);
		public static final double driveKa = (0.27 / 12);

		/* current limiting */
		public static final SupplyCurrentLimitConfiguration angleCurrentLimit = new SupplyCurrentLimitConfiguration(
			true, 25, 40, 0.1);
		public static final SupplyCurrentLimitConfiguration driveCurrentLimit = new SupplyCurrentLimitConfiguration(
			true, 35, 60, 0.1);
	}
}
