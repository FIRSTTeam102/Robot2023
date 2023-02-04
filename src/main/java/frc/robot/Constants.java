package frc.robot;

import frc.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.revrobotics.CANSparkMax.IdleMode;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double loopPeriod_s = LoggedRobot.defaultPeriodSecs; // edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod

	public static final RobotMode robotMode = RobotMode.ACTIVE;

	public enum RobotMode {
		ACTIVE, REPLAY;
	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorControllerPort = 1;
		public static final double stickDeadband = 0.1;
	}

	public static final int pigeonId = 20;

	public static class SwerveConstants {
		// FL, FR, BR, BL
		public static final SwerveModuleConstants moduleConstants[] = {
			new SwerveModuleConstants(21, 22, 23, 3.6954),
			new SwerveModuleConstants(24, 25, 26, 2.5786),
			new SwerveModuleConstants(27, 28, 29, 3.0173),
			new SwerveModuleConstants(30, 31, 32, 2.2473)
		};

		// the left-to-right distance between the drivetrain wheels, should be measured from center to center
		public static final double trackWidth_m = 0.502;
		// the front-to-back distance between the drivetrain wheels, should be measured from center to center
		public static final double wheelBase_m = 0.502;

		// indexes must match moduleConstants
		public static final Translation2d[] moduleTranslations = {
			new Translation2d(wheelBase_m / 2.0, trackWidth_m / 2.0),
			new Translation2d(wheelBase_m / 2.0, -trackWidth_m / 2.0),
			new Translation2d(-wheelBase_m / 2.0, -trackWidth_m / 2.0),
			new Translation2d(-wheelBase_m / 2.0, trackWidth_m / 2.0),
		};

		/**
		 * MK4i module config
		 * @see https://github.com/SwerveDriveSpecialties/swerve-lib/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/SdsModuleConfigurations.java
		 */
		public static final double wheelDiameter_m = 0.10033;
		public static final double wheelRadius_m = wheelDiameter_m / 2;
		public static final double wheelCircumference_m = wheelDiameter_m * Math.PI;
		public static final double driveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
		public static final boolean driveInverted = true;
		public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

		public static final double angleGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
		public static final boolean angleInverted = true;
		public static final IdleMode angleIdleMode = IdleMode.kCoast;
		public static final double angleMaxPercentOutput = 0.5;
		public static final boolean encoderInverted = false;
		public static final double angleEncoderPositionFactor_rad = (2 * Math.PI);
		public static final double angleEncoderVelocityFactor_radps = (2 * Math.PI) / 60.0;

		public static final double maxVelocity_mps = 6380.0 /* Falcon max RPM */
			/ 60.0 / driveGearRatio * wheelCircumference_m;
		public static final double maxAngularVelocity_radps = 5676 /* NEO max RPM */
			/ 60.0 / angleGearRatio
			/ Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
		public static final double maxCoastVelocity_mps = 0.05;

		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		/* angle motor PID values */
		public static final double angleKp = 4.0; // todo: calibrate
		public static final double angleKi = 0.0;
		public static final double angleKd = 0.0;
		public static final double angleKf = 0.0;
		/* sim angle motor PID values */
		public static final double simAngleKp = 12.0; // todo: make sim use these, they need to be really ramped up
		public static final double simAngleKi = 0.0;
		public static final double simAngleKd = 0.0;
		public static final double simAngleKf = 0.0;

		/* drive motor PID values */
		public static final double driveKp = 0.05; // todo: calibrate
		public static final double driveKi = 0.0;
		public static final double driveKd = 0.0;
		public static final double driveKf = 0.0;
		/* sim drive motor PID values */
		public static final double simDriveKp = 0.8;
		public static final double simDriveKi = 0.0;
		public static final double simDriveKd = 0.0;
		public static final double simDriveKf = 0.0;

		/* drive motor characterization values (feed forward) */
		public static final double driveKs = 0.0029032;
		public static final double driveKv = 0.016325;
		public static final double driveKa = 0.00035285;
		/* sim drive motor characterization values */
		public static final double simDriveKs = 0.116970; // todo: do we need these?
		public static final double simDriveKv = 0.133240;
		public static final double simDriveKa = 0.0;

		// V = kS + kV * v + kA * a
		public static final double turnKs = 0.1;
		public static final double turnKv = 0.005;
		// public static final double turnKa = 0.000115;

		/* auto path PID values */
		public static final double autoDriveKp = 5.0;
		public static final double autoDriveKi = 0.0;
		public static final double autoDriveKd = 0.0;
		public static final double autoTurnKp = 0.5;
		public static final double autoTurnKi = 0.0;
		public static final double autoTurnKd = 0.0;

		/* current limiting */
		public static final int angleCurrentLimit_amp = 25;
		public static final SupplyCurrentLimitConfiguration driveCurrentLimit = new SupplyCurrentLimitConfiguration(
			true, 35, 60, 0.1);
	}
}
