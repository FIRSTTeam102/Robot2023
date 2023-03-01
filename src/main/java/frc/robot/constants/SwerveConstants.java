package frc.robot.constants;

import frc.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.revrobotics.CANSparkMax.IdleMode;

public final class SwerveConstants {
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
	public static final double driveOpenLoopRamp = 0.25;
	public static final double driveClosedLoopRamp = 0.0;

	public static final double angleGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
	public static final boolean angleInverted = true;
	public static final IdleMode angleIdleMode = IdleMode.kCoast;
	// public static final double angleMaxPercentOutput = 0.5;
	public static final double angleRampTime_s = 0.5;
	public static final boolean angleEncoderInverted = false;
	// public static final double angleSparkEncoderPositionFactor_rad = (2 * Math.PI);
	// public static final double angleSparkEncoderVelocityFactor_radps = (2 * Math.PI) / 60.0;

	public static final double maxVelocity_mps = 6380 /* Falcon max RPM */
		/ 60.0 / driveGearRatio * wheelCircumference_m;
	public static final double maxAngularVelocity_radps = 5676 /* NEO max RPM */
		/ 60.0 / angleGearRatio / Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
	public static final double maxCoastVelocity_mps = 0.05;

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

	/* angle motor PID values */
	public static final double angleKp = 5.7;
	public static final double angleKi = 0.0;
	public static final double angleKd = 0.05;
	public static final double angleKf = 0.0;
	/* sim angle motor PID values */
	public static final double simAngleKp = 18.0;
	public static final double simAngleKi = 0.0;
	public static final double simAngleKd = 0.0;
	public static final double simAngleKf = 0.0;

	/* drive motor characterization (feed forward) */
	public static final double driveKs = 0.0029032;
	public static final double driveKv = 0.016325;
	public static final double driveKa = 0.00035285;
	/* sim drive motor characterization */
	public static final double simDriveKs = 0.117;
	public static final double simDriveKv = 0.133;
	public static final double simDriveKa = 0.0;

	/* angle motor characterization */
	// public static final double angleKs = 0;
	// public static final double angleKv = 0;
	// public static final double angleKa = 0;

	/* current limiting */
	public static final SupplyCurrentLimitConfiguration driveCurrentLimit = new SupplyCurrentLimitConfiguration(
		true, 35, 60, 0.1);
	public static final int angleCurrentLimit_amp = 25;
};
