package frc.robot;

import frc.robot.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
		public static final double stickDeadband = 0.02;
	}

	public static final int pigeonId = 20;

	public static class SwerveConstants {
		public static final SwerveModuleConstants frontLeft = new SwerveModuleConstants(21, 22, 23, 0.0);
		public static final SwerveModuleConstants frontRight = new SwerveModuleConstants(24, 25, 26, .0);
		public static final SwerveModuleConstants backRight = new SwerveModuleConstants(27, 28, 29, 0.0);
		public static final SwerveModuleConstants backLeft = new SwerveModuleConstants(30, 31, 32, 0.0);

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
		public static final double wheelRadius_m = wheelDiameter_m / 2;
		public static final double wheelCircumference_m = wheelDiameter_m * Math.PI;
		public static final double driveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
		public static final boolean driveInverted = true;
		public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

		public static final double angleGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
		public static final boolean angleInverted = true;
		public static final IdleMode angleIdleMode = IdleMode.kCoast;
		// todo: fix with pid or something, test once actual robot is built to see if there's any overshooting problems
		public static final double angleMaxPercentOutput = 0.5;
		public static final boolean encoderInverted = false;
		public static final double angleEncoderPositionFactor_rad = (2 * Math.PI);
		public static final double angleEncoderVelocityFactor_radps = (2 * Math.PI) / 60.0;

		public static final double maxVelocity_mps = 6380.0 /* Falcon max RPM */
			/ 60.0 / driveGearRatio * wheelCircumference_m;
		public static final double maxAngularVelocity_radps = maxVelocity_mps
			/ Math.hypot(trackWidth_m / 2.0, wheelBase_m / 2.0);
		public static final double maxCoastVelocity_mps = 0.05;

		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		/* angle motor PID values */
		public static final double angleKp = 0.01; // todo: calibrate
		public static final double angleKi = 0.0;
		public static final double angleKd = 0.005;
		public static final double angleKf = 0.0;
		/* sim angle motor PID values */
		public static final double simAngleKp = 12.0;
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
		public static final double driveKs = (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
		public static final double driveKv = (2.44 / 12);
		public static final double driveKa = (0.27 / 12);
		/* sim drive motor characterization values */
		public static final double simDriveKs = 0.116970;
		public static final double simDriveKv = 0.133240;
		public static final double simDriveKa = 0.0;

		/* auto path PID values */
		public static final double autoDriveKp = 5.0;
		public static final double autoDriveKi = 0.0;
		public static final double autoDriveKd = 0.0;
		public static final double autoTurnKp = 0.5;
		public static final double autoTurnKi = 0.0;
		public static final double autoTurnKd = 0.0;

		/* current limiting */
		// public static final SupplyCurrentLimitConfiguration angleCurrentLimit = new SupplyCurrentLimitConfiguration(
		// true, 25, 40, 0.1);
		public static final SupplyCurrentLimitConfiguration driveCurrentLimit = new SupplyCurrentLimitConfiguration(
			true, 35, 60, 0.1);
	}

	public static class ElevatorConstants {
		/* ports */
		public static final int motorPort = 21;
		public static final int topSwitchPort = 1;
		public static final int bottomSwitchPort = 2;

		/* speeds */
		public static final double percentSpeed = 0.6;

		public static final double kMinOuput = -1;
		public static final double kMaxOutput = 1;

		/* pid stuff */
		public static final double kP = 0.0;
		public static final double kI = 0.0;
		public static final double kIZone = 0.0;
		public static final double kD = 0.0;

		public static final double feedForward_V = 0.0;

		/**
		 * characterization values (feed forward)
		 * {@link edu.wpi.first.math.controller.ElevatorFeedforward#ElevatorFeedforward(double, double, double, double)}
		 */
		// public static final double kS = 0.2;
		// public static final double kG = 1.0;
		// public static final double kV = 5.0;
		// public static final double kA = 0.2;

		/* sizes */
		public static final double conversionFactor_ft_per_rotation = 1;
		public static final double minHeight_ft = 0.0;
		public static final double maxHeight_ft = 50 / 12;
		public static final double gearRatio = 1.0; // fixme:
		public static final double carriageMass_kg = 1.0; // fixme:
		public static final double drumRadius_m = Units.inchesToMeters(0.5);

		public static final double midHeight_ft = 2;
		public static final double highHeight_ft = 4;
	}
}
