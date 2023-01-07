package frc.robot.subsystems;

import frc.robot.Constants.SwerveConstants;
import frc.robot.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.Pigeon2;

/*
 * @see https://github.com/Team364/BaseFalconSwerve
 */
public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry odometry;
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.moduleTranslations);
	public SwerveModule[] modules = {
		new SwerveModule(0, SwerveConstants.frontLeft),
		new SwerveModule(1, SwerveConstants.frontRight),
		new SwerveModule(2, SwerveConstants.backRight),
		new SwerveModule(3, SwerveConstants.backLeft)
	};
	private SwerveModulePosition[] modulePositions = {
		new SwerveModulePosition(),
		new SwerveModulePosition(),
		new SwerveModulePosition(),
		new SwerveModulePosition()
	};
	// private SwerveModuleState[] moduleStates;
	private Pose2d[] modulePoses = {
		new Pose2d(),
		new Pose2d(),
		new Pose2d(),
		new Pose2d()
	};
	Field2d fieldSim = new Field2d();

	public Pigeon2 gyro;

	/** Creates a new Swerve. */
	public Swerve() {
		gyro = new Pigeon2(SwerveConstants.pigeonId);
		gyro.configFactoryDefault();
		zeroGyro();

		odometry = new SwerveDriveOdometry(kinematics, getYaw(), modulePositions);

		SmartDashboard.putData("Field", fieldSim);
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				translation.getX(),
				translation.getY(),
				rotation,
				getYaw())
				: new ChassisSpeeds(
					translation.getX(),
					translation.getY(),
					rotation));
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed_mps);
		// System.out.format("x%f y%f r%f\n", translation.getX(), translation.getY(), rotation);
		// System.out.format("a%f s%f\n", swerveModuleStates[0].angle.getDegrees(),
		// swerveModuleStates[0].speedMetersPerSecond);

		for (SwerveModule mod : modules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed_mps);

		for (SwerveModule mod : modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(getYaw(), modulePositions, pose);
	}

	public SwerveModuleState[] getStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	public Rotation2d getYaw() {
		return (SwerveConstants.gyroInverted)
			? Rotation2d.fromDegrees(360 - gyro.getYaw())
			: Rotation2d.fromDegrees(gyro.getYaw());
	}

	@Override
	public void periodic() {
		for (SwerveModule mod : modules) {
			modulePositions[mod.moduleNumber] = mod.getPosition();

			// update shuffleboard
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " encoder", mod.getEncoderPos().getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " integrated", mod.getState().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " velocity", mod.getState().speedMetersPerSecond);
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " position", mod.getPosition().distanceMeters);
			// SmartDashboard.putData("Mod " + mod.moduleNumber, mod.getState().angle);

			// update field pose
			var modulePositionFromChassis = SwerveConstants.moduleTranslations[mod.moduleNumber]
				.rotateBy(getYaw())
				.plus(getPose().getTranslation());
			modulePoses[mod.moduleNumber] = new Pose2d(modulePositionFromChassis,
				modules[mod.moduleNumber].getState().angle.plus(getPose().getRotation()));
		}

		// update odometry
		odometry.update(getYaw(), modulePositions);

		fieldSim.setRobotPose(getPose());
		fieldSim.getObject("Swerve Modules").setPoses(modulePoses);
	}

	@Override
	public void simulationPeriodic() {
		for (SwerveModule mod : modules) {
			mod.simulationPeriodic(0.02);
		}
	}
}
