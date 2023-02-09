package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Robot;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOInputsAutoLogged;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleIOReal;
import frc.robot.swerve.SwerveModuleIOSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import org.littletonrobotics.junction.Logger;

import lombok.Setter;

public class Swerve extends SubsystemBase implements AutoCloseable {
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

	public SwerveModule[] modules;
	public SwerveModulePosition[] modulePositions = {new SwerveModulePosition(), new SwerveModulePosition(),
		new SwerveModulePosition(), new SwerveModulePosition()};
	public SwerveModuleState[] moduleStates = {new SwerveModuleState(), new SwerveModuleState(),
		new SwerveModuleState(), new SwerveModuleState()};

	public SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		kinematics, new Rotation2d(), modulePositions, new Pose2d());

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public Pose2d[] modulePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
	public Field2d fieldSim = new Field2d();

	private Translation2d centerRotation = new Translation2d(0, 0); // position the robot rotates around

	// used for pose estimation
	private Timer timer = new Timer();
	public double disabledTimeStart = 0.0;

	// configurable stuff
	private boolean brakeMode = true;
	@Setter
	private boolean fieldRelative = false;

	public GyroIO gyroIO;
	public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private double gyroOffset_deg = 0.0;

	public final PIDController autoXController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public final PIDController autoYController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public final PIDController autoRotationController = new PIDController(autoTurnKp, autoTurnKi, autoTurnKd);

	public Swerve(GyroIO gyroIO) {
		modules = new SwerveModule[moduleConstants.length];
		int moduleNumber = 0;
		for (var mod : moduleConstants) {
			modules[moduleNumber] = new SwerveModule(moduleNumber, Robot.isReal()
				? new SwerveModuleIOReal(mod, moduleNumber)
				: new SwerveModuleIOSim());
			moduleNumber++;
		}

		this.gyroIO = gyroIO;
		zeroYaw();

		timer.reset();
		timer.start();

		SmartDashboard.putData("Field", fieldSim);
	}

	/** update and return states */
	public SwerveModuleState[] getStates() {
		for (SwerveModule mod : modules) {
			moduleStates[mod.moduleNumber] = mod.getState();
		}
		return moduleStates;
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxVelocity_mps);

		for (SwerveModule mod : modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], false, false);
		}
	}

	public void setChasisSpeeds(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerRotation);
		setModuleStates(states);
	}

	/** @return the estimated pose of the robot on the field */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public SwerveModulePosition[] getPositions() {
		for (SwerveModule mod : modules) {
			modulePositions[mod.moduleNumber] = mod.getPosition();
		}
		return modulePositions;
	}

	/**
	 * sets the odometry of the robot to the specified PathPlanner state
	 * 
	 * this should only be done when the rotation of the robot is known
	 * (like at the start of an autonomous path)
	 */
	public void resetOdometry(PathPlannerState state) {
		setGyroOffset_deg(state.holonomicRotation.getDegrees());

		// estimatedPoseWithoutGyro = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
		poseEstimator.resetPosition(
			getYaw(), getPositions(),
			new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
	}

	public void addVisionMeasurement(Pose2d pose) {
		poseEstimator.addVisionMeasurement(pose, timer.get());
	}

	/** @return gyro yaw including code offset */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(gyroInputs.yaw_deg + gyroOffset_deg);
	}

	/** sets the rotation of the robot to the specified value by changing code gyro offset */
	public void setGyroOffset_deg(double expectedYaw_deg) {
		gyroOffset_deg = expectedYaw_deg - gyroInputs.yaw_deg;
	}

	public void zeroYaw() {
		gyroIO.setYaw(0);
	}

	/** @return rotation around side-to-side axis (leaning forward/backward) */
	public double getPitch_rad() {
		return MathUtil.angleModulus(Units.degreesToRadians(gyroInputs.pitch_deg));
	}

	/** @return rotation around front-to-back axis (leaning left/right) */
	public double getRoll_rad() {
		return MathUtil.angleModulus(Units.degreesToRadians(gyroInputs.roll_deg));
	}

	/** turn off brake mode if we're disabled for long enough and not moving */
	private void updateBrakeMode() {
		if (DriverStation.isEnabled() && !brakeMode) {
			setBrakeMode(true);
		} else {
			boolean stillMoving = false;

			// update if we're still moving for braking
			for (var mod : moduleStates)
				if (mod.speedMetersPerSecond > maxCoastVelocity_mps) {
					stillMoving = true;
					break;
				}

			if (brakeMode && !stillMoving
			// wait for scores to be finalized during a match
				&& (!DriverStation.isFMSAttached() || (Timer.getFPGATimestamp() - disabledTimeStart) < 4.0))
				setBrakeMode(false);
		}
	}

	private void setBrakeMode(boolean enable) {
		brakeMode = enable;
		for (SwerveModule mod : modules) {
			mod.setDriveBrakeMode(enable);
		}
	}

	public void toggleFieldRelative() {
		this.fieldRelative = !this.fieldRelative;
	}

	public void setCenterRotation(double x, double y) {
		centerRotation = new Translation2d(x, y);
	}

	/**
	 * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
	 * rotational directions. The velocities may be specified from either the robot's frame of
	 * reference of the field's frame of reference. In the robot's frame of reference, the positive x
	 * direction is forward; the positive y direction, left; position rotation, CCW. In the field
	 * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
	 * field to the driver's right). Zero is away from the driver and increases in the CCW direction.
	 */
	public void drive(Translation2d translation, double rotation) {
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(
					translation.getX(),
					translation.getY(),
					rotation,
					getYaw())
				: new ChassisSpeeds(
					translation.getX(),
					translation.getY(),
					rotation),
			centerRotation);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity_mps);

		for (SwerveModule mod : modules) {
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false, false);
		}
	}

	public void stop() {
		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerRotation);
		setModuleStates(states);
	}

	/**
	 * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
	 * to make an 'X', which makes it more difficult for other robots to push the robot.
	 */
	// todo: move to command
	private void setXStance() {
		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerRotation);
		states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[3].angle = new Rotation2d(Math.PI * 3.0 / 2.0 - Math.atan(trackWidth_m / wheelBase_m));
		for (SwerveModule swerveModule : modules) {
			swerveModule.setDesiredState(states[swerveModule.moduleNumber], true, true);
		}
	}

	public void runCharacterization(double voltage) {
		for (var mod : modules)
			mod.runCharacterization(voltage);
	}

	// radps
	public double getCharacterizationVelocity() {
		double driveVelocityAverage = 0.0;
		for (var module : modules)
			driveVelocityAverage += module.getCharacterizationVelocity();
		return driveVelocityAverage / modules.length;
	}

	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.getInstance().processInputs("Gyro", gyroInputs);
		for (SwerveModule mod : modules) {
			mod.periodic();

			modulePositions[mod.moduleNumber] = mod.getPosition();
			moduleStates[mod.moduleNumber] = mod.getState();

			// log outputs
			Logger.getInstance().recordOutput("Swerve/State " + mod.moduleNumber, moduleStates[mod.moduleNumber]);
		}

		// update odometry
		poseEstimator.updateWithTime(timer.get(), getYaw(), modulePositions);
		var pose = poseEstimator.getEstimatedPosition();
		// todo: estimate without using gyro?

		Logger.getInstance().recordOutput("Odometry/Robot", pose);
		// Logger.getInstance().recordOutput("3DField", new Pose3d(pose));

		// update field pose
		for (int i = 0; i < modules.length; i++) {
			modulePoses[i] = new Pose2d(
				moduleTranslations[i]
					.rotateBy(getYaw())
					.plus(pose.getTranslation()),
				moduleStates[i].angle
					.plus(pose.getRotation())
					// show movement direction instead of physical module direction since it's optimized
					.plus(Rotation2d.fromRadians(moduleStates[i].speedMetersPerSecond < 0 ? Math.PI : 0)));
		}
		fieldSim.setRobotPose(pose);
		fieldSim.getObject("Swerve Modules").setPoses(modulePoses);
	}

	@Override
	public void close() throws Exception {
		for (var mod : modules)
			mod.close();
	}
}
