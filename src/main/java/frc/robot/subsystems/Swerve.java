package frc.robot.subsystems;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOInputsAutoLogged;
import frc.robot.io.VisionIO.FieldVisionPipeline;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleIOReal;
import frc.robot.swerve.SwerveModuleIOSim;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import org.littletonrobotics.junction.Logger;

import lombok.Getter;

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

	@Getter
	private Translation2d centerRotation = new Translation2d(0, 0); // position the robot rotates around

	// used for pose estimation
	private Timer timer = new Timer();
	public double disabledTimeStart = 0.0;

	private boolean brakeMode = true;

	public GyroIO gyroIO;
	public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private double gyroOffset_deg = 0.0;

	public double translationY;
	public double translationX;

	private Vision vision;

	public Swerve(GyroIO gyroIO, Vision vision) {
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

		this.vision = vision;
	}

	/** update and return states */
	public SwerveModuleState[] getStates() {
		for (SwerveModule mod : modules) {
			moduleStates[mod.moduleNumber] = mod.getState();
		}
		return moduleStates;
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		setModuleStates(desiredStates, false, false);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean forceAngle) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxVelocity_mps);
		for (SwerveModule mod : modules)
			mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, forceAngle);
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
		// fixme:?
		// setGyroOffset_deg(state.holonomicRotation.getDegrees()); // (Robot.isBlue() ? 0 : 180)
		// gyroIO.setYaw(0);

		// estimatedPoseWithoutGyro = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
		poseEstimator.resetPosition(
			getYaw(), getPositions(),
			new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
	}

	public void resetModuleOffsets() {
		for (var mod : modules)
			mod.io.setOffset(0);
	}

	/** @return gyro yaw including code offset */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(gyroInputs.yaw_deg + gyroOffset_deg);
	}

	public double getTilt_rad() {
		var yaw = getYaw().getRadians();
		return (gyroInputs.pitch_rad * Math.sin(yaw)) + (gyroInputs.roll_rad * -Math.cos(yaw));
	}

	/** sets the rotation of the robot to the specified value by changing code gyro offset */
	public void setGyroOffset_deg(double expectedYaw_deg) {
		gyroOffset_deg = expectedYaw_deg - gyroInputs.yaw_deg;
	}

	public void zeroYaw() {
		gyroIO.setYaw(0);
	}

	/** turn off brake mode if we're disabled for long enough and not moving */
	// todo:
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

	public void setCenterRotation(double x, double y) {
		centerRotation = new Translation2d(x, y);
	}

	/**
	 * sets the velocity of the robot
	 * 
	 * <p>if robot oriented, +x is forward, +y is left, +rotation is CCW;
	 * <p>if field oriented, the origin of the field is the lower left corner (corner of the field to the driver's right);
	 * for rotation zero is away from the driver, positive is CCW
	 */
	public void drive(Translation2d translation_mps, double rotation_radps, boolean fieldRelative) {
		SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(
					translation_mps.getX(),
					translation_mps.getY(),
					rotation_radps,
					getYaw())
				: new ChassisSpeeds(
					translation_mps.getX(),
					translation_mps.getY(),
					rotation_radps),
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

	public void runCharacterization(double voltage) {
		for (var mod : modules)
			mod.runCharacterization(voltage);
	}

	// radps
	public double getCharacterizationVelocity() {
		double driveVelocityAverage = 0.0;
		for (var module : modules)
			driveVelocityAverage += module.io.getCharacterizationVelocity_radps();
		return driveVelocityAverage / modules.length;
	}

	public SwerveModuleState[] getXStanceStates() {
		var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0), new Translation2d(0, 0));
		states[0].angle = new Rotation2d(3 * Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[3].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackWidth_m / wheelBase_m));
		states[2].angle = new Rotation2d(Math.PI / 2 - Math.atan(trackWidth_m / wheelBase_m));
		return states;
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
		translationY = pose.getY();
		translationX = pose.getX();
		// todo: estimate without using gyro?

		// Every 0.02s, updating pose2d
		if (vision.inputs.fieldVisionPipeline == FieldVisionPipeline.AprilTag.value && vision.isPipelineReady()
			&& vision.inputs.fieldVisionTarget == true
			&& (vision.inputs.fieldVisionBotpose_FieldspaceTranslationX_m < VisionConstants.botpose_fieldBlueCommunityGeoFenceX_m
				|| vision.inputs.fieldVisionBotpose_FieldspaceTranslationX_m > VisionConstants.botpose_fieldRedCommunityGeoFenceX_m)) {
			var visionPose = new Pose2d(vision.inputs.fieldVisionBotpose_FieldspaceTranslationX_m,
				vision.inputs.fieldVisionBotpose_FieldspaceTranslationY_m,
				new Rotation2d(vision.inputs.fieldVisionBotpose_FieldspaceRotationZ_rad));
			Logger.getInstance().recordOutput("Odometry/VisionPose", visionPose);
			poseEstimator.addVisionMeasurement(visionPose, timer.get() - vision.inputs.fieldVisionBotpose_Latency_s);
		}

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
