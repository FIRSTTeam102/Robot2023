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

	// we can also mix in vision measurements to make it more accurate to the field
	public SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
		kinematics, new Rotation2d(), modulePositions, new Pose2d());

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public Pose2d[] modulePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
	public Field2d fieldSim = new Field2d();

	private Translation2d centerRotation = new Translation2d(0, 0); // position the robot rotates around

	private Timer timer;

	// configurable stuff
	private boolean brakeMode = true;
	@Setter
	private boolean fieldRelative = false;

	public GyroIO gyroIO;
	public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private double gyroOffset = 0.0;

	public final PIDController autoXController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public final PIDController autoYController = new PIDController(autoDriveKp, autoDriveKi, autoDriveKd);
	public final PIDController autoRotationController = new PIDController(autoTurnKp, autoTurnKi, autoTurnKd);

	/** Creates a new Swerve. */
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

		timer = new Timer();
		timer.reset();
		timer.start();

		SmartDashboard.putData("Field", fieldSim);
	}

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

	/**
	 * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
	 * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
	 * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
	 *
	 * @return the pose of the robot
	 */
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
	 * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
	 * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
	 * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
	 * right). Zero degrees is away from the driver and increases in the CCW direction.
	 *
	 * @param state the specified PathPlanner state to which is set the odometry
	 */
	public void resetOdometry(PathPlannerState state) {
		setGyroOffset(state.holonomicRotation.getDegrees());

		// estimatedPoseWithoutGyro = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
		poseEstimator.resetPosition(
			getYaw(), getPositions(),
			new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
	}

	public void addVisionMeasurement(Pose2d pose) {
		poseEstimator.addVisionMeasurement(pose, timer.get());
	}

	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(gyroInputs.yaw_deg + gyroOffset);
	}

	public void zeroYaw() {
		gyroIO.setYaw(0);
	}

	// rotating around side-to-side axis (leaning forward/backward)
	public double getPitch_rad() {
		return MathUtil.angleModulus(Units.degreesToRadians(gyroInputs.pitch_deg));
	}

	// rotating around front-to-back axis (leaning left/right)
	public double getRoll_rad() {
		return MathUtil.angleModulus(Units.degreesToRadians(gyroInputs.roll_deg));
	}

	/**
	 * Sets the rotation of the robot to the specified value. This method should only be invoked when
	 * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
	 * facing away from the driver station; CCW is positive.
	 *
	 * @param expectedYaw the rotation of the robot (in degrees)
	 */
	public void setGyroOffset(double expectedYaw) {
		gyroOffset = expectedYaw - gyroInputs.yaw_deg;
	}

	/**
	 * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
	 * stopped moving, and brake mode is enabled, disable it.
	 */
	private void updateBrakeMode() {
		if (DriverStation.isEnabled() && !brakeMode) {
			setBrakeMode(true);
		} else {
			boolean stillMoving = false;
			for (SwerveModuleState state : moduleStates) {
				if (Math.abs(state.speedMetersPerSecond) > maxCoastVelocity_mps)
					stillMoving = true;
			}

			if (brakeMode && !stillMoving)
				setBrakeMode(false);
		}
	}

	private void setBrakeMode(boolean enable) {
		brakeMode = enable;
		for (SwerveModule mod : modules) {
			mod.setDriveBrakeMode(enable);
		}
	}

	// handle by the scheduler instead
	// public enum Mode {
	// normal, balance, xStance
	// }
	// @Getter
	// @Setter
	// private Mode mode = Mode.normal;

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
	 * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
	 * direction.
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
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true, false);
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

	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);
		Logger.getInstance().processInputs("Gyro", gyroInputs);
		for (SwerveModule mod : modules) {
			mod.updateInputs();

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

		// update the brake mode based on the robot's velocity and state (enabled/disabled)
		updateBrakeMode();

		// update field pose
		for (int i = 0; i < modules.length; i++) {
			modulePoses[i] = new Pose2d(
				moduleTranslations[i]
					.rotateBy(getYaw())
					.plus(pose.getTranslation()),
				moduleStates[i].angle
					.plus(pose.getRotation()));
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
