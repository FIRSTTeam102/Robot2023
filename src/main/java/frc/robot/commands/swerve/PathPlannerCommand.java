package frc.robot.commands.swerve;

import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import lombok.SneakyThrows;

/**
 * instructs the swerve subsystem to follow the specified trajectory 
 */
public class PathPlannerCommand extends PPSwerveControllerCommand {
	private Swerve swerve;
	private PathPlannerTrajectory trajectory;
	private boolean resetOdometry;

	/**
	 * @param trajectory the specified trajectory created by PathPlanner
	 * @param swerve the drivetrain subsystem required by this command
	 * @param transformForAlliance will mirror the trajectory if we're on the other alliance
	 */
	public PathPlannerCommand(PathPlannerTrajectory trajectory, Swerve swerve, boolean transformForAlliance) {
		this(trajectory, swerve, transformForAlliance, false);
	}

	/**
	 * @see {@link #PathPlannerCommand(PathPlannerTrajectory, Swerve, boolean)}
	 * @param firstPathEver true, if this trajectory is the first in a sequence of trajectories or the only trajectory,
	 * in which case the gyro and odometry will be initialized to match the start of trajectory;
	 * false, if this trajectory is a subsequent trajectory in which case the gyro and odometry will not be re-initialized
	 */
	public PathPlannerCommand(PathPlannerTrajectory trajectory, Swerve swerve, boolean transformForAlliance,
		boolean firstPathEver) {
		super(
			susTransformTrajectory(trajectory, transformForAlliance),
			swerve::getPose,
			swerve.kinematics,
			Autos.ppXController,
			Autos.ppYController,
			Autos.ppRotationController,
			swerve::setModuleStates,
			false, // transformForAlliance,
			swerve);

		this.swerve = swerve;
		// why can't it be a public 😭 i just want it in initialize
		this.trajectory = susTransformTrajectory(trajectory, transformForAlliance);
		this.resetOdometry = firstPathEver;
	}

	private static PathPlannerTrajectory susTransformTrajectory(PathPlannerTrajectory traj,
		boolean transformForAlliance) {
		// return PathPlanner.transformTrajectoryForAlliance(traj, Robot.isBlue() ? Alliance.Blue : Alliance.Red)

		// pathplanner flips along y so red (0,0) is on red side
		// we want (0,0) to always be blue bottom corner
		// https://github.com/mjansen4857/pathplanner/issues/297
		return transformForAlliance && DriverStation.getAlliance() == DriverStation.Alliance.Red
			? ReflectedTransform.reflectiveTransformTrajectory(traj)
			: traj;
	}

	@Override
	public void initialize() {
		super.initialize();

		// reset odometry to the starting pose of the trajectory
		if (resetOdometry)
			swerve.resetOdometry(trajectory.getInitialState());

		// reset controller such that old accumulated PID values aren't used with the new path
		// this doesn't matter if only the P value is non-zero
		Autos.ppXController.reset();
		Autos.ppYController.reset();
		Autos.ppRotationController.reset();
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		super.end(interrupted);
	}

	/**
	* Provides utilities for converting blue-side PathPlanner objects to red-side. Reflection is needed
	* due to private or protected fields within PathPlanner's API.
	*
	* <p>These transformations assume an absolute field origin on the blue alliance driver station, on
	* the scoring table side (away from the human player station).
	* <p>+X is the direction from blue alliance driver station to red alliance driver station.
	* <p>+Y is the direction from scoring table to human player station.
	* 
	* {@see https://github.com/FRC2713/Robot2023/blob/main/src/main/java/frc/robot/util/ReflectedTransform.java}
	*/
	public static class ReflectedTransform {
		private static Field deltaPosField;
		private static Field curveRadiusField;
		private static Constructor<PathPlannerTrajectory> constructor;

		// Reflection is expensive. Create declared objects at startup instead of per state or per trajectory.
		static {
			try {
				// Set all `.deltaPos` on PathPlannerState objects to be public
				deltaPosField = PathPlannerState.class.getDeclaredField("deltaPos");
				deltaPosField.setAccessible(true);

				// Set all `.curveRadius` on PathPlannerState objects to be public
				curveRadiusField = PathPlannerState.class.getDeclaredField("curveRadius");
				curveRadiusField.setAccessible(true);

				// Access the private constructor that builds a trajectory from states
				constructor = PathPlannerTrajectory.class.getDeclaredConstructor(
					List.class, List.class, StopEvent.class, StopEvent.class, boolean.class);
				constructor.setAccessible(true);
			} catch (NoSuchFieldException | SecurityException | NoSuchMethodException e) {
				System.err.println("Could not access private fields via reflection in PathPlannerTrajectory.");
				e.printStackTrace(System.err);
			}
		}

		/**
		 * Transforms a blue-side PathPlannerState to a red-side PathPlannerState. This should not need to
		 * be called from outside this class.
		 *
		 * @param state The blue-side state of a blue-side trajectory
		 * @return A new red-side state, or the same state if the DriverStation is set to blue.
		 */
		@SneakyThrows
		private static PathPlannerState reflectiveTransformState(PathPlannerState state) {
			PathPlannerState transformedState = new PathPlannerState();

			// Move it to the other side of the field, with an absolute origin on blue side
			// Mirror the X, keep the Y the same.
			Translation2d transformedTranslation = new Translation2d(
				FieldConstants.fieldLengthX_m - state.poseMeters.getX(), state.poseMeters.getY());

			// The instantaneous heading of the trajectory needs to be negated
			Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
			// The holonomic heading needs to be negated and rotated
			Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1).plus(Rotation2d.fromDegrees(180));

			transformedState.timeSeconds = state.timeSeconds;
			// Negate the velocity. If traveling from community to mid field on blue, the +X
			// velocity is
			// positive. If doing so on red, the +X velocity is negative.
			transformedState.velocityMetersPerSecond = -state.velocityMetersPerSecond;
			transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
			transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
			transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
			transformedState.holonomicRotation = transformedHolonomicRotation;
			transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
			transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

			// transformedState.deltaPos = state.deltaPos;
			deltaPosField.set(transformedState, deltaPosField.get(state));

			// transformedState.curveRadius = -state.curveRadius;
			curveRadiusField.set(transformedState, (-1) * (Double) curveRadiusField.get(state));

			return transformedState;
		}

		/**
		 * Transforms a blue-side PathPlannerTrajectory to a red-side PathPlannerTrajectory. In the event
		 * where this fails for any reason, an empty trajectory is returned so as not cause unpredictable
		 * behavior.
		 *
		 * @param trajectory the blue-side trajectory to transform
		 * @return the equivalent red-side trajectory
		 */
		@SneakyThrows
		public static PathPlannerTrajectory reflectiveTransformTrajectory(
			PathPlannerTrajectory trajectory) {
			List<Trajectory.State> transformedStates = new ArrayList<>();

			try {
				// Convert all the trajectory states to red-side
				for (Trajectory.State s : trajectory.getStates()) {
					PathPlannerState state = (PathPlannerState) s;
					transformedStates.add(reflectiveTransformState(state));
				}

				// Call the now unhidden constructor
				return constructor.newInstance(
					transformedStates,
					trajectory.getMarkers(),
					trajectory.getStartStopEvent(),
					trajectory.getEndStopEvent(),
					trajectory.fromGUI);
			} catch (IllegalArgumentException
				| IllegalAccessException
				| InstantiationException
				| InvocationTargetException e) {
				DriverStation.reportError(e.getMessage(), e.getStackTrace());
				// if this fails, return an empty trajectory instead of crashing
				return new PathPlannerTrajectory();
			}
		}
	}
}
