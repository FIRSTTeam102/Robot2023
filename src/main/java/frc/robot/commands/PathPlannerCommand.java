package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This command instructs the swerve subsystem to follow the specified trajectory, presumably during the autonomous period. 
 */
public class PathPlannerCommand extends PPSwerveControllerCommand {
	private Swerve swerve;
	private PathPlannerTrajectory trajectory;
	private boolean initialPath;

	/**
	 * @param trajectory the specified trajectory created by PathPlanner
	 * @param subsystem the drivetrain subsystem required by this command
	 * @param initialPath true, if this trajectory is the first in a sequence of trajectories or the
	 *     only trajectory, in which case the gyro and odometry will be initialized to match the start
	 *     of trajectory; false, if this trajectory is a subsequent trajectory in which case the gyro
	 *     and odometry will not be re-initialized in order to ensure a smooth transition between
	 *     trajectories
	 */
	public PathPlannerCommand(PathPlannerTrajectory trajectory, Swerve subsystem, boolean initialPath) {
		super(
			trajectory,
			subsystem::getPose,
			subsystem.autoXController,
			subsystem.autoYController,
			subsystem.autoRotationController,
			subsystem::setChasisSpeeds,
			subsystem);

		this.swerve = subsystem;
		this.trajectory = trajectory;
		this.initialPath = initialPath;
	}

	/**
	 * This method is invoked once when this command is scheduled. If the trajectory is the first in a
	 * sequence of trajectories or the only trajectory, initialize the gyro and odometry to match the
	 * start of trajectory. PathPlanner sets the origin of the field to the lower left corner (i.e.,
	 * the corner of the field to the driver's right). Zero degrees is away from the driver and
	 * increases in the CCW direction. It is critical that this initialization occurs in this method
	 * and not the constructor as this object is constructed well before the command is scheduled.
	 */
	@Override
	public void initialize() {
		super.initialize();

		if (initialPath) {
			// reset odometry to the starting pose of the trajectory
			swerve.resetOdometry(trajectory.getInitialState());
		}

		// reset the theta controller such that old accumulated ID values aren't used with the new path
		// this doesn't matter if only the P value is non-zero, which is the current behavior
		swerve.autoXController.reset();
		swerve.autoYController.reset();
		swerve.autoRotationController.reset();
	}

	/**
	 * This method will be invoked when this command finishes or is interrupted. It stops the motion
	 * of the drivetrain.
	 *
	 * @param interrupted true if the command was interrupted by another command being scheduled
	 */
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		super.end(interrupted);
	}
}