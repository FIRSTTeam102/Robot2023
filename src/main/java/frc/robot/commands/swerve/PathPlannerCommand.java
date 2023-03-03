package frc.robot.commands.swerve;

import frc.robot.Autos;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.littletonrobotics.junction.Logger;

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
			trajectory,
			swerve::getPose,
			swerve.kinematics,
			Autos.ppXController,
			Autos.ppYController,
			Autos.ppRotationController,
			swerve::setModuleStates,
			transformForAlliance,
			swerve);

		this.swerve = swerve;
		this.trajectory = transformForAlliance
			? PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance())
			: trajectory;
		this.resetOdometry = firstPathEver;
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

		Logger.getInstance().recordOutput("Odometry/PathPlanner", trajectory);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		super.end(interrupted);
	}
}