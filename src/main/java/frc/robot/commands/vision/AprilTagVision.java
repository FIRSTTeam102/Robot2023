package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.swerve.PathPlannerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AprilTagVision extends CommandBase {
	private Vision vision;
	private Routine routine;
	private boolean shouldRegeneratePaths = true;
	private double xDistanceGoTo_m;
	private double yDistanceGoTo_m;
	private PathPlannerCommand ppCommand;
	private Swerve swerve;

	public enum Routine {
		Left, Middle, Right
	}

	public AprilTagVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.AprilTag);
		shouldRegeneratePaths = vision.inputs.botpose_targetspaceTranslationZ_m < Constants.VisionConstants.maxZDistanceGridAprilTag_m;
		cancelPPCommand();
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady() || !shouldRegeneratePaths) {
			return;
		}

		switch (routine) {
			case Left:
				if (vision.inputs.targetAprilTag == 5 || vision.inputs.targetAprilTag == 4) {
					System.out.println("Left Doublesubstation");
					xDistanceGoTo_m = ;
					yDistanceGoTo_m = ;
				} else if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					System.out.println("Left Grid");
					xDistanceGoTo_m = ;
					yDistanceGoTo_m = ;
				}
				break;

			case Middle:
				if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					System.out.println("Middle Grid");
					xDistanceGoTo_m = ;
					yDistanceGoTo_m = ;
				}
				break;

			case Right:
				if (vision.inputs.targetAprilTag == 5 || vision.inputs.targetAprilTag == 4) {
					System.out.println("Right Doublesubstation");
					xDistanceGoTo_m = ;
					yDistanceGoTo_m = ;
				} else if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					System.out.println("Right Grid");
					xDistanceGoTo_m = ;
					yDistanceGoTo_m = ;
				}
				break;

			default:
				return;
		}

		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(Constants.AutoConstants.maxVelocity_mps, Constants.AutoConstants.maxAcceleration_mps2),
			new PathPoint(swerve.getPose().getTranslation(), swerve.getPose().getRotation()), // start at current pos
			new PathPoint(new Translation2d(xDistanceGoTo_m, yDistanceGoTo_m),
				Rotation2d.fromRadians(0)));

		cancelPPCommand();
		ppCommand = new PathPlannerCommand(trajectory, swerve, false);
		ppCommand.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		cancelPPCommand();
	}

	@Override
	public boolean isFinished() {
		// don't end automatically if subcommand is running
		if (ppCommand != null && !ppCommand.isFinished())
			return false;

		return false;
	}

	private void cancelPPCommand() {
		if (ppCommand != null)
			ppCommand.cancel();
	}
}
