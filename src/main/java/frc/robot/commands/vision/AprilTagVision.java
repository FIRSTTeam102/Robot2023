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
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Blue Doublesubstation Left");
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 6.13;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Red Doublesubstation Left");
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 7.33;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.87;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.19;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Left");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 0.51;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.97;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.29;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Left");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.61;
				}
				break;

			case Middle:
				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.75;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Middle");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.06;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.40;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.74;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Middle");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.05;
				}
				break;

			case Right:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Blue Doublesubstation Right");
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 7.47;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Red Doublesubstation Right");
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 6.00;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.98;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.86;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Right");
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.62;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.85;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.17;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Right");
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 0.49;
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
