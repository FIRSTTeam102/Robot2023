package frc.robot.commands.vision;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
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
		// Sets pipeline to Apriltag
		vision.setPipeline(Pipeline.AprilTag);

		// If we are too far away VisionConstants.maxZDistanceAprilTag_m from the Apriltag, AprilTagVision will
		// not execute
		shouldRegeneratePaths = vision.inputs.botpose_targetspaceTranslationZ_m < VisionConstants.maxZDistanceAprilTag_m;
		cancelPPCommand();
	}

	@Override
	public void execute() {
		// If pipeline switch time error has been reached and we are within VisionConstants.maxZDistanceAprilTag_m
		// from the Apriltag, begin execute
		if (!vision.isPipelineReady() || !shouldRegeneratePaths) {
			return;
		}

		// When we see a substation apriltag, we will position to it and rotate to alliance color's side and put elevator up
		// When we see a grid apriltag, we will position to it and rotate to alliance color's side
		switch (routine) {
			case Left:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Blue Doublesubstation Left"); // Blue. Doublesubstation. Robot POV left.
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 6.13;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Red Doublesubstation Left"); // Red. Doublesubstation. Robot POV left.
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 7.33;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Left"); // Blue. Grid nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.87;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Left"); // Blue. Grid second nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.19;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Left"); // Blue. Grid third nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 0.51;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Left"); // Red. Grid nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.97;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Left"); // Red. Grid second nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.29;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Left"); // Red. Grid third nearest to double substation. Robot POV left.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.61;
				}
				break;

			case Middle:
				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Middle"); // Blue. Grid nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.42;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Middle"); // Blue. Grid second nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 2.75;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Middle"); // Blue. Grid third nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.06;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Middle"); // Red. Grid nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 4.40;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Middle"); // Red. Grid second nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.74;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Middle"); // Red. Grid third nearest to double substation. Robot POV middle.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 1.05;
				}
				break;

			case Right:
				if (vision.inputs.targetAprilTag == 5) {
					System.out.println("Blue Doublesubstation Right"); // Blue. Doublesubstation. Robot POV right.
					xDistanceGoTo_m = 0.70;
					yDistanceGoTo_m = 7.47;
				}
				if (vision.inputs.targetAprilTag == 4) {
					System.out.println("Red Doublesubstation Right"); // Red. Doublesubstation. Robot POV right.
					xDistanceGoTo_m = 15.84;
					yDistanceGoTo_m = 6.00;
				}

				if (vision.inputs.targetAprilTag == 6) {
					System.out.println("Blue Grid 1 Right"); // Blue. Grid nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 4.98;
				}
				if (vision.inputs.targetAprilTag == 7) {
					System.out.println("Blue Grid 2 Right"); // Blue. Grid second nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 3.86;
				}
				if (vision.inputs.targetAprilTag == 8) {
					System.out.println("Blue Grid 3 Right"); // Blue. Grid third nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 1.80;
					yDistanceGoTo_m = 1.62;
				}
				if (vision.inputs.targetAprilTag == 3) {
					System.out.println("Red Grid 1 Right"); // Red. Grid nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 3.85;
				}
				if (vision.inputs.targetAprilTag == 2) {
					System.out.println("Red Grid 2 Right"); // Red. Grid second nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 2.17;
				}
				if (vision.inputs.targetAprilTag == 1) {
					System.out.println("Red Grid 3 Right"); // Red. Grid third nearest to double substation. Robot POV right.
					xDistanceGoTo_m = 14.73;
					yDistanceGoTo_m = 0.49;
				}
				break;

			default:
				return;
		}

		// Generate a path using from "Every 0.02s, updating pose2d" to "substation apriltag" or "grid apriltag"
		PathPlannerTrajectory trajectory = PathPlanner.generatePath(
			new PathConstraints(AutoConstants.maxVelocity_mps, AutoConstants.maxAcceleration_mps2),
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
		// AprilTag Vision command does not end automatically if subcommand is running
		if (ppCommand != null && !ppCommand.isFinished())
			return false;

		return false;
	}

	private void cancelPPCommand() {
		if (ppCommand != null)
			ppCommand.cancel();
	}
}
