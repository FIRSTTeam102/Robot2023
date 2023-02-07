package frc.robot.commands.vision;

import frc.robot.Constants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.Vector2;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTagVision extends CommandBase {
	private Vision vision;
	private Routine routine;
	private boolean shouldRegeneratePaths = true;

	public enum Routine {
		Left, Middle, Right
	}

	public AprilTagVision(Vision vision, Routine routine) {
		this.vision = vision;
		this.routine = routine;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.AprilTag);
		if (vision.inputs.botpose_targetspaceTranslationZ_m < Constants.VisionConstants.maxZDistanceGridAprilTag_m) {
			shouldRegeneratePaths = true;
		} else {
			shouldRegeneratePaths = false;
		}

	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady() || !shouldRegeneratePaths) {
			return;
		}

		Vector2 GoToPosition = Vision.getGoToPostition();

		Vector2 goal = new Vector2(Constants.VisionConstants.zDistanceGridAprilTag_m, 0.0);

		switch (routine) {
			case Left:
				if (vision.inputs.targetAprilTag == 5 || vision.inputs.targetAprilTag == 4) {
					System.out.println("Left Doublesubstation");

				} else if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					goal = new Vector2(Constants.VisionConstants.zDistanceGridAprilTag_m,
						-Constants.VisionConstants.xDistanceGridAprilTag_m);
					System.out.println("Left Grid");
					break;
				}

			case Middle:
				if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					goal = new Vector2(Constants.VisionConstants.zDistanceGridAprilTag_m, 0);
					System.out.println("Middle Grid");
					break;
				}

			case Right:
				if (vision.inputs.targetAprilTag == 5 || vision.inputs.targetAprilTag == 4) {
					System.out.println("Right Doublesubstation");

				} else if (vision.inputs.targetAprilTag >= 1 && vision.inputs.targetAprilTag <= 3
					|| vision.inputs.targetAprilTag >= 6 && vision.inputs.targetAprilTag <= 8) {
					goal = new Vector2(Constants.VisionConstants.zDistanceGridAprilTag_m,
						Constants.VisionConstants.xDistanceGridAprilTag_m);
					System.out.println("Right Grid");
					break;
				}
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
