package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTagVision extends CommandBase {
	private Vision vision;

	public enum Routine {
		LeftGrid, MiddleGrid, RightGrid
	}

	public AprilTagVision(Vision vision, Routine routine) {
		this.vision = vision;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.AprilTag);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		// AprilTag code
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
