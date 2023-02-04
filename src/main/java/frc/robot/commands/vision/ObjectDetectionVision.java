package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ObjectDetectionVision extends CommandBase {
	private Vision vision;

	public ObjectDetectionVision(Vision vision) {
		this.vision = vision;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		// ObjectDetection code
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
