package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ObjectDetectionVision extends CommandBase {
	private Vision vision;
	private Routine routine;

	public enum Routine {
		Ground
	}

	public ObjectDetectionVision(Vision vision, Routine routine) {
		this.vision = vision;
		this.routine = routine;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		switch (routine) {
			case Ground:
				System.out.println("Ground");
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
