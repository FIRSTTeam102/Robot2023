package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ObjectDetectionVision extends CommandBase {
	private Vision vision;
	private CommandXboxController operatorController;

	public ObjectDetectionVision(Vision vision, CommandXboxController operatorController) {
		this.vision = vision;
		this.operatorController = operatorController;
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
