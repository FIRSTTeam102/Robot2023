package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Vision vision;

	public enum Routine {
		MiddleNode, TopNode
	}

	public RetroreflectiveVision(Vision vision, Routine routine) {
		this.vision = vision;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.Retroreflective);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		// Retroreflective code
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
