package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Vision vision;
	private Routine routine;

	public enum Routine {
		Middle, Top
	}

	public RetroreflectiveVision(Vision vision, Routine routine) {
		this.vision = vision;
		this.routine = routine;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.Retroreflective);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		switch (routine) {
			case Middle:
				System.out.println("Middle");
				break;

			case Top:
				System.out.println("Top");
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
