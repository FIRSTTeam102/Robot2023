package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AprilTagVision extends CommandBase {
	private Vision vision;
	private Routine routine;

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
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		switch (routine) {
			case Left:
				System.out.println("Left");
				break;

			case Middle:
				System.out.println("Middle");
				break;

			case Right:
				System.out.println("Right");
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
