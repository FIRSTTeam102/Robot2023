package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ObjectDetectionVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;

	public enum Routine {
		Ground
	}

	public ObjectDetectionVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
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
