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
				if (vision.visionInputs.targetAprilTag == 5 || vision.visionInputs.targetAprilTag == 4) {
					System.out.println("Left Doublesubstation");

				} else if (vision.visionInputs.targetAprilTag >= 0 && vision.visionInputs.targetAprilTag <= 3 || vision.visionInputs.targetAprilTag 6>= 0 && vision.visionInputs.targetAprilTag <= 8) {
					System.out.println("Left Grid");
				}
				break;

			case Middle:
				if (vision.visionInputs.targetAprilTag >= 0 && vision.visionInputs.targetAprilTag <= 3 || vision.visionInputs.targetAprilTag 6>= 0 && vision.visionInputs.targetAprilTag <= 8) {
					System.out.println("Middle Grid");
				}
				break;

			case Right:
			if (vision.visionInputs.targetAprilTag == 5 || vision.visionInputs.targetAprilTag == 4) {
				System.out.println("Right Doublesubstation");

			} else if (vision.visionInputs.targetAprilTag >= 0 && vision.visionInputs.targetAprilTag <= 3 || vision.visionInputs.targetAprilTag 6>= 0 && vision.visionInputs.targetAprilTag <= 8) {
				System.out.println("Right Grid");
			}
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
