package frc.robot.commands.vision;

import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;

	public enum Routine {
		Middle, Top
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.Retroreflective);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		// Copy Limelight.cpp stuff from last year, (PID calculating motor speed from)
		// Copy YawToTarget.cpp stuff from last year (sending information to swerve)

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
	public void end(boolean interrupted) {
		// Copy YawToTarget.cpp stuff from last year, (stopping information to swerve)
	}

	@Override
	public boolean isFinished() {
		// Copy Limelight.cpp [check function definition] stuff from last year, (check to see if tx is within bounds)
		return false;
	}
}
