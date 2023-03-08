package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private double robotRotateVelocity_mps;

	public enum Routine {
		BlueRedGridMiddle, BlueRedGridTop
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		// Sets pipeline to Retroreflective
		vision.setPipeline(Pipeline.Retroreflective);
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		// robotRotateVelocity_mps and PID loop using PIDcontroller class
		// vision.inputs.crosshairToTargetErrorX_rad and VisionConstants.crosshairTargetBoundRotateX_rad

		// When we see a grid retroreflective, we will rotate to it
		switch (routine) {
			case BlueRedGridMiddle:
				System.out.println("Swerve --> BlueRedGridMiddle, Elevator --> BlueRedGridNodeMiddle");
				System.out.println("crosshairToTargetErrorX_rad: " + vision.inputs.crosshairToTargetErrorX_rad);
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mps, false);
				break;

			case BlueRedGridTop:
				System.out.println("Swerve --> BlueRedGridTop, Elevator --> BlueRedGridNodeTop");
				System.out.println("crosshairToTargetErrorX_rad: " + vision.inputs.crosshairToTargetErrorX_rad);
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mps, false);
				break;
		}
	}

	// Stop swerve and sets pipeline back to Apriltag
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		vision.setPipeline(Pipeline.AprilTag);
	}

	// Feedback loop for PID until we meet crosshairTargetBoundRotateX_rad
	@Override
	public boolean isFinished() {
		return ((-VisionConstants.crosshairTargetBoundRotateX_rad < vision.inputs.crosshairToTargetErrorX_rad)
			&& (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairTargetBoundRotateX_rad));
	}
}
