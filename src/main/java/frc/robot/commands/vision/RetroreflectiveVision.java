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
	private double robotTranslateVelocity_mps;

	public enum Routine {
		BlueRedGridLeftRight
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
		addRequirements(swerve);
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

		// When we see a grid retroreflective, we will rotate to it
		switch (routine) {
			case BlueRedGridLeftRight:
				if (-VisionConstants.crosshairTargetBoundTranslateX_rad > vision.inputs.crosshairToTargetErrorX_rad) {
					robotTranslateVelocity_mps = VisionConstants.retroreflectiveTranslateKp
						* -vision.inputs.crosshairToTargetErrorX_rad
						- VisionConstants.retroreflectiveTranslateKd;
				} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairTargetBoundTranslateX_rad) {
					robotTranslateVelocity_mps = VisionConstants.retroreflectiveTranslateKp
						* vision.inputs.crosshairToTargetErrorX_rad
						+ VisionConstants.retroreflectiveTranslateKd;
				}
				break;

			default:
				return;
		}

		// Generate a translation to retroreflective
		System.out.println("Swerve --> BlueRedGridLeftRight");
		swerve.drive(new Translation2d(robotTranslateVelocity_mps, 0), 0, false);
	}

	// Stop swerve and set pipeline back to Apriltag
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		vision.setPipeline(Pipeline.AprilTag);
	}

	// Feedback loop for PD until we meet crosshairTargetBoundRotateX_rad
	@Override
	public boolean isFinished() {
		// return ((-VisionConstants.crosshairTargetBoundTranslateX_rad < vision.inputs.crosshairToTargetErrorX_rad)
		// && (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairTargetBoundTranslateX_rad));
		return false;
	}
}
