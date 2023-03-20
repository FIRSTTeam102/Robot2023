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
	private double robotTranslate_mps;

	public enum Routine {
		BlueRedGridLeftRight
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Swerve swerve) {
		addRequirements(swerve);
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

		// When we see a grid retroreflective, we will rotate to it
		switch (routine) {
			case BlueRedGridLeftRight:
				if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundTranslateX_rad) {
					robotTranslate_mps = VisionConstants.retroreflectiveTranslateKp
						* vision.inputs.crosshairToTargetErrorX_rad
						- VisionConstants.retroreflectiveTranslateKd;
				} else if (VisionConstants.crosshairTargetBoundTranslateX_rad < vision.inputs.crosshairToTargetErrorX_rad) {
					robotTranslate_mps = VisionConstants.retroreflectiveTranslateKp
						* vision.inputs.crosshairToTargetErrorX_rad
						+ VisionConstants.retroreflectiveTranslateKd;
				}
				robotTranslate_mps *= -1; // go opposite of error
				break;

			default:
				return;
		}

		// Generate a continuously updated translation to retroreflective
		System.out.println("Swerve --> BlueRedGridLeftRight");
		System.out.println(robotTranslate_mps);
		swerve.drive(new Translation2d(0, robotTranslate_mps), 0, false);
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
		return (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundTranslateX_rad)
			|| (VisionConstants.crosshairTargetBoundTranslateX_rad < vision.inputs.crosshairToTargetErrorX_rad);
	}
}
