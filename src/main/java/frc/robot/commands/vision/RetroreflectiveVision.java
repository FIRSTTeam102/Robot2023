package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.FieldVisionPipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private double robotTranslate_mps;
	private boolean isAligned;

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
		vision.setFieldVisionPipeline(FieldVisionPipeline.Retroreflective);
		robotTranslate_mps = 0;
		isAligned = false;
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		isAligned = (vision.inputs.fieldVisionCrosshairToTargetErrorX_rad < VisionConstants.crosshairFieldBoundTranslateX_rad)
			&& (vision.inputs.fieldVisionCrosshairToTargetErrorX_rad > -VisionConstants.crosshairFieldBoundTranslateX_rad);
		Logger.getInstance().recordOutput("FieldVision/isAligned", isAligned);

		// When we see a grid Retroreflective, we will rotate to it
		switch (routine) {
			case BlueRedGridLeftRight:
				if (vision.inputs.fieldVisionCrosshairToTargetErrorX_rad < -VisionConstants.crosshairFieldBoundTranslateX_rad) {
					robotTranslate_mps = VisionConstants.retroreflectiveTranslateKp
						* vision.inputs.fieldVisionCrosshairToTargetErrorX_rad
						- VisionConstants.retroreflectiveTranslateKd;
				} else if (VisionConstants.crosshairFieldBoundTranslateX_rad < vision.inputs.fieldVisionCrosshairToTargetErrorX_rad) {
					robotTranslate_mps = VisionConstants.retroreflectiveTranslateKp
						* vision.inputs.fieldVisionCrosshairToTargetErrorX_rad
						+ VisionConstants.retroreflectiveTranslateKd;
				}
				robotTranslate_mps *= -1; // Translate opposite of error
				break;

			default:
				return;
		}

		// Generate a continuously updated translation to Retroreflective
		System.out.println("Swerve --> BlueRedGridLeftRight");
		if (!isFinished)
			swerve.drive(new Translation2d(0, robotTranslate_mps), 0, false);
	}

	// Stop swerve and set pipeline back to AprilTag
	@Override
	public void end(boolean interrupted) {
		isFinished = false;
		swerve.stop();
		vision.setFieldVisionPipeline(FieldVisionPipeline.AprilTag);
	}

	// Feedback loop for PD until we meet fieldVisionCrosshairToTargetErrorX_rad
	@Override
	public boolean isFinished() {
		return isAligned;
	}
}
