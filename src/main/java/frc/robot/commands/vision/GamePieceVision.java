package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.GamePieceVisionPipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.littletonrobotics.junction.Logger;

public class GamePieceVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private double robotRotate_radps;
	private boolean isAligned;

	public enum Routine {
		GamePieceGround
	}

	public GamePieceVision(Routine routine, Vision vision, Swerve swerve) {
		addRequirements(swerve);
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		vision.setGamePieceVisionPipeline(GamePieceVisionPipeline.GamePiece);
		robotRotate_radps = 0;
		isAligned = false;
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		isAligned = (vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad < VisionConstants.crosshairGamePieceBoundRotateX_rad)
			&& (vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad > -VisionConstants.crosshairGamePieceBoundRotateX_rad);
		Logger.getInstance().recordOutput("GamePieceVision/isAligned", isAligned);

		// When we see a ground GamePiece, we will rotate to it
		switch (routine) {
			case GamePieceGround:
				if (vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad < -VisionConstants.crosshairGamePieceBoundRotateX_rad) {
					robotRotate_radps = VisionConstants.gamePieceRotateKp
						* vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad
						- VisionConstants.gamePieceRotateKd;
				} else if (VisionConstants.crosshairGamePieceBoundRotateX_rad < vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad) {
					robotRotate_radps = VisionConstants.gamePieceRotateKp
						* vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad
						+ VisionConstants.gamePieceRotateKd;
				}
				robotRotate_radps *= -1; // Rotate opposite of error
				break;

			default:
				return;
		}

		// Generate a continuously updated rotation to GamePiece
		swerve.drive(new Translation2d(0, 0), robotRotate_radps, false);
	}

	// Stop swerve set pipeline back to GamePiece
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		vision.setGamePieceVisionPipeline(GamePieceVisionPipeline.GamePiece);
	}

	// Feedback loop for PD until we meet gamePieceVisionCrosshairTargetBoundRotateX_rad
	@Override
	public boolean isFinished() {
		return isAligned;
	}
}
