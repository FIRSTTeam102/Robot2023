package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.GamePieceVisionPipeline;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GamePieceVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private Elevator elevator;
	private Arm arm;
	private Grabber grabber;
	private double robotRotate_radps;
	private double gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad;

	public enum Routine {
		GamePieceGround
	}

	public GamePieceVision(Routine routine, Vision vision, Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		addRequirements(swerve);
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
		this.elevator = elevator;
		this.arm = arm;
		this.grabber = grabber;
	}

	@Override
	public void initialize() {
		vision.setGamePieceVisionPipeline(GamePieceVisionPipeline.GamePiece);
		robotRotate_radps = 0;
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad = (3.98 / 12.16)
			- (6.23 / 12.16) * vision.inputs.gamePieceVisionCrosshairToTargetErrorY_rad;

		// When we see a ground GamePiece, we will rotate to it
		switch (routine) {
			case GamePieceGround:
				if (vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad < -gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad) {
					robotRotate_radps = VisionConstants.gamePieceRotateKp
						* vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad
						- VisionConstants.gamePieceRotateKd;
				} else if (gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad < vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad) {
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
		System.out.println("Swerve --> GamePiece");
		System.out.println(vision.inputs.gamePieceVisionCrosshairToTargetErrorX_rad);
		System.out.println(robotRotate_radps);
		swerve.drive(new Translation2d(0, 0), robotRotate_radps, false);
	}

	// Stop swerve and generate a algorithm to get GamePiece and set pipeline back to GamePiece
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		// System.out.println("Intakeground, Allin");
		// Autos.intakeGround(swerve, elevator, arm, grabber).andThen(Autos.allIn(elevator, arm)).schedule();
		vision.setGamePieceVisionPipeline(GamePieceVisionPipeline.GamePiece);
	}

	// Feedback loop for PD until we meet gamePieceVisionCrosshairTargetBoundRotateX_rad
	@Override
	public boolean isFinished() {
		return (gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad < -VisionConstants.crosshairGamePieceBoundRotateX_rad)
			&& (VisionConstants.crosshairGamePieceBoundRotateX_rad < gamePieceVisionCrosshairToTargetErrorLOSCorrectionX_rad);
	}
}
