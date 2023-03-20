package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
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

	public enum Routine {
		GamepieceGround
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
		vision.setPipeline(Pipeline.GamePiece);
		robotRotate_radps = 0;
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		// When we see a ground gamepiece, we will rotate to it
		switch (routine) {
			case GamepieceGround:
				if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundTranslateX_rad) {
					robotRotate_radps = VisionConstants.gamePieceRotateKp * -vision.inputs.crosshairToTargetErrorX_rad
						- VisionConstants.gamePieceRotateKd;
				} else if (VisionConstants.crosshairTargetBoundTranslateX_rad < vision.inputs.crosshairToTargetErrorX_rad) {
					robotRotate_radps = VisionConstants.gamePieceRotateKp * vision.inputs.crosshairToTargetErrorX_rad
						+ VisionConstants.gamePieceRotateKd;
				}
				robotRotate_radps *= -1; // go opposite of error
				break;

			default:
				return;
		}

		// Generate a continuously updated rotation to gamepiece
		System.out.println("Swerve --> Gamepiece");
		swerve.drive(new Translation2d(0, 0), robotRotate_radps, false);
	}

	// Stop swerve and generate a algorithm to get gamepiece and set pipeline back to Apriltag
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		// System.out.println("Intakeground, Allin");
		// Autos.intakeGround(swerve, elevator, arm, grabber).andThen(Autos.allIn(elevator, arm)).schedule();
		vision.setPipeline(Pipeline.AprilTag);
	}

	@Override
	public boolean isFinished() {
		return (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundTranslateX_rad)
			|| (VisionConstants.crosshairTargetBoundTranslateX_rad < vision.inputs.crosshairToTargetErrorX_rad);
	}
}
