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
	private double robotRotateVelocity_mps;

	public enum Routine {
		GamepieceGround
	}

	public GamePieceVision(Routine routine, Vision vision, Swerve swerve, Elevator elevator, Arm arm, Grabber grabber) {
		this.routine = routine;
		this.vision = vision;
		this.swerve = swerve;
		this.elevator = elevator;
		this.arm = arm;
		this.grabber = grabber;
	}

	@Override
	public void initialize() {
		// Sets pipeline to GamePiece
		vision.setPipeline(Pipeline.GamePiece);
	}

	@Override
	public void execute() {
		// If isPipelineReady true, begin execute
		if (!vision.isPipelineReady())
			return;

		// When we see a ground gamepiece, we will rotate to it
		switch (routine) {
			case GamepieceGround:
				if (-VisionConstants.crosshairGamePieceBoundRotateX_rad > vision.inputs.crosshairToTargetErrorX_rad) {
					robotRotateVelocity_mps = VisionConstants.gamePieceRotateKp * -vision.inputs.crosshairToTargetErrorX_rad
						- VisionConstants.gamePieceRotateKd;
				} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairGamePieceBoundRotateX_rad) {
					robotRotateVelocity_mps = VisionConstants.gamePieceRotateKp * vision.inputs.crosshairToTargetErrorX_rad
						+ VisionConstants.gamePieceRotateKd;
				}
				break;

			default:
				return;
		}

		// Generate a rotation to gamepiece
		System.out.println("Swerve --> Gamepiece");
		swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mps, false);
	}

	// Stop swerve and generate a algorithm to get gamepiece and set pipeline back to Apriltag
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		// System.out.println("Arm --> GroundFar, Elevator --> Ground, Grabber --> GamepieceUntilGrabbed");
		// Autos.intakeGroundFar(swerve, elevator, arm, grabber);
		// System.out.println("Arm --> AllIn, Elevator --> AllIn");
		// Autos.allIn(elevator, arm).schedule();
		vision.setPipeline(Pipeline.AprilTag);
	}

	// Feedback loop for PD until we meet crosshairTargetBoundRotateX_rad
	@Override
	public boolean isFinished() {
		return ((-VisionConstants.crosshairGamePieceBoundRotateX_rad < vision.inputs.crosshairToTargetErrorX_rad)
			&& (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairGamePieceBoundRotateX_rad));
	}
}
