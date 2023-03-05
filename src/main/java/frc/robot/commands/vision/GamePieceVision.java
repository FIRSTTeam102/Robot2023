package frc.robot.commands.vision;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.CloseGrabber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GamePieceVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Elevator elevator;
	private Arm arm;
	private Grabber grabber;
	private Swerve swerve;
	private double robotRotateVelocity_mps;
	private double robotTranslateVelocity_mps;

	public enum Routine {
		Gamepiece
	}

	public GamePieceVision(Routine routine, Vision vision, Elevator elevator, Arm arm, Grabber grabber,
		Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.elevator = elevator;
		this.arm = arm;
		this.grabber = grabber;
		this.swerve = swerve;
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

		// robotRotateVelocity_mps and PID loop using PIDcontroller class
		// vision.inputs.crosshairToTargetErrorX_rad and VisionConstants.crosshairTargetBoundRotateX_rad

		// robotTranslateVelocity_mps and PID loop using PIDcontroller class
		// vision.inputs.botpose_targetspaceTranslationZ_m and VisionConstants.crosshairObjectBoundTranslateZ_m

		// When we see a ground gamepiece, we will translate and rotate to it and put elevator down and put scissor arm out
		switch (routine) {
			case Gamepiece:
				System.out.println("Swerve --> Gamepiece, Elevator --> Gamepiece, Arm --> Gamepiece, Grabber --> Gamepiece");
				System.out.println("botpose_targetspaceRotationZ_rad: " + vision.inputs.botpose_fieldRotationZ_rad
					+ " crosshairToTargetErrorX_rad: " + vision.inputs.crosshairToTargetErrorX_rad);
				swerve.drive(new Translation2d(0, robotTranslateVelocity_mps), robotRotateVelocity_mps, false);
				new SetElevatorPosition(elevator, ElevatorConstants.groundHeight_m);
				new SetArmPosition(arm, ArmConstants.groundExtension_m);
				new CloseGrabber(grabber, .5, GrabberConstants.closingTime_s);
				break;
		}
	}

	// Stop swerve and sets pipeline back to Apriltag
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		vision.setPipeline(Pipeline.AprilTag);
	}

	// Feedback loop for PID until we meet crosshairTargetBoundRotateX_rad and crosshairObjectBoundTranslateZ_m
	@Override
	public boolean isFinished() {
		return ((-VisionConstants.crosshairObjectBoundRotateX_rad < vision.inputs.crosshairToTargetErrorX_rad)
			&& (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairObjectBoundRotateX_rad))
			&& (vision.inputs.botpose_fieldRotationZ_rad < VisionConstants.crosshairObjectBoundTranslateZ_m);
	}
}
