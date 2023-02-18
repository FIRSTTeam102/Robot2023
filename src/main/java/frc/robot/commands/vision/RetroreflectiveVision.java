package frc.robot.commands.vision;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Elevator elevator;
	private Swerve swerve;
	private double robotRotateVelocity_mpers;

	public enum Routine {
		Middle, Top
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Elevator elevator, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.elevator = elevator;
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

		// Outputs a robotRotateVelocity_mpers that updates every 0.02s for the motor. rotateKp, rotateKi, rotateKd must be
		// tuned and can not be calculated in a spreadsheet as rotateErrorIntegral and rotateErrorDerivative are based on
		// the last 0.02s VisionConstants.rotatekP * crosshairToTargetOffsetX_rad. We will not know what this data will be
		// for the last 0.02s on a spreadsheet because we do not know what the robotRotateVelocity_mpers was the last 0.02s.
		if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundRotateX_rad) {
			robotRotateVelocity_mpers = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				- VisionConstants.rotateKi * vision.rotateErrorIntegral
				- VisionConstants.rotateKd * vision.rotateErrorDerivative;
		} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairTargetBoundRotateX_rad) {
			robotRotateVelocity_mpers = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				+ VisionConstants.rotateKi * vision.rotateErrorIntegral
				+ VisionConstants.rotateKd * vision.rotateErrorDerivative;
		}

		// When we see a grid retroreflective, we will rotate to it and put elevator up
		switch (routine) {
			case Middle:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator Middle Node");
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mpers);
				elevator.setPosition(ElevatorConstants.middleNodeHeight_m);
				break;

			case Top:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator Top Node");
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mpers);
				elevator.setPosition(ElevatorConstants.topNodeHeight_m);
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
