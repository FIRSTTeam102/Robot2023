package frc.robot.commands.vision;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.elevator.SetElevatorPosition;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Elevator elevator;
	private Swerve swerve;
	private double robotRotateVelocity_mps;

	public enum Routine {
		MiddleGrid, TopDoublesubstation, TopGrid
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

		// Outputs a robotRotateVelocity_mps that updates every 0.02s for the motor. rotateKp, rotateKi, rotateKd must be
		// tuned and can not be calculated in a spreadsheet as rotateErrorIntegral and rotateErrorDerivative are based on
		// the last 0.02s VisionConstants.rotatekP * crosshairToTargetOffsetX_rad. We will not know what this data will be
		// for the last 0.02s on a spreadsheet because we do not know what the robotRotateVelocity_mps was the last 0.02s.
		if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundRotateX_rad) {
			robotRotateVelocity_mps = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				- VisionConstants.rotateKi * vision.rotateErrorIntegral
				- VisionConstants.rotateKd * vision.rotateErrorDerivative;
		} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairTargetBoundRotateX_rad) {
			robotRotateVelocity_mps = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				+ VisionConstants.rotateKi * vision.rotateErrorIntegral
				+ VisionConstants.rotateKd * vision.rotateErrorDerivative;
		}

		// When we see a grid retroreflective, we will rotate to it and put elevator up
		// When are at a doublesubstation apriltag, we will put elevator up
		switch (routine) {
			case MiddleGrid:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator MiddleNode");
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mps, false);
				new SetElevatorPosition(elevator, ElevatorConstants.middleNodeHeight_m);
				break;

			case TopDoublesubstation:
				System.out.println("Elevator Doublesubstation");
				new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m);
				break;

			case TopGrid:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator TopNode");
				swerve.drive(new Translation2d(0, 0), robotRotateVelocity_mps, false);
				new SetElevatorPosition(elevator, ElevatorConstants.topNodeHeight_m);
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
