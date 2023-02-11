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
	private double robotVelocity_mpers;

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
		if (!vision.isPipelineReady())
			return;

		// Outputs a robotVelocity_mpers that updates every 0.02s for the motor. kP, kI, kP must be tuned and can not be
		// calculated in a spreadsheet as vision.errorIntegral and vision.errorDerivative are based on the last 0.02s
		// VisionConstants.visionTurnkP * vision.inputs.crosshairToTargetOffsetX_rad. We will not know what this data will
		// be for the last 0.02s on a spreadsheet because we do not know what the robotVelocity_mpers was the last 0.02s.
		if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairTargetBoundX_rad) {
			robotVelocity_mpers = VisionConstants.visionTurnKp * vision.inputs.crosshairToTargetErrorX_rad
				- VisionConstants.visionTurnKi * vision.errorIntegral
				- VisionConstants.visionTurnKd * vision.errorDerivative;
		} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairTargetBoundX_rad) {
			robotVelocity_mpers = VisionConstants.visionTurnKp * vision.inputs.crosshairToTargetErrorX_rad
				+ VisionConstants.visionTurnKi * vision.errorIntegral
				+ VisionConstants.visionTurnKd * vision.errorDerivative;
		}

		// When we see a grid retroreflective, we will rotate to it and put elevator up
		switch (routine) {
			case Middle:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator Middle Node");
				swerve.drive(new Translation2d(0, 0), robotVelocity_mpers);
				elevator.setPosition(ElevatorConstants.middleNodeHeight_m);
				break;

			case Top:
				System.out.println("Rotate: " + vision.inputs.crosshairToTargetErrorX_rad + "Elevator Top Node");
				swerve.drive(new Translation2d(0, 0), robotVelocity_mpers);
				elevator.setPosition(ElevatorConstants.topNodeHeight_m);
				break;
		}
	}

	// Stop swerve isFinished
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	// Feedback loop for PID
	@Override
	public boolean isFinished() {
		return ((-VisionConstants.crosshairTargetBoundX_rad < vision.inputs.crosshairToTargetErrorX_rad)
			&& (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairTargetBoundX_rad));
	}
}
