package frc.robot.commands.vision;

import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetroreflectiveVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Swerve swerve;
	private double motorPower;

	public enum Routine {
		Middle, Top
	}

	public RetroreflectiveVision(Routine routine, Vision vision, Swerve swerve) {
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
		if (!vision.isPipelineReady())
			return;

		// Outputs a motorPower that updates every 0.02s for the motor. kP, kI, kP must be tuned and can not be calculated
		// in a spreadsheet as vision.errorSum and vision.errorDifference are based on the last 0.02s
		// VisionConstants.visionTurnkP * vision.inputs.crosshairToTargetOffsetX_rad. We will not know what this
		// data will be for the last 0.02s on a spreadsheet because we do not know what the motorPower was the last 0.02s.
		if (vision.inputs.crosshairToTargetOffsetX_rad < -2.0) {
			motorPower = VisionConstants.visionTurnkP * vision.inputs.crosshairToTargetOffsetX_rad
				- VisionConstants.visionTurnkI * vision.errorSum
				- VisionConstants.visionTurnkD * vision.errorDifference;
		} else if (vision.inputs.crosshairToTargetOffsetX_rad > 2.0) {
			motorPower = VisionConstants.visionTurnkP * vision.inputs.crosshairToTargetOffsetX_rad
				+ VisionConstants.visionTurnkI * vision.errorSum
				+ VisionConstants.visionTurnkD * vision.errorDifference;
		}

		// Copy Limelight.cpp stuff from last year, (PID calculating motor speed from)
		// Copy YawToTarget.cpp stuff from last year (sending information to swerve)

		// When we see a grid retroreflective, we will rotate to it and put elevator up
		switch (routine) {
			case Middle:
				System.out.println("Middle");
				break;

			case Top:
				System.out.println("Top");
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		// Copy YawToTarget.cpp stuff from last year, (stopping information to swerve)
	}

	@Override
	public boolean isFinished() {
		// Copy Limelight.cpp [check function definition] stuff from last year, (check to see if tx is within bounds)
		return false;
	}
}
