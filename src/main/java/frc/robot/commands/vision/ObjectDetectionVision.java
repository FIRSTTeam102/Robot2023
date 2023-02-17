package frc.robot.commands.vision;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ObjectDetectionVision extends CommandBase {
	private Routine routine;
	private Vision vision;
	private Elevator elevator;
	private Arm arm;
	private Swerve swerve;
	private double robotRotateVelocity_mpers;
	private double robotTranslateVelocity_mpers;

	public enum Routine {
		Ground
	}

	public ObjectDetectionVision(Routine routine, Vision vision, Elevator elevator, Arm arm, Swerve swerve) {
		this.routine = routine;
		this.vision = vision;
		this.elevator = elevator;
		this.arm = arm;
		this.swerve = swerve;
	}

	@Override
	public void initialize() {
		// Sets pipeline to ObjectDetection
		vision.setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void execute() {
		if (!vision.isPipelineReady())
			return;

		// Outputs a robotRotateVelocity_mpers that updates every 0.02s for the motor. rotateKp, rotateKi, rotateKd must be
		// tuned and can not be calculated in a spreadsheet as vision.rotateErrorIntegral and vision.rotateErrorDerivative
		// are based on the last 0.02s VisionConstants.rotatekP * vision.inputs.crosshairToTargetOffsetX_rad. We will not
		// know what this data will be for the last 0.02s on a spreadsheet because we do not know what the
		// robotRotateVelocity_mpers was the last 0.02s.
		if (vision.inputs.crosshairToTargetErrorX_rad < -VisionConstants.crosshairObjectBoundRotateX_rad) {
			robotRotateVelocity_mpers = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				- VisionConstants.rotateKd * vision.translateErrorDerivative;
		} else if (vision.inputs.crosshairToTargetErrorX_rad > VisionConstants.crosshairObjectBoundRotateX_rad) {
			robotRotateVelocity_mpers = VisionConstants.rotateKp * vision.inputs.crosshairToTargetErrorX_rad
				+ VisionConstants.rotateKi * vision.translateErrorIntegral
				+ VisionConstants.rotateKd * vision.translateErrorDerivative;
		}

		// Outputs a robotTranslateVelocity_mpers that updates every 0.02s for the motor. translateKp, translateKi,
		// translateKd must be tuned and can not be calculated in a spreadsheet as vision.translateErrorIntegral and
		// vision.translateErrorDerivative are based on the last 0.02s VisionConstants.translatekP *
		// vision.inputs.botpose_targetspaceTranslationZ_m. We will not know what this data will be for the last 0.02s on a
		// spreadsheet because we do not know what the robotTranslateVelocity_mpers was the last 0.02s.
		if (vision.inputs.botpose_targetspaceTranslationZ_m < VisionConstants.crosshairObjectBoundTranslateZ_m) {
			robotTranslateVelocity_mpers = VisionConstants.translateKp * vision.inputs.botpose_targetspaceTranslationZ_m
				- VisionConstants.translateKd * vision.translateErrorDerivative;
		}

		// When we see a ground objectdetection, we will translate and rotate to it and put elevator down and put scissor
		// arm out
		switch (routine) {
			case Ground:
				System.out.println("Translate: " + vision.inputs.botpose_targetspaceRotationZ_rad + " Rotate: "
					+ vision.inputs.crosshairToTargetErrorX_rad + "Swerve Object + Elevator Object + Arm Object + Arm Object");
				swerve.drive(new Translation2d(0, robotTranslateVelocity_mpers), robotRotateVelocity_mpers);
				elevator.setPosition(ElevatorConstants.resetHeight_m);
				arm.setPosition(ArmConstants.groundObjectLength_m);
				break;
		}
	}

	// Stop swerve isFinished
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		// Sets pipeline back to Apriltag
		vision.setPipeline(Pipeline.AprilTag);
	}

	// Feedback loop for PID
	@Override
	public boolean isFinished() {
		return ((-VisionConstants.crosshairObjectBoundRotateX_rad < vision.inputs.crosshairToTargetErrorX_rad)
			&& (vision.inputs.crosshairToTargetErrorX_rad < VisionConstants.crosshairObjectBoundRotateX_rad))
			&& (vision.inputs.botpose_targetspaceRotationZ_rad < VisionConstants.crosshairObjectBoundTranslateZ_m);
	}
}
