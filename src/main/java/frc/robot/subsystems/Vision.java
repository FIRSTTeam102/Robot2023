package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private VisionIO io = new VisionIO();
	public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

	Timer pipelineSwitchTimer = new Timer();

	public Vision() {
		setPipeline(Pipeline.AprilTag);
		setPipeline(Pipeline.Retroreflective);
		setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void periodic() {

		// Every 0.02s, updating networktable variables
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		// Every 0.02s, updating pose2d
		if (inputs.pipeline == Pipeline.AprilTag.value && isPipelineReady())
			RobotContainer.getInstance().swerve.addVisionMeasurement(
				new Pose2d(inputs.botpose_fieldTranslationX_m, inputs.botpose_fieldTranslationY_m,
					new Rotation2d(inputs.botpose_fieldRotationZ_rad)));
	}

	// Creates set pipeline
	public void setPipeline(Pipeline pipeline) {
		pipelineSwitchTimer.reset();
		pipelineSwitchTimer.start();
		io.setPipeline(pipeline);
	}

	// Allows AprilTag commands to begin after pipeline switch time error
	public boolean isPipelineReady() {
		if (pipelineSwitchTimer.hasElapsed(0.5)) {
			pipelineSwitchTimer.stop();
			return true;
		}
		return false;
	}
}
