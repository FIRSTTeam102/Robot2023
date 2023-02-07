package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.math.Vector2;
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

	private double zGoTo;
	private double xGoTo;

	public Vector2 getGoToPostition() {
		Vector2 position = new Vector2(zGoTo, xGoTo);
		return;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		if (inputs.pipeline == Pipeline.AprilTag.value && isPipelineReady())
			RobotContainer.getInstance().swerve.addVisionMeasurement(
				new Pose2d(inputs.botpose_targetspaceTranslationX_m, inputs.botpose_targetspaceTranslationY_m,
					new Rotation2d(inputs.botpose_targetspaceRotationZ_rad)));
	}

	public void setPipeline(Pipeline pipeline) {
		pipelineSwitchTimer.reset();
		pipelineSwitchTimer.start();
		io.setPipeline(pipeline);
	}

	public boolean isPipelineReady() {
		if (pipelineSwitchTimer.hasElapsed(0.5)) {
			pipelineSwitchTimer.stop();
			return true;
		}
		return false;
	}
}
