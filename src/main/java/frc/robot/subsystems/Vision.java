package frc.robot.subsystems;

import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

	private VisionIO io = new VisionIO();
	public VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

	Timer pipelineSwitchTimer = new Timer();

	public Vision() {
		setPipeline(Pipeline.AprilTag);
		setPipeline(Pipeline.Retroreflective);
		setPipeline(Pipeline.ObjectDetection);
	}

	@Override
	public void periodic() {
		io.updateInputs(visionInputs);
		Logger.getInstance().processInputs(getName(), visionInputs);
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
