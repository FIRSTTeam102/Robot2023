package frc.robot.subsystems;

import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.Pipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private VisionIO io = new VisionIO();
	public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

	Timer pipelineSwitchTimer = new Timer();

	public Vision() {
		setPipeline(Pipeline.AprilTag);
	}

	@Override
	public void periodic() {

		// Every 0.02s, updating networktable variables
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);
	}

	// Creates set pipeline
	public void setPipeline(Pipeline pipeline) {
		pipelineSwitchTimer.reset();
		pipelineSwitchTimer.start();
		io.setPipeline(pipeline);
	}

	long lastPipeline = 0;

	// Allows AprilTag commands to begin after pipeline switch time error
	public boolean isPipelineReady() {
		if (lastPipeline == inputs.pipeline || pipelineSwitchTimer.hasElapsed(0.9)) {
			pipelineSwitchTimer.stop();
			lastPipeline = inputs.pipeline;
			return true;
		}
		return false;
	}
}
