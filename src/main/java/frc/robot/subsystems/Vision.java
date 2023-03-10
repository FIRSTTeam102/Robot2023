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
		// every 0.02s, updating networktable variables
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		Lights.setStatus(Lights.Group.LMAprilTag, switch ((int) inputs.targetAprilTag) {
			case 1, 6 -> Lights.Status.Right; // right
			case 2, 7 -> Lights.Status.Center; // center
			case 3, 8 -> Lights.Status.Left; // left
			case 4, 5 -> Lights.Status.LeftRight; // substation
			default -> Lights.Status.None;
		});
		Lights.setStatus(Lights.Group.LMRetroreflective, (inputs.pipeline == Pipeline.Retroreflective.value &&
			inputs.target)
				? (inputs.crosshairToTargetErrorX_rad < 0 ? Lights.Status.Left
					: inputs.crosshairToTargetErrorX_rad > 0 ? Lights.Status.Right
						: Lights.Status.All)
				: Lights.Status.None);
		Lights.setStatus(Lights.Group.Coral.value,
			inputs.pipeline == Pipeline.GamePiece.value ? (int) inputs.targetObject : Lights.Status.None.value);
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
			pipelineSwitchTimer.reset();
			lastPipeline = inputs.pipeline;
			return true;
		}
		pipelineSwitchTimer.start();
		return false;
	}
}
