package frc.robot.subsystems;

import frc.robot.io.VisionIO;
import frc.robot.io.VisionIO.FieldVisionPipeline;
import frc.robot.io.VisionIO.GamePieceString;
import frc.robot.io.VisionIO.GamePieceVisionPipeline;
import frc.robot.io.VisionIOInputsAutoLogged;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
	private VisionIO io = new VisionIO();
	public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

	public Vision() {
		setFieldVisionPipeline(FieldVisionPipeline.AprilTag);
		setGamePieceVisionPipeline(GamePieceVisionPipeline.GamePiece);
	}

	@Override
	public void periodic() {
		// every 0.02s, updating networktable variables
		io.updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		Lights.setStatus(Lights.Group.LMAprilTag, switch ((int) inputs.fieldVisionTargetAprilTag) {
			case 1, 6 -> Lights.Status.Right; // right grid
			case 2, 7 -> Lights.Status.Center; // center grid
			case 3, 8 -> Lights.Status.Left; // left grid
			case 4, 5 -> Lights.Status.LeftRight; // double substation
			default -> Lights.Status.None;
		});
		Lights.setStatus(Lights.Group.LMRetroreflective,
			(inputs.fieldVisionPipeline == FieldVisionPipeline.RetroReflective.value &&
				inputs.fieldVisionTarget)
					? (inputs.fieldVisionCrosshairToTargetErrorX_rad < 0 ? Lights.Status.Left
						: inputs.fieldVisionCrosshairToTargetErrorX_rad > 0 ? Lights.Status.Right
							: Lights.Status.All)
					: Lights.Status.None);
		Lights.setStatus(Lights.Group.Coral, switch (inputs.gamePieceVisionTargetObjectClass) {
			case GamePieceString.Cube -> Lights.Status.Left;
			case GamePieceString.Cone -> Lights.Status.Right;
			case GamePieceString.Merge -> Lights.Status.LeftRight;
			default -> Lights.Status.None;
		});
	}

	// Creates setFieldVisionPipeline
	public void setFieldVisionPipeline(FieldVisionPipeline pipeline) {
		fieldVisionPipelineSwitchTimer.reset();
		fieldVisionPipelineSwitchTimer.start();
		io.setFieldVisionPipeline(pipeline);
	}

	// Creates setGamePieceVisionPipeline
	public void setGamePieceVisionPipeline(GamePieceVisionPipeline pipeline) {
		// gamePieceVisionPipelineSwitchTimer.reset();
		// gamePieceVisionPipelineSwitchTimer.start();
		io.setGamePieceVisionPipeline(pipeline);
	}

	Timer fieldVisionPipelineSwitchTimer = new Timer();
	// Timer gamePieceVisionPipelineSwitchTimer = new Timer();
	long lastFieldVisionPipeline = 0;
	// long lastGamePieceVisionPipeline = 0;

	// Allows AprilTag commands to begin after pipeline switch time error
	public boolean isPipelineReady() {
		if (lastFieldVisionPipeline == inputs.fieldVisionPipeline || fieldVisionPipelineSwitchTimer.hasElapsed(0.8)) {
			fieldVisionPipelineSwitchTimer.stop();
			fieldVisionPipelineSwitchTimer.reset();
			lastFieldVisionPipeline = inputs.fieldVisionPipeline;
			return true;
		}
		fieldVisionPipelineSwitchTimer.start();
		return false;
	}
}
