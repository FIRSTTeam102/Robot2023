package frc.robot.io;

import frc.robot.constants.FieldConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		/* fieldvision */
		public long fieldVisionPipeline = 0;
		public boolean fieldVisionTarget = false;
		public long fieldVisionTargetAprilTag = 0;

		public double fieldVisionCrosshairToTargetErrorX_rad = 0.0;
		public double fieldVisionCrosshairToTargetErrorY_rad = 0.0;
		public double fieldVisionTargetArea = 0.0;

		public double fieldVisionBotpose_TargetspaceTranslationX_m = 0.0;
		public double fieldVisionBotpose_TargetspaceTranslationY_m = 0.0;
		public double fieldVisionBotpose_TargetspaceTranslationZ_m = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationX_rad = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationY_rad = 0.0;
		public double fieldVisionBotpose_TargetspaceRotationZ_rad = 0.0;

		public double fieldVisionBotpose_FieldspaceTranslationX_m = 0.0;
		public double fieldVisionBotpose_FieldspaceTranslationY_m = 0.0;
		public double fieldVisionBotpose_FieldspaceTranslationZ_m = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationX_rad = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationY_rad = 0.0;
		public double fieldVisionBotpose_FieldspaceRotationZ_rad = 0.0;

		public double fieldVisionBotpose_Latency_s = 0.0;

		/* gamepiecevision */
		public long gamePieceVisionPipeline = 0;
		public boolean gamePieceVisionTarget = false;
		public String gamePieceVisionTargetObjectClass = "";

		public double gamePieceVisionCrosshairToTargetErrorX_rad = 0.0;
		public double gamePieceVisionCrosshairToTargetErrorY_rad = 0.0;
		public double gamePieceVisionTargetArea = 0.0;
	}

	/* fieldvision */
	private NetworkTable tableFieldVision = NetworkTableInstance.getDefault().getTable("limelight-fv");

	private NetworkTableEntry pipelineEntryFieldVision = tableFieldVision.getEntry("pipeline");
	private NetworkTableEntry tvEntryFieldVision = tableFieldVision.getEntry("tv");
	private NetworkTableEntry tidEntryFieldVision = tableFieldVision.getEntry("tid");

	private NetworkTableEntry txEntryFieldVision = tableFieldVision.getEntry("tx");
	private NetworkTableEntry tyEntryFieldVision = tableFieldVision.getEntry("ty");
	private NetworkTableEntry taEntryFieldVision = tableFieldVision.getEntry("ta");

	private NetworkTableEntry botpose_targetspaceEntryFieldVision = tableFieldVision.getEntry("botpose_targetspace");
	private double[] botpose_targetspaceCacheFieldVision = new double[6];

	private NetworkTableEntry botpose_fieldspaceEntryFieldVision = tableFieldVision.getEntry("botpose");
	private double[] botpose_fieldspaceCacheFieldVision = new double[7];

	/* gamepiecevision */
	private NetworkTable tableGamePieceVision = NetworkTableInstance.getDefault().getTable("limelight-gpv");

	private NetworkTableEntry pipelineEntryGamePieceVision = tableGamePieceVision.getEntry("pipeline");
	private NetworkTableEntry tvEntryGamePieceVision = tableGamePieceVision.getEntry("tv");
	private NetworkTableEntry tclassEntryGamePieceVision = tableGamePieceVision.getEntry("tclass");

	private NetworkTableEntry txEntryGamePieceVision = tableGamePieceVision.getEntry("tx");
	private NetworkTableEntry tyEntryGamePieceVision = tableGamePieceVision.getEntry("ty");
	private NetworkTableEntry taEntryGamePieceVision = tableGamePieceVision.getEntry("ta");

	public void updateInputs(VisionIOInputs inputs) {
		/* fieldvision */
		inputs.fieldVisionPipeline = pipelineEntryFieldVision.getNumber(inputs.fieldVisionPipeline).intValue();
		inputs.fieldVisionTarget = tvEntryFieldVision.getDouble(0) == 1;
		inputs.fieldVisionTargetAprilTag = tidEntryFieldVision.getNumber(inputs.fieldVisionTargetAprilTag).intValue();

		inputs.fieldVisionCrosshairToTargetErrorX_rad = Math.toRadians(txEntryFieldVision.getDouble(0));
		inputs.fieldVisionCrosshairToTargetErrorY_rad = Math.toRadians(tyEntryFieldVision.getDouble(0));
		inputs.fieldVisionTargetArea = taEntryFieldVision.getDouble(inputs.fieldVisionTargetArea);

		botpose_targetspaceCacheFieldVision = botpose_targetspaceEntryFieldVision
			.getDoubleArray(botpose_targetspaceCacheFieldVision);
		if (botpose_targetspaceCacheFieldVision.length > 0) {
			inputs.fieldVisionBotpose_TargetspaceTranslationX_m = botpose_targetspaceCacheFieldVision[0];
			inputs.fieldVisionBotpose_TargetspaceTranslationY_m = botpose_targetspaceCacheFieldVision[1];
			inputs.fieldVisionBotpose_TargetspaceTranslationZ_m = botpose_targetspaceCacheFieldVision[2];
			inputs.fieldVisionBotpose_TargetspaceRotationX_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[3]);
			inputs.fieldVisionBotpose_TargetspaceRotationY_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[4]);
			inputs.fieldVisionBotpose_TargetspaceRotationZ_rad = Math.toRadians(botpose_targetspaceCacheFieldVision[5]);
		} else
			DriverStation.reportWarning("Invalid botpose array from limelight", true);

		botpose_fieldspaceCacheFieldVision = botpose_fieldspaceEntryFieldVision
			.getDoubleArray(botpose_fieldspaceCacheFieldVision);
		inputs.fieldVisionBotpose_FieldspaceTranslationX_m = botpose_fieldspaceCacheFieldVision[0]
			+ FieldConstants.fieldLengthX_m / 2;
		inputs.fieldVisionBotpose_FieldspaceTranslationY_m = botpose_fieldspaceCacheFieldVision[1]
			+ FieldConstants.fieldLengthY_m / 2;
		inputs.fieldVisionBotpose_FieldspaceTranslationZ_m = botpose_fieldspaceCacheFieldVision[2];
		inputs.fieldVisionBotpose_FieldspaceRotationX_rad = Math.toRadians(botpose_fieldspaceCacheFieldVision[3]);
		inputs.fieldVisionBotpose_FieldspaceRotationY_rad = Math.toRadians(botpose_fieldspaceCacheFieldVision[4]);
		inputs.fieldVisionBotpose_FieldspaceRotationZ_rad = Math.toRadians(botpose_fieldspaceCacheFieldVision[5]);
		inputs.fieldVisionBotpose_Latency_s = botpose_fieldspaceCacheFieldVision[6] / 1000;

		/* gamepiecevision */
		inputs.gamePieceVisionPipeline = pipelineEntryGamePieceVision.getNumber(inputs.gamePieceVisionPipeline).intValue();
		inputs.gamePieceVisionTarget = tvEntryGamePieceVision.getDouble(0) == 1;
		inputs.gamePieceVisionTargetObjectClass = tclassEntryGamePieceVision.getString("");

		inputs.gamePieceVisionCrosshairToTargetErrorX_rad = Math.toRadians(txEntryGamePieceVision.getDouble(0));
		inputs.gamePieceVisionCrosshairToTargetErrorY_rad = Math.toRadians(tyEntryGamePieceVision.getDouble(0));
		inputs.gamePieceVisionTargetArea = taEntryGamePieceVision.getDouble(inputs.gamePieceVisionTargetArea);
	}

	/* fieldvision */
	public enum FieldVisionPipeline {
		AprilTag(0), Retroreflective(1);

		public final int value;

		FieldVisionPipeline(int value) {
			this.value = value;
		}
	};

	public void setFieldVisionPipeline(FieldVisionPipeline pipeline) {
		pipelineEntryFieldVision.setNumber(pipeline.value);
	}

	/* gamepiecevision */
	public enum GamePieceVisionPipeline {
		GamePiece(0);

		public final int value;

		GamePieceVisionPipeline(int value) {
			this.value = value;
		}
	};

	public void setGamePieceVisionPipeline(GamePieceVisionPipeline pipeline) {
		pipelineEntryGamePieceVision.setNumber(pipeline.value);
	}
}
