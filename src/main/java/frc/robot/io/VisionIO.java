package frc.robot.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		public double target = 0.0;

		public double crosshairToTargetOffsetX_rad = 0.0;
		public double crosshairToTargetOffsetY_rad = 0.0;
		public double targetArea = 0.0;

		public double botpose_targetspaceTranslationX_m = 0.0;
		public double botpose_targetspaceTranslationY_m = 0.0;
		public double botpose_targetspaceTranslationZ_m = 0.0;
		public double botpose_targetspaceRotationX_rad = 0.0;
		public double botpose_targetspaceRotationY_rad = 0.0;
		public double botpose_targetspaceRotationZ_rad = 0.0;

		public double targetAprilTag = 0.0;
		public double targetObject = 0.0;

		public long pipeline = 0;
	}

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	private NetworkTableEntry tvEntry = table.getEntry("tv");

	private NetworkTableEntry txEntry = table.getEntry("tx");
	private NetworkTableEntry tyEntry = table.getEntry("ty");
	private NetworkTableEntry taEntry = table.getEntry("ta");

	private NetworkTableEntry botpose_targetspaceEntry = table.getEntry("botpose_targetspace");
	private double[] botpose_targetspaceCache = new double[6];

	private NetworkTableEntry tidEntry = table.getEntry("tid");
	private NetworkTableEntry tclassEntry = table.getEntry("tclass");
	private NetworkTableEntry pipelineEntry = table.getEntry("pipeline");

	public void updateInputs(VisionIOInputs inputs) {
		inputs.target = tvEntry.getDouble(inputs.target);

		inputs.crosshairToTargetOffsetX_rad = Math.toRadians(txEntry.getDouble(0));
		inputs.crosshairToTargetOffsetY_rad = Math.toRadians(tyEntry.getDouble(0));
		inputs.targetArea = taEntry.getDouble(inputs.targetArea);

		botpose_targetspaceCache = botpose_targetspaceEntry.getDoubleArray(botpose_targetspaceCache);
		inputs.botpose_targetspaceTranslationX_m = botpose_targetspaceCache[0];
		inputs.botpose_targetspaceTranslationY_m = botpose_targetspaceCache[1];
		inputs.botpose_targetspaceTranslationZ_m = botpose_targetspaceCache[2];
		inputs.botpose_targetspaceRotationX_rad = Math.toRadians(botpose_targetspaceCache[3]);
		inputs.botpose_targetspaceRotationY_rad = Math.toRadians(botpose_targetspaceCache[4]);
		inputs.botpose_targetspaceRotationZ_rad = Math.toRadians(botpose_targetspaceCache[5]);

		inputs.targetAprilTag = tidEntry.getDouble(inputs.targetAprilTag);
		inputs.targetObject = tclassEntry.getDouble(inputs.targetObject);

		inputs.pipeline = pipelineEntry.getNumber(inputs.pipeline).intValue();
	}

	public enum Pipeline {
		AprilTag(0), Retroreflective(1), ObjectDetection(2);

		public final int value;

		Pipeline(int value) {
			this.value = value;
		}
	};

	public void setPipeline(Pipeline pipeline) {
		pipelineEntry.setNumber(pipeline.value);
	}
}
