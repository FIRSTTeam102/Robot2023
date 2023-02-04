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

		public double botposeTranslationX_m = 0.0;
		public double botposeTranslationY_m = 0.0;
		public double botposeTranslationZ_m = 0.0;
		public double botposeRotationX_rad = 0.0;
		public double botposeRotationY_rad = 0.0;
		public double botposeRotationZ_rad = 0.0;

		public double targetAprilTag = 0.0;
		public double targetObject = 0.0;

		public int pipeline = 0;
	}

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	private NetworkTableEntry tvEntry = table.getEntry("tv");

	private NetworkTableEntry txEntry = table.getEntry("tx");
	private NetworkTableEntry tyEntry = table.getEntry("ty");
	private NetworkTableEntry taEntry = table.getEntry("ta");

	private NetworkTableEntry botposEntry = table.getEntry("botpose");
	private double[] botposeCache = new double[6];

	private NetworkTableEntry tidEntry = table.getEntry("tid");
	private NetworkTableEntry tclassEntry = table.getEntry("tclass");
	private NetworkTableEntry pipelineEntry = table.getEntry("pipeline");

	public void updateInputs(VisionIOInputs inputs) {
		inputs.target = tvEntry.getDouble(inputs.target);

		inputs.crosshairToTargetOffsetX_rad = Math.toRadians(txEntry.getDouble(0));
		inputs.crosshairToTargetOffsetY_rad = Math.toRadians(tyEntry.getDouble(0));
		inputs.targetArea = taEntry.getDouble(inputs.targetArea);

		botposeCache = botposEntry.getDoubleArray(botposeCache);
		inputs.botposeTranslationX_m = botposeCache[0];
		inputs.botposeTranslationY_m = botposeCache[1];
		inputs.botposeTranslationZ_m = botposeCache[2];
		inputs.botposeRotationX_rad = Math.toRadians(botposeCache[3]);
		inputs.botposeRotationY_rad = Math.toRadians(botposeCache[4]);
		inputs.botposeRotationZ_rad = Math.toRadians(botposeCache[5]);

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
