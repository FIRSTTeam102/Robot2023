package frc.robot.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		public double target = 0.0;

		public double crosshairToTargetOffsetX = 0.0;
		public double crosshairToTargetOffsetY = 0.0;
		public double targetArea = 0.0;

		public double botposeTranslationX = 0.0;
		public double botposeTranslationY = 0.0;
		public double botposeTranslationZ = 0.0;
		public double botposeRotationX = 0.0;
		public double botposeRotationY = 0.0;
		public double botposeRotationZ = 0.0;

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

		inputs.crosshairToTargetOffsetX = txEntry.getDouble(inputs.crosshairToTargetOffsetX);
		inputs.crosshairToTargetOffsetY = tyEntry.getDouble(inputs.crosshairToTargetOffsetY);
		inputs.targetArea = taEntry.getDouble(inputs.targetArea);

		botposeCache = botposEntry.getDoubleArray(botposeCache);
		inputs.botposeTranslationX = botposeCache[0];
		inputs.botposeTranslationY = botposeCache[1];
		inputs.botposeTranslationZ = botposeCache[2];
		inputs.botposeRotationX = botposeCache[3];
		inputs.botposeRotationY = botposeCache[4];
		inputs.botposeRotationZ = botposeCache[5];

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
