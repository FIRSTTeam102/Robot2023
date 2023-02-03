package frc.robot.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.littletonrobotics.junction.AutoLog;

public class VisionIO {

	@AutoLog
	public static class VisionIOInputs {
		public double tx = 0.0;
		public double ty = 0.0;
		public double ta = 0.0;
		public double tid = 0.0;

		public double botposeTranslationX = 0.0;
		public double botposeTranslationY = 0.0;
		public double botposeTranslationZ = 0.0;
		public double botposeRotationX = 0.0;
		public double botposeRotationY = 0.0;
		public double botposeRotationZ = 0.0;

		public double tv = 0.0;
	}

	private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry txEntry = table.getEntry("tx");
	private NetworkTableEntry tyEntry = table.getEntry("ty");
	private NetworkTableEntry taEntry = table.getEntry("ta");
	private NetworkTableEntry tidEntry = table.getEntry("tid");

	private NetworkTableEntry botposEntry = table.getEntry("botpose");
	private double[] botposeCache = new double[6];

	private NetworkTableEntry tclassEntry = table.getEntry("tclass");

	private NetworkTableEntry tvEntry = table.getEntry("tv");

	public void updateInputs(VisionIOInputs inputs) {
		inputs.tx = txEntry.getDouble(inputs.tx);
		inputs.ty = txEntry.getDouble(inputs.ty);
		inputs.ta = txEntry.getDouble(inputs.ta);
		inputs.tid = txEntry.getDouble(inputs.tid);

		botposeCache = botposEntry.getDoubleArray(botposeCache);
		inputs.botposeTranslationX = botposeCache[0];
		// repeat above for botpost items line 18-23

		inputs.tv = txEntry.getDouble(inputs.tv);

	}
}
