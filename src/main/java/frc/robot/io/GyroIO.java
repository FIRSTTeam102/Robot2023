package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
	/**
	 * pitch = rotating around side-to-side axis
	 * yaw = rotating around vertical axis
	 * roll = rotating around front-to-back axis
	 */
	@AutoLog
	public static class GyroIOInputs {
		public boolean connected = false;
		public double pitch_deg = 0.0;
		public double pitch_dps = 0.0;
		public double yaw_deg = 0.0;
		public double yaw_dps = 0.0;
		public double roll_deg = 0.0;
		public double roll_dps = 0.0;
	}

	public default void updateInputs(GyroIOInputs inputs) {}

	public default void setYaw(double yaw_deg) {}

	public default void resetPitchRoll() {}
}