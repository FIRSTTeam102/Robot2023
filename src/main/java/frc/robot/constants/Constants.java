package frc.robot.constants;

public final class Constants {
	public static final double loopPeriod_s = org.littletonrobotics.junction.LoggedRobot.defaultPeriodSecs; // edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod

	public static final RobotMode robotMode = RobotMode.Active;

	public enum RobotMode {
		Active, Replay;
	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorConsolePort = 1;
		public static final int operatorJoystickPort = 2;

		public static final double xboxStickDeadband = 0.1;
		public static final double operatorJoystickDeadband = 0.2;
		public static final double boolTriggerThreshold = 0.3;
	}

	public static class CameraConstants {
		// must be a supported resolution or will fail
		public static final int width = 426;
		public static final int height = 240;
		public static final int fps = 15;
		public static final int compression = 80;
		// @fieldcal config exposure at comp
		public static final int exposure = -1;
	}

	public static final int pigeonId = 20;

	public static class ShuffleboardConstants {
		public static final String driveTab = "drive";
	}
}
