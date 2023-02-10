package frc.robot.constants;

public final class Constants {
	public static final double loopPeriod_s = org.littletonrobotics.junction.LoggedRobot.defaultPeriodSecs; // edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod

	public static final RobotMode robotMode = RobotMode.ACTIVE;

	public enum RobotMode {
		ACTIVE, REPLAY;
	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
		public static final int operatorControllerPort = 1;
		public static final double stickDeadband = 0.1;
	}

	public static final int pigeonId = 20;
}
