package frc.robot.io;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class GyroIOSim implements GyroIO {
	private RobotContainer robo;

	public GyroIOSim(RobotContainer robo) {
		this.robo = robo;
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		if (robo.swerve == null)
			return;
		inputs.connected = true;
		inputs.yaw_dps = Math
			.toDegrees(robo.swerve.kinematics.toChassisSpeeds(robo.swerve.moduleStates).omegaRadiansPerSecond);
		inputs.yaw_deg += inputs.yaw_dps * Constants.loopPeriod_s;
		// todo: sim other rotations if needed
	}

	@Override
	public void setYaw(double yaw_deg) {
		// todo:
	}
}
