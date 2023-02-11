package frc.robot.io;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;

public class GyroIOSim implements GyroIO {
	public GyroIOSim() {}

	Swerve swerve;

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		if (swerve == null)
			swerve = RobotContainer.getInstance().swerve;
		if (swerve == null)
			return;

		inputs.connected = true;
		inputs.yaw_dps = Math
			.toDegrees(swerve.kinematics.toChassisSpeeds(swerve.moduleStates).omegaRadiansPerSecond);
		inputs.yaw_deg += inputs.yaw_dps * Constants.loopPeriod_s;
		// todo: sim other rotations if needed
	}

	@Override
	public void setYaw(double yaw_deg) {
		// todo:
	}
}
