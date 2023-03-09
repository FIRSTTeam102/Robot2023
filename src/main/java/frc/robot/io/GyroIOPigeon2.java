package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 gyro;
	private final double[] ypr_deg = new double[3];
	private final double[] xyz_dps = new double[3];

	public GyroIOPigeon2(int deviceNumber) {
		gyro = new Pigeon2(deviceNumber);
		gyro.configFactoryDefault();
		gyro.configMountPose(192.807, 0, 0);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		gyro.getRawGyro(xyz_dps);
		gyro.getYawPitchRoll(ypr_deg);
		inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
		inputs.yaw_deg = ypr_deg[0];
		inputs.yaw_dps = xyz_dps[0];
		inputs.pitch_rad = MathUtil.angleModulus(Units.degreesToRadians(ypr_deg[1]));
		inputs.pitch_radps = Units.degreesToRadians(xyz_dps[1]);
		inputs.roll_rad = MathUtil.angleModulus(Units.degreesToRadians(ypr_deg[2]));
		inputs.roll_radps = Units.degreesToRadians(xyz_dps[2]);
		inputs.temperature_C = gyro.getTemp();
	}

	@Override
	public void setYaw(double yaw_deg) {
		gyro.setYaw(yaw_deg);
	}

	@Override
	public void resetPitchRoll() {
		gyro.configMountPosePitch(gyro.getPitch());
		gyro.configMountPoseRoll(gyro.getRoll());
	}
}
