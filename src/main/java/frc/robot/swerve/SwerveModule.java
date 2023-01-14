package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.maxVelocity_mps;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {
	public int moduleNumber;
	private Rotation2d lastAngle;
	private final SwerveModuleIO io;
	private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

	public SwerveModule(int moduleNumber, SwerveModuleIO io) {
		this.moduleNumber = moduleNumber;
		this.io = io;

		lastAngle = getState().angle;
	}

	/**
	 * Set this swerve module to the specified speed and angle.
	 *
	 * @param desiredState the desired state of the module
	 * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
	 * 	velocity; if false, the drive motor will set to the specified velocity using a closed-loop
	 * 	controller (PID)
	 * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
	 * 	false, the module will not rotate if the velocity is less than 1% of the max velocity
	 */
	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {
		// custom hardware-specific optimize command
		desiredState = io.optimize(desiredState, getState().angle);

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / maxVelocity_mps;
			io.setDriveMotorPercentage(percentOutput);
		} else {
			io.setDriveVelocity(desiredState.speedMetersPerSecond);
		}

		/*
		 * Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is too low.
		 * This prevents jittering if the controller isn't tuned perfectly. Perhaps more
		 * importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
		 * during pauses (e.g., multi-segmented auto paths).
		 */
		var angle = (!forceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (maxVelocity_mps * 0.05))
			? lastAngle
			: desiredState.angle;

		io.setAnglePosition(angle.getDegrees());
		lastAngle = angle;
	}

	/**
	 * Set the drive motor to the specified voltage. This is only used for characterization via the
	 * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
	 * characterization; as a result, the wheels don't need to be clamped to hold them straight.
	 *
	 * @param voltage the specified voltage for the drive motor
	 */
	public void setVoltageForCharacterization(double voltage) {
		io.setAnglePosition(0.0);
		lastAngle = Rotation2d.fromDegrees(0.0);
		io.setDriveMotorPercentage(voltage / 12.0);
	}

	public SwerveModuleState getState() {
		double velocity = inputs.driveVelocityMetersPerSec;
		Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModulePosition getPosition() {
		double distance = inputs.driveDistanceMeters;
		Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
		return new SwerveModulePosition(distance, angle);
	}

	public void updateInputs() {
		io.updateInputs(inputs);
		Logger.getInstance().processInputs("SwerveModule " + moduleNumber, inputs);
	}

	public void setDriveBrakeMode(boolean enable) {
		io.setDriveBrakeMode(enable);
	}

	public void setAngleBrakeMode(boolean enable) {
		io.setAngleBrakeMode(enable);
	}
}