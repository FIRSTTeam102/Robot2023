package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Conversions;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import org.littletonrobotics.junction.Logger;

public class SwerveModule implements AutoCloseable {
	public int moduleNumber;
	private Rotation2d lastAngle;
	private final SwerveModuleIO io;
	private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
	private ProfiledPIDController anglePIDController = new ProfiledPIDController(angleKp, angleKi, angleKd,
		new TrapezoidProfile.Constraints(
			maxAngularVelocity_radps,
			maxAngularVelocity_radps)); // fixme:

	public SwerveModule(int moduleNumber, SwerveModuleIO io) {
		this.moduleNumber = moduleNumber;
		this.io = io;

		anglePIDController.enableContinuousInput(0, Conversions.twoPi);

		lastAngle = getState().angle;
	}

	private SwerveModuleState optimizedState = new SwerveModuleState();
	private SwerveModuleState desiredState = new SwerveModuleState();
	private boolean isOpenLoop = false;
	private boolean forceAngle = false;

	/**
	 * Set this swerve module to the specified speed and angle.
	 *
	 * @param desiredState the desired state of the module
	 * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
	 * 	velocity; if false, the drive motor will set to the specified velocity using a closed-loop
	 * 	controller (PID)
	 * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
	 * 	false, the module will not rotate if the velocity is too low
	 */
	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {
		this.desiredState = desiredState;
		this.isOpenLoop = isOpenLoop;
		this.forceAngle = forceAngle;
	}

	/**
	 * Set the drive motor to the specified voltage. This is only used for characterization via the
	 * FeedForwardCharacterization command. The module will be set to 0 throughout the
	 * characterization; as a result, the wheels don't need to be clamped to hold them straight.
	 *
	 * @param voltage the specified voltage for the drive motor
	 */
	public void setVoltageForCharacterization(double voltage) {
		lastAngle = Rotation2d.fromRadians(0.0);
		// io.setAnglePosition(lastAngle); // fixme:
		io.setDriveMotorPercentage(voltage / 12.0);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			inputs.driveVelocity_mps,
			Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			inputs.driveDistance_m,
			Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.getInstance().processInputs("SwerveModule " + moduleNumber, inputs);

		optimizedState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(inputs.angleAbsolutePosition_rad));

		/*
		 * Unless the angle is forced (like X-stance), don't rotate if speed is too low.
		 * This prevents jittering if the controller isn't tuned perfectly.
		 * It also allows for smooth repeated movement as the wheel direction doesn't reset during pauses.
		 */
		var angle = (!forceAngle && Math.abs(optimizedState.speedMetersPerSecond) <= (maxVelocity_mps * 0.05))
			? lastAngle
			: optimizedState.angle;

		// run turn
		// io.setAnglePosition(angle);
		io.setAngleVoltage(
			anglePIDController.calculate(inputs.angleAbsolutePosition_rad,
				Conversions.angleModulus2pi(angle.getRadians())));

		// update velocity based on angle error
		optimizedState.speedMetersPerSecond *= Math.cos(anglePIDController.getPositionError());

		// run drive
		if (isOpenLoop) {
			double percentOutput = optimizedState.speedMetersPerSecond / maxVelocity_mps;
			io.setDriveMotorPercentage(percentOutput);
		} else {
			io.setDriveVelocity(optimizedState.speedMetersPerSecond);
		}

		lastAngle = angle;
	}

	public void setDriveBrakeMode(boolean enable) {
		io.setDriveBrakeMode(enable);
	}

	@Override
	public void close() throws Exception {
		io.close();
	}
}