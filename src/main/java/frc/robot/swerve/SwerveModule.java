package frc.robot.swerve;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.Conversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

public class SwerveModule implements AutoCloseable {
	public int moduleNumber;
	private Rotation2d lastAngle;
	private final SwerveModuleIO io;
	private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
	private PIDController anglePIDController;

	public SwerveModule(int moduleNumber, SwerveModuleIO io) {
		this.moduleNumber = moduleNumber;
		this.io = io;

		var isReal = io.getClass() == SwerveModuleIOReal.class;
		anglePIDController = new PIDController(
			isReal ? angleKp : simAngleKp,
			isReal ? angleKi : simAngleKi,
			isReal ? angleKd : simAngleKd);
		// new TrapezoidProfile.Constraints(
		// maxAngularVelocity_radps,
		// maxAngularVelocity_radps));
		anglePIDController.enableContinuousInput(0, Conversions.twoPi);

		lastAngle = getState().angle;

		SmartDashboard.putData("SwerveModule " + moduleNumber, anglePIDController);
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
	 * Set the drive motor to the specified voltage and angle position to 0.
	 * 
	 * This is used for characterization by the FeedForwardCharacterization command.
	 *
	 * @param voltage specified voltage for the drive motor
	 */
	public void runCharacterization(double voltage) {
		setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)), true, true);
		io.setDriveMotorPercentage(voltage / 12.0);
	}

	public double getCharacterizationVelocity() {
		return io.getCharacterizationVelocity();
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
		Logger.getInstance().recordOutput("SwerveModule " + moduleNumber + "/targetAngle_rad",
			Conversions.angleModulus2pi(angle.getRadians()));
	}

	public void setDriveBrakeMode(boolean enable) {
		io.setDriveBrakeMode(enable);
	}

	@Override
	public void close() throws Exception {
		io.close();
	}
}