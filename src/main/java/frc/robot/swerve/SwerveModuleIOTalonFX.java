package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CAN coder.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
	private TalonFX angleMotor;
	private TalonFX driveMotor;
	private CANCoder angleEncoder;
	private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
	private double angleOffsetDeg;

	/**
	 * Make a new SwerveModuleIOTalonFX object.
	 *
	 * @param SwerveModuleConstants module config
	 */
	public SwerveModuleIOTalonFX(SwerveModuleConstants moduleConstants) {
		this.angleOffsetDeg = moduleConstants.angleOffset_deg();

		angleEncoder = new CANCoder(moduleConstants.encoderId());
		angleEncoder.configFactoryDefault();
		var angleEncoderConfig = new CANCoderConfiguration();
		angleEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		angleEncoderConfig.sensorDirection = encoderInverted;
		angleEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		angleEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		angleEncoder.configAllSettings(angleEncoderConfig);

		angleMotor = new TalonFX(moduleConstants.angleMotorId());
		angleMotor.configFactoryDefault();
		angleMotor.config_kP(0, angleKp);
		angleMotor.config_kI(0, angleKi);
		angleMotor.config_kD(0, angleKd);
		angleMotor.config_kF(0, angleKf);
		// angleMotor.configSupplyCurrentLimit(angleCurrentLimit);
		angleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		angleMotor.setInverted(angleInverted);
		angleMotor.setNeutralMode(angleNeutralMode);
		setFalconInternalAngleToCanCoder(); // reset to absolute position

		driveMotor = new TalonFX(moduleConstants.driveMotorId());
		driveMotor.configFactoryDefault();
		driveMotor.config_kP(0, driveKp);
		driveMotor.config_kI(0, driveKi);
		driveMotor.config_kD(0, driveKd);
		driveMotor.config_kF(0, driveKf);
		// driveMotor.configSupplyCurrentLimit(driveCurrentLimit);
		driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		driveMotor.configOpenloopRamp(openLoopRamp);
		driveMotor.configClosedloopRamp(closedLoopRamp);
		driveMotor.setInverted(driveInverted);
		driveMotor.setNeutralMode(driveNeutralMode);
		driveMotor.setSelectedSensorPosition(0);
	}

	private void setFalconInternalAngleToCanCoder() {
		angleMotor.setSelectedSensorPosition(
			Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffsetDeg, angleGearRatio));
	}

	private Rotation2d getCanCoder() {
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
	}

	private double calculateFeedforward(double velocity) {
		double percentage = feedForward.calculate(velocity);
		return Math.min(percentage, 1.0);
	}

	/** Updates the set of inputs. */
	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// sync cancoder reading to internal falcon reading
		setFalconInternalAngleToCanCoder();

		inputs.drivePositionDeg = Conversions.falconToDegrees(
			driveMotor.getSelectedSensorPosition(), driveGearRatio);
		inputs.driveDistanceMeters = Conversions.falconToMeters(
			driveMotor.getSelectedSensorPosition(),
			wheelCircumference_m,
			driveGearRatio);
		inputs.driveVelocityMetersPerSec = Conversions.falconToMps(
			driveMotor.getSelectedSensorVelocity(),
			wheelCircumference_m,
			driveGearRatio);
		inputs.driveAppliedPercentage = driveMotor.getMotorOutputPercent();
		inputs.driveCurrentAmps = new double[] {driveMotor.getStatorCurrent()};
		// inputs.driveTempCelsius = new double[] {driveMotor.getTemperature()};

		inputs.angleAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
		inputs.anglePositionDeg = Conversions.falconToDegrees(
			angleMotor.getSelectedSensorPosition(), angleGearRatio);
		inputs.angleVelocityRevPerMin = Conversions.falconToRpm(
			angleMotor.getSelectedSensorVelocity(), angleGearRatio);
		inputs.angleAppliedPercentage = angleMotor.getMotorOutputPercent();
		inputs.angleCurrentAmps = new double[] {angleMotor.getStatorCurrent()};
		// inputs.angleTempCelsius = new double[] {angleMotor.getTemperature()};
	}

	/** Run the drive motor at the specified percentage of full power. */
	@Override
	public void setDriveMotorPercentage(double percentage) {
		driveMotor.set(ControlMode.PercentOutput, percentage);
	}

	/** Run the drive motor at the specified velocity. */
	@Override
	public void setDriveVelocity(double velocity) {
		double ticksPerSecond = Conversions.mpsToFalcon(
			velocity,
			wheelCircumference_m,
			driveGearRatio);
		driveMotor.set(
			ControlMode.Velocity,
			ticksPerSecond,
			DemandType.ArbitraryFeedForward,
			calculateFeedforward(velocity));
	}

	/** Run the turn motor to the specified angle. */
	@Override
	public void setAnglePosition(double degrees) {
		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(degrees, angleGearRatio));
	}

	/** Enable or disable brake mode on the drive motor. */
	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
	}

	/** Enable or disable brake mode on the turn motor. */
	@Override
	public void setAngleBrakeMode(boolean enable) {
		// always leave the angle motor in coast mode
		angleMotor.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * Minimize the change in heading the desired swerve module state would require by potentially
	 * reversing the direction the wheel spins. Customized from WPILib's version to include placing
	 * in appropriate scope for CTRE onboard control.
	 *
	 * @param desiredState desired state
	 * @param currentAngle current module angle
	 */
	@Override
	public SwerveModuleState optimize(SwerveModuleState desiredState,
		Rotation2d currentAngle) {
		double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(),
			desiredState.angle.getDegrees());
		double targetSpeed = desiredState.speedMetersPerSecond;
		double delta = targetAngle - currentAngle.getDegrees();
		if (Math.abs(delta) > 90) {
			targetSpeed = -targetSpeed;
			targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
		}
		return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
	}

	/**
	 * @param scopeReference current angle
	 * @param newAngle target angle
	 * @return closest angle within scope
	 */
	private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
		double lowerBound;
		double upperBound;
		double lowerOffset = scopeReference % 360;
		if (lowerOffset >= 0) {
			lowerBound = scopeReference - lowerOffset;
			upperBound = scopeReference + (360 - lowerOffset);
		} else {
			upperBound = scopeReference - lowerOffset;
			lowerBound = scopeReference - (360 + lowerOffset);
		}
		while (newAngle < lowerBound) {
			newAngle += 360;
		}
		while (newAngle > upperBound) {
			newAngle -= 360;
		}
		if (newAngle - scopeReference > 180) {
			newAngle -= 360;
		} else if (newAngle - scopeReference < -180) {
			newAngle += 360;
		}
		return newAngle;
	}
}
