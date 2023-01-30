package frc.robot.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import frc.robot.BuildManager;
import frc.robot.Conversions;
import frc.robot.SendableSparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with
 * a Falcon 500 motor (TalonFX) for drive, NEO (SparkMax) for turn, and a CAN coder.
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
	private WPI_TalonFX driveMotor;
	private CANCoder angleEncoder;
	private CANSparkMax angleMotor;
	private SparkMaxPIDController anglePidController;
	private SparkMaxAbsoluteEncoder angleMotorAbsoluteEncoder;
	private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(driveKs, driveKv, driveKa);
	private double angleOffset_rad; // todo: implement

	/**
	 * Make a new SwerveModuleIOTalonFX object.
	 *
	 * @param SwerveModuleConstants module config
	 */
	public SwerveModuleIOReal(SwerveModuleConstants moduleConstants, int moduleNumber) {
		this.angleOffset_rad = moduleConstants.angleOffset_rad();

		angleEncoder = new CANCoder(moduleConstants.encoderId());
		var angleEncoderConfig = new CANCoderConfiguration();
		angleEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		angleEncoderConfig.sensorDirection = encoderInverted;
		angleEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		angleEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		angleEncoderConfig.sensorCoefficient = Conversions.twoPi / Conversions.cancoderCountsPerRotation;
		angleEncoderConfig.unitString = "rad";
		angleEncoder.configAllSettings(angleEncoderConfig);

		angleMotor = new CANSparkMax(moduleConstants.angleMotorId(), MotorType.kBrushless);
		angleMotor.restoreFactoryDefaults();
		anglePidController = angleMotor.getPIDController();
		anglePidController.setP(angleKp);
		anglePidController.setI(angleKi);
		anglePidController.setD(angleKd);
		anglePidController.setFF(angleKf);
		anglePidController.setOutputRange(-angleMaxPercentOutput, angleMaxPercentOutput);
		angleMotor.setSmartCurrentLimit(angleCurrentLimit_amp);
		// angleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		angleMotor.setInverted(angleInverted);
		angleMotor.setIdleMode(angleIdleMode);
		angleMotorAbsoluteEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		angleMotorAbsoluteEncoder.setPositionConversionFactor(angleEncoderPositionFactor_rad);
		angleMotorAbsoluteEncoder.setVelocityConversionFactor(angleEncoderVelocityFactor_radps);
		anglePidController.setFeedbackDevice(angleMotorAbsoluteEncoder);
		// wrap betwee 0 and 2pi radians
		anglePidController.setPositionPIDWrappingEnabled(true);
		anglePidController.setPositionPIDWrappingMinInput(0);
		anglePidController.setPositionPIDWrappingMaxInput(Conversions.twoPi);
		BuildManager.burnSpark(angleMotor);

		// custom shuffleboard pid controller
		var anglePIDSendable = new SendableSparkMaxPIDController(anglePidController, CANSparkMax.ControlType.kPosition,
			"Swerve turn " + moduleNumber);
		Shuffleboard.getTab("PID").add(anglePIDSendable);

		// todo: fix sync angle from cancoder -> spark
		// setFalconInternalAngleToCanCoder(); // reset to absolute position
		// angleMotorAbsoluteEncoder.setZeroOffset(angleOffset_rad);

		driveMotor = new WPI_TalonFX(moduleConstants.driveMotorId());
		var driveMotorConfig = new TalonFXConfiguration();
		driveMotorConfig.slot0.kP = driveKp;
		driveMotorConfig.slot0.kI = driveKi;
		driveMotorConfig.slot0.kD = driveKd;
		driveMotorConfig.slot0.kF = driveKf;
		driveMotorConfig.supplyCurrLimit = driveCurrentLimit;
		driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		driveMotorConfig.openloopRamp = openLoopRamp;
		driveMotorConfig.closedloopRamp = closedLoopRamp;
		driveMotor.configAllSettings(driveMotorConfig);
		driveMotor.setNeutralMode(driveNeutralMode);
		driveMotor.setSelectedSensorPosition(0);
		driveMotor.setInverted(driveInverted);
	}

	// private void setFalconInternalAngleToCanCoder() {
	// angleMotor.setSelectedSensorPosition(
	// Conversions.degreesToFalcon(getCanCoder().getDegrees(), angleGearRatio));
	// }

	private Rotation2d getCanCoder() {
		return Rotation2d.fromRadians(angleEncoder.getAbsolutePosition());
	}

	private double calculateFeedforward(double velocity) {
		double percentage = feedForward.calculate(velocity);
		return Math.min(percentage, 1.0);
	}

	/** Updates the set of inputs. */
	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// todo: sync cancoder reading to internal falcon reading
		// setFalconInternalAngleToCanCoder();

		inputs.drivePosition_deg = Conversions.falconToDegrees(
			driveMotor.getSelectedSensorPosition(), driveGearRatio);
		inputs.driveDistance_m = Conversions.falconToMeters(
			driveMotor.getSelectedSensorPosition(),
			wheelCircumference_m,
			driveGearRatio);
		inputs.driveVelocity_mps = Conversions.falconToMps(
			driveMotor.getSelectedSensorVelocity(),
			wheelCircumference_m,
			driveGearRatio);
		inputs.driveAppliedPercentage = driveMotor.getMotorOutputPercent();
		inputs.driveCurrentAmps = new double[] {driveMotor.getStatorCurrent()};
		// inputs.driveTempCelsius = new double[] {driveMotor.getTemperature()};

		inputs.angleAbsolutePosition_rad = angleEncoder.getAbsolutePosition();
		// inputs.angleAbsolutePosition_rad = angleMotorAbsoluteEncoder.getPosition();
		inputs.anglePosition_rad = angleEncoder.getPosition();
		inputs.angleVelocity_rpm = angleEncoder.getVelocity();
		inputs.angleAppliedPercentage = angleMotor.getAppliedOutput();
		inputs.angleCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
		// inputs.angleTempCelsius = new double[] {angleMotor.getMotorTemperature()};
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
	public void setAnglePosition(Rotation2d angle) {
		// todo: verify if we need radians or rotations
		anglePidController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
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
		angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	@Override
	public void close() throws Exception {
		driveMotor.DestroyObject();
		angleEncoder.DestroyObject();
		angleMotor.close();
	}
}
