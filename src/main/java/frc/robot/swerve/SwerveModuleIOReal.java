package frc.robot.swerve;

import static frc.robot.constants.SwerveConstants.*;

import frc.robot.util.BuildManager;
import frc.robot.util.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;

/**
 * Implementation of the SwerveModuleIO interface for MK4i Swerve Modules with
 * a Falcon 500 motor (TalonFX) for drive, NEO (SparkMax) for turn, and a CANcoder
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
	private WPI_TalonFX driveMotor;
	private CANCoder angleCancoder;
	private CANSparkMax angleMotor;
	private SparkMaxPIDController angleSparkPidController;
	// private SparkMaxAbsoluteEncoder angleMotorAbsoluteEncoder;
	// private RelativeEncoder angleMotorRelativeEncoder;
	private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveKs, driveKv);
	private double angleOffset_rad;

	// private SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(turnKs, turnKv);

	/**
	 * @param moduleConstants module config
	 * @param moduleNumber module number used for identification
	 */
	public SwerveModuleIOReal(SwerveModuleConstants moduleConstants, int moduleNumber) {
		this.angleOffset_rad = moduleConstants.angleOffset_rad();

		angleCancoder = new CANCoder(moduleConstants.encoderId());
		var angleCancoderConfig = new CANCoderConfiguration();
		angleCancoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		angleCancoderConfig.sensorDirection = angleEncoderInverted;
		angleCancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		angleCancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
		angleCancoderConfig.sensorCoefficient = Conversions.twoPi / Conversions.cancoderCountsPerRotation;
		angleCancoderConfig.unitString = "rad";
		angleCancoder.configAllSettings(angleCancoderConfig);

		angleMotor = new CANSparkMax(moduleConstants.angleMotorId(), CANSparkMax.MotorType.kBrushless);
		angleMotor.restoreFactoryDefaults();
		angleSparkPidController = angleMotor.getPIDController();
		// angleSparkPidController.setP(angleKp);
		// angleSparkPidController.setI(angleKi);
		// angleSparkPidController.setD(angleKd);
		// angleSparkPidController.setFF(angleKf);
		// angleSparkPidController.setOutputRange(-angleMaxPercentOutput, angleMaxPercentOutput);
		angleMotor.setSmartCurrentLimit(angleCurrentLimit_amp);
		angleMotor.setInverted(angleInverted);
		angleMotor.setIdleMode(angleIdleMode);
		angleMotor.setClosedLoopRampRate(angleRampTime_s);
		angleMotor.enableVoltageCompensation(12);

		// angleMotorAbsoluteEncoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		// angleMotorAbsoluteEncoder.setPositionConversionFactor(angleEncoderPositionFactor_rad);
		// angleMotorAbsoluteEncoder.setVelocityConversionFactor(angleEncoderVelocityFactor_radps);
		// angleMotorRelativeEncoder = angleMotor.getEncoder();
		// angleSparkPidController.setFeedbackDevice(angleMotorRelativeEncoder);

		// wrap betwee 0 and 2pi radians
		// angleSparkPidController.setPositionPIDWrappingEnabled(true);
		// angleSparkPidController.setPositionPIDWrappingMinInput(0);
		// angleSparkPidController.setPositionPIDWrappingMaxInput(Conversions.twoPi);

		// turn down frequency as we only log them, not needed for calculations
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40); // percent output
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40); // velocity, current
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500); // position
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); // alternate encoder
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); // duty cycle position
		angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity

		BuildManager.burnSpark(angleMotor);

		driveMotor = new WPI_TalonFX(moduleConstants.driveMotorId());
		var driveMotorConfig = new TalonFXConfiguration();
		driveMotorConfig.slot0.kP = driveKp;
		driveMotorConfig.slot0.kI = driveKi;
		driveMotorConfig.slot0.kD = driveKd;
		driveMotorConfig.slot0.kF = driveKf;
		driveMotorConfig.supplyCurrLimit = driveCurrentLimit;
		driveMotorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		driveMotorConfig.openloopRamp = driveOpenLoopRamp;
		driveMotorConfig.closedloopRamp = driveClosedLoopRamp;
		driveMotor.configAllSettings(driveMotorConfig);
		driveMotor.setNeutralMode(driveNeutralMode);
		driveMotor.setSelectedSensorPosition(0);
		driveMotor.setInverted(driveInverted);
		// only needed for logging
		driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
		driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 40);
	}

	private double getAbsoluteCancoder_rad() {
		return Conversions.angleModulus2pi(angleCancoder.getAbsolutePosition() - angleOffset_rad);
	}

	private double calculateDriveFeedforward(double velocity) {
		double percentage = driveFeedforward.calculate(velocity);
		return Math.min(percentage, 1.0);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		inputs.angleAbsolutePosition_rad = getAbsoluteCancoder_rad();

		inputs.drivePosition_rad = Conversions.falconToRadians(
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
		inputs.driveCurrent_A = driveMotor.getStatorCurrent();
		inputs.driveTemperature_C = driveMotor.getTemperature();

		// inputs.angleAbsolutePosition_rad = angleMotorAbsoluteEncoder.getPosition();
		inputs.anglePosition_rad = angleCancoder.getPosition();
		inputs.angleVelocity_radps = angleCancoder.getVelocity();
		inputs.angleAppliedPercentage = angleMotor.getAppliedOutput();
		inputs.angleCurrent_A = angleMotor.getOutputCurrent();
		inputs.angleTemperature_C = angleMotor.getMotorTemperature();
	}

	@Override
	public void setDriveMotorPercentage(double percentage) {
		driveMotor.set(ControlMode.PercentOutput, percentage);
	}

	@Override
	public void setDriveVelocity(double velocity) {
		driveMotor.set(
			ControlMode.Velocity,
			Conversions.mpsToFalcon(velocity, wheelCircumference_m, driveGearRatio),
			DemandType.ArbitraryFeedForward,
			calculateDriveFeedforward(velocity));
	}

	@Override
	public void setAngleVoltage(double voltage) {
		angleSparkPidController.setReference(voltage, CANSparkMax.ControlType.kVoltage);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
	}

	@Override
	public double getCharacterizationVelocity() {
		return Conversions.falconToRadians(driveMotor.getSelectedSensorVelocity(), driveGearRatio);
	}

	@Override
	public void close() throws Exception {
		driveMotor.DestroyObject();
		angleCancoder.DestroyObject();
		angleMotor.close();
	}
}
