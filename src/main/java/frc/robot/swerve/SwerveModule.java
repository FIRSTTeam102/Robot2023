package frc.robot.swerve;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Conversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
	public int moduleNumber;

	private double angleOffset;
	private WPI_TalonFX angleMotor;
	private WPI_TalonFX driveMotor;
	private WPI_CANCoder angleEncoder;
	private Rotation2d lastAngle;

	// todo: get real sim paramaters
	TalonFXSimCollection angleMotorSim;
	FlywheelSim angleWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.angleGearRatio, 2);
	TalonFXSimCollection driveMotorSim;
	FlywheelSim driveWheelSim = new FlywheelSim(DCMotor.getFalcon500(1), SwerveConstants.driveGearRatio, 2);
	CANCoderSimCollection angleEncoderSim;

	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,
		SwerveConstants.driveKv, SwerveConstants.driveKa);

	public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
		this.moduleNumber = moduleNumber;
		angleOffset = moduleConstants.angleOffset_deg();

		angleEncoder = new WPI_CANCoder(moduleConstants.encoderId());
		angleEncoder.configFactoryDefault();
		angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
		angleEncoder.configSensorDirection(SwerveConstants.encoderInverted);
		angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		angleEncoderSim = angleEncoder.getSimCollection();

		angleMotor = new WPI_TalonFX(moduleConstants.angleMotorId());
		angleMotor.configFactoryDefault();
		angleMotor.config_kP(0, SwerveConstants.angleKp);
		angleMotor.config_kI(0, SwerveConstants.angleKi);
		angleMotor.config_kD(0, SwerveConstants.angleKd);
		angleMotor.config_kF(0, SwerveConstants.angleKf);
		// angleMotor.configSupplyCurrentLimit(SwerveConstants.angleCurrentLimit);
		angleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		angleMotor.setInverted(SwerveConstants.angleInverted);
		angleMotor.setNeutralMode(SwerveConstants.angleNeutralMode);
		resetToAbsolute();
		angleMotorSim = angleMotor.getSimCollection();

		driveMotor = new WPI_TalonFX(moduleConstants.driveMotorId());
		driveMotor.configFactoryDefault();
		driveMotor.config_kP(0, SwerveConstants.driveKp);
		driveMotor.config_kI(0, SwerveConstants.driveKi);
		driveMotor.config_kD(0, SwerveConstants.driveKd);
		driveMotor.config_kF(0, SwerveConstants.driveKf);
		// driveMotor.configSupplyCurrentLimit(SwerveConstants.driveCurrentLimit);
		driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		driveMotor.configOpenloopRamp(SwerveConstants.openLoopRamp);
		driveMotor.configClosedloopRamp(SwerveConstants.closedLoopRamp);
		driveMotor.setInverted(SwerveConstants.driveInverted);
		driveMotor.setNeutralMode(SwerveConstants.driveNeutralMode);
		driveMotor.setSelectedSensorPosition(0);
		driveMotorSim = driveMotor.getSimCollection();

		lastAngle = getState().angle;
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		// custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
		desiredState = CTREModuleState.optimize(desiredState, getState().angle);
		// System.out.format("a%f s%f\n", desiredState.angle.getDegrees(), desiredState.speedMetersPerSecond);

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed_mps;
			// System.out.format("%d percent %f; state angle %f\n", moduleNumber, percentOutput,
			// desiredState.angle.getDegrees());
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			double velocity = Conversions.mpsToFalcon(desiredState.speedMetersPerSecond,
				SwerveConstants.wheelCircumference_m, SwerveConstants.driveGearRatio);
			// System.out.format("%d velocity %f\n", moduleNumber, velocity);
			driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
				feedforward.calculate(desiredState.speedMetersPerSecond));
		}

		// prevent jitter if speed is less then 1%
		var angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed_mps * 0.01))
			? lastAngle
			: desiredState.angle;
		angleMotor.set(ControlMode.Position,
			Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio)); // todo: is this right?
		lastAngle = angle;
	}

	private void resetToAbsolute() {
		double absolutePosition = getEncoderPos().getDegrees() - angleOffset;
		angleMotor.setSelectedSensorPosition(absolutePosition);
	}

	// todo: what are we actually using the cancoder for?
	public Rotation2d getEncoderPos() {
		return Rotation2d
			.fromDegrees(Conversions.cancoderToDegrees(angleEncoder.getAbsolutePosition(), SwerveConstants.angleGearRatio));
	}

	public SwerveModuleState getState() {
		double velocity = Conversions.falconToMps(driveMotor.getSelectedSensorVelocity(),
			SwerveConstants.wheelCircumference_m, SwerveConstants.driveGearRatio);
		Rotation2d angle = Rotation2d.fromDegrees(Conversions
			.falconToDegrees(angleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModulePosition getPosition() {
		double position = Conversions.falconToM(driveMotor.getSelectedSensorPosition(),
			SwerveConstants.wheelCircumference_m, SwerveConstants.driveGearRatio);
		Rotation2d angle = Rotation2d.fromDegrees(Conversions
			.falconToDegrees(angleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
		return new SwerveModulePosition(position, angle);
	}

	/**
	 * @param dt tick time (s)
	 */
	public void simulationPeriodic(double dt) {
		angleMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
		angleWheelSim.setInputVoltage(angleMotorSim.getMotorOutputLeadVoltage());
		// System.out.println(angleMotorSim.getMotorOutputLeadVoltage());
		angleWheelSim.update(dt);
		angleMotorSim.setIntegratedSensorVelocity(
			(int) Conversions.rpmToFalcon(angleWheelSim.getAngularVelocityRPM(), SwerveConstants.angleGearRatio));
		angleMotorSim.addIntegratedSensorPosition(
			(int) Conversions.mToFalcon(angleWheelSim.getAngularVelocityRPM() * (dt * 60), 1,
				SwerveConstants.angleGearRatio));

		angleEncoderSim.setVelocity(
			(int) Conversions.rpmToCancoder(angleWheelSim.getAngularVelocityRPM(), SwerveConstants.angleGearRatio));
		angleEncoderSim.addPosition(
			(int) Conversions.mToCancoder(angleWheelSim.getAngularVelocityRPM() * (dt * 60),
				1, SwerveConstants.angleGearRatio));

		driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
		driveWheelSim.setInputVoltage(driveMotorSim.getMotorOutputLeadVoltage());
		driveWheelSim.update(dt);
		driveMotorSim.setIntegratedSensorVelocity(
			(int) Conversions.rpmToFalcon(driveWheelSim.getAngularVelocityRPM(), SwerveConstants.driveGearRatio));
		driveMotorSim.addIntegratedSensorPosition((int) Conversions
			.mToFalcon(driveWheelSim.getAngularVelocityRPM() * (dt * 60), 1, SwerveConstants.driveGearRatio));

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(angleWheelSim.getCurrentDrawAmps(),
				driveWheelSim.getCurrentDrawAmps()));
	}
}