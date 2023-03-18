package frc.robot.subsystems;

import static frc.robot.constants.ElevatorConstants.*;

import frc.robot.Robot;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.util.BuildManager;
import frc.robot.util.ScoringMechanism2d;
import frc.robot.util.SendableSparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import lombok.Getter;
import lombok.Setter;

public class Elevator extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, MotorType.kBrushless);
	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch topSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch bottomSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	// feedforward is for velocity control mode
	// private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

	@Getter
	private double targetPosition_m = 0.0;

	@Getter
	// if within module bounds so arm knows to not go down too far
	private static boolean inDangerZone = false;

	@Setter
	private DoubleSupplier manualModeInput = null;
	public boolean inManualMode = true;

	public Elevator() {
		motor.setIdleMode(IdleMode.kBrake);

		motor.setSmartCurrentLimit(50);
		motor.setSecondaryCurrentLimit(65);

		topSwitch.enableLimitSwitch(true);
		bottomSwitch.enableLimitSwitch(true);

		pidController.setP(kP);
		pidController.setD(kD);
		pidController.setI(kI);
		pidController.setIZone(kIZone);
		pidController.setOutputRange(minOuput, mMaxOutput);

		encoder.setPositionConversionFactor(conversionFactor_m_per_rotation);
		encoder.setVelocityConversionFactor(conversionFactor_mps_per_rpm);

		BuildManager.burnSpark(motor);

		SmartDashboard.putData(new SendableSparkMaxPIDController(pidController, ControlType.kPosition, "elevator pid"));

		// if (Robot.isSimulation()) revPhysicsSim.addSparkMax(motor, DCMotor.getNEO(1));
	}

	public boolean getTopSwitch() {
		return inputs.topSwitch;
	}

	public boolean getBottomSwitch() {
		return inputs.bottomSwitch;
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, ControlType.kDutyCycle, 0, feedForward_V);
	}

	public void stop() {
		pidController.setReference(0, ControlType.kDutyCycle, 0, feedForward_V);
	}

	public void killMotor() {
		inManualMode = true;
		pidController.setReference(0, ControlType.kVoltage, 0, 0);
	}

	public void setPosition(double position_m) {
		targetPosition_m = position_m;
		// double feed = feedforward.calculate();
		pidController.setReference(
			MathUtil.clamp(targetPosition_m, Arm.isInDangerZone() ? dangerZone_m : 0, maxHeight_m),
			ControlType.kPosition, 0, feedForward_V);
	}

	public boolean withinTargetPosition() {
		return Math.abs(inputs.position_m - targetPosition_m) < AutoConstants.elevatorTolerance_m;
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		Logger.getInstance().recordOutput("Elevator/targetPosition_m", targetPosition_m);

		ScoringMechanism2d.elevator.setLength(inputs.position_m);

		// these won't do anything in simulation
		if (inputs.bottomSwitch)
			encoder.setPosition(minHeight_m);
		if (inputs.topSwitch)
			encoder.setPosition(maxHeight_m);

		inDangerZone = (encoder.getPosition() < dangerZone_m);

		if (manualModeInput != null
			&& Math.abs(manualModeInput.getAsDouble()) > OperatorConstants.operatorJoystickDeadband)
			inManualMode = true;
	}

	/**
	 * inputs
	 */
	public final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

	@AutoLog
	public static class ElevatorIOInputs {
		public double position_m = 0.0;
		public double velocity_mps = 0.0;
		public double current_A = 0.0;
		public double dutyCycle = 0;
		public double busVoltage_V = 0;
		public double temperature_C = 0.0;
		public boolean topSwitch = false;
		public boolean bottomSwitch = false;
	}

	// rev has pretty bad simulation support, so we can just completely bypass it during sim
	// REVPhysicsSim revPhysicsSim = new REVPhysicsSim();
	ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(1),
		gearRatio,
		carriageMass_kg,
		drumRadius_m,
		minHeight_m,
		maxHeight_m,
		false // switch if we're using velocity control
	);

	private void updateInputs(ElevatorIOInputs inputs) {
		// todo: sim position control
		if (Robot.isSimulation()) {
			elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());
			elevatorSim.update(Constants.loopPeriod_s);

			inputs.position_m = elevatorSim.getPositionMeters();
			inputs.velocity_mps = elevatorSim.getVelocityMetersPerSecond();
			inputs.current_A = elevatorSim.getCurrentDrawAmps();
			inputs.topSwitch = elevatorSim.hasHitUpperLimit();
			inputs.bottomSwitch = elevatorSim.hasHitLowerLimit();

			encoder.setPosition(inputs.position_m);
			// motor.setVoltage(motor.get() * RobotController.getBatteryVoltage());
			// revPhysicsSim.run();

			RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(inputs.current_A));
		} else {
			inputs.position_m = encoder.getPosition();
			inputs.velocity_mps = encoder.getVelocity();
			inputs.current_A = motor.getOutputCurrent();
			inputs.dutyCycle = motor.getAppliedOutput();
			inputs.busVoltage_V = motor.getBusVoltage();
			inputs.temperature_C = motor.getMotorTemperature();
			inputs.topSwitch = topSwitch.isPressed();
			inputs.bottomSwitch = bottomSwitch.isPressed();
		}
	}
}