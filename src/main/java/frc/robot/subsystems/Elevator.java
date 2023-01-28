package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(ElevatorConstants.motorPort, MotorType.kBrushless);
	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch topSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch bottomSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	// todo: feedforward is for velocity control mode so we do we want to or just use position
	// private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS,
	// ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

	private double targetPosition_m = 0.0;

	/** Creates a new Elevator. */
	public Elevator() {
		// todo: do we want to fall back down when not being told to hold?
		motor.setIdleMode(IdleMode.kCoast);

		topSwitch.enableLimitSwitch(true);
		bottomSwitch.enableLimitSwitch(true);

		pidController.setP(ElevatorConstants.kP);
		pidController.setD(ElevatorConstants.kD);
		pidController.setI(ElevatorConstants.kI);
		pidController.setIZone(ElevatorConstants.kIZone);
		pidController.setOutputRange(ElevatorConstants.kMinOuput, ElevatorConstants.kMaxOutput);

		encoder.setPositionConversionFactor(ElevatorConstants.conversionFactor_meters_per_rotation);

		// if (Robot.isSimulation())
		// revPhysicsSim.addSparkMax(motor, DCMotor.getNEO(1));

		SmartDashboard.putData("Elevator", mech);
	}

	public boolean getTopSwitch() {
		return inputs.topSwitch;
	}

	public boolean getBottomSwitch() {
		return inputs.bottomSwitch;
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	public void setPosition(double position_m) {
		targetPosition_m = position_m;
		// double feed = feedforward.calculate();
		pidController.setReference(position_m, CANSparkMax.ControlType.kSmartMotion, 0, 0.0);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		// these won't do anything in simulation
		if (inputs.bottomSwitch)
			encoder.setPosition(ElevatorConstants.minHeight_m);
		if (inputs.topSwitch)
			encoder.setPosition(ElevatorConstants.maxHeight_m);

		mechElevator.setLength(inputs.position_m);
	}

	/**
	 * inputs
	 */
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

	@AutoLog
	public static class ElevatorIOInputs {
		public double position_m = 0.0;
		public double velocity_mps = 0.0;
		public boolean topSwitch = false;
		public boolean bottomSwitch = false;
	}

	// rev has pretty bad simulation support, so we can just completely bypass it during sim
	// REVPhysicsSim revPhysicsSim = new REVPhysicsSim();
	ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(1),
		ElevatorConstants.gearRatio,
		ElevatorConstants.carriageMass_kg,
		ElevatorConstants.drumRadius_m,
		ElevatorConstants.minHeight_m,
		ElevatorConstants.maxHeight_m,
		false // todo: switch if we're using velocity mode
	);

	private void updateInputs(ElevatorIOInputs inputs) {
		// todo: sim position control?
		if (Robot.isSimulation()) {
			elevatorSim.setInput(motor.get() * RobotController.getBatteryVoltage());
			elevatorSim.update(Constants.loopPeriod_s);

			inputs.position_m = elevatorSim.getPositionMeters();
			inputs.velocity_mps = elevatorSim.getVelocityMetersPerSecond();
			inputs.topSwitch = elevatorSim.hasHitUpperLimit();
			inputs.bottomSwitch = elevatorSim.hasHitLowerLimit();

			encoder.setPosition(inputs.position_m);
			// motor.setVoltage(motor.get() * RobotController.getBatteryVoltage());
			// revPhysicsSim.run();

			RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
		} else {
			inputs.position_m = encoder.getPosition();
			inputs.velocity_mps = encoder.getVelocity();
			inputs.topSwitch = topSwitch.isPressed();
			inputs.bottomSwitch = bottomSwitch.isPressed();
		}
	}

	/**
	 * mechanism preview
	 * todo: combine with scissor and grabber when finished
	 */
	private final Mechanism2d mech = new Mechanism2d(3, 4);
	private final MechanismRoot2d mechRoot = mech.getRoot("elevator root", 2, 0);
	private final MechanismLigament2d mechElevator = mechRoot
		.append(new MechanismLigament2d("elevator", inputs.position_m, 90));

	// todo: figure out livewindow vs advantagekit
	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addBooleanProperty("at top", () -> inputs.topSwitch, null);
		builder.addBooleanProperty("at bottom", () -> inputs.bottomSwitch, null);
		builder.addDoubleProperty("position", () -> inputs.position_m, null); // todo: setter
	}
}