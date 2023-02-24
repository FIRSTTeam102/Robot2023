package frc.robot.subsystems;

import static frc.robot.constants.GrabberConstants.motorId;

import frc.robot.Robot;
import frc.robot.ScoringMechanism2d;
import frc.robot.constants.GrabberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase implements AutoCloseable {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);

	public Grabber() {
		motor.setInverted(true);

		// sim only works with velocity control
		// if (Robot.isSimulation())
		// REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNeo550(1));
	}

	public boolean currentLimitReached = false;

	public void move(double speed) {
		motor.set(speed);
	}

	public void stop() {
		motor.set(0);
	}

	public void hold() {
		motor.set(.12);
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		ScoringMechanism2d.setGrabber(inputs.percentOutput);

		// read current once every cycle so results don't change during a cycle
		currentLimitReached = (inputs.current_A >= GrabberConstants.currentLimit_A);

		// if (currentLimitReached)
		// stop();
	}

	/**
	 * inputs
	 */
	@AutoLog
	public static class GrabberIOInputs {
		public double percentOutput = 0.0;
		public double current_A = 0.0;
		public double temperature_C = 0.0;
	}

	public GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

	private void updateInputs(GrabberIOInputs inputs) {
		inputs.percentOutput = Robot.isReal() ? motor.getAppliedOutput() : motor.get();
		inputs.current_A = motor.getOutputCurrent();
		inputs.temperature_C = motor.getMotorTemperature();
	}

	@Override
	public void close() {
		motor.close();
	}
}
