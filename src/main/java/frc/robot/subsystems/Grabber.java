package frc.robot.subsystems;

import static frc.robot.constants.GrabberConstants.*;

import frc.robot.Robot;
import frc.robot.util.BuildManager;
import frc.robot.util.ScoringMechanism2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase implements AutoCloseable {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);

	public Grabber() {
		motor.setInverted(true);

		motor.setSmartCurrentLimit(smartCurrentLimit_A);
		motor.setSecondaryCurrentLimit(hardCurrentLimit_A);

		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		BuildManager.burnSpark(motor);

		// sim only works with velocity control
		// if (Robot.isSimulation())
		// REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNeo550(1));
	}

	public void move(double speed) {
		motor.set(speed);
	}

	public void stop() {
		motor.set(0);
	}

	public void hold() {
		motor.set(holdSpeed);
	}

	private int overCurrentCounter = 0;

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		ScoringMechanism2d.setGrabber(inputs.percentOutput);

		if (inputs.current_A > smartCurrentLimit_A) {
			overCurrentCounter++;
			if (overCurrentCounter > 10) {
				overCurrentCounter = 0;
				DriverStation.reportWarning("grabber over current!", false);
				stop();
			}
		}
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
