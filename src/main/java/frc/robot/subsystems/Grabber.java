package frc.robot.subsystems;

import static frc.robot.constants.GrabberConstants.motorId;

import frc.robot.Robot;
import frc.robot.constants.GrabberConstants;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase implements AutoCloseable {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);
	// private DigitalInput closedSensor = new DigitalInput(closedSensorPin);
	// private DigitalInput objectSensor = new DigitalInput(objectSensorPin);

	// private GenericEntry shuffleboardClosed;

	public Grabber() {
		var shuffleboardGroup = Shuffleboard.getTab("Drive").getLayout("Grabber", BuiltInLayouts.kList);
		// shuffleboardClosed = shuffleboardGroup
		// .add("closed", false)
		// .withWidget(BuiltInWidgets.kBooleanBox)
		// .getEntry();
	}

	// Depracated: no sensor
	//
	// public boolean objectDetected() {
	// return inputs.objectDetected;
	// }

	// Depracated: no sensor
	//
	// public boolean isClosed() {
	// return inputs.closed;
	// }

	public boolean currentLimitReached = false;

	public void move(double speed) {
		motor.set(speed);
	}

	public void stop() {
		motor.set(0);
	}

	// CloseGrabber closeGrabber = new CloseGrabber(this);

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		// Read current once every cycle so results don't change during a cycle
		currentLimitReached = (motor.getOutputCurrent() >= GrabberConstants.currentLimit_A);

		// if (inputs.objectDetected && !inputs.closed && !closeGrabber.isScheduled())
		// closeGrabber.schedule();

		// shuffleboardClosed.setBoolean(inputs.closed);

		if (currentLimitReached) {
			stop();
		}
	}

	/**
	 * inputs
	 */
	@AutoLog
	public static class GrabberIOInputs {
		// public boolean objectDetected = false;
		// public boolean closed = false;
		public double percentOutput = 0.0;
	}

	public GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

	private void updateInputs(GrabberIOInputs inputs) {
		inputs.percentOutput = motor.get(); // getAppliedOutput(); ?

		if (Robot.isReal()) {
			// inputs.objectDetected = objectSensor.get();
			// inputs.closed = closedSensor.get();
		} else {
			// inputs.closed = inputs.percentOutput > 0 ? false
			// : inputs.percentOutput < 0 ? true
			// : inputs.closed;
		}
	}

	@Override
	public void close() {
		motor.close();
		// objectSensor.close();
		// closedSensor.close();
		// closeGrabber.cancel();
	}
}
