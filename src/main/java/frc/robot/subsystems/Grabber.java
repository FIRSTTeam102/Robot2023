package frc.robot.subsystems;

import static frc.robot.constants.GrabberConstants.*;

import frc.robot.Robot;

import frc.robot.commands.grabber.CloseGrabber;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Grabber extends SubsystemBase implements AutoCloseable {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);
	private DigitalInput closedSensor = new DigitalInput(closedSensorPin);
	private DigitalInput objectSensor = new DigitalInput(objectSensorPin);

	private GenericEntry shuffleboardClosed;

	public Grabber() {
		var shuffleboardGroup = Shuffleboard.getTab("Drive").getLayout("Grabber", BuiltInLayouts.kList);
		shuffleboardClosed = shuffleboardGroup
			.add("closed", false)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.getEntry();
	}

	public boolean objectDetected() {
		return inputs.objectDetected;
	}

	public boolean isClosed() {
		return inputs.closed;
	}

	public void moveInward() {
		motor.set(speed);
	}

	public void moveOutward() {
		motor.set(-speed);
	}

	public void stop() {
		motor.set(0);
	}

	CloseGrabber closeGrabber = new CloseGrabber(this);

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.getInstance().processInputs(getName(), inputs);

		if (inputs.objectDetected && !inputs.closed && !closeGrabber.isScheduled())
			closeGrabber.schedule();

		shuffleboardClosed.setBoolean(inputs.closed);
	}

	/**
	 * inputs
	 */
	@AutoLog
	public static class GrabberIOInputs {
		public boolean objectDetected = false;
		public boolean closed = false;
		public double percentOutput = 0.0;
	}

	public GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

	private void updateInputs(GrabberIOInputs inputs) {
		inputs.percentOutput = motor.get(); // getAppliedOutput(); ?

		if (Robot.isReal()) {
			inputs.objectDetected = objectSensor.get();
			inputs.closed = closedSensor.get();
		} else {
			inputs.closed = inputs.percentOutput > 0 ? false
				: inputs.percentOutput < 0 ? true
					: inputs.closed;
		}
	}

	@Override
	public void close() {
		motor.close();
		objectSensor.close();
		closedSensor.close();
		closeGrabber.cancel();
	}
}
