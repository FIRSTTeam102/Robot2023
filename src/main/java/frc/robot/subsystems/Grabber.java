package frc.robot.subsystems;

import frc.robot.Constants.GrabberConstants;
import frc.robot.commands.CloseGrabber;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

public class Grabber extends SubsystemBase {
	private DigitalInput closedSensor = new DigitalInput(GrabberConstants.closedSensorPin);
	private CANSparkMax motor = new CANSparkMax(GrabberConstants.motorId, CANSparkMax.MotorType.kBrushless);
	private DigitalInput objectSensor = new DigitalInput(GrabberConstants.objectSensorPin);
	private GenericEntry ShuffleboardEntry;

	/** Creates a new Grabber. */
	public Grabber() {
		motor.set(0);
		ShuffleboardEntry = Shuffleboard.getTab("Drive").add("Is Closed?", false)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.getEntry(); // Get an entry widget
	}

	public boolean objectDetected() {
		return objectSensor.get();
	}

	public boolean isClosed() {
		return closedSensor.get();
	}

	public void moveInward() {
		motor.set(GrabberConstants.speed);
	}

	public void moveOutward() {
		motor.set(-GrabberConstants.speed);
	}

	public void stop() {
		motor.set(0);
	}

	CloseGrabber closeGrabber = new CloseGrabber(this);

	@Override
	public void periodic() {
		if (objectDetected() && !closeGrabber.isScheduled()) {
			closeGrabber.schedule();
		}
		ShuffleboardEntry.setBoolean(isClosed());
	}
}
