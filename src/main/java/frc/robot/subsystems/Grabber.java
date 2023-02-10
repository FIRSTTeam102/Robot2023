package frc.robot.subsystems;

import static frc.robot.constants.GrabberConstants.*;

import frc.robot.commands.grabber.CloseGrabber;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

public class Grabber extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(motorId, CANSparkMax.MotorType.kBrushless);
	private DigitalInput closedSensor = new DigitalInput(closedSensorPin);
	private DigitalInput objectSensor = new DigitalInput(objectSensorPin);

	private GenericEntry shuffleboardClosed;

	public Grabber() {
		var shuffleboardGroup = Shuffleboard.getTab("Drive").getLayout("Grabber");
		shuffleboardClosed = shuffleboardGroup
			.add("closed", false)
			.withWidget(BuiltInWidgets.kBooleanBox)
			.getEntry();
	}

	public boolean objectDetected() {
		return objectSensor.get();
	}

	public boolean isClosed() {
		return closedSensor.get();
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
		if (objectDetected() && !closeGrabber.isScheduled())
			closeGrabber.schedule();

		shuffleboardClosed.setBoolean(isClosed());
	}
}
