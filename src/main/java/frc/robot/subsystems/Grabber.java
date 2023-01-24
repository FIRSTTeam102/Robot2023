package frc.robot.subsystems;

import frc.robot.Constants.GrabberConstants;
import frc.robot.commands.CloseGrabber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

public class Grabber extends SubsystemBase {
	public boolean isClosed = false;
	private CANSparkMax motor = new CANSparkMax(GrabberConstants.motorId, CANSparkMax.MotorType.kBrushless);
	private DigitalInput sensor = new DigitalInput(GrabberConstants.sensorPin);

	/** Creates a new Grabber. */
	public Grabber() {
		motor.set(0);
	}

	public boolean isGrabbed() {
		return sensor.get();
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

	CloseGrabber closeGrabber = new CloseGrabber(this, 250);

	@Override
	public void periodic() {

	}
}
