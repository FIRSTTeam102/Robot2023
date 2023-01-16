package frc.robot.subsystems;

import frc.robot.Constants.ClawConstants;
import frc.robot.commands.CloseClaw;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Claw extends SubsystemBase {
	public boolean isClosed = false;
	private TalonSRX motor = new TalonSRX(ClawConstants.motorId);
	private DigitalInput sensor = new DigitalInput(ClawConstants.sensorPin);

	/** Creates a new Claw. */
	public Claw() {}

	public void moveInward() {
		motor.set(ControlMode.PercentOutput, ClawConstants.speed);
	}

	public void moveOutward() {
		motor.set(ControlMode.PercentOutput, -ClawConstants.speed);
	}

	public void stop() {
		motor.set(ControlMode.PercentOutput, 0);
	}

	CloseClaw closeClaw = new CloseClaw(this, 250);

	@Override
	public void periodic() {
		// get sensor input and enter conditional if something returns
		if (sensor.get() && !closeClaw.isScheduled()) {
			closeClaw.schedule(); // close or moveinward
		}
	}
}
