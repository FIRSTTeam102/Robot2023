package frc.robot.subsystems;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * Passes status information to the Arduino coprocessor.
 * @see https://github.com/FIRSTTeam102/ArduinoLights2023
 * 
 * Each subsystem has it's own block of lights, and it/its
 * commands can update them independantly of other subsystems.
 */
public class Lights {
	private static SerialPort serial = new SerialPort(9600, SerialPort.Port.kMXP,
		8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);

	private Lights() {}

	public enum Subsystem {
		Control(0), Vision(1), Grabber(2), Elevator(3), Arm(4);

		public final int value;

		Subsystem(int value) {
			this.value = value;
		}
	};

	public enum Status {
		None(0b000), Right(0b001), Center(0b010), CenterRight(0b011), Left(0b100), LeftRight(0b101), LeftCenter(0b110), All(
			0b111);
		// todo: patterns up to 0b1111

		public final int value;

		Status(int value) {
			this.value = value;
		}
	};

	public enum ControlMode {
		Off(1), Regular(2), DisabledRed(3), DisabledBlue(4);

		public final int value;

		ControlMode(int value) {
			this.value = value;
		}
	}

	private static void setStatus(int subsystem, int status) {
		if (subsystem < 0 || subsystem > 0b1111 || status < 0 || status > 0b1111)
			throw new RuntimeException("invalid lights status message");
		Integer message = (status << 4) ^ subsystem;
		System.out.println("Lights serial message: " + message);
		if (Robot.isReal())
			serial.write(new byte[] {message.byteValue()}, 1);
	}

	public static void setStatus(Subsystem subsystem, Status status) {
		setStatus(subsystem.value, status.value);
	}

	public static void setControlMode(ControlMode controlMode) {
		setStatus(Subsystem.Control.value, controlMode.value);
	}
}
