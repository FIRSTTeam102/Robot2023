package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Passes status information to the Arduino coprocessor.
 * Each subsystem has it's own block of lights, and it/its
 * commands can update them independantly of other subsystems.
 * @see https://github.com/FIRSTTeam102/ArduinoLights2023
 */
public class Lights {
	private static SPI spi = new SPI(SPI.Port.kOnboardCS0);
	private static int[] spiCache = new int[16];

	static {
		spi.setClockRate(4000000);
		spi.setMode(SPI.Mode.kMode0);
		spi.setChipSelectActiveLow();
	}

	private Lights() {}

	public enum Group {
		Control(0), LMRetroreflective(1), LMAprilTag(2), Coral(3), ElevatorArm(4), Grabber(5), HumanPlayer(6);

		public final int value;

		Group(int value) {
			this.value = value;
		}
	};

	public enum Status {
		None(0b000), Right(0b001), Center(0b010), CenterRight(0b011), Left(0b100), LeftRight(0b101), LeftCenter(0b110), All(
			0b111),
		// game piece colors
		Cone(0b1000), Cube(0b1001);
		// todo: patterns up to 0b1111

		public final int value;

		Status(int value) {
			this.value = value;
		}
	};

	public enum ControlMode {
		Off(1), Regular(2), DisabledRed(3), DisabledBlue(4), Cube(5), Cone(6), Victory(7);

		public final int value;

		ControlMode(int value) {
			this.value = value;
		}
	}

	public static void setStatus(int group, int status) {
		if (group < 0 || group > 0b1111 || status < 0 || status > 0b1111) {
			DriverStation.reportError("invalid lights status message", true);
			return;
		}

		// already sent
		if (spiCache[group] == status)
			return;
		spiCache[group] = status;

		Integer message = (status << 4) ^ group;

		System.out.println("lights serial message: " + message);
		if (Robot.isReal())
			for (int i = 0; i < 3; i++)
				spi.write(new byte[] {message.byteValue()}, 1);
	}

	public static void setStatus(Group group, Status status) {
		setStatus(group.value, status.value);
	}

	public static void setControlMode(ControlMode controlMode) {
		setStatus(Group.Control.value, controlMode.value);
	}

	private static Status requestedGamePiece = Status.None;
	private static Timer requestedGamePieceClear = new Timer();

	public static Command requestGamePiece(Status piece) {
		return new InstantCommand(() -> {
			if (requestedGamePiece == piece) {
				requestedGamePiece = Status.None; // toggle request
				requestedGamePieceClear.stop();
				requestedGamePieceClear.reset();
			} else {
				requestedGamePiece = piece;
				requestedGamePieceClear.restart();
			}
			setStatus(Group.HumanPlayer, requestedGamePiece);
		});
	}

	public static void periodic() {
		if (requestedGamePieceClear.hasElapsed(10)) {
			requestedGamePiece = Status.None;
			setStatus(Group.HumanPlayer, requestedGamePiece);
			requestedGamePieceClear.stop();
			requestedGamePieceClear.reset();
		}

		// todo: not doing this at midnight, should be within auto error and methods in respictive classes,
		// offload to lights periodic
		byte eleArmStatus = 0;
		if (RobotContainer.getInstance().elevator.withinTargetPosition())
			eleArmStatus += 0b001;
		if (RobotContainer.getInstance().arm.withinTargetPosition())
			eleArmStatus += 0b100;
		if (eleArmStatus == 0b101)
			eleArmStatus = 0b111;
		Lights.setStatus(Lights.Group.ElevatorArm.value, eleArmStatus);
	}
}
