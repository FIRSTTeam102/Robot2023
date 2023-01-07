package frc.robot.autos;

import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
	/** Example static factory for an autonomous command. */
	public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
		return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
