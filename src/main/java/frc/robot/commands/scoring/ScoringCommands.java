package frc.robot.commands.scoring;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoringCommands {
	public static CommandBase downThenRelease(Elevator elevator, Grabber grabber) {
		return sequence();
	}
}
