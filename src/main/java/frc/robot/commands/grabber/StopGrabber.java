package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class StopGrabber extends InstantCommand {
	Grabber grabber;

	public StopGrabber(Grabber grabber) {
		this.grabber = grabber;
	}

	@Override
	public void initialize() {
		grabber.stop();
	}
}
