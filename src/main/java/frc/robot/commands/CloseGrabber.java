package frc.robot.commands;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;

	/**
	 * @param time_ms how long to close (in ms)
	*/
	public CloseGrabber(Grabber grabber, int time_ms) {
		this.grabber = grabber; // creates an instance of grabber from the subsystem
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.moveInward();
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() { // trigger to start the end
		return grabber.isGrabbed();
	}

	@Override
	public void end(boolean interrupted) {
		grabber.stop();
	}

}
