package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;

	public CloseGrabber(Grabber grabber) {
		this.grabber = grabber;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.moveInward();
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return grabber.isClosed();
	}

	@Override
	public void end(boolean interrupted) {
		grabber.stop();
	}
}
