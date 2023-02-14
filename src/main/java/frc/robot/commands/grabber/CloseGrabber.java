package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseGrabber extends CommandBase {
	private Grabber grabber;
	private double speed;

	public CloseGrabber(Grabber grabber, double speed) {
		this.grabber = grabber;
		this.speed = speed;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		grabber.move(speed);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return grabber.currentLimitReached();
	}

	@Override
	public void end(boolean interrupted) {
		grabber.stop();
	}
}
