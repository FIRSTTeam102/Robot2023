package frc.robot.commands.grabber;

import frc.robot.constants.GrabberConstants;
import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabGrabber extends CommandBase {
	protected Grabber grabber;
	private double speed;

	public GrabGrabber(Grabber grabber, double speed) {
		this.grabber = grabber;
		this.speed = speed;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
		if (grabber.hasGrabbed(GrabberConstants.grabbedTicks * 2))
			grabber.move(Math.max(GrabberConstants.holdSpeed, speed * 0.5)); // prevent it from stalling too much
		else
			grabber.move(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		grabber.hold();
	}
}
