package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

public class GrabGrabberUntilGrabbed extends GrabGrabber {
	public GrabGrabberUntilGrabbed(Grabber grabber) {
		super(grabber);
	}

	@Override
	public boolean isFinished() {
		return grabber.hasGrabbed();
	}
}
