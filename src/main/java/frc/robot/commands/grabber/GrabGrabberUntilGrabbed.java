package frc.robot.commands.grabber;

import frc.robot.constants.GrabberConstants;
import frc.robot.subsystems.Grabber;

public class GrabGrabberUntilGrabbed extends GrabGrabber {
	private int grabbedTicks;

	public GrabGrabberUntilGrabbed(Grabber grabber, double speed) {
		this(grabber, speed, GrabberConstants.grabbedTicks);
	}

	public GrabGrabberUntilGrabbed(Grabber grabber, double speed, int grabbedTicks) {
		super(grabber, speed);
		this.grabbedTicks = grabbedTicks;
	}

	@Override
	public boolean isFinished() {
		return grabber.hasGrabbed(grabbedTicks);
	}
}
