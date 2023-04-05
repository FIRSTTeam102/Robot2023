package frc.robot.commands.grabber;

import frc.robot.constants.GrabberConstants;
import frc.robot.subsystems.Grabber;

public class GrabGrabberUntilGrabbed extends GrabGrabber {
	public GrabGrabberUntilGrabbed(Grabber grabber, double speed) {
		super(grabber, speed);
	}

	@Override
	public boolean isFinished() {
		return grabber.hasGrabbed(GrabberConstants.grabbedTicks);
	}
}
