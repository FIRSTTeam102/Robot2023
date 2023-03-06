package frc.robot.commands.grabber;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabGrabber extends CommandBase {
	private Grabber grabber;
	private double speed;

	private double counter;

	/**
	 * 1st press: run at grab
	 * 1st hold->release: run at hold
	 * 2nd press: stop
	 */
	public GrabGrabber(Grabber grabber) {
		this(grabber, 0.3);
	}

	public GrabGrabber(Grabber grabber, double speed) {
		this.grabber = grabber;
		this.speed = speed;
		addRequirements(grabber);
	}

	private boolean stop = false;

	@Override
	public void initialize() {
		stop = grabber.inputs.percentOutput > 0;

		if (stop)
			grabber.stop();
		else
			grabber.move(speed);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		if (!stop)
			grabber.hold();
	}
}
