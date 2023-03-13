package frc.robot.commands.grabber;

import static frc.robot.constants.GrabberConstants.grabSpeed;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabGrabber extends CommandBase {
	protected Grabber grabber;
	private double speed;

	public GrabGrabber(Grabber grabber) {
		this(grabber, grabSpeed);
	}

	public GrabGrabber(Grabber grabber, double speed) {
		this.grabber = grabber;
		this.speed = speed;
		addRequirements(grabber);
	}

	@Override
	public void initialize() {
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
